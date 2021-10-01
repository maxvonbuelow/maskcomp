
#include "openjpeg.h"
#include "image.h"
#include <vector>
#include <cstring>
#include <cstdlib>
#include <stdexcept>
#include <sstream>
#include <chrono>
#include <atomic>
#include <sys/stat.h>
#include "image_io.h"

static void error_callback(const char *msg, void *client_data)
{
	std::cerr << "Error: " << msg << std::endl;
}
static void warning_callback(const char *msg, void *client_data)
{
	std::cout << "Warning: " << msg << std::endl;
}
static void info_callback(const char *msg, void *client_data)
{
	std::cout << "Info: " << msg << std::endl;
}

struct ImageViewJP2MASTER {
	const uint16_t *data;
	uint32_t _w, _h;
	uint32_t _nc, _bitdepth;

	uint32_t w() const
	{
		return _w;
	}
	uint32_t h() const
	{
		return _h;
	}
	uint32_t nc() const
	{
		return _nc;
	}
	uint32_t bitdepth() const
	{
		return _bitdepth;
	}

	ImageViewJP2MASTER(const uint16_t *data, uint32_t w, uint32_t h, uint32_t nc, uint32_t bitdepth) : data(data), _w(w), _h(h), _nc(nc), _bitdepth(bitdepth)
	{}
	uint16_t get(uint32_t x, uint32_t y, uint32_t c) const
	{
		uint32_t i = x + y * _w;
		return data[i * _nc + c];
	}
};

struct ImageViewLossless {
	const uint16_t *data;
	const uint8_t *mask;
	uint32_t _w, _h;
	uint32_t _nc, _bitdepth;

	uint32_t w() const
	{
		return _w;
	}
	uint32_t h() const
	{
		return _h;
	}
	uint32_t nc() const
	{
		return _nc + 1;
	}
	uint32_t bitdepth() const
	{
		return _bitdepth;
	}

	ImageViewLossless(const uint16_t *data, const uint8_t *mask, uint32_t w, uint32_t h, uint32_t nc, uint32_t bitdepth) : data(data), mask(mask), _w(w), _h(h), _nc(nc), _bitdepth(bitdepth)
	{}
	uint16_t get(uint32_t x, uint32_t y, uint32_t c) const
	{
		uint32_t i = x + y * _w;
		if (c == _nc) return !!mask[i];
		return mask[i] ? data[i * _nc + c] : 0;
	}
};
struct ImageViewLossy {
	const uint16_t *data;
	const uint8_t *mask;
	uint32_t _w, _h;
	uint32_t _nc, _bitdepth;
	int _r;

	uint32_t w() const
	{
		return _w;
	}
	uint32_t h() const
	{
		return _h;
	}
	uint32_t nc() const
	{
		return _nc;
	}
	uint32_t bitdepth() const
	{
		return _bitdepth;
	}

	ImageViewLossy(const uint16_t *data, const uint8_t *mask, uint32_t w, uint32_t h, uint32_t nc, uint32_t bitdepth, uint32_t radius) : data(data), mask(mask), _w(w), _h(h), _nc(nc), _r(radius), _bitdepth(bitdepth)
	{}
	uint8_t get_mask_erosion(int x, int y) const
	{
		for (int yy = -_r; yy <= _r; ++yy) {
			for (int xx = -_r; xx <= _r; ++xx) {
				if (xx * xx + yy * yy > _r * _r) continue; // circle
				int xxx = x + xx, yyy = y + yy;
				if (xxx < 0 || yyy < 0 || xxx >= _w || yyy >= _h) continue;
				uint32_t i = xxx + yyy * _w;
				if (!mask[i]) return 0;
			}
		}
		return 255;
	}
	uint16_t get(uint32_t x, uint32_t y, uint32_t c) const
	{
		uint32_t i = x + y * _w;
		return !get_mask_erosion(x, y) ? data[i * _nc + c] : 0;
	}
};

image_s bayer(const image_s &img)
{
	image_s datav(img.width() * 2, img.height() * 2);
	for (int y = 0; y < img.height(); ++y) {
		for (int x = 0; x < img.width(); ++x) {
			for (int c = 0; c < img.channels(); ++c) {
				uint16_t v = img.at2d(x, y, c);
				datav.at2d(x * 2, y * 2) = std::max((int)img.at2d(x, y, 0) - (1 << 10), 0);
				datav.at2d(x * 2 + 1, y * 2) = std::max((int)img.at2d(x, y, 1) - (1 << 10), 0);
				datav.at2d(x * 2, y * 2 + 1) = std::max((int)img.at2d(x, y, 2) - (1 << 10), 0);
				datav.at2d(x * 2 + 1, y * 2 + 1) = std::max((int)img.at2d(x, y, 3) - (1 << 10), 0);
			}
		}
	}
	return datav;
}
image_b get_vis(const image_s &img)
{
	image_b datav(img.width(), img.height(), 3);
	for (int i = 0; i < img.pixels(); ++i) {
		for (int c = 0; c < img.channels(); ++c) {
			uint16_t v = img.at(i, c);
			if (c != 2 && img.channels() == 4) datav.at(i, c > 1 ? c - 1 : c) = std::min(1., (double)v / std::numeric_limits<uint16_t>::max() * 8) * 255.f;
			if (img.channels() == 3) datav.at(i, c) = v;
		}
	}
	return datav;
}
image_s get_visraw(const image_s &img)
{
	return img;
}

struct ImageWriter {
	image_s data;
	image_b mask;

	bool inited = false;
	bool is_lossless;

	void init(uint32_t w, uint32_t h, uint32_t nc, uint32_t bitdepth)
	{
		if (inited) return;
		inited = true;
		data.resize(w, h, nc, bitdepth);
		mask.resize(w, h);
	}
	void set(uint32_t i, uint32_t c, uint16_t v)
	{
		if (is_lossless && c == data.channels()) {
			mask.at(i) = v * 255;
		} else if (is_lossless || !mask.at(i)) {
			data.at(i, c) = v;
		}
	}

	void lossless()
	{
		is_lossless = true;
	}
	void lossy()
	{
		is_lossless = false;
	}

	void tofs()
	{
		image_io::save(get_visraw(data), "data.tiff");
		image_io::save(mask, "mask.png");
	}
};

struct JP2Conf {
	bool lossy;
	int numres;
	float rate;
};

extern "C" {
OPJ_SIZE_T writefn(void *p_buffer, OPJ_SIZE_T p_nb_bytes, void *p_user_data)
{
	std::ostream *os = (std::ostream*)p_user_data;
	std::size_t before = os->tellp();
	os->write((const char*)p_buffer, p_nb_bytes);
	return os->tellp() - before;
}
void freefn(void *p_user_data)
{}
OPJ_BOOL seekfn(OPJ_OFF_T p_nb_bytes, void *p_user_data)
{
	std::ostream *os = (std::ostream*)p_user_data;
	os->seekp(p_nb_bytes, std::ios_base::beg);
	return !!*os;
}
OPJ_OFF_T skipfn(OPJ_OFF_T p_nb_bytes, void *p_user_data)
{
	std::ostream *os = (std::ostream*)p_user_data;
	os->seekp(p_nb_bytes, std::ios_base::cur);
	return p_nb_bytes;
}
}

// jp2 and jp2i are adopted from the JPEG2000 example
// https://github.com/uclouvain/openjpeg/blob/master/src/bin/jp2/opj_dump.c
// BSD-2 licensed
//  * Copyright (c) 2010, Mathieu Malaterre, GDCM
//  * Copyright (c) 2011-2012, Centre National d'Etudes Spatiales (CNES), France
//  * Copyright (c) 2012, CS Systemes d'Information, France
//  * All rights reserved.
template <typename T>
void jp2(const T &img, const JP2Conf &conf, std::ostream *ptr)
{
    opj_cparameters_t parameters;
    opj_stream_t *l_stream = 0;
    opj_codec_t* l_codec = 0;
    int ret = 0;
	OPJ_BOOL succ;

    int num_threads = 4;

    opj_set_default_encoder_parameters(&parameters);
    parameters.tcp_mct = conf.lossy;// LOSSY == 1(char)
	parameters.tcp_numlayers = 0;
	parameters.tcp_rates[parameters.tcp_numlayers++] = conf.lossy ? conf.rate : 1;

	parameters.cp_disto_alloc = 1;
	parameters.numresolution = conf.numres;
	parameters.irreversible = conf.lossy;

    std::vector<opj_image_cmptparm_t> cmptparm(img.nc());
// 	std::memset((char*)cmptparm.data(), 0, cmptparm.size() * sizeof(opj_image_cmptparm_t));
    opj_image_t * image = NULL;
    for (int i = 0; i < img.nc(); i++) {
        cmptparm[i].prec = (OPJ_UINT32)((img.bitdepth() + 7) / 8 * 8);
        cmptparm[i].bpp = (OPJ_UINT32)img.bitdepth(); // floor log2max
        cmptparm[i].sgnd = (OPJ_UINT32)0;
        cmptparm[i].x0 = (OPJ_UINT32)0;
        cmptparm[i].y0 = (OPJ_UINT32)0;
        cmptparm[i].dx = (OPJ_UINT32)1;
        cmptparm[i].dy = (OPJ_UINT32)1;
        cmptparm[i].w = (OPJ_UINT32)img.w();
        cmptparm[i].h = (OPJ_UINT32)img.h();
    }
    OPJ_COLOR_SPACE color_space = img.nc() == 3 ? OPJ_CLRSPC_SRGB : OPJ_CLRSPC_UNKNOWN;
    image = opj_image_create((OPJ_UINT32)img.nc(), cmptparm.data(), color_space);
	if (!image) std::cerr << "ERR" << std::endl;
    image->x0 = (OPJ_UINT32)0;
    image->y0 = (OPJ_UINT32)0;
    image->x1 = (OPJ_UINT32)img.w();
    image->y1 = (OPJ_UINT32)img.h();
	for (int y = 0; y < img.h(); ++y) {
		for (int x = 0; x < img.w(); ++x) {
			for (int i = 0; i < img.nc(); i++) {
				image->comps[i].data[x + y * img.w()] = img.get(x, y, i);
			}
		}
	}

	l_codec = opj_create_compress(OPJ_CODEC_JP2);

	opj_set_info_handler(l_codec, info_callback, 0);
	opj_set_warning_handler(l_codec, warning_callback, 0);
	opj_set_error_handler(l_codec, error_callback, 0);

	if (! opj_setup_encoder(l_codec, &parameters, image)) {
		fprintf(stderr, "failed to encode image: opj_setup_encoder\n");
		opj_destroy_codec(l_codec);
		opj_image_destroy(image);
		ret = 1;
		goto fin;
	}

	if (num_threads >= 1 && !opj_codec_set_threads(l_codec, num_threads)) {
		fprintf(stderr, "failed to set number of threads\n");
		opj_destroy_codec(l_codec);
		opj_image_destroy(image);
		ret = 1;
		goto fin;
	}

	l_stream = opj_stream_default_create(OPJ_FALSE);
	opj_stream_set_seek_function(l_stream, &seekfn);
	opj_stream_set_skip_function(l_stream, &skipfn);
	opj_stream_set_user_data(l_stream, ptr, &freefn);
// 	opj_stream_set_user_data_length(l_stream, sizeof(ptr));
	opj_stream_set_write_function(l_stream, &writefn);
	if (! l_stream) {
		ret = 1;
		goto fin;
	}

	succ = opj_start_compress(l_codec, image, l_stream);
	if (!succ)  {
		fprintf(stderr, "failed to encode image: opj_start_compress\n");
	}
	succ = succ && opj_encode(l_codec, l_stream);
	if (!succ)  {
		fprintf(stderr, "failed to encode image: opj_encode\n");
	}

	succ = succ && opj_end_compress(l_codec, l_stream);
	if (!succ)  {
		fprintf(stderr, "failed to encode image: opj_end_compress\n");
	}

	if (!succ)  {
		opj_stream_destroy(l_stream);
		opj_destroy_codec(l_codec);
		opj_image_destroy(image);
		fprintf(stderr, "failed to encode image\n");
		remove(parameters.outfile);
		ret = 1;
		goto fin;
	}

	opj_stream_destroy(l_stream);
	opj_destroy_codec(l_codec);
	opj_image_destroy(image);

    ret = 0;

fin:
    if (parameters.cp_comment) {
        free(parameters.cp_comment);
    }
    if (parameters.cp_matrice) {
        free(parameters.cp_matrice);
    }
}

template <typename T>
void jp2i(const char *in, T &img)
{
	opj_dparameters_t parameters;
	opj_set_default_decoder_parameters(&parameters);
	opj_image_t *image = NULL;
	opj_stream_t *l_stream = NULL;
	opj_codec_t *l_codec = NULL;

	l_stream = opj_stream_create_default_file_stream(in, 1);
	if (!l_stream) {
		fprintf(stderr, "ERROR -> failed to create the stream from the file %s\n",
				in);

		goto fin;
	}
	l_codec = opj_create_decompress(OPJ_CODEC_JP2);
	opj_set_info_handler(l_codec, info_callback, 00);
	opj_set_warning_handler(l_codec, warning_callback, 00);
	opj_set_error_handler(l_codec, error_callback, 00);

	if (!opj_setup_decoder(l_codec, &parameters)) {
		fprintf(stderr, "ERROR -> opj_decompress: failed to setup the decoder\n");
		opj_stream_destroy(l_stream);
		opj_destroy_codec(l_codec);

		goto fin;
	}

	/* Read the main header of the codestream and if necessary the JP2 boxes*/
	if (! opj_read_header(l_stream, l_codec, &image)) {
		fprintf(stderr, "ERROR -> opj_decompress: failed to read the header\n");
		opj_stream_destroy(l_stream);
		opj_destroy_codec(l_codec);
		opj_image_destroy(image);

		goto fin;
	}

	if (!(opj_decode(l_codec, l_stream, image) &&
			opj_end_decompress(l_codec,   l_stream))) {
		fprintf(stderr, "ERROR -> opj_decompress: failed to decode image!\n");
		opj_destroy_codec(l_codec);
		opj_stream_destroy(l_stream);
		opj_image_destroy(image);

		goto fin;
	}
	std::cout << image->comps[0].bpp << std::endl;
	img.init(image->comps[0].w, image->comps[0].h, image->numcomps - 1, /*image->comps[0].bpp*/14);
	for (int i = 0; i < image->numcomps; ++i) {
		for (int j = 0; j < image->comps[i].w * image->comps[i].h; ++j) {
			img.set(j, i, image->comps[i].data[j]);
		}
	}
	opj_stream_destroy(l_stream);
	if (l_codec) {
		opj_destroy_codec(l_codec);
	}
	opj_image_destroy(image);
fin:;
}
std::string get_mask_fn(std::string fn)
{
	return fn.substr(0, fn.find_last_of('.')) + "_mask.png";
}
std::string get_mask_fn2(std::string fn)
{
	return fn.substr(0, fn.find_last_of('.')) + "_mask.png";
}
std::string get_name(std::string fn)
{
	std::string f = fn.substr(fn.find_last_of('/') + 1);
	return f.substr(0, f.find_last_of('.'));
}
std::string get_dir(std::string fn)
{
	return fn.substr(0, fn.find_last_of('/'));
}

struct Eval {
	uint64_t t;
	std::size_t s;
	void p()
	{
		std::cout << ((int)((double)s / 1024 / 1024 * 100) / 100.f) << " M; " << t << " s" << std::endl;
	}
};
long GetFileSize(const char *filename)
{
    struct stat stat_buf;
    int rc = stat(filename, &stat_buf);
    return rc == 0 ? stat_buf.st_size : -1;
}

Eval compress(const image_s &img, const image_b &mask, const char *name, const char *outdir, float rate)
{
	if (img.width() != mask.width() || img.height() != mask.height()) throw std::runtime_error("dims");
	JP2Conf conf{ false, 5 };
	JP2Conf conf_lossy{ true, 5, rate };
	ImageViewLossless imgd(img.data(), mask.data(), img.width(), img.height(), img.channels(), img.bitdepth());
	ImageViewLossy imgd_lossy(img.data(), mask.data(), img.width(), img.height(), img.channels(), img.bitdepth(), conf_lossy.numres);

	std::ofstream oslossy((std::string(outdir) + '/' + name + ".bg.jp2").c_str());
	std::ofstream oslossless((std::string(outdir) + '/' + name + ".fg.jp2").c_str());
	std::stringstream ss;
	auto start = std::chrono::high_resolution_clock::now();
	jp2(imgd, conf, &oslossless);
	jp2(imgd_lossy, conf_lossy, &oslossy);

	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	oslossless.flush();
	oslossy.flush();
	return Eval{ elapsed.count(), oslossy.tellp() + oslossless.tellp() };
}
void eval_rates(const image_s &img, const image_b &mask, const char *outdir)
{
	ImageWriter writer;
	writer.init(img.width(), img.height(), img.channels(), /*image->comps[0].bpp*/14);
	writer.lossless();
	for (int i = 0; i < img.pixels(); ++i) {
		for (int c = 0; c < img.channels(); ++c) {
			writer.set(i, c, img.at(i, c));
		}
		writer.set(i, img.channels(), mask.at(i));
	}
	writer.lossy();
	for (float rate = 10; rate < 200; rate += 10) {
		JP2Conf conf_lossy{ true, 5, rate };
		std::string tmp = std::string(outdir) + "/tmp.jp2";
		std::ofstream oslossy(tmp.c_str());
		ImageViewLossy imgd_lossy(img.data(), mask.data(), img.width(), img.height(), img.channels(), img.bitdepth(), conf_lossy.numres);
		jp2(imgd_lossy, conf_lossy, &oslossy);
		jp2i(tmp.c_str(), writer);
		oslossy.flush();
		double mse = 0, mae = 0;
		double n = ((1 << img.bitdepth()) - 1);
		n=1;
		for (int i = 0; i < img.size(); ++i) {
			double v0 = (double)img[i] / n;
			double v1 = (double)writer.data[i] / n;
			double d = v0 - v1;
			mse += d * d;
			mae += std::abs(d);
		}
		mse /= img.size();
		mae /= img.size();
		double rmse = std::sqrt(mse * n * n);
		std::cout << "MSE " << rate << " RSME: " << rmse << " MAE: " << mae << " " << oslossy.tellp() / 1024 << std::endl;
	}
}
Eval comp_png(const image_s &img, const char *name, const char *outdir)
{
	std::string fn = std::string(outdir) + '/' + name + ".png";
	auto start = std::chrono::high_resolution_clock::now();
	image_io::save(img, fn.c_str());
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	return Eval{ elapsed.count(), GetFileSize(fn.c_str()) };
}
Eval comp_jp2(const image_s &img, const char *name, const char *outdir)
{
	std::string fn = std::string(outdir) + '/' + name + ".jp2";
	JP2Conf conf{ false, 5 };
	ImageViewJP2MASTER imgdM(img.data(), img.width(), img.height(), img.channels(), img.bitdepth());
	std::ofstream os(fn.c_str());
	auto start = std::chrono::high_resolution_clock::now();
	jp2(imgdM, conf, &os);
	auto end = std::chrono::high_resolution_clock::now();
	std::chrono::milliseconds elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
	return Eval{ elapsed.count(), GetFileSize(fn.c_str()) };
}

int main(int argc, const char **argv)
{
	const char *outdir = argv[1];
#define NRATES 5
	std::atomic<std::size_t> sum_s_raw(0);
	std::atomic<std::size_t> sum_s_our[NRATES];
	std::atomic<uint64_t> sum_t_our[NRATES];
	for (int i = 0; i < NRATES; ++i) {
		sum_s_our[i] = 0;
		sum_t_our[i] = 0;
	}
	std::atomic<std::size_t> sum_s_png(0);
	std::atomic<std::size_t> sum_s_jp2(0);
	std::atomic<uint64_t> sum_t_png(0);
	std::atomic<uint64_t> sum_t_jp2(0);
	int jp2rates[5] = { 8, 16, 32, 64, 128 };

	for (int i = 2; i < argc; ++i) {
		image_s img = mvc::load_raw(argv[i]);
		std::string name = get_name(argv[i]);
		std::string dir = get_dir(argv[i]);
		image_b mask2 = image_io::load((dir + '/' + name + "_mask.png").c_str());
		image_b mask = image_manip::rescale_half_gaussian(mask2);
		std::size_t uncomp = ((std::size_t)img.width() * (std::size_t)img.height() * (std::size_t)img.channels() * (std::size_t)img.bitdepth()) / 8;
		for (int i = 0; i < NRATES; ++i) {
			Eval eour = compress(img, mask, name.c_str(), outdir, jp2rates[i]);
			sum_s_our[i] += eour.s;
			sum_t_our[i] += eour.t;
		}
		Eval epng = comp_png(img, name.c_str(), outdir);
		Eval ejp2 = comp_jp2(img, name.c_str(), outdir);

		sum_s_raw += uncomp;
		sum_s_png += epng.s;
		sum_s_jp2 += ejp2.s;
		sum_t_png += epng.t;
		sum_t_jp2 += ejp2.t;
	}
	double ratepng = (double)sum_s_raw / sum_s_png;
	double ratejp2 = (double)sum_s_raw / sum_s_jp2;
	for (int i = 0; i < NRATES; ++i) {
		double rateour = (double)sum_s_raw / sum_s_our[i];
		std::cout << "Our[" << jp2rates[i] << "]: 1:" << rateour << " @ " << sum_t_our[i] << " ms" << std::endl;
	}
	std::cout << "PNG: 1:" << ratepng << " @ " << sum_t_png << " ms" << std::endl;
	std::cout << "JP2: 1:" << ratejp2 << " @ " << sum_t_jp2 << " ms" << std::endl;
}
