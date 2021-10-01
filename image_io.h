#include "image.h"
#include "libraw/libraw.h"
#include <cstring>

namespace mvc {

void load_dims(const char *fn, uint32_t &w, uint32_t &h, uint8_t &nc, uint8_t &bitdepth, bool mosa = false, int shift = 0)
{
	try {
		image_b img = image_io::load(fn); // TODO: don't load pixels
		w = img.width();
		h = img.height();
		nc = img.channels();
		if (mosa) {
			w /= 2;
			h /= 2;
			nc = 4;
		}
		bitdepth = img.bps() * 8;
	} catch (...) {
		LibRaw img;
		img.open_file(fn);
		w = img.imgdata.sizes.raw_width / 2;
		h = img.imgdata.sizes.raw_height / 2;
		nc = 4; // assume bayer
		int max = img.imgdata.color.maximum;
		bitdepth = 0;
		while (max > 0) {
			max >>= 1;
			++bitdepth;
		}
	}
	bitdepth -= shift;
}

template <typename Tin, typename Tout>
void copy_mosaiced(Tin *in, int ow, int oh, int stride, Tout *out, int w, int h, int nc)
{
// 	std::cout << sizeof(Tin) << " " << sizeof(Tout) << std::endl;
	for (int j = 0; j < std::min(nc, 4); ++j) {
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int ox = j & 1, oy = j >> 1;
				int xx = std::min(x, ow / 2 - 1) * 2 + ox, yy = std::min(y, oh / 2 - 1) * 2 + oy;
				out[x + y * w + j * w * h] = in[xx + yy * stride];
			}
		}
	}
}
template <typename Tin, typename Tout>
void copy_demosaiced(Tin *in, int ow, int oh, int stride, Tout *out, int w, int h, int nc)
{
	for (int j = 0; j < nc; ++j) {
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int i = x + y * w;
				int xx = std::min(x, (int)ow - 1), yy = std::min(y, (int)oh - 1);
				out[j * w * h + i] = in[xx * nc + yy * stride + j];
			}
		}
	}
}
template <typename Tin>
void copy_mosaiced2(Tin *in, int ow, int oh, int stride, image_s &out, int w, int h, int nc, int shift, int bitdepth, int black)
{
	uint16_t mask = (1 << bitdepth) - 1;
// 	std::cout << sizeof(Tin) << " " << sizeof(Tout) << std::endl;
	for (int j = 0; j < std::min(nc, 4); ++j) {
		for (int y = 0; y < h; ++y) {
			for (int x = 0; x < w; ++x) {
				int ox = j & 1, oy = j >> 1;
				int xx = std::min(x, ow / 2 - 1) * 2 + ox, yy = std::min(y, oh / 2 - 1) * 2 + oy;
// 				if (x == 0 && y == 0) { std::cout << (in[xx + yy * stride] >> shift) << std::endl; }
				out.at2d(x, y, j) = /*std::max(*/((/*(int)*/in[xx + yy * stride] >> shift) & mask)/* - black, 0)*/;
				if (shift != 0 && (in[xx + yy * stride] & ((1 << shift) - 1)) != 0) {
					std::cout << "Not lossless..." << std::endl;
					std::exit(1);
				}
			}
		}
	}
}

template <typename Tin, typename Tout>
void wrapper(Tin *in, int ow, int oh, int stride, Tout *out, int w, int h, int nc, bool mosa)
{
	if (mosa) copy_mosaiced<Tin, Tout>(in, ow, oh, stride, out, w, h, nc);
	else copy_demosaiced<Tin, Tout>(in, ow, oh, stride, out, w, h, nc);
}

image_s load_raw(const char *filename)
{
	bool iiq_support = LIBRAW_CHECK_VERSION(0, 18, 0);
	std::size_t l = std::strlen(filename);
	bool is_iiq = filename[l - 4] == '.' && std::tolower(filename[l - 3]) == 'i' && std::tolower(filename[l - 2]) == 'i' && std::tolower(filename[l - 1]) == 'q';
	bool is_cr2 = filename[l - 4] == '.' && std::tolower(filename[l - 3]) == 'c' && std::tolower(filename[l - 2]) == 'r' && std::tolower(filename[l - 1]) == '2';
	int shift = is_iiq && !iiq_support ? 2 : 0;

	LibRaw img;
	img.open_file(filename);
// 	std::cout << "ow: " <<" " << img.imgdata.sizes.raw_height << "    " << img.imgdata.sizes.top_margin << std::endl;
// 	std::cout << "ow: " <<" " << img.imgdata.sizes.raw_width << "    " << img.imgdata.sizes.left_margin << std::endl;
	if (is_cr2) {
// 		img.imgdata.sizes.top_margin = 104;
		img.imgdata.sizes.left_margin = 104;

		img.imgdata.sizes.width = img.imgdata.sizes.raw_width - img.imgdata.sizes.left_margin * 2;
		img.imgdata.sizes.height = img.imgdata.sizes.raw_height - img.imgdata.sizes.top_margin * 2;
	}
	int ow = img.imgdata.sizes.width;
	int oh = img.imgdata.sizes.height;
	int w = ow / 2;
	int h = oh / 2;
	img.unpack();
// 	for (int i = 0; i < 0x10000; ++i) {
// 		std::cout << i << ": " << img.imgdata.color.curve[i] << std::endl;
// 	}
// 	std::cout << "RAWBPS: " << img.imgdata.color.linear_max[0] << " " << img.imgdata.color.linear_max[1] << " " << img.imgdata.color.linear_max[2] << " " << img.imgdata.color.linear_max[3] << " " << std::endl;
// 	std::cout << img.imgdata.color.cblack[0] << " " << img.imgdata.color.cblack[1] << " " << img.imgdata.color.cblack[2] << " " << img.imgdata.color.cblack[3] << std::endl; std::exit(1);
	int stride = img.imgdata.sizes.raw_pitch / 2;

// 	std::cout << "ow: " << ow << " " << oh << " " << img.imgdata.sizes.raw_height << "    " << img.imgdata.sizes.top_margin << std::endl;
// 	std::cout << "ow: " << ow << " " << oh << " " << img.imgdata.sizes.raw_width << "    " << img.imgdata.sizes.left_margin << std::endl;

	int max = 0;
	uint32_t ored = 0;
	for (int y = img.imgdata.sizes.top_margin; y < img.imgdata.sizes.height; ++y) {
		for (int x = img.imgdata.sizes.left_margin; x < img.imgdata.sizes.width; ++x) {
			int cur = img.imgdata.rawdata.raw_image[x + y * stride] >> shift;
			max = std::max(max, cur);
			ored |= cur;
		}
	}
// 	std::cout << "max2: " << max << std::endl;
	int bitdepth = 0;
	while (max > 0) {
		max >>= 1;
		++bitdepth;
	}
	if (bitdepth & 1) ++bitdepth;
// 	std::cout << "VERSIONCHECK: " << iiq_support << " BD: " << bitdepth << " OR: " << ored << std::endl;

	image_s image(w, h, 4, bitdepth);
// 	image_s image1(ow, oh);
// 	for (int y = 0; y < oh; ++y) {
// 		for (int x = 0; x < ow; ++x) {
// 			int cur = img.imgdata.rawdata.raw_image[x + y * stride]/* >> shift*/;
// 			image1.at2d(x, y) = cur;
// 		}
// 	}
// 	image_io::save(image1, "imgmosa.png");
// 	std::exit(1);
	copy_mosaiced2<uint16_t>(img.imgdata.rawdata.raw_image + img.imgdata.sizes.top_margin * stride + img.imgdata.sizes.left_margin, ow, oh, stride, image, w, h, 4, shift, bitdepth, img.imgdata.color.black);

// 	img.raw2image();
// 	image_s test(w, h);
// 	for (int y = 0; y < h; ++y) {
// 		for (int x = 0; x < w; ++x) {
// 			test.at2d(x, y, 0) = img.imgdata.image[x + y * w][0] * 4;
// 		}
// 	}
// 	image_io::save(test, "test.png");std::exit(1);

	return image;
}


template <typename T>
void load_image(const char *fn, int w, int h, int nc, T *out, bool mosa = false, int shift = 0)
{
	typedef typename std::make_unsigned<T>::type TU;
	try {
		imgdta img = image_io::load(fn);

		if (img.bps() <= 1) wrapper<uint8_t, T>((uint8_t*)img.data(), img.width(), img.height(), img.stride(), out, w, h, nc, mosa);
		else if (img.bps() <= 2) wrapper<uint16_t, T>((uint16_t*)img.data(), img.width(), img.height(), img.stride(), out, w, h, nc, mosa);
	} catch (...) {
		LibRaw img;
		img.open_file(fn);
		int ow = img.imgdata.sizes.raw_width;
		int oh = img.imgdata.sizes.raw_height;
		img.unpack();
		int stride = img.imgdata.sizes.raw_pitch / 2;

		copy_mosaiced<uint16_t, T>(img.imgdata.rawdata.raw_image, ow, oh, stride, out, w, h, nc);
	}

	if (shift > 0) {
// 		std::cout << "Do shift by " << shift << std::endl;
		int mask = (1 << shift) - 1;
		for (int j = 0; j < w * h * nc; ++j) {
			TU v = (TU)out[j];
			if (v & mask) throw std::runtime_error("Cannot perform lossless shift");
			out[j] = v >> shift;
		}
	}
}

}
