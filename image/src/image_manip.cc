#include "image.h"

#define IMG_LOOP(E) for (std::size_t i = 0; i < src.pixels(); ++i) { dst[i] = E; }

namespace image_manip {

image_f grayscale(const image_b &src)
{
	image_f dst(src.width(), src.height());

	float sc = (1 << (src.bps() * 8));
	switch (src.channels()) {
	case 1:
		IMG_LOOP(src[i] / sc);
		break;
	case 2:
		IMG_LOOP((src.at(i, 0) / sc) * (src.at(i, 1) / sc));
		break;
	case 3:
		IMG_LOOP(src.at(i, 0) / sc * .3f + src.at(i, 1) / sc * .59f + src.at(i, 2) / sc * .11f);
		break;
	case 4:
		IMG_LOOP((src.at(i, 0) / sc * .3f + src.at(i, 1) / sc * .59f + src.at(i, 2) / sc * .11f) * (src.at(i, 1) / sc));
		break;
	}

	return dst;
}

}
