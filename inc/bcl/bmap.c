#include "bmap.h"

/* writes a bitmap _b to the file _fp */
void export_bitmap(char* _fp, bitmap* _b) {
	FILE* outf = fopen(_fp, "wb");
	fwrite(&_b->file_header, 1, 14, outf);
	fwrite(&_b->info, 1, _b->info.length, outf);
	if (_b->extra.NO_PALETTE) fwrite(_b->palette, 1, _b->info.num_colours * sizeof(struct rgba_pixel_32), outf);
	fwrite(_b->rdata, 1, _b->extra.padded_length, outf);
	fclose(outf);
}

/* Returns a bitmap image (32-bit, 24-bit, 16-bit, 8-bit, or paletted),
 * as a 2D 32-bit RGBA pixel array. */
bitmap import_bitmap(char* _fp) {
	bitmap bmp;
	FILE* inf = fopen(_fp, "rb");
	
	if (!inf) {
		ERROR_CODE = ERR_FILE_NOT_FOUND;
		return (bitmap) {0};
	}
	
	// load bitmap file header
	fread(&bmp.file_header, 1, 14, inf);
	
	// load the minimum DIB size first, then using the loaded data determine any further data that is required to be loaded
	fread(&bmp.info, 1, 12, inf);
	switch (bmp.info.length) {
		case 12:
			bmp.extra.header_format = BITMAPCOREHEADER;
			break;
		case 40:
			bmp.extra.header_format = BITMAPINFOHEADER;
			fread((char*) &bmp.info + 12, 1, 28, inf);
			break;
		case 64:
			bmp.extra.header_format = OS22XBITMAPHEADER;
			fread((char*) &bmp.info + 12, 1, 52, inf);
			break;
		case 108:
			bmp.extra.header_format = BITMAPV4HEADER;
			fread((char*) &bmp.info + 12, 1, 96, inf);
			break;
		case 124:
			bmp.extra.header_format = BITMAPV5HEADER;
			fread((char*) &bmp.info + 12, 1, 112, inf);
			break;
		default:
			ERROR_CODE = ERR_UNKNOWN_HEADER;
			#ifndef __BMAPCLIB_ATTEMPT_ANYWAYS__
			return (bitmap) {0};
			#endif
			bmp.extra.header_format = UNKNOWN;
	}
	
	// if the number of colours in the colour palette is more than the maximum colours able to be stored in one pixel then there
	// is likely an error (potentially corrupt file). so we check that num_colours is less than the maximum number of colours in
	// the palette (2 ^ colour depth)
	if (bmp.info.num_colours > (1 << bmp.info.bpp)) {
		ERROR_CODE = ERR_INVALID_PALETTE_SIZE;
		#ifndef __BMAPCLIB_ATTEMPT_ANYWAYS__
		return (bitmap) {0};
		#endif
	}
	
	// calculate extra data
	bmp.extra.bit_width = bmp.info.width * bmp.info.bpp;
	bmp.extra.bit_length = bmp.extra.bit_width * bmp.info.height;
	bmp.extra.padded_width = (bmp.info.width * bmp.info.bpp + 31) / 32 * 4;
	bmp.extra.padded_length = bmp.extra.padded_width * bmp.info.height;
	bmp.extra.padding = (32 - (bmp.extra.bit_width % 32)) % 32;
	if (bmp.info.bpp != 32) bmp.extra.padded = 1;
	bmp.plen = bmp.info.height * bmp.info.width;
	
	// keep track of the expected file length
	long expected_length = 14 + bmp.info.length + bmp.extra.padded_length;
	
	// by default, some colour depths will be interpreted with a colour palette,
	// though when no palette is present they can be interpreted differently
	bmp.extra.NO_PALETTE = 1;
	
	// for bitmaps other than those with 24, or 32-bit colour depths, colour palette data must be copied
	if (bmp.info.bpp != 32 && bmp.info.bpp != 24) {
		// if the colour depth was set to default to 2 ^ n (a value of 0), then set it to 2 ^ n
		if (!bmp.info.num_colours) bmp.info.num_colours = 1 << bmp.info.bpp;
		
		// check whether or not the expected colour palette is present
		if (bmp.info.num_colours * sizeof(struct rgba_pixel_32) + bmp.info.length != bmp.file_header.offset) {
			// as an exception, if the bitmap has a 16 or 8-bit colour depth, it may be interpreted as 16 or 8-bit RGBA instead
			if (bmp.info.bpp != 8 && bmp.info.bpp != 16) {
				ERROR_CODE = ERR_EXPECTED_COLOUR_PALETTE_NOT_PRESENT;
				#ifndef __BMAPCLIB_ATTEMPT_ANYWAYS__
				return (bitmap) {0};
				#endif
			}
		} else bmp.extra.NO_PALETTE = 0;
		
		// add to the expected length of the file
		expected_length += bmp.info.num_colours * sizeof(struct rgba_pixel_32);
		
		// copy colour table into bitmap structure
		bmp.palette = malloc(sizeof(struct rgba_pixel_32) * bmp.info.num_colours);
		fread(bmp.palette, 1, sizeof(struct rgba_pixel_32) * bmp.info.num_colours, inf);
	}
	
	// check that the file is is the expected length, if not then the file is likely corrupt or not a bitmap
	fseek(inf, 0, SEEK_END);
	if (ftell(inf) != expected_length) {
		ERROR_CODE = ERR_CONFLICTING_HEADER_INFORMATION;
		#ifndef __BMAPCLIB_ATTEMPT_ANYWAYS__
		return (bitmap) {0};
		#endif
	}
	
	// copy pixel data into bitmap structure
	bmp.rdata = malloc(bmp.extra.padded_length);
	fseek(inf, bmp.file_header.offset, SEEK_SET);
	fread(bmp.rdata, 1, bmp.extra.padded_length, inf);
	fclose(inf);
	
	// allocate memory for a pointer array to access rdata as a 2D-array
	bmp.data = malloc(bmp.info.height * sizeof(void*));
	
	// all bitmaps are converted to 32-bit RGBA when imported
	switch (bmp.info.bpp) {
		case 24:
			bitmap24_to_32rgba(&bmp);
			break;
		case 32:
			// no conversion needed
			// allow access to pixel data as 2D-array
			for (int i = 0; i < bmp.info.height; i++)
				bmp.data[i] = (((struct rgba_pixel_32*) bmp.rdata) + i * bmp.info.width);
			break;
		case 8:
			if (bmp.extra.NO_PALETTE) {
				bitmap8_to_32rgba(&bmp);
				break;
			}
		case 16:
			if (bmp.extra.NO_PALETTE) {
				bitmap16_to_32rgba(&bmp);
				break;
			}
		default:
			// convert the colour table index data to 32-bit RGBA pixel array
			bitmap_expand_from_colour_table(&bmp);
	}
	
	return bmp;
}

bitmap bitmap_copy(bitmap* _b) {
	bitmap new = *_b;
	new.rdata = memfork(_b->rdata, _b->plen * sizeof(struct rgba_pixel_32));
	
	// allow acces to pixel data as 2D-array
	new.data = malloc(new.info.height * sizeof(void*));
	for (int i = 0; i < new.info.height; i++)
		new.data[i] = (((struct rgba_pixel_32*) new.rdata) + i * new.info.width);
	
	return new;
}

/* Recomputes data for a bitmap based on updates to other parameters */
void bitmap_recalculate_extra_data(bitmap* _b) {
	_b->extra.bit_width = _b->info.width * _b->info.bpp;
	_b->extra.bit_length = _b->extra.bit_width * _b->info.height;
	_b->extra.padded_width = (_b->info.width * _b->info.bpp + 31) / 32 * 4;
	_b->extra.padded_length = _b->extra.padded_width * _b->info.height;
	_b->extra.padding = (32 - (_b->extra.bit_width % 32)) % 32;
	_b->plen = _b->info.height * _b->info.width;
}

/* Replaces the data associated with bitmap _b with new data stored
 * in _p */
void bitmap_replace_data(bitmap* _b, void* _p) {
	free(_b->rdata);
	_b->rdata = (BYTE*) _p;
	
	// allow acces to pixel data as 2D-array
	for (int i = 0; i < _b->info.height; i++)
		_b->data[i] = (((struct rgba_pixel_32*) _b->rdata) + i * _b->info.width);
}

/* Convers a paletted bitmap (image data holds indices in the colour table)
 * to a 32-bit RGBA bitmap - this is so the structure of all imported images
 * is consistant for the user */
void bitmap_expand_from_colour_table(bitmap* _b) {
	DWORD* new_data = calloc(_b->plen, sizeof(struct rgba_pixel_32));
	char bit = 0;
	BYTE m = 0x80;
	
	// split up the byte array (pixel data) into sets of n bits for a n-bit colour depth,
	// look these values up the the colour palette, and replace the pixel with the
	// corresponding 32-bit rgba colour value
	for (int set = 0, n = 0; set < _b->extra.padded_length; n++) {
		for (char i = 0; i < _b->info.bpp; i++, bit++) {
			if (!(i % 8)) new_data[n] <<= 8;
			new_data[n] |= m & _b->rdata[set];
			if (m == 1) set++, bit = -1;
			rorb(&m, 1);
		}
		rold(&new_data[n], (bit - _b->info.bpp + 8) % 8);
		new_data[n] >>= 8 - _b->info.bpp;
		new_data[n] = *(((DWORD*) _b->palette) + new_data[n]);
	}
	_b->info.bpp = 32;
	_b->extra.padded = 0;
	bitmap_recalculate_extra_data(_b);
	
	bitmap_replace_data(_b, new_data);
}

/* Bitmaps are padded so rows are a multiple for four bytes,
 * to process the image we need to remove this */
void bitmap_remove_padding(bitmap* _b) {
	char* new_data = malloc(_b->plen * sizeof(struct rgba_pixel_32));
	int c = 0;
	for (int i = 0; i < _b->extra.padded_length; i++) {
		if (i && i % (_b->info.bpp / 8 * _b->info.width) == 0) c += _b->extra.padding / 8;
		new_data[i] = *((char*) _b->rdata + i + c);
	}
	bitmap_replace_data(_b, new_data);
	_b->extra.padded = 0;
}

/* Converts an 8-bit RGBA bitmap image into a 32-bit RGBA one */
void bitmap8_to_32rgba(bitmap* _b) {
	bitmap_remove_padding(_b);
	
	struct rgba_pixel_32* new_data = malloc(_b->plen * sizeof(struct rgba_pixel_32));
	for (int i = 0; i < _b->plen; i++) {
		new_data[i].r = (((struct rgba_pixel_8*) _b->rdata) + i)->r * 85;
		new_data[i].g = (((struct rgba_pixel_8*) _b->rdata) + i)->g * 85;
		new_data[i].b = (((struct rgba_pixel_8*) _b->rdata) + i)->b * 85;
		new_data[i].a = (((struct rgba_pixel_8*) _b->rdata) + i)->a * 85;
	}
	_b->info.bpp = 32;
	_b->extra.padded = 0;
	bitmap_recalculate_extra_data(_b);
	
	bitmap_replace_data(_b, new_data);
}

/* Converts an 16-bit RGBA bitmap image into a 32-bit RGBA one */
void bitmap16_to_32rgba(bitmap* _b) {
	struct rgba_pixel_32* new_data = malloc(_b->plen * sizeof(struct rgba_pixel_32));
	bitmap_remove_padding(_b);
	
	for (int i = 0; i < _b->plen; i++) {
		new_data[i].r = (((struct rgba_pixel_16*) _b->rdata) + i)->r * 17;
		new_data[i].g = (((struct rgba_pixel_16*) _b->rdata) + i)->g * 17;
		new_data[i].b = (((struct rgba_pixel_16*) _b->rdata) + i)->b * 17;
		new_data[i].a = (((struct rgba_pixel_16*) _b->rdata) + i)->a * 17;
	}
	_b->info.bpp = 32;
	_b->extra.padded = 0;
	bitmap_recalculate_extra_data(_b);
	
	bitmap_replace_data(_b, new_data);
}

/* Converts an 24-bit RGB bitmap image into a 32-bit RGBA one */
void bitmap24_to_32rgba(bitmap* _b) {
	struct rgba_pixel_32* new_data = malloc(_b->plen * sizeof(struct rgba_pixel_32));
	bitmap_remove_padding(_b);
	
	for (int i = 0; i < _b->plen; i++) {
		new_data[i].r = (((struct rgb_pixel_24*) _b->rdata) + i)->r;
		new_data[i].g = (((struct rgb_pixel_24*) _b->rdata) + i)->g;
		new_data[i].b = (((struct rgb_pixel_24*) _b->rdata) + i)->b;
		new_data[i].a = 255;
	}
	_b->info.bpp = 32;
	_b->extra.padded = 0;
	bitmap_recalculate_extra_data(_b);
	
	bitmap_replace_data(_b, new_data);
}

/* Obvious */
void bitmap_free(bitmap* _b) {
	if (_b->data) free(_b->data);
	if (_b->rdata) free(_b->rdata);
}

/* test function, replaces bitmap data with a checkerboard */
void bitmap_checker(bitmap* _b) {
	for (int c = 0, i = 0; i < _b->plen; i++) {
		if (i % _b->info.width == 0 && _b->info.width % 2 == 0) c = !c;
		*(((struct rgba_pixel_32*) _b->rdata) + i) = ((i + c) % 2 == 0) ? (struct rgba_pixel_32) {255, 255, 255, 255} : (struct rgba_pixel_32) {0, 0, 0, 255};
	}
}

/* greyscales a bitmap using the specified format
 * (unknown format default to 'A'):
 *   'L' -> use realistic greyscaling
 *   'M' -> use arithmetic mean of channels
 *   'A' -> use alpha channel
 *   'R' -> use red channel
 *   'G' -> use green channel
 *   'B' -> use blue channel */
void bitmap_greyscale(bitmap* _b, char _fmt) {
	struct rgba_pixel_32* data = (struct rgba_pixel_32*) _b->rdata;
	for (int i = 0; i < _b->plen; i++) {
		switch (_fmt) {
			default:
			case 'M': // average channels
				data[i].r = (data[i].r + data[i].g + data[i].b) / 3;
				break;
			case 'L': // get pixel intensity (I = 0.2126 * R + 0.7152 * G + 0.0722 * B)
				data[i].r = 0.2126 * data[i].r + 0.7152 * data[i].g + 0.0722 * data[i].b;
				break;
			case 'A': // A channel only
				data[i].r = data[i].b;
				break;
			case 'R': // R channel only
				break;
			case 'G': // G channel only
				data[i].r = data[i].g;
				break;
			case 'B': // B channel only
				data[i].r = data[i].b;
				break;
		}
		data[i].b = data[i].g = data[i].r;
	}
}

/* blurs a bitmap _b, with _n passes */
void bitmap_blur(bitmap* _b, int _n) {
	// do x and y direction seperately
	// (gaussian kernel is seperable)
	
	// rows
	for (int n = 0; n < _n; n++) {
		for (int y = 0; y < _b->info.height; y++) {
			_b->data[y][0].r = (2 * _b->data[y][0].r + _b->data[y][1].r) / 3;
			_b->data[y][0].g = (2 * _b->data[y][0].g + _b->data[y][1].g) / 3;
			_b->data[y][0].b = (2 * _b->data[y][0].b + _b->data[y][1].b) / 3;
			_b->data[y][_b->info.width - 1].r = (2 * _b->data[y][_b->info.width - 2].r + _b->data[y][_b->info.width - 1].r) / 3;
			_b->data[y][_b->info.width - 1].g = (2 * _b->data[y][_b->info.width - 2].g + _b->data[y][_b->info.width - 1].g) / 3;
			_b->data[y][_b->info.width - 1].b = (2 * _b->data[y][_b->info.width - 2].b + _b->data[y][_b->info.width - 1].b) / 3;
			for (int x = 1; x < _b->info.width - 1; x++) {
				_b->data[y][x].r = (_b->data[y][x - 1].r + 2 * _b->data[y][x].r + _b->data[y][x + 1].r) / 4;
				_b->data[y][x].g = (_b->data[y][x - 1].g + 2 * _b->data[y][x].g + _b->data[y][x + 1].g) / 4;
				_b->data[y][x].b = (_b->data[y][x - 1].b + 2 * _b->data[y][x].b + _b->data[y][x + 1].b) / 4;
			}
		}
	}
	
	// cols
	for (int n = 0; n < _n; n++) {
		for (int x = 0; x < _b->info.width; x++) {
			_b->data[0][x].r = (2 * _b->data[0][x].r + _b->data[1][x].r) / 3;
			_b->data[0][x].g = (2 * _b->data[0][x].g + _b->data[1][x].g) / 3;
			_b->data[0][x].b = (2 * _b->data[0][x].b + _b->data[1][x].b) / 3;
			_b->data[_b->info.height - 1][x].r = (2 * _b->data[_b->info.height - 2][x].r + _b->data[_b->info.height - 1][x].r) / 3;
			_b->data[_b->info.height - 1][x].g = (2 * _b->data[_b->info.height - 2][x].g + _b->data[_b->info.height - 1][x].g) / 3;
			_b->data[_b->info.height - 1][x].b = (2 * _b->data[_b->info.height - 2][x].b + _b->data[_b->info.height - 1][x].b) / 3;
			for (int y = 1; y < _b->info.height - 1; y++) {
				_b->data[y][x].r = (_b->data[y - 1][x].r + 2 * _b->data[y][x].r + _b->data[y + 1][x].r) / 4;
				_b->data[y][x].g = (_b->data[y - 1][x].g + 2 * _b->data[y][x].g + _b->data[y + 1][x].g) / 4;
				_b->data[y][x].b = (_b->data[y - 1][x].b + 2 * _b->data[y][x].b + _b->data[y + 1][x].b) / 4;
			}
		}
	}
}

/* Resizes a bitmap _b to new dimensions _x by _y using
 * nearest-neighbour interpolation */
void bitmap_resize_nn(bitmap* _b, int _x, int _y) {
	int x0, y0, dclen = _x * _y * sizeof(struct rgba_pixel_32);
	struct rgba_pixel_32* dcopy = malloc(dclen);
	double xscale = _x / (double) _b->info.width;
	double yscale = _y / (double) _b->info.height;
	for (int i = 0; i < _y; i++) {
		y0 = i / yscale;
		for (int j = 0; j < _x; j++) {
			x0 = j / xscale;
			dcopy[i * _x + j] = _b->data[y0][x0];
		}
	}
	_b->info.width = _x;
	_b->info.height = _y;
	bitmap_recalculate_extra_data(_b);
	bitmap_replace_data(_b, dcopy);
}

/* Resizes a bitmap _b to new dimensions _x by _y using
 * bi-cubic interpolation */
void bitmap_resize_cubic(bitmap* _b, int _x, int _y) {
}

//int main(void) { return 0; }
