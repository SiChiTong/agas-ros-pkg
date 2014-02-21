#include "puma2config.h"
#include "ImageWriter.h"
#include "PumaException.h"
#include "PumaMessages.h"

#include <cstdio>
#include <cerrno>
#include <cstring>

using namespace std;
using namespace puma2;

#ifdef HAVE_IMAGEMAGICK
#  include <Magick++/Image.h>
static bool writeImageGrayMagick(const GrayLevelImage8 &img, string filename);
static bool writeImageGrayMagick(const GrayLevelImage16 &img, string filename);
static bool writeImageColorMagick(const ColorImageRGB8 &img, string filename);
static bool writeImageColorMagick(const ColorImageRGBa8 &img, string filename);
#endif

// forward declarations
static bool writeImageGrayBuiltinPNM(const GrayLevelImage8 &img, string &filename);
static bool writeImageColorBuiltinPNM(const ColorImageRGB8 &img, string &filename);
#define USE_BUILTIN_PNM_WRITER	1


bool ImageWriter::writeImage(const GrayLevelImage8 &img, string filename)
{
  bool success = false;
  
#ifdef USE_BUILTIN_PNM_WRITER
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0)) {
    success = writeImageGrayBuiltinPNM(img, filename);
  }
#endif

#ifdef HAVE_IMAGEMAGICK
  if (! success) {
    success = writeImageGrayMagick(img, filename);
  }
#endif
  if (! success) {
    string msg ("failed to write file.");
    throw PumaException(PumaException::intolerable, msg);
    success = false;
  }
  return success;
}

bool ImageWriter::writeImage(const GrayLevelImage16 &img, string filename)
{
  bool success = false;
  
#ifdef HAVE_IMAGEMAGICK
  if (! success) {
    success = writeImageGrayMagick(img, filename);
  }
#endif
  if (! success) {
    string msg ("failed to write file.");
    throw PumaException(PumaException::intolerable, msg);
    success = false;
  }
  return success;
}

bool ImageWriter::writeImage(const ColorImageRGB8 &img, string filename)
{
  bool success = false;
  
#ifdef USE_BUILTIN_PNM_WRITER
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0)) {
    success = writeImageColorBuiltinPNM(img, filename);
  }
#endif

#ifdef HAVE_IMAGEMAGICK
  if (! success) {
    success = writeImageColorMagick(img, filename);
  }
#endif
  if (! success) {
    string msg ("failed to write file.");
    throw PumaException(PumaException::intolerable, msg);
    success = false;
  }
  return success;
}

bool ImageWriter::writeImage(const ColorImageRGBa8 &img, string filename)
{
  bool success = false;
  
#ifdef USE_BUILTIN_PNM_WRITER_NOAPLHAINPNM
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0)) {
    success = writeImageColorBuiltinPNM(img, filename);
  }
#endif

#ifdef HAVE_IMAGEMAGICK
  if (! success) {
    success = writeImageColorMagick(img, filename);
  }
#endif
  if (! success) {
    string msg ("failed to write file.");
    throw PumaException(PumaException::intolerable, msg);
    success = false;
  }
  return success;
}

// --------------------------------------------------------
// writer for PNM files (no libraries, builtin code)
// --------------------------------------------------------

static bool writeImageGrayBuiltinPNM(const GrayLevelImage8 &img, string &filename)
{   
  FILE *filePointer = fopen (filename.c_str(), "wb"); // "b" for binary mode on non-Unix
  if (filePointer == NULL) {
    string msg ("cannot write to file: ");
    msg += filename;
    msg += strerror(errno);
    throw PumaException(PumaException::intolerable, msg);
    return false;
  }
  
  bool success = true;
  int wid = img.getWidth();
  int hig = img.getHeight();
  char outputMode = filename[filename.length()-2];
  int valMax = int(img.getValueRangeMaximum());
  int depth = 255;
  int denominator = 1;
  int bytesPerRow, bytesWritten = 0;
  if (valMax < 255) {
    depth = valMax;
    assert (valMax > 0);	// this could fail for float or double images
  }
  else {
    denominator = valMax / 255;
    assert (valMax == 255); // > 255 can't happen for type 'byte' in GrayLevelImage8
  }
  switch (outputMode) {
    case 'g':
    case 'G':
    case 'n':
    case 'N':
      {
	fprintf (filePointer, "P5\n%d %d\n%d\n", wid, hig, depth);
	// data must be scaled down here for valMax > 255 (that is for 16bit etc.) !!
	// that is all values devided by denominator !
	const GrayLevelImage8::ElementType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
          bytesWritten += fwrite (ptr[y], 1, wid, filePointer);
	}
      }
      break;
    
    case 'p':
    case 'P':
      {
	fprintf (filePointer, "P6\n%d %d\n%d\n", wid, hig, depth);
	const GrayLevelImage8::ElementType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
          const GrayLevelImage8::ElementType *linePtr = ptr[y];
	  for (int x = 0; x < wid; x++) {
            // data must be scaled down here for valMax > 255 !!
	    byte val = *linePtr++  / denominator;
	    putc(val, filePointer);
	    putc(val, filePointer);
	    putc(val, filePointer);
	  }
	  bytesWritten += 3 * wid;
	}
      }
      break;
    
    case 'b':
    case 'B':
      {
	fprintf (filePointer, "P4\n%d %d\n", wid, hig);
	denominator = (valMax + 1) / 2; // so "/ denom" yields [0 | 1], threshold 50%
	bytesPerRow = (wid + 7) / 8;
	byte bitRow[bytesPerRow+10];
	const GrayLevelImage8::ElementType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
          const GrayLevelImage8::ElementType *linePtr = ptr[y];
	  byte *bitPtr = bitRow;
	  byte curByte = 0;
	  int x;
	  for (x = 0; x < wid; x++) {
            // data must be scaled down here for valMax > 1 !!
	    curByte <<= 1;
	    int val = (unsigned(*linePtr++)  / denominator);
	    curByte |= (val > 0) ? 0 : 1;
	    if ((x & 0x07) == 7) {
	      *bitPtr++ = curByte;
	      curByte = 0;
	    }
	  }
	  if ((x & 0x07) != 0) { // ship remaining bits in curByte
	    curByte <<= (8 - (x & 0x07));
	    *bitPtr++ = curByte;
	  }
          bytesWritten += fwrite (bitRow, 1, bytesPerRow, filePointer);
	}
      }
      break;
      
    default:
      // TODO PumaMessage (msgImportant, "file is no recognized PNM file");
      success = false;
      break;
  }
  fclose (filePointer);
  return success;
}

static bool writeImageColorBuiltinPNM(const ColorImageRGB8 &img, string &filename)
{   
  FILE *filePointer = fopen (filename.c_str(), "wb"); // "b" for binary mode on non-Unix
  if (filePointer == NULL) {
    string msg ("cannot write to file: ");
    msg += filename;
    msg += strerror(errno);
    throw PumaException(PumaException::intolerable, msg);
    return false;
  }
  
  bool success = true;
  int wid = img.getWidth();
  int hig = img.getHeight();
  char outputMode = filename[filename.length()-2];
  int valMax = int(img.getValueRangeMaximum());
  int depth = 255;
  int denominator = 1;
  int bytesWritten = 0;
  if (valMax < 255) {
    depth = valMax;
    assert (valMax > 0);	// this could fail for float or double images
  }
  else {
    denominator = valMax / 255;
    assert (valMax == 255); // > 255 can't happen for type 'byte' in GrayLevelImage8
  }
  switch (outputMode) {
    case 'g':
    case 'G':
      {
	denominator *= 1000;	// 299 + 587 + 114 == 1000
	fprintf (filePointer, "P5\n%d %d\n%d\n", wid, hig, depth);
	// data must be scaled down here for valMax > 255 (that is for 16bit etc.) !!
	// that is all values devided by denominator !
	const ColorImageRGB8::PixelType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
	  const ColorImageRGB8::ElementType *linePtr = (ColorImageRGB8::ElementType*)(ptr[y]);
	  for (int x = 0; x < wid; x++) {
	    int val = 299 * *linePtr++;
	    val += 587 * *linePtr++;
	    val += 114 * *linePtr++;
	    putc(val / denominator, filePointer);
	  }
	  bytesWritten += wid;
	}
      }
      break;
    
    case 'p':
    case 'P':
    case 'n':
    case 'N':
      {
	fprintf (filePointer, "P6\n%d %d\n%d\n", wid, hig, depth);
	const ColorImageRGB8::PixelType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
	  // data should be scaled down here for valMax > 255 (that is for 16bit etc.) !!
	  // that is all values devided by denominator !
          bytesWritten += fwrite (ptr[y], 1, wid * 3, filePointer);
	}
      }
      break;
    
    case 'b':
    case 'B':
      {
	fprintf (filePointer, "P4\n%d %d\n", wid, hig);
	denominator = (valMax + 1) / 2; // so "/ denom" yields [0 | 1], threshold 50%
	denominator *= 1000;	// 299 + 587 + 114 == 1000
	const ColorImageRGB8::PixelType **ptr = img.unsafeRowPointerArray();
	for (int y = 0; y < hig; y++) {
	  const ColorImageRGB8::ElementType *linePtr = (ColorImageRGB8::ElementType*)(ptr[y]);
	  int bytesPerRow = (wid + 7) / 8;
	  byte bitRow[bytesPerRow];
	  byte *bitPtr = bitRow;
	  byte curByte = 0;
	  int x;
	  for (x = 0; x < wid; x++) {
            // data must be scaled down here for valMax > 1 !!
	    curByte <<= 1;
	    int val = 299 * *linePtr++;
	    val += 587 * *linePtr++;
	    val += 114 * *linePtr++;
	    curByte |= ((val  / denominator) > 0) ? 0 : 1;
	    if ((x & 0x07) == 7) {
	      *bitPtr++ = curByte;
	      curByte = 0;
	    }
	  }
	  if ((x & 0x07) != 0) { // ship remaining bits in curByte
	    curByte <<= (8 - (x & 0x07));
	    *bitPtr++ = curByte;
	  }
          bytesWritten += fwrite (bitRow, 1, bytesPerRow, filePointer);
	}
      }
      break;
    
    default:
      // TODO PumaMessage (msgImportant, "file is no recognized PNM file");
      success = false;
      break;
  }
  fclose (filePointer);
  return success;
}

// --------------------------------------------------------
// writer for other image files (using libMagick++ from www.imagemagick.org)
// --------------------------------------------------------

#ifdef HAVE_IMAGEMAGICK

static bool writeImageGrayMagick(const GrayLevelImage8 &img, string filename)
{
  Magick::Image *magickImage;
  const GrayLevelImage8::PixelType *byteDataPointer = img.unsafeRowPointerArray()[0];
  byte *copiedData = NULL;

  if (img.isSubImage()) {
    // very ugly: no way to tell ImageMagick about subimages.
	// must make a copy of it so ImageMagick can work on it. :-(
	
	const int lineWid = img.getWidth() * sizeof(GrayLevelImage8::PixelType);
	copiedData = new byte[img.getHeight() * lineWid];
	for (int y = 0; y < img.getHeight(); y ++) {
	  memcpy (&(copiedData[y*lineWid]), img.unsafeRowPointerArray()[y], lineWid);
	}
	byteDataPointer = (GrayLevelImage8::PixelType*)copiedData;
  }
  try {
    magickImage = new Magick::Image(
    	img.getWidth(),
    	img.getHeight(),
	"I",
	Magick::CharPixel,
	byteDataPointer		//	&(img.sample(0,0,0))
	);
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch( Magick::Exception &error_ ) {
    string msg ("Caught Image Magic Exception: ");
    msg += error_.what();
    throw PumaException(PumaException::intolerable, msg);
  }

  // chose "RAW" for PNM images. Magick++ does this if compressionType
  // is set to anything else but "NoCompression". 
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0))
    magickImage->compressType(Magick::ZipCompression);

  int bitDepth = sizeof(img[0][0]) * 8;
  if (0 /* depth() really used by ImageMagick? I doubt it */ ) {
    unsigned int valMax = int(img.getValueRangeMaximum());
    while (valMax > 0) {
      bitDepth++;
      valMax >>= 1;
    }
  }
  magickImage->compressType(Magick::ZipCompression); // try allways to compress
  magickImage->depth(bitDepth);
  magickImage->write(filename);
  if (copiedData)
    delete [] copiedData;
  delete magickImage;
  return true;
}

static bool writeImageGrayMagick(const GrayLevelImage16 &img, string filename)
{
  Magick::Image *magickImage;
  const GrayLevelImage16::PixelType *byteDataPointer = img.unsafeRowPointerArray()[0];
  byte *copiedData = NULL;

  if (img.isSubImage()) {
    // very ugly: no way to tell ImageMagick about subimages.
	// must make a copy of it so ImageMagick can work on it. :-(
	
	const int lineWid = img.getWidth() * sizeof(GrayLevelImage16::PixelType);
	copiedData = new byte[img.getHeight() * lineWid];
	for (int y = 0; y < img.getHeight(); y ++) {
	  memcpy (&(copiedData[y*lineWid]), img.unsafeRowPointerArray()[y], lineWid);
	}
	byteDataPointer = (GrayLevelImage16::PixelType*)copiedData;
  }

  try {
    magickImage = new Magick::Image(
    	img.getWidth(),
    	img.getHeight(),
	"I",
	Magick::ShortPixel,
	byteDataPointer	//	&(img.sample(0,0,0))
	);
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch( Magick::Exception &error_ ) {
    string msg ("Caught Image Magic Exception: ");
    msg += error_.what();
    throw PumaException(PumaException::intolerable, msg);
  }

  // chose "RAW" for PNM images. Magick++ does this if compressionType
  // is set to anything else but "NoCompression". 
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0))
    magickImage->compressType(Magick::ZipCompression);

  int bitDepth = sizeof(img[0][0]) * 8;
  if (0 /* depth() really used by ImageMagick? I doubt it */ ) {
    unsigned int valMax = int(img.getValueRangeMaximum());
    while (valMax > 0) {
      bitDepth++;
      valMax >>= 1;
    }
  }
  magickImage->compressType(Magick::ZipCompression); // try allways to compress
  magickImage->depth(bitDepth);
  magickImage->write(filename);
  if (copiedData)
    delete [] copiedData;
  delete magickImage;
  return true;
}

static bool writeImageColorMagick(const ColorImageRGB8 &img, string filename)
{
  Magick::Image *magickImage;
  const ColorImageRGB8::PixelType *byteDataPointer = img.unsafeRowPointerArray()[0];
  byte *copiedData = NULL;

  if (img.isSubImage()) {
    // very ugly: no way to tell ImageMagick about subimages.
	// must make a copy of it so ImageMagick can work on it. :-(
	
	const int lineWid = img.getWidth() * sizeof(ColorImageRGB8::PixelType);
	copiedData = new byte[img.getHeight() * lineWid];
	for (int y = 0; y < img.getHeight(); y ++) {
	  memcpy (&(copiedData[y*lineWid]), img.unsafeRowPointerArray()[y], lineWid);
	}
	byteDataPointer = (ColorImageRGB8::PixelType*)copiedData;
  }

  try {
    magickImage = new Magick::Image(
    	img.getWidth(),
    	img.getHeight(),
	"RGB",
	Magick::CharPixel,
	byteDataPointer	//	&(img.sample(0,0,0))
      );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch( Magick::Exception &error_ ) {
    string msg ("Caught Image Magic Exception: ");
    msg += error_.what();
    throw PumaException(PumaException::intolerable, msg);
  }

  // chose "RAW" for PNM images. Magick++ does this if compressionType
  // is set to anything else but "NoCompression". 
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0))
    magickImage->compressType(Magick::ZipCompression);

  magickImage->compressType(Magick::ZipCompression); // try allways to compress
  magickImage->depth(8);
  magickImage->write(filename);
  if (copiedData)
    delete [] copiedData;
  delete magickImage;
  return true;
}

static bool writeImageColorMagick(const ColorImageRGBa8 &img, string filename)
{
  Magick::Image *magickImage;
  const ColorImageRGBa8::PixelType *byteDataPointer = img.unsafeRowPointerArray()[0];
  byte *copiedData = NULL;

  if (img.isSubImage()) {
    // very ugly: no way to tell ImageMagick about subimages.
	// must make a copy of it so ImageMagick can work on it. :-(
	
	const int lineWid = img.getWidth() * sizeof(ColorImageRGBa8::PixelType);
	copiedData = new byte[img.getHeight() * lineWid];
	for (int y = 0; y < img.getHeight(); y ++) {
	  memcpy (&(copiedData[y*lineWid]), img.unsafeRowPointerArray()[y], lineWid);
	}
	byteDataPointer = (ColorImageRGBa8::PixelType*)copiedData;
  }

  try {
    magickImage = new Magick::Image(
    	img.getWidth(),
    	img.getHeight(),
	"RGBA",
	Magick::CharPixel,
	byteDataPointer	//	&(img.sample(0,0,0))
      );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch( Magick::Exception &error_ ) {
    string msg ("Caught Image Magic Exception: ");
    msg += error_.what();
    throw PumaException(PumaException::intolerable, msg);
  }

  // chose "RAW" for PNM images. Magick++ does this if compressionType
  // is set to anything else but "NoCompression". 
  int slen = filename.length();
  if ((filename.compare(slen-4, 4, ".pbm") == 0)
   || (filename.compare(slen-4, 4, ".pgm") == 0)
   || (filename.compare(slen-4, 4, ".ppm") == 0)
   || (filename.compare(slen-4, 4, ".pnm") == 0))
    magickImage->compressType(Magick::ZipCompression);

  magickImage->compressType(Magick::ZipCompression); // try allways to compress
  magickImage->depth(8);
  magickImage->write(filename);
  if (copiedData)
    delete [] copiedData;
  delete magickImage;
  return true;
}
#endif  /* HAVE_IMAGEMAGICK */
