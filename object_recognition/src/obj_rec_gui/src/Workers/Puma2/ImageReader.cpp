#include "puma2config.h"
#include "ImageReader.h"
#include "PumaException.h"
#include "PumaMessages.h"

#include <cstdio>
#include <cerrno>
#include <string>

using namespace std;
using namespace puma2;

#ifdef HAVE_LIBPNG
# include <png.h>
static bool readImageGrayPNG ( GrayLevelImage8 &img, FILE *fp );
static bool readImageColorPNG ( ColorImageRGB8 &img, FILE *fp );
#endif

#ifdef HAVE_LIBJPEG
//extern "C"
//{
# include <jpeglib.h>
  // special in memory jpeg lib src, see jmemsrc.c
  void jpeg_memory_src ( j_decompress_ptr cinfo, JOCTET * ptr, size_t numBytes );
//};

static bool readImageGrayJPEG ( GrayLevelImage8 &img, FILE *fp );
static bool readImageColorJPEGFromFile ( ColorImageRGB8 &img, FILE *fp );
static bool readImageColorJPEGFromMemory ( ColorImageRGB8 &img, JOCTET * ptr, size_t numBytes );
static bool readImageColorJPEG_generic ( ColorImageRGB8 &img, struct jpeg_decompress_struct &cinfo );
#endif

#ifdef HAVE_IMAGEMAGICK
#  include <Magick++/Image.h>
static bool readImageGrayMagick ( GrayLevelImage8 &img, string filename );
static bool readImageGrayMagick ( GrayLevelImage16 &img, string filename );
static bool readImageColorMagick ( ColorImageRGB8 &img, string filename );
static bool readImageColorMagick ( ColorImageRGBa8 &img, string filename );
#endif

// forward declarations
static bool readImageGrayBuiltinPNM ( GrayLevelImage8 &img, FILE *fp );
static bool readImageColorBuiltinPNM ( ColorImageRGB8 &img, FILE *fp );


bool ImageReader::readImage ( GrayLevelImage8 &img, string filename )
{
  const int lookAheadLength = 8;
  byte magicChars [lookAheadLength];
  int returnValue;
  bool success = false;

  FILE *filePointer = fopen ( filename.c_str(), "rb" ); // "b" for binary mode on non-Unix
  if ( filePointer == NULL )
  {
    string msg ( "cannot open file: " );
    msg += filename;
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  returnValue = fread ( magicChars, lookAheadLength, 1, filePointer );
  if ( returnValue != 1 )
  {
    string msg ( "can't read from file: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }
  rewind ( filePointer );
  if ( ftell ( filePointer ) != 0 )
  {
    string msg ( "can't peek into file start, rewind failed: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }

#ifdef HAVE_LIBPNG
  if ( png_sig_cmp ( magicChars, 0, lookAheadLength ) == 0 )
  {
    success = readImageGrayPNG ( img, filePointer );
  }
  else
#endif
#ifdef HAVE_LIBJPEG
    if ( ( magicChars[0] == 0xff ) && ( magicChars[1] == 0xd8 ) )
    {
      success = readImageGrayJPEG ( img, filePointer );
    }
    else
#endif
      if ( ( magicChars[0] == 'P' )
           && ( ( magicChars[1] >= '1' ) && ( magicChars[1] <= '6' ) )
           && ( ( magicChars[2] == ' ' ) || ( magicChars[2] == '\n' ) || ( magicChars[2] == '\r' ) || ( magicChars[2] == '\t' ) )
         )
      {
        success = readImageGrayBuiltinPNM ( img, filePointer );
      }
  fclose ( filePointer );

#ifdef HAVE_IMAGEMAGICK
  if ( ! success ) // ImageMagick++ has no method to read from a file pointer :-(
  {
    success = readImageGrayMagick ( img, filename );
  }
#endif
  if ( ! success )
  {
    string msg ( "unknown/unsupported image format, can't read file." );
    throw PumaException ( PumaException::intolerable, msg );
    success = false;
  }
  return success;
}

bool ImageReader::readImage ( GrayLevelImage16 &img, string filename )
{
  const int lookAheadLength = 8;
  byte magicChars [lookAheadLength];
  int returnValue;
  bool success = false;

  FILE *filePointer = fopen ( filename.c_str(), "rb" ); // "b" for binary mode on non-Unix
  if ( filePointer == NULL )
  {
    string msg ( "cannot open file: " );
    msg += filename;
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  returnValue = fread ( magicChars, lookAheadLength, 1, filePointer );
  if ( returnValue != 1 )
  {
    string msg ( "can't read from file: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }
  rewind ( filePointer );
  if ( ftell ( filePointer ) != 0 )
  {
    string msg ( "can't peek into file start, rewind failed: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }

#ifdef HAVE_IMAGEMAGICK
  if ( ! success ) // ImageMagick++ has no method to read from a file pointer :-(
  {
    success = readImageGrayMagick ( img, filename );
  }
#endif
  if ( ! success )
  {
    string msg ( "unknown/unsupported image format, can't read file." );
    throw PumaException ( PumaException::intolerable, msg );
    success = false;
  }
  return success;
}

bool ImageReader::readImage ( ColorImageRGB8 &img, string filename )
{
  const int lookAheadLength = 8;
  byte magicChars [lookAheadLength];
  int returnValue;
  bool success = false;

  FILE *filePointer = fopen ( filename.c_str(), "rb" ); // "b" for binary mode on non-Unix
  if ( filePointer == NULL )
  {
    string msg ( "cannot open file: " );
    msg += filename;
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  returnValue = fread ( magicChars, lookAheadLength, 1, filePointer );
  if ( returnValue != 1 )
  {
    string msg ( "can't read from file: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }
  rewind ( filePointer );
  if ( ftell ( filePointer ) != 0 )
  {
    string msg ( "can't peek into file start, rewind failed: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }

#ifdef HAVE_LIBPNG
  if ( png_sig_cmp ( magicChars, 0, lookAheadLength ) == 0 )
  {
    success = readImageColorPNG ( img, filePointer );
  }
  else
#endif
#ifdef HAVE_LIBJPEG
    if ( ( magicChars[0] == 0xff ) && ( magicChars[1] == 0xd8 ) )
    {
      success = readImageColorJPEGFromFile ( img, filePointer );
    }
    else
#endif
      if ( ( magicChars[0] == 'P' )
           && ( ( magicChars[1] >= '1' ) && ( magicChars[1] <= '6' ) )
           && ( ( magicChars[2] == ' ' ) || ( magicChars[2] == '\n' ) || ( magicChars[2] == '\r' ) || ( magicChars[2] == '\t' ) )
         )
      {
        success = readImageColorBuiltinPNM ( img, filePointer );
      }
  fclose ( filePointer );

#ifdef HAVE_IMAGEMAGICK
  if ( ! success ) // ImageMagick++ has no method to read from a file pointer :-(
  {
    success = readImageColorMagick ( img, filename );
  }
#endif
  if ( ! success )
  {
    string msg ( "unknown image format, can't read file '" );
    msg += filename + "'.";
    throw PumaException ( PumaException::intolerable, msg );
    success = false;
  }
  return success;
}

bool ImageReader::loadJPEGImage ( ColorImageRGB8  &img, byte* ptr, size_t numBytes )
{
#ifdef HAVE_LIBJPEG
  if ( ptr == NULL )
  {
    string msg ( "can't load data from NULL pointer" );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }
  return readImageColorJPEGFromMemory ( img, ptr, numBytes );
#else
  string msg ( "libpuma2 compiled without libjpeg support, loadJPEGImage() not available" );
  throw PumaException ( PumaException::intolerable, msg );
#endif
}

bool ImageReader::readImage ( ColorImageRGBa8 &img, string filename )
{
  const int lookAheadLength = 8;
  byte magicChars [lookAheadLength];
  int returnValue;
  bool success = false;

  FILE *filePointer = fopen ( filename.c_str(), "rb" ); // "b" for binary mode on non-Unix
  if ( filePointer == NULL )
  {
    string msg ( "cannot open file: " );
    msg += filename;
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  returnValue = fread ( magicChars, lookAheadLength, 1, filePointer );
  if ( returnValue != 1 )
  {
    string msg ( "can't read from file: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }
  rewind ( filePointer );
  if ( ftell ( filePointer ) != 0 )
  {
    string msg ( "can't peek into file start, rewind failed: " );
    msg += strerror ( errno );
    throw PumaException ( PumaException::intolerable, msg );
  }

#ifdef HAVE_LIBPNG_ANDISIMPLEMENTED
  if ( png_sig_cmp ( magicChars, 0, lookAheadLength ) == 0 )
  {
    success = readImageColorPNG ( img, filePointer );
  }
  else
#endif
#ifdef HAVE_LIBJPEG_ANDISIMPLEMENTED
    if ( ( magicChars[0] == 0xff ) && ( magicChars[1] == 0xd8 ) )
    {
      success = readImageColorJPEGFromFile ( img, filePointer );
    }
    else
#endif
#ifdef USE_BUILTIN_PNM_ANDISIMPLEMENTED
      if ( ( magicChars[0] == 'P' )
           && ( ( magicChars[1] >= '1' ) && ( magicChars[1] <= '6' ) )
           && ( ( magicChars[2] == ' ' ) || ( magicChars[2] == '\n' ) || ( magicChars[2] == '\r' ) || ( magicChars[2] == '\t' ) )
         )
      {
        success = readImageColorBuiltinPNM ( img, filePointer );
      }
#endif
  fclose ( filePointer );

#ifdef HAVE_IMAGEMAGICK
  if ( ! success ) // ImageMagick++ has no method to read from a file pointer :-(
  {
    success = readImageColorMagick ( img, filename );
  }
#endif
  if ( ! success )
  {
    string msg ( "unknown image format, can't read file '" );
    msg += filename + "'.";
    throw PumaException ( PumaException::intolerable, msg );
    success = false;
  }
  return success;
}

// --------------------------------------------------------
// reader for PNM files
// --------------------------------------------------------

/**
 * Skip comments in a PNM (PBM, PGM, PPM) file header (or body for P1-P3).
 * Reads and discards characters from a file pointer until end-of-line
 * or and-of-file is reached.
 * @param[in]   fp    File pointer to read from
 * @pre  comment char '#' was just read before with getc(fp)
 * @post newline char of this comment was read as last character or
 * end-of-file was reached.
 */
static void skipPNMcomment ( FILE *fp )
{
  int c = getc ( fp );
  while ( c != '\n' && c != EOF )
  {
    c = getc ( fp );
  }
}

/**
 * Read an integer value form the header of a PNM (PBM, PGM, PPM) file
 * header (or body for P1-P3).
 * Skips any white space and comments, then reads a sequence of digits.
 * @param[in]   fp    File pointer to read from
 * @return  returns the integer value of the number read.
 * @pre  fp points somwhere were the next integer is expected
 * @post  character right behind the last digit was consumed by getc(),
 * or end-of-file was reached.
 */
static int getPNMinteger ( FILE *fp )
{
  int ch = ' ';

  while ( 1 ) /* eat comments and white space */
  {
    if ( ch == '#' ) /* if we're at a comment, read to end of line */
      skipPNMcomment ( fp );
    if ( ch == EOF ) return 0;
    if ( ch >= '0' && ch <= '9' ) break; /* found a digit */
    ch = getc ( fp );
  }

  /* we're at the start of a number, continue until we hit a non-digit */
  int value = 0;
  while ( 1 )
  {
    value = ( value * 10 ) + ( ch - '0' );
    ch = getc ( fp );
    if ( ch == EOF ) return value;
    if ( ch < '0' || ch > '9' ) break;
  }
  return value;
}


static bool readPNMheader ( FILE *filePointer, int &pCode, int &pWid, int &pHig, int &pDep )
{
  int ch;

  ch = getc ( filePointer );
  if ( ch != 'P' )
  {
    // TODO PumaMessage ( msgImportant, "file is not a PNM file" );
    return false;
  }
  pCode = getc ( filePointer ) - '0';
  if ( ( pCode < 1 ) || ( pCode > 6 ) )
  {
    // TODO  PumaMessage ( msgImportant, "file is not a classic PNM file" );
    return false;
  }
  pWid  = getPNMinteger ( filePointer );
  pHig = getPNMinteger ( filePointer );
  if ( ( pWid < 1 ) || ( pHig < 1 ) )
  {
    // TODO PumaMessage ( msgImportant, "file is not a valid PNM file (width or height < 1)" );
    return false;
  }
  if ( ( pCode == 1 ) || ( pCode == 4 ) )
  {
    pDep = 1;
  }
  else
  {
    pDep = getPNMinteger ( filePointer );
    if ( pDep > 255 )
    {
      // TODO PumaMessage ( msgNotice, "reading PNM files with depth > 255 not yet supported by builtin" );
      return false;
    }
  }
  return true;
}

static bool readImageGrayBuiltinPNM ( GrayLevelImage8 &img,
                                      FILE *filePointer )
{
  int pnmCode;
  int pnmWidth, pnmHeight, pnmDepth;

  if ( ! readPNMheader ( filePointer, pnmCode, pnmWidth, pnmHeight, pnmDepth ) )
    return false;

  img.resize ( pnmWidth, pnmHeight );
  int bytesRead = 0;
  switch ( pnmCode )
  {
      // =============================================================================
    case 1:   // PBM in ASCII, depth is 1 by definition, 1 is black, 0 is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PBM (P1) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 2:   // PGM in ASCII, depth from file, 0 is black, depth is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PGM (P2) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 3:   // PPM in ASCII, depth from file, 0 is black, depth is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PPM (P3) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 4:   // PBM binary, depth is 1 by definition, 1 is black, 0 is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PBM (P4) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 5:   // PGM binary, depth from file, 0 is black, depth is white
    {
      img.setValueRangeMaximum ( pnmDepth );
      GrayLevelImage8::ElementType **ptr = img.unsafeRowPointerArray();
      for ( int y = 0; y < pnmHeight; y++ )
      {
        bytesRead += fread ( ptr[y], 1, pnmWidth * img.numberOfChannels(), filePointer );
      }
      if ( bytesRead != pnmWidth * pnmHeight * img.numberOfChannels() )
      {
        string msg ( "File appears to be truncated: " );
        // msg += strerror(errno); "(got %d - expected %d bytes)\n", bytesRead, xs*ys);
        throw PumaException ( PumaException::intolerable, msg );
      }
      break;
    }
    // =============================================================================
    case 6:   // PPM binary, depth from file, RGB, 0 is black, depth is white
    {
      img.setValueRangeMaximum ( pnmDepth );
      // TODO PumaMessage ( msgNotice, "reading PPM image into GrayLevelImage, discarding color" );
      for ( int y = 0; y < pnmHeight; y++ )
      {
        for ( int x = 0; x < pnmWidth; x++ )
        {
          int val = 299 * getc ( filePointer );
          val += 587 * getc ( filePointer );
          val += 114 * getc ( filePointer );
          img.sample ( x, y, 0 ) = byte ( val / 1000 );
          bytesRead += 3;
        }
      }
      if ( bytesRead != pnmWidth * pnmHeight * 3 )
      {
        string msg ( "File appears to be truncated: " );
        // msg += strerror(errno); "(got %d - expected %d bytes)\n", bytesRead, xs*ys);
        throw PumaException ( PumaException::intolerable, msg );
      }
      break;
    }
    // =============================================================================
    case 7:   // PAM, totally different, yet more flexible format of NetPBM
    {
      // TODO PumaMessage ( msgNotice, "builtin PNM reader can't handle PNM:PAM files" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    default:   // unknown
    {
      // TODO PumaMessage ( msgNotice, "builtin PNM reader: unknown PNM format" );
      return false; // let ImageMagick try it
      break;
    }
  }
  return true;
}

static bool readImageColorBuiltinPNM ( ColorImageRGB8 &img,
                                       FILE *filePointer )
{
  int pnmCode;
  int pnmWidth, pnmHeight, pnmDepth;

  if ( ! readPNMheader ( filePointer, pnmCode, pnmWidth, pnmHeight, pnmDepth ) )
    return false;

  img.resize ( pnmWidth, pnmHeight );
  int bytesRead = 0;
  switch ( pnmCode )
  {
      // =============================================================================
    case 1:   // PBM in ASCII, depth is 1 by definition, 1 is black, 0 is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PBM (P1) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 2:   // PGM in ASCII, depth from file, 0 is black, depth is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PGM (P2) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 3:   // PPM in ASCII, depth from file, 0 is black, depth is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PPM (P3) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 4:   // PBM binary, depth is 1 by definition, 1 is black, 0 is white
    {
      // TODO PumaMessage ( msgNotice, "builtin reading PBM (P4) files not yet implemented" );
      return false; // let ImageMagick try it
      break;
    }
    // =============================================================================
    case 5:   // PGM binary, depth from file, 0 is black, depth is white
    {
      img.setValueRangeMaximum ( pnmDepth );
      ColorImageRGB8::PixelType **ptr = img.unsafeRowPointerArray();
      byte *readBuf = new byte[pnmWidth];
      for ( int y = 0; y < pnmHeight; y++ )
      {
        ColorImageRGB8::ElementType *sptr = & ( ptr[y][0][0] );
        fread ( readBuf, pnmWidth, 1, filePointer );
        byte * valp = readBuf;
        for ( int x = 0; x < pnmWidth; x++ )
        {
          *sptr++ = *valp;
          *sptr++ = *valp;
          *sptr++ = *valp++;
          bytesRead += 1;
        }
      }
      delete readBuf;
      if ( bytesRead != pnmWidth * pnmHeight * 1 )
      {
        string msg ( "File appears to be truncated: " );
        // msg += strerror(errno); "(got %d - expected %d bytes)\n", bytesRead, xs*ys);
        throw PumaException ( PumaException::intolerable, msg );
      }
      break;
    }
    // =============================================================================
    case 6:   // PPM binary, depth from file, RGB, 0 is black, depth is white
    {
      img.setValueRangeMaximum ( pnmDepth );
      ColorImageRGB8::PixelType **ptr = img.unsafeRowPointerArray();
      for ( int y = 0; y < pnmHeight; y++ )
      {
        bytesRead += fread ( ptr[y], 1, pnmWidth * img.numberOfChannels(), filePointer );
      }
      if ( bytesRead != pnmWidth * pnmHeight * img.numberOfChannels() )
      {
        string msg ( "File appears to be truncated: " );
        // msg += strerror(errno); "(got %d - expected %d bytes)\n", bytesRead, xs*ys);
        throw PumaException ( PumaException::intolerable, msg );
      }
      break;
    }
    // =======================================================================
    case 7:   // PAM, totally different, yet more flexible format of NetPBM
    {
      // TODO PumaMessage ( msgNotice, "builtin PNM reader can't handle PNM:PAM files" );
      return false; // let ImageMagick try it
      break;
    }
    // =======================================================================
    default:   // unknown
    {
      // TODO PumaMessage ( msgNotice, "builtin PNM reader: unknown PNM format" );
      return false; // let ImageMagick try it
      break;
    }
  }
  return true;
}


// --------------------------------------------------------
// specific reader for PNG files (using libpng from www.libpng.org)
// --------------------------------------------------------

#ifdef HAVE_LIBJPEG

#if BITS_IN_JSAMPLE != 8
# error "can't yet deal with JPEG samples other than 8 Bit"
#endif

static bool readImageGrayJPEG ( GrayLevelImage8 &img, FILE *filePointer )
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  rewind ( filePointer );
  cinfo.err = jpeg_std_error ( &jerr );
  jpeg_create_decompress ( &cinfo );
  jpeg_stdio_src ( &cinfo, filePointer );
  int ret = jpeg_read_header ( &cinfo, TRUE );
  if ( ret != JPEG_HEADER_OK )
  {
    string msg ( "can't read JPEG header." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

#if JPEG_LIB_VERSION <= 62
  // jpeglib v.6b will fail on these color spaces and raise an exception
  // when calling jpeg_start_decompress(). So let's check before ...
  // Apparently there is no implicit/automatic conversion CMYK -> RGB
  if ( ( cinfo.jpeg_color_space == JCS_CMYK )
       || ( cinfo.jpeg_color_space == JCS_YCCK ) )
  {
    string msg ( "can't read JPEG files with CMYC or YCCK color space natively." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }
#endif

  // set non-default decompression parameters here: force gray scale
  if ( cinfo.output_components != 1 )
    // TODO PumaMessage ( msgNotice, "reading JPEG image into GrayLevelImage, "
//"discarding color" );
  cinfo.out_color_space = JCS_GRAYSCALE;
  cinfo.out_color_components = 1;
  cinfo.output_components = 1;

  int wid = cinfo.image_width;
  int hig = cinfo.image_height;

  jpeg_start_decompress ( &cinfo );

  img.resize ( wid, hig );
  // byte ** row_ptrs = img;
  byte ** row_ptrs = img.unsafeRowPointerArray();
  while ( cinfo.output_scanline < cinfo.output_height )
  {
    jpeg_read_scanlines ( &cinfo, &row_ptrs[cinfo.output_scanline],
                          cinfo.output_height - cinfo.output_scanline );
  }

  jpeg_finish_decompress ( &cinfo );
  jpeg_destroy_decompress ( &cinfo );
  return true;
}

static bool readImageColorJPEGFromFile ( ColorImageRGB8 &img, FILE *filePointer )
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  rewind ( filePointer );
  cinfo.err = jpeg_std_error ( &jerr );
  jpeg_create_decompress ( &cinfo );
  jpeg_stdio_src ( &cinfo, filePointer );

  return readImageColorJPEG_generic ( img, cinfo );
}

static bool readImageColorJPEGFromMemory ( ColorImageRGB8 &img, JOCTET * ptr, size_t numBytes )
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error ( &jerr );
  jpeg_create_decompress ( &cinfo );
  jpeg_memory_src ( &cinfo, ptr, numBytes );

  return readImageColorJPEG_generic ( img, cinfo );
}

static bool readImageColorJPEG_generic ( ColorImageRGB8 &img, struct jpeg_decompress_struct &cinfo )
{
  int ret = jpeg_read_header ( &cinfo, TRUE );
  if ( ret != JPEG_HEADER_OK )
  {
    string msg ( "can't read JPEG header." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

#if JPEG_LIB_VERSION <= 62
  // jpeglib v.6b will fail on these color spaces and raise an exception
  // when calling jpeg_start_decompress(). So let's check before ...
  // Apparently there is no implicit/automatic conversion CMYK -> RGB
  if ( ( cinfo.jpeg_color_space == JCS_CMYK )
       || ( cinfo.jpeg_color_space == JCS_YCCK ) )
  {
    string msg ( "can't read JPEG files with CMYC or YCCK color space natively." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }
#endif

  // set non-default decompression parameters here: force gray scale
  if ( cinfo.output_components == 1 )
    // TODO PumaMessage ( msgNotice, "expanding graylevel JPEG image to RGB8 color image" );
  cinfo.out_color_space = JCS_RGB;
  cinfo.out_color_components = 3;
  cinfo.output_components = 3;

  int wid = cinfo.image_width;
  int hig = cinfo.image_height;

  jpeg_start_decompress ( &cinfo );

  img.resize ( wid, hig );
  // byte ** row_ptrs = img;
  byte ** row_ptrs = ( JSAMPLE ** ) img.unsafeRowPointerArray();
  while ( cinfo.output_scanline < cinfo.output_height )
  {
    jpeg_read_scanlines ( &cinfo, &row_ptrs[cinfo.output_scanline],
                          cinfo.output_height - cinfo.output_scanline );
  }

  jpeg_finish_decompress ( &cinfo );
  jpeg_destroy_decompress ( &cinfo );
  return true;
}
#endif  /* HAVE_LIBJPEG */

// --------------------------------------------------------
// specific reader for JPEG files (using libjpeg from www.ijg.org)
// --------------------------------------------------------

#ifdef HAVE_LIBPNG

static bool readImageGrayPNG ( GrayLevelImage8 &img, FILE *filePointer )
{
  enum { cs_gray, cs_rgb, cs_rgba } outColorSpace = cs_gray;
  enum { st_8bit, st_16bit, st_other } outSampleType = st_8bit;

  png_structp png_ptr = png_create_read_struct
                        ( PNG_LIBPNG_VER_STRING, ( png_voidp ) NULL,
                          NULL, NULL );
  if ( !png_ptr )
  {
    string msg ( "can't read PNG header." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  png_infop info_ptr = png_create_info_struct ( png_ptr );
  if ( !info_ptr )
  {
    png_destroy_read_struct ( &png_ptr,
                              ( png_infopp ) NULL, ( png_infopp ) NULL );
    string msg ( "error setting up PNG structures." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  png_infop end_info = png_create_info_struct ( png_ptr );
  if ( !end_info )
  {
    png_destroy_read_struct ( &png_ptr, &info_ptr,
                              ( png_infopp ) NULL );
    string msg ( "error setting up PNG end structures." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  if ( setjmp ( png_jmpbuf ( png_ptr ) ) )
  {
    png_destroy_read_struct ( &png_ptr, &info_ptr,
                              &end_info );
    string msg ( "severe error reading PNG file." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  rewind ( filePointer );
  png_init_io ( png_ptr, filePointer );
  png_read_info ( png_ptr, info_ptr );

  int wid = png_get_image_width ( png_ptr, info_ptr );
  int hig = png_get_image_height ( png_ptr, info_ptr );
  int color_type = png_get_color_type ( png_ptr, info_ptr );
  int bit_depth = png_get_bit_depth ( png_ptr, info_ptr );
  img.resize ( wid, hig );

  if ( color_type == PNG_COLOR_TYPE_PALETTE )
    png_set_palette_to_rgb ( png_ptr );

  if ( color_type == PNG_COLOR_TYPE_GRAY
       && bit_depth < 8 )
    png_set_expand_gray_1_2_4_to_8 ( png_ptr );

  if ( ( outSampleType == st_8bit ) && ( bit_depth == 16 ) )
    png_set_strip_16 ( png_ptr );

  if ( ( outColorSpace != cs_rgba ) && ( color_type & PNG_COLOR_MASK_ALPHA ) )
    png_set_strip_alpha ( png_ptr );

  if ( bit_depth < 8 )
    png_set_packing ( png_ptr );

  if ( ( outColorSpace == cs_gray )
       && ( color_type == PNG_COLOR_TYPE_RGB ||
            color_type == PNG_COLOR_TYPE_RGB_ALPHA ) )
  {
    // must use error_action == 2, since =1 may cause abort due to bug in libpng 1.2.18
    png_set_rgb_to_gray_fixed ( png_ptr, 2, 29900, 58700 );
  }

  // int number_of_passes = png_set_interlace_handling(png_ptr);
  png_read_update_info ( png_ptr, info_ptr );

  // byte ** row_ptrs = img;
  byte ** row_ptrs = img.unsafeRowPointerArray();
  png_read_image ( png_ptr, row_ptrs );

  png_read_end ( png_ptr, end_info );
  png_destroy_read_struct ( &png_ptr, &info_ptr, &end_info );
  return true;
}

static bool readImageColorPNG ( ColorImageRGB8 &img, FILE *filePointer )
{
  enum { cs_gray, cs_rgb, cs_rgba } outColorSpace = cs_rgb;
  enum { st_8bit, st_16bit, st_other } outSampleType = st_8bit;

  png_structp png_ptr = png_create_read_struct( PNG_LIBPNG_VER_STRING, NULL, NULL, NULL );

  if ( !png_ptr )
  {
    string msg ( "can't read PNG header." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  png_infop info_ptr = png_create_info_struct ( png_ptr );
  if ( !info_ptr )
  {
    png_destroy_read_struct ( &png_ptr,
                              ( png_infopp ) NULL, ( png_infopp ) NULL );
    string msg ( "error setting up PNG structures." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  png_infop end_info = png_create_info_struct ( png_ptr );
  if ( !end_info )
  {
    png_destroy_read_struct ( &png_ptr, &info_ptr,
                              ( png_infopp ) NULL );
    string msg ( "error setting up PNG end structures." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  if ( setjmp ( png_jmpbuf ( png_ptr ) ) )
  {
    png_destroy_read_struct ( &png_ptr, &info_ptr,
                              &end_info );
    string msg ( "severe error reading PNG file." );
    throw PumaException ( PumaException::intolerable, msg );
    return false;
  }

  rewind ( filePointer );
  png_init_io ( png_ptr, filePointer );
  png_read_info ( png_ptr, info_ptr );

  int wid = png_get_image_width ( png_ptr, info_ptr );
  int hig = png_get_image_height ( png_ptr, info_ptr );
  int color_type = png_get_color_type ( png_ptr, info_ptr );
  int bit_depth = png_get_bit_depth ( png_ptr, info_ptr );
  img.resize ( wid, hig );

  if ( color_type == PNG_COLOR_TYPE_PALETTE )
    png_set_palette_to_rgb ( png_ptr );

  if ( color_type == PNG_COLOR_TYPE_GRAY
       && bit_depth < 8 )
    png_set_expand_gray_1_2_4_to_8 ( png_ptr );

  if ( ( outSampleType == st_8bit ) && ( bit_depth == 16 ) )
    png_set_strip_16 ( png_ptr );

  if ( ( outColorSpace != cs_rgba ) && ( color_type & PNG_COLOR_MASK_ALPHA ) )
    png_set_strip_alpha ( png_ptr );

  if ( bit_depth < 8 )
    png_set_packing ( png_ptr );

  if ( ( outColorSpace == cs_gray )
       && ( color_type == PNG_COLOR_TYPE_RGB ||
            color_type == PNG_COLOR_TYPE_RGB_ALPHA ) )
    png_set_rgb_to_gray_fixed ( png_ptr, 1, 29900, 58700 );

  if ( ( outColorSpace == cs_rgb )
       && ( color_type == PNG_COLOR_TYPE_GRAY ||
            color_type == PNG_COLOR_TYPE_GRAY_ALPHA ) )
    png_set_gray_to_rgb ( png_ptr );

  // int number_of_passes = png_set_interlace_handling(png_ptr);
  png_read_update_info ( png_ptr, info_ptr );

  // byte ** row_ptrs = img;
  byte ** row_ptrs = ( byte** ) ( img.unsafeRowPointerArray() );
  png_read_image ( png_ptr, row_ptrs );

  png_read_end ( png_ptr, end_info );
  png_destroy_read_struct ( &png_ptr, &info_ptr, &end_info );
  return true;
}

#endif  /* HAVE_LIBPNG */


// --------------------------------------------------------
// reader for other image files (using libMagick++ from www.imagemagick.org)
// --------------------------------------------------------

#ifdef HAVE_IMAGEMAGICK

static bool readImageGrayMagick ( GrayLevelImage8 &img, string filename )
{
  Magick::Image magickImage;

  try
  {
    magickImage.read ( filename );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch ( Magick::Exception &error_ )
  {
    string msg ( "Caught Image Magic Exception: " );
    msg += error_.what();
    throw PumaException ( PumaException::intolerable, msg );
  }

  unsigned int wid = magickImage.columns();
  unsigned int hig = magickImage.rows();
  double scaleQuotient = 1.0 / 255.0;

  img.resize ( wid, hig );
  if ( magickImage.type() != Magick::GrayscaleType ) // found a color image
  {
    scaleQuotient *= 1000; // 299 + 587 + 114 == 1000
    PumaMessage ( msgNotice, "reading Magick color image into GrayLevelImage, discarding color" );
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::ColorRGB col = magickImage.pixelColor ( x, y );
        double val = 299 * col.red()
                     + 587 * col.green()
                     + 114 * col.blue();
        img.sample ( x, y, 0 ) = GrayLevelImage8::ElementType ( val / scaleQuotient );
      }
  }
  else   // found a gray-level image
  {
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::ColorRGB col = magickImage.pixelColor ( x, y );
        img.sample ( x, y, 0 ) = GrayLevelImage8::ElementType ( col.red() / scaleQuotient );
      }
  }

  return true;
}

static bool readImageGrayMagick ( GrayLevelImage16 &img, string filename )
{
  Magick::Image magickImage;

  try
  {
    magickImage.read ( filename );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch ( Magick::Exception &error_ )
  {
    string msg ( "Caught Image Magic Exception: " );
    msg += error_.what();
    throw PumaException ( PumaException::intolerable, msg );
  }

  unsigned int wid = magickImage.columns();
  unsigned int hig = magickImage.rows();
  double scaleQuotient = 1.0 / 65535.0;

  img.resize ( wid, hig );
  if ( magickImage.type() != Magick::GrayscaleType ) // found a color image
  {
    scaleQuotient *= 1000; // 299 + 587 + 114 == 1000
    PumaMessage ( msgNotice, "reading Magick color image into GrayLevelImage, discarding color" );
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::ColorRGB col = magickImage.pixelColor ( x, y );
        double val = 299 * col.red()
                     + 587 * col.green()
                     + 114 * col.blue();
        img.sample ( x, y, 0 ) = GrayLevelImage8::ElementType ( val / scaleQuotient );
      }
  }
  else   // found a gray-level image
  {
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::ColorRGB col = magickImage.pixelColor ( x, y );
        img.sample ( x, y, 0 ) = GrayLevelImage16::ElementType ( col.red() / scaleQuotient );
      }
  }

  return true;
}

static bool readImageColorMagick ( ColorImageRGB8 &img, string filename )
{
  Magick::Image magickImage;

  try
  {
    magickImage.read ( filename );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch ( Magick::Exception &error_ )
  {
    string msg ( "Caught Image Magic Exception: " );
    msg += error_.what();
    throw PumaException ( PumaException::intolerable, msg );
  }

  unsigned int wid = magickImage.columns();
  unsigned int hig = magickImage.rows();
  long scaleQuotient = 1;
  if ( magickImage.depth() == 16 )
    scaleQuotient = 257; // yes, really 257, to map 65535 -> 255 !!

  img.resize ( wid, hig );
  if ( magickImage.type() != Magick::GrayscaleType ) // found a color image
  {
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::Color col = magickImage.pixelColor ( x, y );
        img.sample ( x, y, 0 ) = ColorImageRGB8::ElementType ( col.redQuantum() / scaleQuotient );
        img.sample ( x, y, 1 ) = ColorImageRGB8::ElementType ( col.greenQuantum() / scaleQuotient );
        img.sample ( x, y, 2 ) = ColorImageRGB8::ElementType ( col.blueQuantum() / scaleQuotient );
      }
  }
  else   // found a gray-level image
  {
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::Color col = magickImage.pixelColor ( x, y );
        img.sample ( x, y, 0 ) =
          img.sample ( x, y, 1 ) =
            img.sample ( x, y, 2 ) =
              ColorImageRGB8::ElementType ( col.redQuantum() / scaleQuotient );
      }
  }

  return true;
}

static bool readImageColorMagick ( ColorImageRGBa8 &img, string filename )
{
  Magick::Image magickImage;

  try
  {
    magickImage.read ( filename );
  }
//   catch( Magick::Warning &warning_ ) {
//     cout << "Caught Image Magic Warning: " << warning_.what() << endl;
//   }
  catch ( Magick::Exception &error_ )
  {
    string msg ( "Caught Image Magic Exception: " );
    msg += error_.what();
    throw PumaException ( PumaException::intolerable, msg );
  }

  unsigned int wid = magickImage.columns();
  unsigned int hig = magickImage.rows();
  bool hasAlpha = magickImage.matte();
  const byte fullyOpaque = byte ( img.getValueRangeMaximum() );
  long scaleQuotient = 1;
  if ( magickImage.depth() == 16 )
    scaleQuotient = 257; // yes, really 257, to map 65535 -> 255 !!

  img.resize ( wid, hig );
  if ( magickImage.type() != Magick::GrayscaleType ) // found a color image
  {
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        Magick::Color col = magickImage.pixelColor ( x, y );
        RGBa8 &pixel = img.sample ( x, y, 0 );
        pixel.r = ColorImageRGB8::ElementType ( col.redQuantum() / scaleQuotient );
        pixel.g = ColorImageRGB8::ElementType ( col.greenQuantum() / scaleQuotient );
        pixel.b = ColorImageRGB8::ElementType ( col.blueQuantum() / scaleQuotient );
        pixel.a = ( hasAlpha )
                  ? ColorImageRGB8::ElementType ( col.alphaQuantum() / scaleQuotient )
                  : fullyOpaque ;
      }
  }
  else   // found a gray-level image
  {
    Magick::Color col;
    byte val;
    for ( unsigned int y = 0; y < hig; y++ )
      for ( unsigned int x = 0; x < wid; x++ )
      {
        col = magickImage.pixelColor ( x, y );
        val = col.redQuantum() / scaleQuotient;
        img.sample ( x, y, 0 ) = val;
        img.sample ( x, y, 1 ) = val;
        img.sample ( x, y, 2 ) = val;
        img.sample ( x, y, 3 ) = ( hasAlpha )
                                 ? ColorImageRGB8::ElementType ( col.alphaQuantum() / scaleQuotient )
                                 : fullyOpaque ;
      }
  }

  return true;
}
#endif  /* HAVE_IMAGEMAGICK */
