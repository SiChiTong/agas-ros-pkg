/*
 * jmemsrc.c, based on jdatasrc.c
 *
 * jdatasrc.c: Copyright (C) 1994-1996, Thomas G. Lane.
 */
 
/* Provide a data source for libjpeg to read data from memory
 * and not from some file (descriptor).
 * For Robbie 11 tests
 * June 2008, Detlev Droege, droege@uni-koblenz.de
 */

/* this is not a core library module, so it doesn't define JPEG_INTERNALS */
#include "jinclude.h"
#include "jpeglib.h"
#include "jerror.h"


/* Expanded data source object for memory input */

typedef struct {
  struct jpeg_source_mgr pub;	/* public fields */

  size_t bytesInBuffer;		/* source data size */
  JOCTET * buffer;		/* start of buffer */
  boolean start_of_file;	/* have we gotten any data yet? */
  JOCTET eofBuffer[4];		/* buffer for EOI marker if empty input */
} mem_source_mgr;

typedef mem_source_mgr * mem_src_ptr;

/*
 * Initialize source --- called by jpeg_read_header
 * before any data is actually read.
 */

METHODDEF(void)
mem_init_source (j_decompress_ptr cinfo)
{
  mem_src_ptr src = (mem_src_ptr) cinfo->src;

  /* We reset the empty-input-file flag for each image,
   * but we don't clear the input buffer.
   * This is correct behavior for reading a series of images from one source.
   */
  src->start_of_file = TRUE;
}

METHODDEF(boolean)
mem_fill_input_buffer (j_decompress_ptr cinfo)
{
  mem_src_ptr src = (mem_src_ptr) cinfo->src;
  size_t nbytes   = src->bytesInBuffer;

  src->pub.next_input_byte = src->buffer;
  if (src->buffer == NULL) {
    if (src->start_of_file)	/* Treat empty input file as fatal error */
      ERREXIT(cinfo, JERR_INPUT_EMPTY);
    WARNMS(cinfo, JWRN_JPEG_EOF);
    /* Insert a fake EOI marker */
    src->eofBuffer[0] = (JOCTET) 0xFF;
    src->eofBuffer[1] = (JOCTET) JPEG_EOI;
    src->buffer = src->eofBuffer;
    nbytes = 2;
  }

  src->pub.bytes_in_buffer = nbytes;
  src->start_of_file = FALSE;

  return TRUE;
}

METHODDEF(void)
mem_skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
  mem_src_ptr src = (mem_src_ptr) cinfo->src;

  /* Quite simple in memory ....
   */
  if (num_bytes > 0) {
    src->pub.next_input_byte += (size_t) num_bytes;
    src->pub.bytes_in_buffer -= (size_t) num_bytes;
  }
}

METHODDEF(void)
mem_term_source (j_decompress_ptr cinfo)
{
  (void) cinfo;

  /* no work necessary here */
}

/*
 * Prepare for input from memory.
 * The caller must have already stored the data, and is responsible
 * for deallocating the used memory for it after finishing decompression.
 */

GLOBAL(void)
jpeg_memory_src (j_decompress_ptr cinfo, JOCTET * ptr, size_t numBytes)
{
  mem_src_ptr src;

  /* The source object is made permanent so that a series
   * of JPEG images can be read from the same buffer by calling jpeg_memory_src
   * only before the first one. 
   * This makes it unsafe to use this manager and a different source
   * manager serially with the same JPEG object.  Caveat programmer.
   */
  if (cinfo->src == NULL) {	/* first time for this JPEG object? */
    cinfo->src = (struct jpeg_source_mgr *)
      (*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				  SIZEOF(mem_source_mgr));
    src = (mem_src_ptr) cinfo->src;
    src->buffer = ptr;
  }

  src = (mem_src_ptr) cinfo->src;
  src->pub.init_source = mem_init_source;
  src->pub.fill_input_buffer = mem_fill_input_buffer;
  src->pub.skip_input_data = mem_skip_input_data;
  src->pub.resync_to_restart = jpeg_resync_to_restart; /* use default method */
  src->pub.term_source = mem_term_source;
  src->bytesInBuffer = numBytes;
  src->pub.bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
  src->pub.next_input_byte = NULL; /* until buffer loaded */
}
