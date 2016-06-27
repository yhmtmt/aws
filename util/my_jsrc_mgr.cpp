#include "stdafx.h"
// Copyright(c) 2012 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// my_jsrc_mgr is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// my_jsrc_mgr is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with my_jsrc_mgr.  If not, see <http://www.gnu.org/licenses/>. 
#ifdef SANYO_HD5400
#include <iostream>
using namespace std;

#include "jinclude.h"
#include "jpeglib.h"
#include "jerror.h"
#include "my_jsrc_mgr.h"

#define INPUT_BUF_SIZE  20480	/* choose an efficiently fread'able size */

METHODDEF (void)
	init_source (j_decompress_ptr cinfo)
{
}

METHODDEF (boolean)
	fill_input_buffer(j_decompress_ptr cinfo)
{
	// special case for io-suspension
	// cinfo.src->buffer should be filled by outer loop. (maybe write_data call back of libcurl)
	return FALSE;
}

METHODDEF (void)
	skip_input_data (j_decompress_ptr cinfo, long num_bytes)
{
	s_jsrc_mgr * psrc = (s_jsrc_mgr*) cinfo->src;

	if(num_bytes > psrc->pub.bytes_in_buffer){
		// special case for io-suspension
		psrc->skip_bytes = (unsigned int) (num_bytes - psrc->pub.bytes_in_buffer);
		psrc->pub.bytes_in_buffer = 0; 
		psrc->buffer = NULL;
	}else{
		psrc->pub.bytes_in_buffer -= num_bytes;
		psrc->pub.next_input_byte += num_bytes;
	}
}

METHODDEF (void)
	term_source (j_decompress_ptr cinfo)
{
	// no-op
}

GLOBAL (void)
	jpeg_my_src(j_decompress_ptr cinfo)
{
	s_jsrc_mgr * psrc;

	if(cinfo->src == NULL){
			cinfo->src = (struct jpeg_source_mgr *) 
				(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				SIZEOF(s_jsrc_mgr));
			psrc = (s_jsrc_mgr *) cinfo->src;
			psrc->buffer = (JOCTET *)
				(*cinfo->mem->alloc_small) ((j_common_ptr) cinfo, JPOOL_PERMANENT,
				INPUT_BUF_SIZE * SIZEOF(JOCTET));
	}

	psrc = (s_jsrc_mgr*) cinfo->src;
	psrc->skip_bytes = 0;
	psrc->pub.init_source = init_source;
	psrc->pub.skip_input_data = skip_input_data;
	psrc->pub.fill_input_buffer = fill_input_buffer;
	psrc->pub.resync_to_restart = jpeg_resync_to_restart;
	psrc->pub.term_source = term_source;
	psrc->pub.bytes_in_buffer = 0;
	psrc->pub.next_input_byte = NULL;
}

#endif