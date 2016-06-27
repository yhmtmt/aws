// Copyright(c) 2014 Yohei Matsumoto, Tokyo University of Marine
// Science and Technology, All right reserved. 

// my_jsrc_mgr.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Publica License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// my_jsrc_mgr.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with my_jsrc_mgr.h.  If not, see <http://www.gnu.org/licenses/>. 

#ifdef SANYO_HD5400
GLOBAL (void) jpeg_my_src(j_decompress_ptr cinfo);

typedef struct {
	struct jpeg_source_mgr pub;
	JOCTET * buffer;		/* start of buffer */
	unsigned int skip_bytes; /* special treatment for io-suspension in skip_input_data */
} s_jsrc_mgr;
#endif
