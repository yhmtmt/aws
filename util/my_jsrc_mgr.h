#ifdef SANYO_HD5400
GLOBAL (void) jpeg_my_src(j_decompress_ptr cinfo);

typedef struct {
	struct jpeg_source_mgr pub;
	JOCTET * buffer;		/* start of buffer */
	unsigned int skip_bytes; /* special treatment for io-suspension in skip_input_data */
} s_jsrc_mgr;
#endif
