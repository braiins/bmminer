#ifndef __CONSTRUCT_H__
#define __CONSTRUCT_H__

#include <string.h>

/**
 * Motivation for Construct Buffer
 *
 * Construct Buffer is a little handy structure that serves as an aid in
 * incremental string creation.  It's main feature is that it looks like a
 * "output" handle, into which you can write/printf to, and that it handles
 * overflows gracefully.
 *
 * The buffer where construction happens is allocated statically by caller
 * (cnstrct_init). When you overflow the buffer, an "overflow" flag is set and
 * all subsequent operation are "NO-OPs", until this flag is cleared (which is
 * _handy_, because you don't have to check return values/error until the end).
 *
 * A buffer can be extracted at the end of construction.
 */

struct construct_buf {
	int overflow;
	char *buf, *ptr, *end;
};

/**
 * Append printf-style formatted data to buffer
 *
 * @param cbuf Initialized construct structure
 * @param fmt Format string
 * @param ... ...
 */
int cnstrct_printf(struct construct_buf *cbuf, const char *fmt, ...);

/**
 * Initialize construct buffer with memory
 *
 * @param cbuf Uninitialized construct structure
 * @param buf Data buffer
 * @param size Size of @c buf
 */
static inline void cnstrct_init(struct construct_buf *cbuf, char *buf, int size)
{
	cbuf->overflow = 0;
	cbuf->buf = cbuf->ptr = buf;
	cbuf->end = buf + size;
}

/**
 * Append character into construct buffer
 *
 * @param cbuf Initialized construct structure
 * @param x A character
 *
 * @returns 1 on overflow, 0 otherwise
 */
static inline int cnstrct_putc(struct construct_buf *cbuf, char x)
{
	if (cbuf->overflow || cbuf->ptr >= cbuf->end) {
		cbuf->overflow = 1;
		return 1;
	}
	*cbuf->ptr++ = x;
	return 0;
}

/**
 * Query whether construct buffer encountered an overflow
 *
 * @param cbuf Initialized construct structure
 *
 * @returns 1 on overflow, 0 otherwise
 *
 * @see https://www.jstor.org/stable/453244?seq=1#page_scan_tab_contents
 */
static inline int cnstrct_has_overflown(struct construct_buf *cbuf)
{
	return !!cbuf->overflow;
}

/**
 * Get number of characters constructed in buffer
 *
 * @param cbuf Initialized construct structure
 *
 * @returns 0 on overflow, number of characters otherwise
 */
static inline size_t cnstrct_get_len(struct construct_buf *cbuf)
{
	if (cbuf->overflow)
		return 0;
	return cbuf->ptr - cbuf->buf;
}

int cnstrct_print_hex(struct construct_buf *cbuf, void *mem, int len);
int cnstrct_json_quote(struct construct_buf *cbuf, char *buf, int len);

static inline int cnstrct_json_quote_str(struct construct_buf *cbuf, char *str)
{
	return cnstrct_json_quote(cbuf, str, strlen(str));
}

#endif /* __CONSTRUCT_H__ */
