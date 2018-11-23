/*
 * Copyright 2011-2015 Con Kolivas
 * Copyright 2011-2015 Andrew Smith
 * Copyright 2010 Jeff Garzik
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <jansson.h>
#ifdef HAVE_LIBCURL
#include <curl/curl.h>
#endif
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>


#include <fcntl.h>

#ifdef __linux
# include <sys/prctl.h>
# endif

# include <sys/socket.h>
# include <netinet/in.h>
# include <netinet/tcp.h>
# include <netdb.h>


#include <sched.h>

#include "miner.h"
#include "elist.h"
#include "compat.h"
#include "util.h"

#define DEFAULT_SOCKWAIT 60

bool successful_connect = false;

int no_yield(void)
{
    return 0;
}

int (*selective_yield)(void) = &no_yield;

unsigned char bit_swap_table[256] =
{
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};

static void keep_sockalive(SOCKETTYPE fd)
{
    const int tcp_one = 1;
    const int tcp_keepidle = 45;
    const int tcp_keepintvl = 30;

    int flags = fcntl(fd, F_GETFL, 0);

    fcntl(fd, F_SETFL, O_NONBLOCK | flags);


    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, (const void *)&tcp_one, sizeof(tcp_one));

    if (!opt_delaynet)
    {
        fcntl(fd, F_SETFD, FD_CLOEXEC);
    }

    setsockopt(fd, SOL_TCP, TCP_NODELAY, (const void *)&tcp_one, sizeof(tcp_one));
    setsockopt(fd, SOL_TCP, TCP_KEEPCNT, &tcp_one, sizeof(tcp_one));
    setsockopt(fd, SOL_TCP, TCP_KEEPIDLE, &tcp_keepidle, sizeof(tcp_keepidle));
    setsockopt(fd, SOL_TCP, TCP_KEEPINTVL, &tcp_keepintvl, sizeof(tcp_keepintvl));

}


void *_cgmalloc(uint32_t size, const char *file, const char *func, const int line)
{
    void *ret;

    align_len(&size);
    ret = malloc(size);

    if (unlikely(!ret)) {
        quit(1, "Failed to malloc size %d from %s %s:%d", size, file, func, line);
    }

    return ret;
}

void *_cgcalloc(int memb, size_t size, const char *file, const char *func, const int line)
{
    void *ret;

    align_len(&size);
    ret = calloc(memb, size);

    if (unlikely(!ret)) {
        quit(1, "Failed to calloc memb %d size %d from %s %s:%d", memb, size, file, func, line);
    }

    return ret;
}

void *_cgrealloc(void *ptr, size_t size, const char *file, const char *func, const int line)
{
    void *ret;

    align_len(&size);
    ret = realloc(ptr, size);

    if (unlikely(!ret)) {
        quit(1, "Failed to realloc size %d from %s %s:%d", size, file, func, line);
    }

    return ret;
}

struct tq_ent {

    void *data;
    struct list_head q_node;
};


#ifdef HAVE_LIBCURL
struct timeval nettime;

struct data_buffer
{
    void        *buf;
    size_t      len;
};

struct upload_buffer
{
    const void  *buf;
    size_t      len;
};

struct header_info
{
    char        *lp_path;
    int     rolltime;
    char        *reason;
    char        *stratum_url;
    bool        hadrolltime;
    bool        canroll;
    bool        hadexpire;
};

static void databuf_free(struct data_buffer *db)
{
    if (!db)
        return;

    free(db->buf);

    memset(db, 0, sizeof(*db));
}

static size_t all_data_cb(const void *ptr, size_t size, size_t nmemb,
                          void *user_data)
{
    struct data_buffer *db = user_data;
    size_t len = size * nmemb;
    size_t oldlen, newlen;
    void *newmem;
    static const unsigned char zero = 0;

    oldlen = db->len;
    newlen = oldlen + len;

	newmem = cgrealloc(db->buf, newlen + 1);
    db->buf = newmem;
    db->len = newlen;
	cg_memcpy(db->buf + oldlen, ptr, len);
	cg_memcpy(db->buf + newlen, &zero, 1);	/* null terminate */

    return len;
}

static size_t upload_data_cb(void *ptr, size_t size, size_t nmemb,
                             void *user_data)
{
    struct upload_buffer *ub = user_data;
    unsigned int len = size * nmemb;

    if (len > ub->len)
        len = ub->len;

	if (len)
	{
		cg_memcpy(ptr, ub->buf, len);
        ub->buf += len;
        ub->len -= len;
    }

    return len;
}

static size_t resp_hdr_cb(void *ptr, size_t size, size_t nmemb, void *user_data)
{
    struct header_info *hi = user_data;
    size_t remlen, slen, ptrlen = size * nmemb;
    char *rem, *val = NULL, *key = NULL;
    void *tmp;

	val = cgcalloc(1, ptrlen);
	key = cgcalloc(1, ptrlen);

    tmp = memchr(ptr, ':', ptrlen);
    if (!tmp || (tmp == ptr))   /* skip empty keys / blanks */
        goto out;
    slen = tmp - ptr;
    if ((slen + 1) == ptrlen)   /* skip key w/ no value */
        goto out;
	cg_memcpy(key, ptr, slen);		/* store & nul term key */
    key[slen] = 0;

    rem = ptr + slen + 1;       /* trim value's leading whitespace */
    remlen = ptrlen - slen - 1;
    while ((remlen > 0) && (isspace(*rem)))
    {
        remlen--;
        rem++;
    }

	cg_memcpy(val, rem, remlen);	/* store value, trim trailing ws */
    val[remlen] = 0;
    while ((*val) && (isspace(val[strlen(val) - 1])))
        val[strlen(val) - 1] = 0;

    if (!*val)          /* skip blank value */
        goto out;

    if (opt_protocol)
        applog(LOG_DEBUG, "HTTP hdr(%s): %s", key, val);

    if (!strcasecmp("X-Roll-Ntime", key))
    {
        hi->hadrolltime = true;
        if (!strncasecmp("N", val, 1))
            applog(LOG_DEBUG, "X-Roll-Ntime: N found");
        else
        {
            hi->canroll = true;

            /* Check to see if expire= is supported and if not, set
             * the rolltime to the default scantime */
            if (strlen(val) > 7 && !strncasecmp("expire=", val, 7))
            {
                sscanf(val + 7, "%d", &hi->rolltime);
                hi->hadexpire = true;
            }
            else
				hi->rolltime = max_scantime;
            applog(LOG_DEBUG, "X-Roll-Ntime expiry set to %d", hi->rolltime);
        }
    }

    if (!strcasecmp("X-Long-Polling", key))
    {
        hi->lp_path = val;  /* steal memory reference */
        val = NULL;
    }

    if (!strcasecmp("X-Reject-Reason", key))
    {
        hi->reason = val;   /* steal memory reference */
        val = NULL;
    }

    if (!strcasecmp("X-Stratum", key))
    {
        hi->stratum_url = val;
        val = NULL;
    }

out:
    free(key);
    free(val);
    return ptrlen;
}

static void last_nettime(struct timeval *last)
{
    rd_lock(&netacc_lock);
    last->tv_sec = nettime.tv_sec;
    last->tv_usec = nettime.tv_usec;
    rd_unlock(&netacc_lock);
}

static void set_nettime(void)
{
    wr_lock(&netacc_lock);
    cgtime(&nettime);
    wr_unlock(&netacc_lock);
}

#if CURL_HAS_KEEPALIVE
static void keep_curlalive(CURL *curl)
{
    const int tcp_keepidle = 45;
    const int tcp_keepintvl = 30;
    const long int keepalive = 1;

    curl_easy_setopt(curl, CURLOPT_TCP_KEEPALIVE, keepalive);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPIDLE, tcp_keepidle);
    curl_easy_setopt(curl, CURLOPT_TCP_KEEPINTVL, tcp_keepintvl);
}
#else
static void keep_curlalive(CURL *curl)
{
    SOCKETTYPE sock;

    curl_easy_getinfo(curl, CURLINFO_LASTSOCKET, (long *)&sock);
    keep_sockalive(sock);
}
#endif

static int curl_debug_cb(__maybe_unused CURL *handle, curl_infotype type,
                         __maybe_unused char *data, size_t size, void *userdata)
{
    struct pool *pool = (struct pool *)userdata;

    switch(type)
    {
        case CURLINFO_HEADER_IN:
        case CURLINFO_DATA_IN:
        case CURLINFO_SSL_DATA_IN:
            pool->cgminer_pool_stats.net_bytes_received += size;
            break;
        case CURLINFO_HEADER_OUT:
        case CURLINFO_DATA_OUT:
        case CURLINFO_SSL_DATA_OUT:
            pool->cgminer_pool_stats.net_bytes_sent += size;
            break;
        case CURLINFO_TEXT:
        default:
            break;
    }
    return 0;
}

json_t *json_web_config(const char *url)
{
    struct data_buffer all_data = {NULL, 0};
    char curl_err_str[CURL_ERROR_SIZE];
    long timeout = 60;
    json_error_t err;
    json_t *val;
    CURL *curl;
    int rc;

    memset(&err, 0, sizeof(err));

    curl = curl_easy_init();
    if (unlikely(!curl))
        quithere(1, "CURL initialisation failed");

    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);

    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_ENCODING, "");
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1);

    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, all_data_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &all_data);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_err_str);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
    curl_easy_setopt(curl, CURLOPT_USE_SSL, CURLUSESSL_TRY);

    val = NULL;
    rc = curl_easy_perform(curl);
    curl_easy_cleanup(curl);
    if (rc)
    {
        applog(LOG_ERR, "HTTP config request of '%s' failed: %s", url, curl_err_str);
        goto c_out;
    }

    if (!all_data.buf)
    {
        applog(LOG_ERR, "Empty config data received from '%s'", url);
        goto c_out;
    }

    val = JSON_LOADS(all_data.buf, &err);
    if (!val)
    {
        applog(LOG_ERR, "JSON config decode of '%s' failed(%d): %s", url,
               err.line, err.text);
    }
    databuf_free(&all_data);

c_out:
    return val;
}

json_t *json_rpc_call(CURL *curl, const char *url,
                      const char *userpass, const char *rpc_req,
                      bool probe, bool longpoll, int *rolltime,
                      struct pool *pool, bool share)
{
    long timeout = longpoll ? (60 * 60) : 60;
    struct data_buffer all_data = {NULL, 0};
    struct header_info hi = {NULL, 0, NULL, NULL, false, false, false};
    char len_hdr[64], user_agent_hdr[128];
    char curl_err_str[CURL_ERROR_SIZE];
    struct curl_slist *headers = NULL;
    struct upload_buffer upload_data;
    json_t *val, *err_val, *res_val;
    bool probing = false;
    double byte_count;
    json_error_t err;
    int rc;

    memset(&err, 0, sizeof(err));

    /* it is assumed that 'curl' is freshly [re]initialized at this pt */

    if (probe)
        probing = !pool->probed;
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, timeout);

    // CURLOPT_VERBOSE won't write to stderr if we use CURLOPT_DEBUGFUNCTION
    curl_easy_setopt(curl, CURLOPT_DEBUGFUNCTION, curl_debug_cb);
    curl_easy_setopt(curl, CURLOPT_DEBUGDATA, (void *)pool);
    curl_easy_setopt(curl, CURLOPT_VERBOSE, 1);

    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1);
    curl_easy_setopt(curl, CURLOPT_URL, url);
    curl_easy_setopt(curl, CURLOPT_ENCODING, "");
    curl_easy_setopt(curl, CURLOPT_FAILONERROR, 1);

    /* Shares are staggered already and delays in submission can be costly
     * so do not delay them */
    if (!opt_delaynet || share)
        curl_easy_setopt(curl, CURLOPT_TCP_NODELAY, 1);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, all_data_cb);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &all_data);
    curl_easy_setopt(curl, CURLOPT_READFUNCTION, upload_data_cb);
    curl_easy_setopt(curl, CURLOPT_READDATA, &upload_data);
    curl_easy_setopt(curl, CURLOPT_ERRORBUFFER, curl_err_str);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1);
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, resp_hdr_cb);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &hi);
    curl_easy_setopt(curl, CURLOPT_USE_SSL, CURLUSESSL_TRY);
    if (pool->rpc_proxy)
    {
        curl_easy_setopt(curl, CURLOPT_PROXY, pool->rpc_proxy);
        curl_easy_setopt(curl, CURLOPT_PROXYTYPE, pool->rpc_proxytype);
    }
    else if (opt_socks_proxy)
    {
        curl_easy_setopt(curl, CURLOPT_PROXY, opt_socks_proxy);
        curl_easy_setopt(curl, CURLOPT_PROXYTYPE, CURLPROXY_SOCKS4);
    }
    if (userpass)
    {
        curl_easy_setopt(curl, CURLOPT_USERPWD, userpass);
        curl_easy_setopt(curl, CURLOPT_HTTPAUTH, CURLAUTH_BASIC);
    }
    if (longpoll)
        keep_curlalive(curl);
    curl_easy_setopt(curl, CURLOPT_POST, 1);

    if (opt_protocol)
        applog(LOG_DEBUG, "JSON protocol request:\n%s", rpc_req);

    upload_data.buf = rpc_req;
    upload_data.len = strlen(rpc_req);
    sprintf(len_hdr, "Content-Length: %lu",
            (unsigned long) upload_data.len);
	sprintf(user_agent_hdr, "User-Agent: %s", PACKAGE_STRING, opt_api_description);

    headers = curl_slist_append(headers,
                                "Content-type: application/json");
    headers = curl_slist_append(headers,
                                "X-Mining-Extensions: longpoll midstate rollntime submitold");

    if (likely(global_hashrate))
    {
        char ghashrate[255];

        sprintf(ghashrate, "X-Mining-Hashrate: %llu", global_hashrate);
        headers = curl_slist_append(headers, ghashrate);
    }

    headers = curl_slist_append(headers, len_hdr);
    headers = curl_slist_append(headers, user_agent_hdr);
    headers = curl_slist_append(headers, "Expect:"); /* disable Expect hdr*/

    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    if (opt_delaynet)
    {
        /* Don't delay share submission, but still track the nettime */
        if (!share)
        {
            long long now_msecs, last_msecs;
            struct timeval now, last;

            cgtime(&now);
            last_nettime(&last);
            now_msecs = (long long)now.tv_sec * 1000;
            now_msecs += now.tv_usec / 1000;
            last_msecs = (long long)last.tv_sec * 1000;
            last_msecs += last.tv_usec / 1000;
            if (now_msecs > last_msecs && now_msecs - last_msecs < 250)
            {
                struct timespec rgtp;

                rgtp.tv_sec = 0;
                rgtp.tv_nsec = (250 - (now_msecs - last_msecs)) * 1000000;
                nanosleep(&rgtp, NULL);
            }
        }
        set_nettime();
    }

    rc = curl_easy_perform(curl);
    if (rc)
    {
        applog(LOG_INFO, "HTTP request failed: %s", curl_err_str);
        goto err_out;
    }

    if (!all_data.buf)
    {
        applog(LOG_DEBUG, "Empty data received in json_rpc_call.");
        goto err_out;
    }

    pool->cgminer_pool_stats.times_sent++;
    if (curl_easy_getinfo(curl, CURLINFO_SIZE_UPLOAD, &byte_count) == CURLE_OK)
        pool->cgminer_pool_stats.bytes_sent += byte_count;
    pool->cgminer_pool_stats.times_received++;
    if (curl_easy_getinfo(curl, CURLINFO_SIZE_DOWNLOAD, &byte_count) == CURLE_OK)
        pool->cgminer_pool_stats.bytes_received += byte_count;

    if (probing)
    {
        pool->probed = true;
        /* If X-Long-Polling was found, activate long polling */
        if (hi.lp_path)
        {
            if (pool->hdr_path != NULL)
                free(pool->hdr_path);
            pool->hdr_path = hi.lp_path;
        }
        else
            pool->hdr_path = NULL;
        if (hi.stratum_url)
        {
            pool->stratum_url = hi.stratum_url;
            hi.stratum_url = NULL;
        }
    }
    else
    {
        if (hi.lp_path)
        {
            free(hi.lp_path);
            hi.lp_path = NULL;
        }
        if (hi.stratum_url)
        {
            free(hi.stratum_url);
            hi.stratum_url = NULL;
        }
    }

    *rolltime = hi.rolltime;
    pool->cgminer_pool_stats.rolltime = hi.rolltime;
    pool->cgminer_pool_stats.hadrolltime = hi.hadrolltime;
    pool->cgminer_pool_stats.canroll = hi.canroll;
    pool->cgminer_pool_stats.hadexpire = hi.hadexpire;

    val = JSON_LOADS(all_data.buf, &err);
    if (!val)
    {
        applog(LOG_INFO, "JSON decode failed(%d): %s", err.line, err.text);

        if (opt_protocol)
            applog(LOG_DEBUG, "JSON protocol response:\n%s", (char *)(all_data.buf));

        goto err_out;
    }

    if (opt_protocol)
    {
        char *s = json_dumps(val, JSON_INDENT(3));

        applog(LOG_DEBUG, "JSON protocol response:\n%s", s);
        free(s);
    }

    /* JSON-RPC valid response returns a non-null 'result',
     * and a null 'error'.
     */
    res_val = json_object_get(val, "result");
    err_val = json_object_get(val, "error");

    if (!res_val ||(err_val && !json_is_null(err_val)))
    {
        char *s;

        if (err_val)
            s = json_dumps(err_val, JSON_INDENT(3));
        else
            s = strdup("(unknown reason)");

        applog(LOG_INFO, "JSON-RPC call failed: %s", s);

        free(s);

        goto err_out;
    }

    if (hi.reason)
    {
        json_object_set_new(val, "reject-reason", json_string(hi.reason));
        free(hi.reason);
        hi.reason = NULL;
    }
    successful_connect = true;
    databuf_free(&all_data);
    curl_slist_free_all(headers);
    curl_easy_reset(curl);
    return val;

err_out:
    databuf_free(&all_data);
    curl_slist_free_all(headers);
    curl_easy_reset(curl);
    if (!successful_connect)
        applog(LOG_DEBUG, "Failed to connect in json_rpc_call");
    curl_easy_setopt(curl, CURLOPT_FRESH_CONNECT, 1);
    return NULL;
}
#define PROXY_HTTP  CURLPROXY_HTTP
#define PROXY_HTTP_1_0  CURLPROXY_HTTP_1_0
#define PROXY_SOCKS4    CURLPROXY_SOCKS4
#define PROXY_SOCKS5    CURLPROXY_SOCKS5
#define PROXY_SOCKS4A   CURLPROXY_SOCKS4A
#define PROXY_SOCKS5H   CURLPROXY_SOCKS5_HOSTNAME
#else /* HAVE_LIBCURL */
#define PROXY_HTTP  0
#define PROXY_HTTP_1_0  1
#define PROXY_SOCKS4    2
#define PROXY_SOCKS5    3
#define PROXY_SOCKS4A   4
#define PROXY_SOCKS5H   5
#endif /* HAVE_LIBCURL */

static struct
{
    const char *name;
    proxytypes_t proxytype;
} proxynames[] =
{
    { "http:",  PROXY_HTTP },
    { "http0:", PROXY_HTTP_1_0 },
    { "socks4:",    PROXY_SOCKS4 },
    { "socks5:",    PROXY_SOCKS5 },
    { "socks4a:",   PROXY_SOCKS4A },
    { "socks5h:",   PROXY_SOCKS5H },
    { NULL, 0 }
};


const char *proxytype(proxytypes_t proxytype)
{
    int i;

    for (i = 0; proxynames[i].name; i++) {
        if (proxynames[i].proxytype == proxytype) {
            return proxynames[i].name;
        }
    }

    return "invalid";
}


char *get_proxy(char *url, struct pool *pool)
{
    pool->rpc_proxy = NULL;

    char *split;
    int plen, len, i;

    for (i = 0; proxynames[i].name; i++)
    {
        plen = strlen(proxynames[i].name);
        if (strncmp(url, proxynames[i].name, plen) == 0)
        {
            if (!(split = strchr(url, '|')))
                return url;

            *split = '\0';
            len = (int) (split - url);
            pool->rpc_proxy = cgmalloc((size_t) 1 + len - plen);
            strcpy(pool->rpc_proxy, url + plen);
            extract_sockaddr(pool->rpc_proxy, &pool->sockaddr_proxy_url, &pool->sockaddr_proxy_port);
            pool->rpc_proxytype = proxynames[i].proxytype;
            url = split + 1;
            break;
        }
    }
    return url;
}

/* Adequate size s==len*2 + 1 must be alloced to use this variant */
void __bin2hex(char *s, const unsigned char *p, size_t len)
{
    int i;
    static const char hex[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

    for (i = 0; i < (int)(len / 1); i++)
    {
        *s++ = hex[p[i] >> 4];
        *s++ = hex[p[i] & 0xF];
    }
    *s++ = '\0';
}

/* Returns a malloced array string of a binary value of arbitrary length. The
 * array is rounded up to a 4 byte size to appease architectures that need
 * aligned array  sizes */
char *bin2hex(const unsigned char *p, size_t len)
{
    ssize_t slen;
    char *s;

    slen = len * 2 + 1;

    if (slen % 4) {
        slen += 4 - (slen % 4);
    }

    s = cgcalloc(slen, (size_t)1);
    __bin2hex(s, p, len);

    return s;
}

static const int hex2bin_tbl[256] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    0,  1,  2,  3,  4,  5,  6,  7,  8,  9, -1, -1, -1, -1, -1, -1,
    -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, 10, 11, 12, 13, 14, 15, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
};


/* Does the reverse of bin2hex but does not allocate any ram */
bool hex2bin(unsigned char *p, const char *hexstr, size_t len)
{
    int nibble1, nibble2;
    unsigned char idx;
    bool ret = false;

    while (*hexstr && len)
    {
        if (unlikely(!hexstr[1]))
        {
            applog(LOG_ERR, "hex2bin str truncated");
            return ret;
        }

        idx = (unsigned char) *hexstr++;
        nibble1 = hex2bin_tbl[idx];
        idx = (unsigned char) *hexstr++;
        nibble2 = hex2bin_tbl[idx];

        if (unlikely((nibble1 < 0) || (nibble2 < 0)))
        {
            applog(LOG_ERR, "hex2bin scan failed");
            return ret;
        }

        *p++ = (((unsigned char)nibble1) << 4) | ((unsigned char)nibble2);
        --len;
    }

    if (likely(len == 0 && *hexstr == 0))
        ret = true;
    return ret;
}

static bool _valid_hex(char *s, const char *file, const char *func, const int line)
{
    bool ret = false;
    int i, len;

    if (unlikely(!s))
    {
        applog(LOG_ERR, "Null string passed to valid_hex from", IN_FMT_FFL, file, func, line);
        return ret;
    }
    len = strlen(s);

    for (i = 0; i < len; i++)
    {
        unsigned char idx = (unsigned char) s[i];

        if (unlikely(hex2bin_tbl[idx] < 0))
        {
            applog(LOG_ERR, "Invalid char 0x%x passed to valid_hex from", IN_FMT_FFL, idx, file, func, line);
            return ret;
        }
    }

    ret = true;
    return ret;
}

#define valid_hex(s) _valid_hex(s, __FILE__, __func__, __LINE__)

static bool _valid_ascii(char *s, const char *file, const char *func, const int line)
{
    bool ret = false;
    int i, len;

    if (unlikely(!s))
    {
        applog(LOG_ERR, "Null string passed to valid_ascii from", IN_FMT_FFL, file, func, line);
        return ret;
    }

    len = strlen(s);

    if (unlikely(!len))
    {
        applog(LOG_ERR, "Zero length string passed to valid_ascii from", IN_FMT_FFL, file, func, line);
        return ret;
    }

    for (i = 0; i < len; i++)
    {
        unsigned char idx = (unsigned char) s[i];

        if (unlikely(idx < 32 || idx > 126))
        {
            applog(LOG_ERR, "Invalid char 0x%x passed to valid_ascii from", IN_FMT_FFL, idx, file, func, line);
            return ret;
        }
    }
    ret = true;
    return ret;
}

#define valid_ascii(s) _valid_ascii(s, __FILE__, __func__, __LINE__)

static const int b58tobin_tbl[] =
{
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1,  0,  1,  2,  3,  4,  5,  6,  7,  8, -1, -1, -1, -1, -1, -1,
    -1,  9, 10, 11, 12, 13, 14, 15, 16, -1, 17, 18, 19, 20, 21, -1,
    22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, -1, -1, -1, -1, -1,
    -1, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, -1, 44, 45, 46,
    47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57
};

/* b58bin should always be at least 25 bytes long and already checked to be
 * valid. */
void b58tobin(unsigned char *b58bin, const char *b58)
{
    uint32_t c, bin32[7];
    int len, i, j;
    uint64_t t;

    memset(bin32, 0, 7 * sizeof(uint32_t));
    len = strlen(b58);

    for (i = 0; i < len; i++) {
        c = (uint32_t) b58[i];
        c = (uint32_t) b58tobin_tbl[c];

        for (j = 6; j >= 0; j--)
        {
            t = ((uint64_t)bin32[j]) * 58 + c;
            c = (t & 0x3f00000000ull) >> 32;
            bin32[j] = t & 0xffffffffull;
        }
    }
    *(b58bin++) = (unsigned char) (bin32[0] & 0xff);
    for (i = 1; i < 7; i++)
    {
        *((uint32_t *)b58bin) = htobe32(bin32[i]);
        b58bin += sizeof(uint32_t);
    }
}

void address_to_pubkeyhash(unsigned char *pkh, const char *addr)
{
    unsigned char b58bin[25];

    memset(b58bin, 0, 25);
    b58tobin(b58bin, addr);
    pkh[0] = 0x76;
    pkh[1] = 0xa9;
    pkh[2] = 0x14;
    cg_memcpy(&pkh[3], &b58bin[1], 20);
    pkh[23] = 0x88;
    pkh[24] = 0xac;
}

/*  For encoding nHeight into coinbase, return how many bytes were used */
int ser_number(unsigned char *s, int32_t val)
{
    int32_t *i32 = (int32_t *)&s[1];
    int len;

    if (val < 128)
        len = 1;
    else if (val < 16512)
        len = 2;
    else if (val < 2113664)
        len = 3;
    else
        len = 4;
    *i32 = htole32(val);
    s[0] = (unsigned char) len++;
    return len;
}

/* For encoding variable length strings */
unsigned char *ser_string(char *s, int *slen)
{
    size_t len = strlen(s);
    unsigned char *ret;

    ret = cgmalloc(1 + len + 8); // Leave room for largest size
    if (len < 253)
    {
        ret[0] = len;
        cg_memcpy(ret + 1, s, (unsigned int)(len/1));
        *slen = len + 1;
    }
    else if (len < 0x10000)
    {
        uint16_t *u16 = (uint16_t *) &ret[1];

        ret[0] = 253;
        *u16 = htobe16(len);
        cg_memcpy(ret + 3, s, (unsigned int)(len/1));
        *slen = len + 3;
    }
    else
    {
        /* size_t is only 32 bit on many platforms anyway */
        uint32_t *u32 = (uint32_t *)&ret[1];

        ret[0] = 254;
        *u32 = htobe32(len);
        cg_memcpy(ret + 5, s, (unsigned int)(len/1));
        *slen = len + 5;
    }
    return ret;
}

bool fulltest(const unsigned char *hash, const unsigned char *target)
{
    uint32_t *hash32 = (uint32_t *)hash;
    uint32_t *target32 = (uint32_t *)target;
    bool rc = true;
    int i;

    for (i = 28 / 4; i >= 0; i--)
    {
        uint32_t h32tmp = le32toh(hash32[i]);
        uint32_t t32tmp = le32toh(target32[i]);

        if (h32tmp > t32tmp)
        {
            rc = false;
            break;
        }

        if (h32tmp < t32tmp)
        {
            rc = true;
            break;
        }
    }

    if (opt_debug)
    {
        unsigned char hash_swap[32], target_swap[32];
        char *hash_str, *target_str;

        swab256(hash_swap, hash);
        swab256(target_swap, target);
        hash_str = bin2hex(hash_swap, (size_t)32);
        target_str = bin2hex(target_swap, (size_t)32);

        applog(LOG_DEBUG, " Proof: %s\nTarget: %s\nTrgVal? %s",
               hash_str,
               target_str,
               rc ? "YES (hash <= target)" :
               "no (false positive; hash > target)");

        free(hash_str);
        free(target_str);
    }

    return rc;
}

struct thread_q *tq_new(void)
{
    struct thread_q *tq;

    tq = cgcalloc(1, sizeof(*tq));
    INIT_LIST_HEAD(&tq->q);
    pthread_mutex_init(&tq->mutex, NULL);
    pthread_cond_init(&tq->cond, NULL);

    return tq;
}

void tq_free(struct thread_q *tq)
{
    struct tq_ent *ent, *iter;

    if (!tq) {
        return;
    }

    list_for_each_entry_safe(ent, iter, &tq->q, q_node)
    {
        list_del(&ent->q_node);
        free(ent);
    }

    pthread_cond_destroy(&tq->cond);
    pthread_mutex_destroy(&tq->mutex);

    memset(tq, 0, sizeof(*tq)); /* poison */
    free(tq);
}

static void tq_freezethaw(struct thread_q *tq, bool frozen)
{
    mutex_lock(&tq->mutex);
    tq->frozen = frozen;
    pthread_cond_signal(&tq->cond);
    mutex_unlock(&tq->mutex);
}

void tq_freeze(struct thread_q *tq)
{
    tq_freezethaw(tq, true);
}

void tq_thaw(struct thread_q *tq)
{
    tq_freezethaw(tq, false);
}

bool tq_push(struct thread_q *tq, void *data)
{
    struct tq_ent *ent;
    bool rc = true;

    ent = cgcalloc(1, sizeof(*ent));
    ent->data = data;
    INIT_LIST_HEAD(&ent->q_node);

    mutex_lock(&tq->mutex);

    if (!tq->frozen)
    {
        list_add_tail(&ent->q_node, &tq->q);
    }
    else
    {
        free(ent);
        rc = false;
    }

    pthread_cond_signal(&tq->cond);
    mutex_unlock(&tq->mutex);

    return rc;
}


void *tq_pop(struct thread_q *tq, const struct timespec *abstime)
{
    struct tq_ent *ent;
    void *rval = NULL;
    int rc;

    mutex_lock(&tq->mutex);
    if (!list_empty(&tq->q)) {
        goto pop;
    }

    if (abstime) {
        rc = pthread_cond_timedwait(&tq->cond, &tq->mutex, abstime);
    }
    else {
        rc = pthread_cond_wait(&tq->cond, &tq->mutex);
    }

    if (rc) {
        goto out;
    }

    if (list_empty(&tq->q)) {
        goto out;
    }
pop:
    ent = list_entry(tq->q.next, struct tq_ent, q_node);
    rval = ent->data;

    list_del(&ent->q_node);
    free(ent);
out:
    mutex_unlock(&tq->mutex);

    return rval;
}

int thr_info_create(struct thr_info *thr, pthread_attr_t *attr, void *(*start) (void *), void *arg)
{
    cgsem_init(&thr->sem);
    return pthread_create(&thr->pth, attr, start, arg);
}

void thr_info_cancel(struct thr_info *thr)
{
    if (!thr) {
        return;
    }

    if (PTH(thr) != 0L)
    {
        pthread_cancel(thr->pth);
        PTH(thr) = 0L;
    }

    cgsem_destroy(&thr->sem);
}

void subtime(struct timeval *a, struct timeval *b)
{
    timersub(a, b, b);
}


void addtime(struct timeval *a, struct timeval *b)
{
    timeradd(a, b, b);
}


bool time_more(struct timeval *a, struct timeval *b)
{
    return timercmp(a, b, >);
}


bool time_less(struct timeval *a, struct timeval *b)
{
    return timercmp(a, b, <);
}


void copy_time(struct timeval *dest, const struct timeval *src)
{
    cg_memcpy(dest, src, sizeof(struct timeval));
}


void timespec_to_val(struct timeval *val, const struct timespec *spec)
{
    val->tv_sec = spec->tv_sec;
    val->tv_usec = spec->tv_nsec / 1000;
}


void timeval_to_spec(struct timespec *spec, const struct timeval *val)
{
    spec->tv_sec = val->tv_sec;
    spec->tv_nsec = val->tv_usec * 1000;
}


void us_to_timeval(struct timeval *val, int64_t us)
{
    lldiv_t tvdiv = lldiv(us, 1000000);

    val->tv_sec = tvdiv.quot;
    val->tv_usec = tvdiv.rem;
}


void us_to_timespec(struct timespec *spec, int64_t us)
{
    lldiv_t tvdiv = lldiv(us, 1000000);

    spec->tv_sec = tvdiv.quot;
    spec->tv_nsec = tvdiv.rem * 1000;
}


void ms_to_timespec(struct timespec *spec, int64_t ms)
{
    lldiv_t tvdiv = lldiv(ms, 1000);

    spec->tv_sec = tvdiv.quot;
    spec->tv_nsec = tvdiv.rem * 1000000;
}


void ms_to_timeval(struct timeval *val, int64_t ms)
{
    lldiv_t tvdiv = lldiv(ms, 1000);

    val->tv_sec = tvdiv.quot;
    val->tv_usec = tvdiv.rem * 1000;
}


static void spec_nscheck(struct timespec *ts)
{
    while (ts->tv_nsec >= 1000000000)
    {
        ts->tv_nsec -= 1000000000;
        ts->tv_sec++;
    }

    while (ts->tv_nsec < 0)
    {
        ts->tv_nsec += 1000000000;
        ts->tv_sec--;
    }
}

void timeraddspec(struct timespec *a, const struct timespec *b)
{
    a->tv_sec += b->tv_sec;
    a->tv_nsec += b->tv_nsec;
    spec_nscheck(a);
}

static int __maybe_unused timespec_to_ms(struct timespec *ts)
{
    return ts->tv_sec * 1000 + ts->tv_nsec / 1000000;
}


/* Subtract b from a */
static void __maybe_unused timersubspec(struct timespec *a, const struct timespec *b)
{
    a->tv_sec -= b->tv_sec;
    a->tv_nsec -= b->tv_nsec;
    spec_nscheck(a);
}

char *Strcasestr(char *haystack, const char *needle)
{
    char *lowhay, *lowneedle, *ret;
    int hlen, nlen, i, ofs;

    if (unlikely(!haystack || !needle)) {
        return NULL;
    }

    hlen = strlen(haystack);
    nlen = strlen(needle);

    if (!hlen || !nlen) {
        return NULL;
    }

    lowhay = alloca((size_t) hlen);
    lowneedle = alloca((size_t) nlen);

    for (i = 0; i < hlen; i++) {
        lowhay[i] = tolower(haystack[i]);
    }

    for (i = 0; i < nlen; i++) {
        lowneedle[i] = tolower(needle[i]);
    }

    ret = strstr(lowhay, lowneedle);

    if (!ret) {
        return ret;
    }

    ofs = (int) (ret - lowhay);
    return haystack + ofs;
}

char *Strsep(char **stringp, const char *delim)
{
    char *ret = *stringp;
    char *p;

    p = (ret != NULL) ? strpbrk(ret, delim) : NULL;

    if (p == NULL) {
        *stringp = NULL;
    }
    else
    {
        *p = '\0';
        *stringp = p + 1;
    }

    return ret;
}


void cgtime(struct timeval *tv)
{
    gettimeofday(tv, NULL);
}


int cgtimer_to_ms(cgtimer_t *cgt)
{
    return timespec_to_ms(cgt);
}


/* Subtracts b from a and stores it in res. */
void cgtimer_sub(cgtimer_t *a, cgtimer_t *b, cgtimer_t *res)
{
    res->tv_sec = a->tv_sec - b->tv_sec;
    res->tv_nsec = a->tv_nsec - b->tv_nsec;
    if (res->tv_nsec < 0)
    {
        res->tv_nsec += 1000000000;
        res->tv_sec--;
    }
}


#if defined(CLOCK_MONOTONIC) && !defined(__FreeBSD__) /* Essentially just linux */
//#ifdef CLOCK_MONOTONIC /* Essentially just linux */
void cgtimer_time(cgtimer_t *ts_start)
{
    clock_gettime(CLOCK_MONOTONIC, ts_start);
}

static void nanosleep_abstime(struct timespec *ts_end)
{
    int ret;

    do
    {
        ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ts_end, NULL);
    }
    while (ret == EINTR);
}

/* Reentrant version of cgsleep functions allow start time to be set separately
 * from the beginning of the actual sleep, allowing scheduling delays to be
 * counted in the sleep. */
void cgsleep_ms_r(cgtimer_t *ts_start, int ms)
{
    struct timespec ts_end;

    ms_to_timespec(&ts_end, ms);
    timeraddspec(&ts_end, ts_start);
    nanosleep_abstime(&ts_end);
}

void cgsleep_us_r(cgtimer_t *ts_start, int64_t us)
{
    struct timespec ts_end;

    us_to_timespec(&ts_end, us);
    timeraddspec(&ts_end, ts_start);
    nanosleep_abstime(&ts_end);
}
#else /* CLOCK_MONOTONIC */
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
void cgtimer_time(cgtimer_t *ts_start)
{
    clock_serv_t cclock;
    mach_timespec_t mts;

    host_get_clock_service(mach_host_self(), SYSTEM_CLOCK, &cclock);
    clock_get_time(cclock, &mts);
    mach_port_deallocate(mach_task_self(), cclock);
    ts_start->tv_sec = mts.tv_sec;
    ts_start->tv_nsec = mts.tv_nsec;
}
#elif !defined(WIN32) /* __MACH__ - Everything not linux/macosx/win32 */
void cgtimer_time(cgtimer_t *ts_start)
{
    struct timeval tv;

    cgtime(&tv);
    ts_start->tv_sec = tv.tv_sec;
    ts_start->tv_nsec = tv.tv_usec * 1000;
}
#endif /* __MACH__ */


static void cgsleep_spec(struct timespec *ts_diff, const struct timespec *ts_start)
{
    struct timespec now;

    timeraddspec(ts_diff, ts_start);
    cgtimer_time(&now);
    timersubspec(ts_diff, &now);
    if (unlikely(ts_diff->tv_sec < 0))
        return;
    nanosleep(ts_diff, NULL);
}

void cgsleep_ms_r(cgtimer_t *ts_start, int ms)
{
    struct timespec ts_diff;

    ms_to_timespec(&ts_diff, (int64_t) ms);
    cgsleep_spec(&ts_diff, ts_start);
}

void cgsleep_us_r(cgtimer_t *ts_start, int64_t us)
{
    struct timespec ts_diff;

    us_to_timespec(&ts_diff, us);
    cgsleep_spec(&ts_diff, ts_start);
}

#endif /* CLOCK_MONOTONIC */

void cgsleep_ms(int ms)
{
    cgtimer_t ts_start;

    cgsleep_prepare_r(&ts_start);
    cgsleep_ms_r(&ts_start, ms);
}

void cgsleep_us(int64_t us)
{
    cgtimer_t ts_start;

    cgsleep_prepare_r(&ts_start);
    cgsleep_us_r(&ts_start, us);
}

/* Returns the microseconds difference between end and start times as a double */
double us_tdiff(struct timeval *end, struct timeval *start)
{
    /* Sanity check. We should only be using this for small differences so
     * limit the max to 60 seconds. */
    if (unlikely(end->tv_sec - start->tv_sec > 60))
        return 60000000;
    return (end->tv_sec - start->tv_sec) * 1000000 + (end->tv_usec - start->tv_usec);
}

/* Returns the milliseconds difference between end and start times */
int ms_tdiff(struct timeval *end, struct timeval *start)
{
    /* Like us_tdiff, limit to 1 hour. */
    if (unlikely(end->tv_sec - start->tv_sec > 3600))
        return 3600000;
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_usec - start->tv_usec) / 1000;
}

/* Returns the seconds difference between end and start times as a double */
double tdiff(struct timeval *end, struct timeval *start)
{
    return end->tv_sec - start->tv_sec + (end->tv_usec - start->tv_usec) / 1000000.0;
}


bool extract_sockaddr(char *url, char **sockaddr_url, char **sockaddr_port)
{
    char *url_begin, *url_end, *ipv6_begin, *ipv6_end, *port_start = NULL;
    char url_address[256], port[6];
    int url_len, port_len = 0;

    *sockaddr_url = url;
    url_begin = strstr(url, "//");
    if (!url_begin) {
        url_begin = url;
    }
    else {
        url_begin += 2;
    }

    /* Look for numeric ipv6 entries */
    ipv6_begin = strstr(url_begin, "[");
    ipv6_end = strstr(url_begin, "]");

    if (ipv6_begin && ipv6_end && ipv6_end > ipv6_begin) {
        url_end = strstr(ipv6_end, ":");
    }
    else {
        url_end = strstr(url_begin, ":");
    }

    if (url_end)
    {
        url_len = (int) (url_end - url_begin);
        port_len = strlen(url_begin) - url_len - 1;

        if (port_len < 1) {
            return false;
        }

        port_start = url_end + 1;
    }
    else
        url_len = strlen(url_begin);

    if (url_len < 1) {
        return false;
    }

    /* Get rid of the [] */
    if (ipv6_begin && ipv6_end && ipv6_end > ipv6_begin)
    {
        url_len -= 2;
        url_begin++;
    }

    snprintf(url_address, 254, "%.*s", url_len, url_begin);

    if (port_len)
    {
        char *slash;

        snprintf(port, 6, "%.*s", port_len, port_start);
        slash = strpbrk(port, "/#");
        if (slash)
            *slash = '\0';
    }
    else {
        strcpy(port, "80");
    }

    *sockaddr_port = strdup(port);
    *sockaddr_url = strdup(url_address);

    return true;
}

enum send_ret
{
    SEND_OK,
    SEND_SELECTFAIL,
    SEND_SENDFAIL,
    SEND_INACTIVE
};

/* Send a single command across a socket, appending \n to it. This should all
 * be done under stratum lock except when first establishing the socket */
static enum send_ret __stratum_send(struct pool *pool, char *s, ssize_t len)
{
    SOCKETTYPE sock = pool->sock;
    ssize_t ssent = 0;

    if (opt_protocol) {
        applog(LOG_DEBUG, "SEND: %s", s);
    }

    strcat(s, "\n");
    len++;

    while (len > 0 )
    {
        struct timeval timeout = {1, 0};
        ssize_t sent;
        fd_set wd;
    retry:
        FD_ZERO(&wd);
        FD_SET(sock, &wd);

        if (select(sock + 1, NULL, &wd, NULL, &timeout) < 1)
        {
            if (interrupted()) {
                goto retry;
            }

            return SEND_SELECTFAIL;
        }

#ifdef __APPLE__
        sent = send(pool->sock, s + ssent, len, SO_NOSIGPIPE);
#elif WIN32
        sent = send(pool->sock, s + ssent, len, 0);
#else
        sent = send(pool->sock, s + ssent, len, MSG_NOSIGNAL);
#endif
        if (sent < 0)
        {
            if (!sock_blocks())
                return SEND_SENDFAIL;
            sent = 0;
        }
        ssent += sent;
        len -= sent;
    }

    pool->cgminer_pool_stats.times_sent++;
    pool->cgminer_pool_stats.bytes_sent += ssent;
    pool->cgminer_pool_stats.net_bytes_sent += ssent;
    return SEND_OK;
}

bool stratum_send(struct pool *pool, char *s, ssize_t len)
{
    enum send_ret ret = SEND_INACTIVE;

    if (opt_protocol) {
        applog(LOG_DEBUG, "SEND: %s", s);
    }

    mutex_lock(&pool->stratum_lock);

    if (pool->stratum_active) {
        ret = __stratum_send(pool, s, len);
    }

    mutex_unlock(&pool->stratum_lock);

    /* This is to avoid doing applog under stratum_lock */
    switch (ret)
    {
        default:
        case SEND_OK:
            break;

        case SEND_SELECTFAIL:
            applog(LOG_DEBUG, "Write select failed on pool %d sock", pool->pool_no);
            suspend_stratum(pool);
            break;

        case SEND_SENDFAIL:
            applog(LOG_DEBUG, "Failed to send in stratum_send");
            suspend_stratum(pool);
            break;

        case SEND_INACTIVE:
            applog(LOG_DEBUG, "Stratum send failed due to no pool stratum_active");
            break;
    }

    return (ret == SEND_OK);
}


static bool socket_full(struct pool *pool, int wait)
{
    SOCKETTYPE sock = pool->sock;
    struct timeval timeout;
    fd_set rd;

    if (unlikely(wait < 0)) {
        wait = 0;
    }

    FD_ZERO(&rd);
    FD_SET(sock, &rd);
    timeout.tv_usec = 0;
    timeout.tv_sec = wait;

    if (select(sock + 1, &rd, NULL, NULL, &timeout) > 0) {
        return true;
    }

    return false;
}

/* Check to see if Santa's been good to you */
bool sock_full(struct pool *pool)
{
    if (strlen(pool->sockbuf)) {
        return true;
    }

    return (socket_full(pool, 0));
}

static void clear_sockbuf(struct pool *pool)
{
    if (likely(pool->sockbuf)) {
        strcpy(pool->sockbuf, "");
    }
}

static void clear_sock(struct pool *pool)
{
    ssize_t n;

    mutex_lock(&pool->stratum_lock);
    do
    {
        if (pool->sock)
            n = recv(pool->sock, pool->sockbuf, RECVSIZE, 0);
        else
            n = 0;
    }
    while (n > 0);
    mutex_unlock(&pool->stratum_lock);

    clear_sockbuf(pool);
}

/* Realloc memory to new size and zero any extra memory added */
void _recalloc(void **ptr, size_t old, size_t news, const char *file, const char *func, const int line)
{
    if (news == old)
        return;
    *ptr = _cgrealloc(*ptr, news, file, func, line);
    if (news > old)
        memset(*ptr + old, 0, news - old);
}

/* Make sure the pool sockbuf is large enough to cope with any coinbase size
 * by reallocing it to a large enough size rounded up to a multiple of RBUFSIZE
 * and zeroing the new memory */
static void recalloc_sock(struct pool *pool, size_t len)
{
    size_t old, news;

    old = strlen(pool->sockbuf);
    news = old + len + 1;
    if (news < pool->sockbuf_size)
        return;
    news = news + (RBUFSIZE - (news % RBUFSIZE));
    // Avoid potentially recursive locking
    // applog(LOG_DEBUG, "Recallocing pool sockbuf to %d", new);
    pool->sockbuf = cgrealloc(pool->sockbuf, news);
    memset(pool->sockbuf + old, 0, news - old);
    pool->sockbuf_size = news;
}

/* Peeks at a socket to find the first end of line and then reads just that
 * from the socket and returns that as a malloced char */
char *recv_line(struct pool *pool)
{
    char *tok, *sret = NULL;
    ssize_t len, buflen;
    int waited = 0;

    if (!strstr(pool->sockbuf, "\n"))
    {
        struct timeval rstart, now;

        cgtime(&rstart);
        if (!socket_full(pool, DEFAULT_SOCKWAIT))
        {
            applog(LOG_DEBUG, "Timed out waiting for data on socket_full");
            goto out;
        }

        do
        {
            char s[RBUFSIZE];
            size_t slen;
            ssize_t n;

            memset(s, 0, RBUFSIZE);
            n = recv(pool->sock, s, RECVSIZE, 0);
            if (!n)
            {
                applog(LOG_DEBUG, "Socket closed waiting in recv_line");
                suspend_stratum(pool);
                break;
            }
            cgtime(&now);
            waited = (int) tdiff(&now, &rstart);
            if (n < 0)
            {
                if (!sock_blocks() || !socket_full(pool, DEFAULT_SOCKWAIT - waited))
                {
                    applog(LOG_DEBUG, "Failed to recv sock in recv_line");
                    suspend_stratum(pool);
                    break;
                }
            }
            else
            {
                slen = strlen(s);
                recalloc_sock(pool, slen);
                strcat(pool->sockbuf, s);
            }
        }
        while (waited < DEFAULT_SOCKWAIT && !strstr(pool->sockbuf, "\n"));
    }

    buflen = strlen(pool->sockbuf);
    tok = strtok(pool->sockbuf, "\n");
    if (!tok)
    {
        applog(LOG_DEBUG, "Failed to parse a \\n terminated string in recv_line");
        goto out;
    }
    sret = strdup(tok);
    len = strlen(sret);

    /* Copy what's left in the buffer after the \n, including the
     * terminating \0 */
    if (buflen > len + 1)
        memmove(pool->sockbuf, pool->sockbuf + len + 1, buflen - len + 1);
    else
        strcpy(pool->sockbuf, "");

    pool->cgminer_pool_stats.times_received++;
    pool->cgminer_pool_stats.bytes_received += len;
    pool->cgminer_pool_stats.net_bytes_received += len;
out:
    if (!sret)
        clear_sock(pool);
    else if (opt_protocol)
        applog(LOG_DEBUG, "RECVD: %s", sret);
    return sret;
}

/* Extracts a string value from a json array with error checking. To be used
 * when the value of the string returned is only examined and not to be stored.
 * See json_array_string below */
static char *__json_array_string(json_t *val, unsigned int entry)
{
    json_t *arr_entry;

    if (json_is_null(val))
        return NULL;
    if (!json_is_array(val))
        return NULL;
    if (entry > json_array_size(val))
        return NULL;
    arr_entry = json_array_get(val, entry);
    if (!json_is_string(arr_entry))
        return NULL;

    return (char *)json_string_value(arr_entry);
}

/* Creates a freshly malloced dup of __json_array_string */
static char *json_array_string(json_t *val, unsigned int entry)
{
    char *buf = __json_array_string(val, entry);

    if (buf)
        return strdup(buf);
    return NULL;
}

static char *blank_merkle = "0000000000000000000000000000000000000000000000000000000000000000";

#define VERSION_BITS_NUM 2
#define VERSION_BITS_THAT_S9_ROLLS 0x00c00000
#if MIDSTATE_NUM != (1UL << VERSION_BITS_NUM)
#error Incompatible number of midstates and version bits to roll!
#endif

struct block_version n_version[MIDSTATE_NUM];

static bool parse_notify(struct pool *pool, json_t *val)
{
    char *job_id, *prev_hash, *coinbase1, *coinbase2, *bbversion, *nbit,
         *ntime, header[228];
    unsigned char *cb1 = NULL, *cb2 = NULL;
    size_t cb1_len, cb2_len, alloc_len;
    bool clean, ret = false;
    int merkles, i;
    json_t *arr;

    arr = json_array_get(val, 4);
    if (!arr || !json_is_array(arr))
        goto out;

    merkles = json_array_size(arr);

    job_id = json_array_string(val, 0);
    prev_hash = __json_array_string(val, 1);
    coinbase1 = json_array_string(val, 2);
    coinbase2 = json_array_string(val, 3);
    bbversion = __json_array_string(val, 5);
    nbit = __json_array_string(val, 6);
    ntime = __json_array_string(val, 7);
    clean = json_is_true(json_array_get(val, 8));

    if (!valid_ascii(job_id) || !valid_hex(prev_hash) || !valid_hex(coinbase1) ||
        !valid_hex(coinbase2) || !valid_hex(bbversion) || !valid_hex(nbit) ||
        !valid_hex(ntime))
    {
        /* Annoying but we must not leak memory */
        free(job_id);
        free(coinbase1);
        free(coinbase2);
        goto out;
    }

    cg_wlock(&pool->data_lock);
    free(pool->swork.job_id);
    pool->swork.job_id = job_id;
    snprintf(pool->prev_hash, 65, "%s", prev_hash);
    cb1_len = strlen(coinbase1) / 2;
    cb2_len = strlen(coinbase2) / 2;
    snprintf(pool->bbversion, 9, "%s", bbversion);
    snprintf(pool->nbit, 9, "%s", nbit);
    snprintf(pool->ntime, 9, "%s", ntime);
    pool->swork.clean = clean;
    if (pool->next_diff > 0) {
        pool->sdiff = pool->next_diff;
    }
    alloc_len = pool->coinbase_len = cb1_len + pool->n1_len + pool->n2size + cb2_len;
    pool->nonce2_offset = cb1_len + pool->n1_len;

    for (i = 0; i < pool->merkles; i++)
        free(pool->swork.merkle_bin[i]);
    if (merkles)
    {
        pool->swork.merkle_bin = cgrealloc(pool->swork.merkle_bin,
                                         sizeof(char *) * merkles + 1);
        for (i = 0; i < merkles; i++)
        {
            char *merkle = json_array_string(arr, (unsigned int) i);

            pool->swork.merkle_bin[i] = cgmalloc((size_t)32);
            if (opt_protocol)
                applog(LOG_DEBUG, "merkle %d: %s", i, merkle);
            ret = hex2bin(pool->swork.merkle_bin[i], merkle, (size_t)32);
            free(merkle);
            if (unlikely(!ret))
            {
                applog(LOG_ERR, "Failed to convert merkle to merkle_bin in parse_notify");
                goto out_unlock;
            }
        }
    }
    pool->merkles = merkles;
    if (pool->merkles < 2)
        pool->bad_work++;
    if (clean)
        pool->nonce2 = 0;
#if 0
    header_len =         strlen(pool->bbversion) +
                         strlen(pool->prev_hash);
    /* merkle_hash */    32 +
    strlen(pool->ntime) +
    strlen(pool->nbit) +
    /* nonce */      8 +
    /* workpadding */    96;
#endif
    snprintf(header, 225,
             "%s%s%s%s%s%s%s",
             pool->bbversion,
             pool->prev_hash,
             blank_merkle,
             pool->ntime,
             pool->nbit,
             "00000000", /* nonce */
             workpadding);
    ret = hex2bin(pool->header_bin, header, (size_t)112);
    if (unlikely(!ret))
    {
        applog(LOG_ERR, "Failed to convert header to header_bin in parse_notify");
        goto out_unlock;
    }

    cb1 = alloca(cb1_len);
    ret = hex2bin(cb1, coinbase1, cb1_len);
    if (unlikely(!ret))
    {
        applog(LOG_ERR, "Failed to convert cb1 to cb1_bin in parse_notify");
        goto out_unlock;
    }
    cb2 = alloca(cb2_len);
    ret = hex2bin(cb2, coinbase2, cb2_len);
    if (unlikely(!ret))
    {
        applog(LOG_ERR, "Failed to convert cb2 to cb2_bin in parse_notify");
        goto out_unlock;
    }
    free(pool->coinbase);
    pool->coinbase = cgcalloc(alloc_len, (size_t)1);
    cg_memcpy(pool->coinbase, cb1, (size_t)cb1_len);
    if (pool->n1_len)
        cg_memcpy(pool->coinbase + cb1_len, pool->nonce1bin, (size_t)pool->n1_len);
    cg_memcpy(pool->coinbase + cb1_len + pool->n1_len + pool->n2size, cb2, (size_t)cb2_len);
    if (opt_debug)
    {
        char *cb = bin2hex(pool->coinbase, (size_t)pool->coinbase_len);

        applog(LOG_DEBUG, "Pool %d coinbase %s", pool->pool_no, cb);
        free(cb);
    }
out_unlock:
    cg_wunlock(&pool->data_lock);

    if (opt_protocol)
    {
        applog(LOG_DEBUG, "job_id: %s", job_id);
        applog(LOG_DEBUG, "prev_hash: %s", prev_hash);
        applog(LOG_DEBUG, "coinbase1: %s", coinbase1);
        applog(LOG_DEBUG, "coinbase2: %s", coinbase2);
        applog(LOG_DEBUG, "bbversion: %s", bbversion);
        applog(LOG_DEBUG, "nbit: %s", nbit);
        applog(LOG_DEBUG, "ntime: %s", ntime);
        applog(LOG_DEBUG, "clean: %s", clean ? "yes" : "no");
    }
    free(coinbase1);
    free(coinbase2);

    /* A notify message is the closest stratum gets to a getwork */
    pool->getwork_requested++;
    total_getworks++;
    if (pool == current_pool())
        opt_work_update = true;
out:
    return ret;
}

static bool parse_version(struct pool *pool, json_t *val)
{
    int i;
    for(i = 0; i < json_array_size(val); i++)
    {
        pool->version[i] = json_integer_value(json_array_get(val, i));
    }
}

static bool parse_diff(struct pool *pool, json_t *val)
{
    double old_diff, diff;

    diff = json_number_value(json_array_get(val, 0));
    if (diff == 0)
        return false;

    cg_wlock(&pool->data_lock);

    if (pool->next_diff > 0)
    {
        old_diff = pool->next_diff;
        pool->next_diff = diff;
    }
    else
    {
        old_diff = pool->sdiff;
        pool->next_diff = pool->sdiff = diff;
    }

    cg_wunlock(&pool->data_lock);

    if (old_diff != diff)
    {
        int idiff = (int) diff;

        if ((double)idiff == diff)
        {
            applog(LOG_NOTICE, "Pool %d difficulty changed to %d", pool->pool_no, idiff);
        }
        else
        {
            applog(LOG_NOTICE, "Pool %d difficulty changed to %.1f", pool->pool_no, diff);
        }

    }
    else
    {
        applog(LOG_DEBUG, "Pool %d difficulty set to %f", pool->pool_no, diff);
    }

    return true;
}

static bool parse_extranonce(struct pool *pool, json_t *val)
{
    int n2size;
    char *nonce1;
    char s[RBUFSIZE];

    nonce1 = json_array_string(val, 0);
    if (!valid_hex(nonce1))
    {
        applog(LOG_INFO, "Failed to get valid nonce1 in parse_extranonce");
       		return false;
    }
    n2size = json_integer_value(json_array_get(val, 1));
    if (!n2size) {
        applog(LOG_INFO, "Failed to get valid n2size in parse_extranonce");
        free(nonce1);
       		return false;
    }

    cg_wlock(&pool->data_lock);
    free(pool->nonce1);
    pool->nonce1 = nonce1;
    pool->n1_len = strlen(nonce1) / 2;
    free(pool->nonce1bin);
    pool->nonce1bin = cgcalloc(pool->n1_len, (size_t)1);
    if (unlikely(!pool->nonce1bin))
        quithere(1, "Failed to calloc pool->nonce1bin");
    hex2bin(pool->nonce1bin, pool->nonce1, pool->n1_len);
    pool->n2size = n2size;
    cg_wunlock(&pool->data_lock);

            	applog(LOG_NOTICE, "Pool %d extranonce change requested", pool->pool_no);
    return true;

}

/**
 * Decodes version mask from json string
 *
 * @param pool
 * @param version_mask_json
 * @return true when the decoded mask is valid and supported by the hardware
 */
static bool decode_version_mask(struct pool *pool, json_t *version_mask_json)
{
    const char *version_mask_str;
    unsigned long version_mask;
    unsigned int i, j, bit_idx, component_idx;
    unsigned long version_bit_components[VERSION_BITS_NUM];
    unsigned long detect_bit_mask;
    bool retval = true;

    if (!version_mask_json) {
        applog(LOG_ERR, "Received empty version mask!");
        retval = false;
        goto invalid_version_mask;
    }

    version_mask_str = json_string_value(version_mask_json);

    version_mask = strtol(version_mask_str, NULL, 16);
    detect_bit_mask = 1;
    bit_idx = 0;
    component_idx = 0;

    /* If version mask doesn't support bits we can roll, bail out */
    if ((version_mask & VERSION_BITS_THAT_S9_ROLLS) != VERSION_BITS_THAT_S9_ROLLS) {
        applog(LOG_ERR, "Received version mask (0x%08lx) doesn't satisfy hardware requirement (0x%08lx)",
                           version_mask, VERSION_BITS_THAT_S9_ROLLS);
        retval = false;
        goto invalid_version_mask;
    }

    /* Scan the version mask and extract masks for individual bits to satisfy
     * the number of version bits */
    while ((bit_idx < (sizeof(version_mask) * 8)) && (component_idx < VERSION_BITS_NUM)) {
        if (version_mask & detect_bit_mask) {
            version_bit_components[component_idx++] = detect_bit_mask;
        }
        detect_bit_mask = detect_bit_mask << 1;
        bit_idx++;
    }

    if (component_idx != VERSION_BITS_NUM) {
        applog(LOG_ERR, "Received version mask (0x%08lx) doesn't satisfy hardware requirement (detected %d bits, "
                           "required %d bits)", version_mask, component_idx, VERSION_BITS_NUM);
        retval = false;
        goto invalid_version_mask;
    }

    /* Pre-generate all permutations of version bits */
    for (i = 0; i < (1UL << VERSION_BITS_NUM); i++) {
        n_version[i].bits = 0;
        for (bit_idx = 0; bit_idx < (sizeof(i) * 8); bit_idx++) {
            if (i & (1UL << bit_idx)) {
                n_version[i].bits |= version_bit_components[bit_idx];
            }
        }
        /* Update mask string used for mining.submit version_bits parameter */
        sprintf(n_version[i].bits_str, "%08x", n_version[i].bits);
    }

  invalid_version_mask:
    return retval;
}

/**
 * Parses mining.set_version_mask message
 *
 * @param pool
 * @param params - array of JSON parameters of mining.set_version_mask
 * @return true when a valid mask has been parsed
 */
static bool parse_set_version_mask(struct pool *pool, json_t *params)
{
    /* Extract the first parameter and delegate further */
    return decode_version_mask(pool, json_array_get(params, 0));
}

static void __suspend_stratum(struct pool *pool)
{
    clear_sockbuf(pool);
    pool->stratum_active = pool->stratum_notify = false;
    if (pool->sock)
        CLOSESOCKET(pool->sock);
    pool->sock = 0;
}

static bool parse_reconnect(struct pool *pool, json_t *val)
{
    char *sockaddr_url, *stratum_port, *tmp;
    char *url, *port, address[256];
    int port_no;

    memset(address, 0, 255);
    url = (char *)json_string_value(json_array_get(val, 0));
    if (!url)
        url = pool->sockaddr_url;
    else
    {
        char *dot_pool, *dot_reconnect;
        dot_pool = strchr(pool->sockaddr_url, '.');
        if (!dot_pool)
        {
            applog(LOG_ERR, "Denied stratum reconnect request for pool without domain '%s'",
                   pool->sockaddr_url);
            return false;
        }
        dot_reconnect = strchr(url, '.');
        if (!dot_reconnect)
        {
            applog(LOG_ERR, "Denied stratum reconnect request to url without domain '%s'",
                   url);
            return false;
        }
        if (strcmp(dot_pool, dot_reconnect))
        {
            applog(LOG_ERR, "Denied stratum reconnect request to non-matching domain url '%s'",
                   pool->sockaddr_url);
            return false;
        }
    }

    port_no = json_integer_value(json_array_get(val, 1));
    if (port_no) {
        port = alloca((size_t)(256));
        sprintf(port, "%d", port_no);
    } else {
    port = (char *)json_string_value(json_array_get(val, 1));
    if (!port)
        port = pool->stratum_port;
    }

    snprintf(address, 254, "%s:%s", url, port);

    if (!extract_sockaddr(address, &sockaddr_url, &stratum_port))
        return false;

    applog(LOG_WARNING, "Stratum reconnect requested from pool %d to %s", pool->pool_no, address);

    clear_pool_work(pool);

    mutex_lock(&pool->stratum_lock);
    __suspend_stratum(pool);
    tmp = pool->sockaddr_url;
    pool->sockaddr_url = sockaddr_url;
    pool->stratum_url = pool->sockaddr_url;
    free(tmp);
    tmp = pool->stratum_port;
    pool->stratum_port = stratum_port;
    free(tmp);
    mutex_unlock(&pool->stratum_lock);

    return restart_stratum(pool);
}

static bool send_version(struct pool *pool, json_t *val)
{
    json_t *id_val = json_object_get(val, "id");
    char s[RBUFSIZE];
    int id;

    if (!id_val)
        return false;
    id = json_integer_value(json_object_get(val, "id"));

    sprintf(s, "{\"id\": %d, \"result\": \""PACKAGE"/"VERSION"\", \"error\": null}", id);
    if (!stratum_send(pool, s, strlen(s)))
        return false;

    return true;
}

static bool send_pong(struct pool *pool, json_t *val)
{
    json_t *id_val = json_object_get(val, "id");
    char s[RBUFSIZE];
    int id;

    if (!id_val)
        return false;
    id = json_integer_value(json_object_get(val, "id"));

    sprintf(s, "{\"id\": %d, \"result\": \"pong\", \"error\": null}", id);
    if (!stratum_send(pool, s, strlen(s)))
        return false;

    return true;
}

static bool show_message(struct pool *pool, json_t *val)
{
    char *msg;

    if (!json_is_array(val))
        return false;
    msg = (char *)json_string_value(json_array_get(val, 0));
    if (!msg)
        return false;
    applog(LOG_NOTICE, "Pool %d message: %s", pool->pool_no, msg);
    return true;
}

bool parse_method(struct pool *pool, char *s)
{
    json_t *val = NULL, *method, *err_val, *params;
    json_error_t err;
    bool ret = false;
    char *buf;

    if (!s)
        goto out;

    val = JSON_LOADS(s, &err);
    if (!val)
    {
        applog(LOG_INFO, "JSON decode failed(%d): %s", err.line, err.text);
        goto out;
    }

    method = json_object_get(val, "method");
    if (!method)
        goto out_decref;
    err_val = json_object_get(val, "error");
    params = json_object_get(val, "params");

    if (err_val && !json_is_null(err_val))
    {
        char *ss;

        if (err_val)
            ss = json_dumps(err_val, JSON_INDENT(3));
        else
            ss = strdup("(unknown reason)");

        applog(LOG_INFO, "JSON-RPC method decode failed: %s", ss);
        free(ss);
        goto out_decref;
    }

    buf = (char *)json_string_value(method);
    if (!buf)
        goto out_decref;

    if (!strncasecmp(buf, "mining.notify", 13))
    {
        if (parse_notify(pool, params))
            pool->stratum_notify = ret = true;
        else
            pool->stratum_notify = ret = false;
        goto out_decref;
    }

    if (!strncasecmp(buf, "mining.set_version_mask", 23)) {
        ret = parse_set_version_mask(pool, params);
        goto out_decref;
    }

    if (!strncasecmp(buf, "mining.set_difficulty", 21))
    {
        ret = parse_diff(pool, params);
        goto out_decref;
    }

    if (!strncasecmp(buf, "mining.set_extranonce", 21))
    {
        ret = parse_extranonce(pool, params);
        goto out_decref;
    }

    if (!strncasecmp(buf, "client.reconnect", 16))
        {
            ret = parse_reconnect(pool, params);
            goto out_decref;
        }

    if (!strncasecmp(buf, "client.get_version", 18))
    {
        ret =  send_version(pool, val);
        goto out_decref;
    }

    if (!strncasecmp(buf, "client.show_message", 19))
    {
        ret = show_message(pool, params);
        goto out_decref;
    }

    if (!strncasecmp(buf, "mining.ping", 11))
    {
        applog(LOG_INFO, "Pool %d ping", pool->pool_no);
        ret = send_pong(pool, val);
        goto out_decref;
    }
out_decref:
    json_decref(val);
out:
    return ret;
}

bool subscribe_extranonce(struct pool *pool)
{
	json_t *val = NULL, *res_val, *err_val;
	char s[RBUFSIZE], *sret = NULL;
	json_error_t err;
	bool ret = false;

	sprintf(s, "{\"id\": %d, \"method\": \"mining.extranonce.subscribe\", \"params\": []}",
		swork_id++);

	if (!stratum_send(pool, s, strlen(s)))
		return ret;

	/* Parse all data in the queue and anything left should be the response */
	while (42) {
		if (!socket_full(pool, DEFAULT_SOCKWAIT / 30)) {
			applog(LOG_DEBUG, "Timed out waiting for response extranonce.subscribe");
			/* some pool doesnt send anything, so this is normal */
			ret = true;
			goto out;
		}

		sret = recv_line(pool);
		if (!sret)
			return ret;
		if (parse_method(pool, sret))
			free(sret);
		else
			break;
	}

	val = JSON_LOADS(sret, &err);
	free(sret);
	res_val = json_object_get(val, "result");
	err_val = json_object_get(val, "error");

	if (!res_val || json_is_false(res_val) || (err_val && !json_is_null(err_val)))  {
		char *ss;

		if (err_val) {
			ss = __json_array_string(err_val, 1);
			if (!ss)
				ss = (char *)json_string_value(err_val);
			if (ss && (strcmp(ss, "Method 'subscribe' not found for service 'mining.extranonce'") == 0)) {
				applog(LOG_INFO, "Cannot subscribe to mining.extranonce for pool %d", pool->pool_no);
				ret = true;
				goto out;
			}
			if (ss && (strcmp(ss, "Unrecognized request provided") == 0)) {
				applog(LOG_INFO, "Cannot subscribe to mining.extranonce for pool %d", pool->pool_no);
				ret = true;
				goto out;
			}
			ss = json_dumps(err_val, JSON_INDENT(3));
		}
		else
			ss = strdup("(unknown reason)");
		applog(LOG_INFO, "Pool %d JSON extranonce subscribe failed: %s", pool->pool_no, ss);
		free(ss);

		goto out;
	}

	ret = true;
	applog(LOG_INFO, "Stratum extranonce subscribe for pool %d", pool->pool_no);

out:
	json_decref(val);
	return ret;
}


bool auth_stratum(struct pool *pool)
{
    json_t *val = NULL, *res_val, *err_val;
    char s[RBUFSIZE], *sret = NULL;
    json_error_t err;
    bool ret = false;

    sprintf(s, "{\"id\": %d, \"method\": \"mining.authorize\", \"params\": [\"%s\", \"%s\"]}",
            swork_id++, pool->rpc_user, pool->rpc_pass);

    if (!stratum_send(pool, s, strlen(s)))
        return ret;

    /* Parse all data in the queue and anything left should be auth */
    while (42)
    {
        sret = recv_line(pool);
        if (!sret)
            return ret;
        if (parse_method(pool, sret))
            free(sret);
        else
            break;
    }

    val = JSON_LOADS(sret, &err);
    free(sret);
    res_val = json_object_get(val, "result");
    err_val = json_object_get(val, "error");

    if (!res_val || json_is_false(res_val) || (err_val && !json_is_null(err_val)))
    {
        char *ss;

        if (err_val)
            ss = json_dumps(err_val, JSON_INDENT(3));
        else
            ss = strdup("(unknown reason)");
        applog(LOG_INFO, "pool %d JSON stratum auth failed: %s", pool->pool_no, ss);
        free(ss);

        suspend_stratum(pool);

        goto out;
    }

    ret = true;
    applog(LOG_INFO, "Stratum authorisation success for pool %d", pool->pool_no);
    pool->probed = true;
    successful_connect = true;
    if (opt_suggest_diff)
    {
        sprintf(s, "{\"id\": %d, \"method\": \"mining.suggest_difficulty\", \"params\": [%d]}",
                swork_id++, opt_suggest_diff);
        stratum_send(pool, s, strlen(s));
    }
    if (opt_multi_version)
    {
        sprintf(s, "{\"id\": %d, \"method\": \"mining.multi_version\", \"params\": [%d]}",
                swork_id++, opt_multi_version);
        stratum_send(pool, s, strlen(s));
    }
out:
    json_decref(val);
    return ret;
}

static int recv_byte(int sockd)
{
    char c;

    if (recv(sockd, &c, 1, 0) != -1)
        return c;

    return -1;
}

static bool http_negotiate(struct pool *pool, int sockd, bool http0)
{
    char buf[1024];
    int i, len;

    if (http0)
    {
        snprintf(buf, 1024, "CONNECT %s:%s HTTP/1.0\r\n\r\n",
                 pool->sockaddr_url, pool->stratum_port);
    }
    else
    {
        snprintf(buf, 1024, "CONNECT %s:%s HTTP/1.1\r\nHost: %s:%s\r\n\r\n",
                 pool->sockaddr_url, pool->stratum_port, pool->sockaddr_url,
                 pool->stratum_port);
    }
    applog(LOG_DEBUG, "Sending proxy %s:%s - %s",
           pool->sockaddr_proxy_url, pool->sockaddr_proxy_port, buf);
    send(sockd, buf, strlen(buf), 0);
    len = recv(sockd, buf, 12, 0);
    if (len <= 0)
    {
        applog(LOG_WARNING, "Couldn't read from proxy %s:%s after sending CONNECT",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
        return false;
    }
    buf[len] = '\0';
    applog(LOG_DEBUG, "Received from proxy %s:%s - %s",
           pool->sockaddr_proxy_url, pool->sockaddr_proxy_port, buf);
    if (strcmp(buf, "HTTP/1.1 200") && strcmp(buf, "HTTP/1.0 200"))
    {
        applog(LOG_WARNING, "HTTP Error from proxy %s:%s - %s",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port, buf);
        return false;
    }

    /* Ignore unwanted headers till we get desired response */
    for (i = 0; i < 4; i++)
    {
        buf[i] = (char) recv_byte(sockd);
        if (buf[i] == (char)-1)
        {
            applog(LOG_WARNING, "Couldn't read HTTP byte from proxy %s:%s",
                   pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
            return false;
        }
    }
    while (strncmp(buf, "\r\n\r\n", 4))
    {
        for (i = 0; i < 3; i++)
            buf[i] = buf[i + 1];
        buf[3] = (char) recv_byte(sockd);
        if (buf[3] == (char)-1)
        {
            applog(LOG_WARNING, "Couldn't read HTTP byte from proxy %s:%s",
                   pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
            return false;
        }
    }

    applog(LOG_DEBUG, "Success negotiating with %s:%s HTTP proxy",
           pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
    return true;
}

static bool socks5_negotiate(struct pool *pool, int sockd)
{
    unsigned char atyp, uclen;
    unsigned short port;
    char buf[515];
    int i, len;

    buf[0] = 0x05;
    buf[1] = 0x01;
    buf[2] = 0x00;
    applog(LOG_DEBUG, "Attempting to negotiate with %s:%s SOCKS5 proxy",
           pool->sockaddr_proxy_url, pool->sockaddr_proxy_port );
    send(sockd, buf, 3, 0);
    if (recv_byte(sockd) != 0x05 || recv_byte(sockd) != buf[2])
    {
        applog(LOG_WARNING, "Bad response from %s:%s SOCKS5 server",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port );
        return false;
    }

    buf[0] = 0x05;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x03;
    len = (strlen(pool->sockaddr_url));
    if (len > 255)
        len = 255;
    uclen = (unsigned char) len;
    buf[4] = (char) (uclen & 0xff);
    cg_memcpy(buf + 5, pool->sockaddr_url, (unsigned int)len);
    port = atoi(pool->stratum_port);
    buf[5 + len] = (char) (port >> 8);
    buf[6 + len] = (char) (port & 0xff);
    send(sockd, buf, (7 + len), 0);
    if (recv_byte(sockd) != 0x05 || recv_byte(sockd) != 0x00)
    {
        applog(LOG_WARNING, "Bad response from %s:%s SOCKS5 server",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port );
        return false;
    }

    recv_byte(sockd);
    atyp = (unsigned char) recv_byte(sockd);
    if (atyp == 0x01)
    {
        for (i = 0; i < 4; i++)
            recv_byte(sockd);
    }
    else if (atyp == 0x03)
    {
        len = recv_byte(sockd);
        for (i = 0; i < len; i++)
            recv_byte(sockd);
    }
    else
    {
        applog(LOG_WARNING, "Bad response from %s:%s SOCKS5 server",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port );
        return false;
    }
    for (i = 0; i < 2; i++)
        recv_byte(sockd);

    applog(LOG_DEBUG, "Success negotiating with %s:%s SOCKS5 proxy",
           pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
    return true;
}

static bool socks4_negotiate(struct pool *pool, int sockd, bool socks4a)
{
    unsigned short port;
    in_addr_t inp;
    char buf[515];
    int i, len;

    buf[0] = 0x04;
    buf[1] = 0x01;
    port = atoi(pool->stratum_port);
    buf[2] = (char) (port >> 8);
    buf[3] = (char) (port & 0xff);
    sprintf(&buf[8], "CGMINER");

    /* See if we've been given an IP address directly to avoid needing to
     * resolve it. */
    inp = inet_addr(pool->sockaddr_url);
    inp = ntohl(inp);
    if ((in_addr_t)inp != -1)
        socks4a = false;
    else
    {
        /* Try to extract the IP address ourselves first */
        struct addrinfo servinfobase, *servinfo, hints;

        servinfo = (struct addrinfo *) &servinfobase;
        memset(&hints, 0, sizeof(struct addrinfo));
        hints.ai_family = AF_INET; /* IPV4 only */
        if (!getaddrinfo(pool->sockaddr_url, NULL, &hints, &servinfo))
        {
            struct sockaddr_in *saddr_in = (struct sockaddr_in *)servinfo->ai_addr;

            inp = ntohl(saddr_in->sin_addr.s_addr);
            socks4a = false;
            freeaddrinfo(servinfo);
        }
    }

    if (!socks4a)
    {
        if ((in_addr_t)inp == -1)
        {
            applog(LOG_WARNING, "Invalid IP address specified for socks4 proxy: %s",
                   pool->sockaddr_url);
            return false;
        }
        buf[4] = (inp >> 24) & 0xFF;
        buf[5] = (inp >> 16) & 0xFF;
        buf[6] = (inp >>  8) & 0xFF;
        buf[7] = (inp >>  0) & 0xFF;
        send(sockd, buf, 16, 0);
    }
    else
    {
        /* This appears to not be working but hopefully most will be
         * able to resolve IP addresses themselves. */
        buf[4] = 0;
        buf[5] = 0;
        buf[6] = 0;
        buf[7] = 1;
        len = strlen(pool->sockaddr_url);
        if (len > 255)
            len = 255;
        cg_memcpy(&buf[16], pool->sockaddr_url, (unsigned int)len);
        len += 16;
        buf[len++] = '\0';
        send(sockd, buf, len, 0);
    }

    if (recv_byte(sockd) != 0x00 || recv_byte(sockd) != 0x5a)
    {
        applog(LOG_WARNING, "Bad response from %s:%s SOCKS4 server",
               pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
        return false;
    }

    for (i = 0; i < 6; i++)
        recv_byte(sockd);

    return true;
}

static void noblock_socket(SOCKETTYPE fd)
{

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, O_NONBLOCK | flags);

}

static void block_socket(SOCKETTYPE fd)
{

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);

}

static bool sock_connecting(void)
{

    return errno == EINPROGRESS;

}
static bool setup_stratum_socket(struct pool *pool)
{
    struct addrinfo *servinfo, hints, *p;
    char *sockaddr_url, *sockaddr_port;
    int sockd;

    mutex_lock(&pool->stratum_lock);
    pool->stratum_active = false;
    if (pool->sock)
        CLOSESOCKET(pool->sock);
    pool->sock = 0;
    mutex_unlock(&pool->stratum_lock);

    memset(&hints, 0, sizeof(struct addrinfo));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    if (!pool->rpc_proxy && opt_socks_proxy)
    {
        pool->rpc_proxy = opt_socks_proxy;
        extract_sockaddr(pool->rpc_proxy, &pool->sockaddr_proxy_url, &pool->sockaddr_proxy_port);
        pool->rpc_proxytype = PROXY_SOCKS5;
    }

    if (pool->rpc_proxy)
    {
        sockaddr_url = pool->sockaddr_proxy_url;
        sockaddr_port = pool->sockaddr_proxy_port;
    }
    else
    {
        sockaddr_url = pool->sockaddr_url;
        sockaddr_port = pool->stratum_port;
    }
    if (getaddrinfo(sockaddr_url, sockaddr_port, &hints, &servinfo) != 0)
    {
        if (!pool->probed)
        {
            applog(LOG_WARNING, "Failed to resolve (?wrong URL) %s:%s",
                   sockaddr_url, sockaddr_port);
            pool->probed = true;
        }
        else
        {
            applog(LOG_INFO, "Failed to getaddrinfo for %s:%s",
                   sockaddr_url, sockaddr_port);
        }
        return false;
    }

    for (p = servinfo; p != NULL; p = p->ai_next)
    {
        sockd = socket(p->ai_family, p->ai_socktype, p->ai_protocol);
        if (sockd == -1)
        {
            applog(LOG_DEBUG, "Failed socket");
            continue;
        }

        /* Iterate non blocking over entries returned by getaddrinfo
         * to cope with round robin DNS entries, finding the first one
         * we can connect to quickly. */
        noblock_socket(sockd);
        if (connect(sockd, p->ai_addr, p->ai_addrlen) == -1)
        {
            struct timeval tv_timeout = {1, 0};
            int selret;
            fd_set rw;

            if (!sock_connecting())
            {
                CLOSESOCKET(sockd);
                applog(LOG_DEBUG, "Failed sock connect");
                continue;
            }
        retry:
            FD_ZERO(&rw);
            FD_SET(sockd, &rw);
            selret = select(sockd + 1, NULL, &rw, NULL, &tv_timeout);
            if  (selret > 0 && FD_ISSET(sockd, &rw))
            {
                socklen_t len;
                int err, n;

                len = sizeof(err);
                n = getsockopt(sockd, SOL_SOCKET, SO_ERROR, (void *)&err, &len);
                if (!n && !err)
                {
                    applog(LOG_DEBUG, "Succeeded delayed connect");
                    block_socket(sockd);
                    break;
                }
            }
            if (selret < 0 && interrupted())
                goto retry;
            CLOSESOCKET(sockd);
            applog(LOG_DEBUG, "Select timeout/failed connect");
            continue;
        }
        applog(LOG_WARNING, "Succeeded immediate connect");
        block_socket(sockd);

        break;
    }
    if (p == NULL)
    {
        applog(LOG_INFO, "Failed to connect to stratum on %s:%s",
               sockaddr_url, sockaddr_port);
        freeaddrinfo(servinfo);
        return false;
    }
    freeaddrinfo(servinfo);

    if (pool->rpc_proxy)
    {
        switch (pool->rpc_proxytype)
        {
            case PROXY_HTTP_1_0:
                if (!http_negotiate(pool, sockd, true))
                    return false;
                break;
            case PROXY_HTTP:
                if (!http_negotiate(pool, sockd, false))
                    return false;
                break;
            case PROXY_SOCKS5:
            case PROXY_SOCKS5H:
                if (!socks5_negotiate(pool, sockd))
                    return false;
                break;
            case PROXY_SOCKS4:
                if (!socks4_negotiate(pool, sockd, false))
                    return false;
                break;
            case PROXY_SOCKS4A:
                if (!socks4_negotiate(pool, sockd, true))
                    return false;
                break;
            default:
                applog(LOG_WARNING, "Unsupported proxy type for %s:%s",
                       pool->sockaddr_proxy_url, pool->sockaddr_proxy_port);
                return false;
                break;
        }
    }

    if (!pool->sockbuf)
    {
        pool->sockbuf = cgcalloc(RBUFSIZE, 1);
        pool->sockbuf_size = RBUFSIZE;
    }

    pool->sock = sockd;
    keep_sockalive(sockd);
    return true;
}

static char *get_sessionid(json_t *val)
{
    char *ret = NULL;
    json_t *arr_val;
    int arrsize, i;

    arr_val = json_array_get(val, 0);
    if (!arr_val || !json_is_array(arr_val))
        goto out;
    arrsize = json_array_size(arr_val);
    for (i = 0; i < arrsize; i++)
    {
        json_t *arr = json_array_get(arr_val, i);
        char *notify;

        if (!arr | !json_is_array(arr))
            break;
        notify = __json_array_string(arr, 0);
        if (!notify)
            continue;
        if (!strncasecmp(notify, "mining.notify", 13))
        {
            ret = json_array_string(arr, 1);
            break;
        }
    }
out:
    return ret;
}

void suspend_stratum(struct pool *pool)
{
    applog(LOG_INFO, "Closing socket for stratum pool %d", pool->pool_no);

    mutex_lock(&pool->stratum_lock);
    __suspend_stratum(pool);
    mutex_unlock(&pool->stratum_lock);
}


#define STRATUM_VERSION_ROLLING "version-rolling"
#define STRATUM_VERSION_ROLLING_LEN (sizeof(STRATUM_VERSION_ROLLING) - 1)

/**
 * Configures stratum mining based on connected hardware capabilities
 * (version rolling etc.)
 *
 * Sample communication
 * Request:
 * {"id": 1, "method": "mining.configure", "params": [ ["version-rolling", "sp-telemetry"], { "version-rolling.bit-count": 2, "version-rolling.mask": "ffffffff" }]}\n
 * Response:
 * {"id": 1, "result": { "version-rolling": True, "sp-telemetry": True }, "error": null}\n
 *
 * @param pool
 *
 *
 * @return
 */
static bool configure_stratum_mining(struct pool *pool)
{
    char s[RBUFSIZE];
    char *response_str = NULL;
    bool config_status = false;
    bool version_rolling_status = false;
    bool version_mask_valid = false;
    const char *key;
    json_t *response, *value, *res_val, *err_val;
    json_error_t err;

    /* TODO JCA: read version rolling bits and mask from the hardware driver
     * or pool parameters? */
    snprintf(s, RBUFSIZE,
             "{\"id\": %d, \"method\": \"mining.configure\", \"params\": "
             "[[\""STRATUM_VERSION_ROLLING"\"], "
                                          "{\""STRATUM_VERSION_ROLLING".bit-count\": %d, "
                                                                      "\""STRATUM_VERSION_ROLLING".mask\": \"%x\"}]}",
             swork_id++, 2, 0xffffffff);

    if (__stratum_send(pool, s, strlen(s)) != SEND_OK) {
        applog(LOG_DEBUG, "Failed to send mining.configure");
        goto out;
    }
    if (!socket_full(pool, DEFAULT_SOCKWAIT)) {
        applog(LOG_DEBUG, "Timed out waiting for response in %s", __FUNCTION__);
        goto out;
    }
    response_str = recv_line(pool);
    if (!response_str)
        goto out;

    response = JSON_LOADS(response_str, &err);
    free(response_str);

    res_val = json_object_get(response, "result");
    err_val = json_object_get(response, "error");

    if (!res_val || json_is_null(res_val) ||
        (err_val && !json_is_null(err_val))) {
        char *ss;

        if (err_val)
            ss = json_dumps(err_val, JSON_INDENT(3));
        else
            ss = strdup("(unknown reason)");

        applog(LOG_INFO, "JSON-RPC decode failed: %s", ss);

        free(ss);

        goto json_response_error;
    }
    json_object_foreach(res_val, key, value)
    {
        if (!strcasecmp(key, STRATUM_VERSION_ROLLING) &&
            strlen(key) == STRATUM_VERSION_ROLLING_LEN) {
            version_rolling_status = json_boolean_value(value);
        }
        else if (!strcasecmp(key, STRATUM_VERSION_ROLLING ".mask")) {
            version_mask_valid = decode_version_mask(pool, value);
        }
        else {
            applog(LOG_ERR, "JSON-RPC unexpected mining.configure value: %s", key);
        }
    }
    /* Valid configuration for now only requires enabled version rolling and valid bit mask */
    config_status = version_rolling_status && version_mask_valid;
    if (config_status) {
        pool->supports_version_rolling = true;
    }

  json_response_error:
    json_decref(response);

  out:
    return config_status;
}

bool initiate_stratum(struct pool *pool)
{
    bool ret = false, recvd = false, noresume = false, sockd = false, notification_received = false;
    char s[RBUFSIZE], *sret = NULL, *nonce1, *sessionid, *tmp;
    json_t *val = NULL, *res_val, *err_val;
    json_error_t err;
    int n2size;

resend:
    if (!setup_stratum_socket(pool))
    {
        sockd = false;
        goto out;
    }

    sockd = true;

    /* Get rid of any crap lying around if we're resending */
    if (recvd) {
        clear_sock(pool);
    }
	/* Attempt to configure stratum protocol feature set first. */
	if (!configure_stratum_mining(pool)) {
		goto out;
	}

	if (recvd) {
        sprintf(s, "{\"id\": %d, \"method\": \"mining.subscribe\", \"params\": []}", swork_id++);
    }
    else
    {
        if (pool->sessionid)
            sprintf(s, "{\"id\": %d, \"method\": \"mining.subscribe\", \"params\": [\""BOS_SMALL_VERSION_STRING"\", \"%s\"]}", swork_id++, pool->sessionid);
        else
            sprintf(s, "{\"id\": %d, \"method\": \"mining.subscribe\", \"params\": [\""BOS_SMALL_VERSION_STRING"\"]}", swork_id++);
    }

    if (__stratum_send(pool, s, strlen(s)) != SEND_OK)
    {
        applog(LOG_DEBUG, "Failed to send s in initiate_stratum");
        goto out;
    }

    if (!socket_full(pool, DEFAULT_SOCKWAIT))
    {
        applog(LOG_DEBUG, "Timed out waiting for response in initiate_stratum");
        goto out;
    }

	/* Filter out all notifications and wait for the response */
	notification_received = true;
	while (notification_received) {
    sret = recv_line(pool);
    if (!sret)
        goto out;
    recvd = true;
		/* Parse any notification including set_version_mask */
		notification_received = parse_method(pool, sret);
	}

    val = JSON_LOADS(sret, &err);
    free(sret);
    if (!val)
    {
        applog(LOG_INFO, "JSON decode failed(%d): %s", err.line, err.text);
        goto out;
    }

    res_val = json_object_get(val, "result");
    err_val = json_object_get(val, "error");

    if (!res_val || json_is_null(res_val) ||
        (err_val && !json_is_null(err_val)))
    {
        char *ss;

        if (err_val)
            ss = json_dumps(err_val, JSON_INDENT(3));
        else
            ss = strdup("(unknown reason)");

        applog(LOG_INFO, "JSON-RPC decode failed: %s", ss);

        free(ss);

        goto out;
    }

    sessionid = get_sessionid(res_val);
    if (!sessionid)
        applog(LOG_DEBUG, "Failed to get sessionid in initiate_stratum");
    nonce1 = json_array_string(res_val, 1);
    if (!valid_hex(nonce1))
    {
        applog(LOG_INFO, "Failed to get valid nonce1 in initiate_stratum");
        free(sessionid);
        goto out;
    }
    n2size = json_integer_value(json_array_get(res_val, 2));
    if (n2size < 2 || n2size > 16)
    {
        applog(LOG_INFO, "Failed to get valid n2size in initiate_stratum");
        free(sessionid);
        free(nonce1);
        goto out;
    }

    if (sessionid && pool->sessionid && !strcmp(sessionid, pool->sessionid)) {
        applog(LOG_NOTICE, "Pool %d successfully negotiated resume with the same session ID",
               pool->pool_no);
    }

    cg_wlock(&pool->data_lock);
    free(pool->nonce1);
    free(pool->sessionid);
    pool->sessionid = sessionid;
    pool->nonce1 = nonce1;
    pool->n1_len = strlen(nonce1) / 2;
    free(pool->nonce1bin);
    pool->nonce1bin = cgcalloc(pool->n1_len, (size_t)1);
    hex2bin(pool->nonce1bin, pool->nonce1, pool->n1_len);
    pool->n2size = n2size;
    cg_wunlock(&pool->data_lock);

    if (sessionid)
        applog(LOG_DEBUG, "Pool %d stratum session id: %s", pool->pool_no, pool->sessionid);

    ret = true;
out:
    if (ret)
    {
        if (!pool->stratum_url)
        {
            pool->stratum_url = pool->sockaddr_url;
        }
        pool->stratum_active = true;
        pool->next_diff = 0;
        pool->sdiff = 1;
        if (opt_protocol)
        {
            applog(LOG_DEBUG, "Pool %d confirmed mining.subscribe with extranonce1 %s extran2size %d", pool->pool_no, pool->nonce1, pool->n2size);
        }
    }
    else
    {
        if (recvd && !noresume)
        {
            /* Reset the sessionid used for stratum resuming in case the pool
            * does not support it, or does not know how to respond to the
            * presence of the sessionid parameter. */
            cg_wlock(&pool->data_lock);
            free(pool->sessionid);
            free(pool->nonce1);
            pool->sessionid = pool->nonce1 = NULL;
            cg_wunlock(&pool->data_lock);

            applog(LOG_DEBUG, "Failed to resume stratum, trying afresh");
            noresume = true;
            json_decref(val);
            goto resend;
        }
        applog(LOG_DEBUG, "Initiate stratum failed");
        if (sockd)
            suspend_stratum(pool);
    }

    json_decref(val);
    return ret;
}

bool restart_stratum(struct pool *pool)
{
    bool ret = false;

    if (pool->stratum_active)
        suspend_stratum(pool);
    if (!initiate_stratum(pool))
        goto out;
    if (pool->extranonce_subscribe && !subscribe_extranonce(pool))
        goto out;
    if (!auth_stratum(pool))
        goto out;
    //extranonce_subscribe_stratum(pool);
    ret = true;
out:
    if (!ret)
        pool_died(pool);
    else
        stratum_resumed(pool);
    return ret;
}

void dev_error(struct cgpu_info *dev, enum dev_reason reason)
{
    dev->device_last_not_well = time(NULL);
    dev->device_not_well_reason = reason;

    switch (reason)
    {
        case REASON_THREAD_FAIL_INIT:
            dev->thread_fail_init_count++;
            break;
        case REASON_THREAD_ZERO_HASH:
            dev->thread_zero_hash_count++;
            break;
        case REASON_THREAD_FAIL_QUEUE:
            dev->thread_fail_queue_count++;
            break;
        case REASON_DEV_SICK_IDLE_60:
            dev->dev_sick_idle_60_count++;
            break;
        case REASON_DEV_DEAD_IDLE_600:
            dev->dev_dead_idle_600_count++;
            break;
        case REASON_DEV_NOSTART:
            dev->dev_nostart_count++;
            break;
        case REASON_DEV_OVER_HEAT:
            dev->dev_over_heat_count++;
            break;
        case REASON_DEV_THERMAL_CUTOFF:
            dev->dev_thermal_cutoff_count++;
            break;
        case REASON_DEV_COMMS_ERROR:
            dev->dev_comms_error_count++;
            break;
        case REASON_DEV_THROTTLE:
            dev->dev_throttle_count++;
            break;
    }
}

/* Realloc an existing string to fit an extra string s, appending s to it. */
void *realloc_strcat(char *ptr, char *s)
{
    size_t old = 0, len = strlen(s);
    char *ret;

    if (!len)
        return ptr;
    if (ptr)
        old = strlen(ptr);

    len += old + 1;
    align_len(&len);

    ret = cgmalloc(len);

    if (ptr)
    {
        sprintf(ret, "%s%s", ptr, s);
        free(ptr);
    }
    else
        sprintf(ret, "%s", s);
    return ret;
}

/* Make a text readable version of a string using 0xNN for < ' ' or > '~'
 * Including 0x00 at the end
 * You must free the result yourself */
void *str_text(char *ptr)
{
    unsigned char *uptr;
    char *ret, *txt;

    if (ptr == NULL)
    {
        ret = strdup("(null)");

        if (unlikely(!ret))
            quithere(1, "Failed to malloc null");
    }

    uptr = (unsigned char *)ptr;

    ret = txt = cgmalloc(strlen(ptr) * 4 + 5); // Guaranteed >= needed

    do
    {
        if (*uptr < ' ' || *uptr > '~')
        {
            sprintf(txt, "0x%02x", *uptr);
            txt += 4;
        }
        else
            *(txt++) = *uptr;
    }
    while (*(uptr++));

    *txt = '\0';

    return ret;
}

void RenameThread(const char* name)
{
    char buf[16];

    snprintf(buf, sizeof(buf), "cg@%s", name);
#if defined(PR_SET_NAME)
    // Only the first 15 characters are used (16 - NUL terminator)
    prctl(PR_SET_NAME, buf, 0, 0, 0);
#elif (defined(__FreeBSD__) || defined(__OpenBSD__))
    pthread_set_name_np(pthread_self(), buf);
#elif defined(MAC_OSX)
    pthread_setname_np(buf);
#else
    // Prevent warnings
    (void)buf;
#endif
}

/* cgminer specific wrappers for true unnamed semaphore usage on platforms
 * that support them and for apple which does not. We use a single byte across
 * a pipe to emulate semaphore behaviour there. */
#ifdef __APPLE__
void _cgsem_init(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
    int flags, fd, i;

    if (pipe(cgsem->pipefd) == -1)
        quitfrom(1, file, func, line, "Failed pipe errno=%d", errno);

    /* Make the pipes FD_CLOEXEC to allow them to close should we call
     * execv on restart. */
    for (i = 0; i < 2; i++)
    {
        fd = cgsem->pipefd[i];
        flags = fcntl(fd, F_GETFD, 0);
        flags |= FD_CLOEXEC;
        if (fcntl(fd, F_SETFD, flags) == -1)
            quitfrom(1, file, func, line, "Failed to fcntl errno=%d", errno);
    }
}

void _cgsem_post(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
    const char buf = 1;
    int ret;

retry:
    ret = write(cgsem->pipefd[1], &buf, 1);
    if (unlikely(ret == 0))
        applog(LOG_WARNING, "Failed to write errno=%d" IN_FMT_FFL, errno, file, func, line);
    else if (unlikely(ret < 0 && interrupted))
        goto retry;
}

void _cgsem_wait(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
    char buf;
    int ret;
retry:
    ret = read(cgsem->pipefd[0], &buf, 1);
    if (unlikely(ret == 0))
        applog(LOG_WARNING, "Failed to read errno=%d" IN_FMT_FFL, errno, file, func, line);
    else if (unlikely(ret < 0 && interrupted))
        goto retry;
}

void cgsem_destroy(cgsem_t *cgsem)
{
    close(cgsem->pipefd[1]);
    close(cgsem->pipefd[0]);
}

/* This is similar to sem_timedwait but takes a millisecond value */
int _cgsem_mswait(cgsem_t *cgsem, int ms, const char *file, const char *func, const int line)
{
    struct timeval timeout;
    int ret, fd;
    fd_set rd;
    char buf;

retry:
    fd = cgsem->pipefd[0];
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    ms_to_timeval(&timeout, ms);
    ret = select(fd + 1, &rd, NULL, NULL, &timeout);

    if (ret > 0)
    {
        ret = read(fd, &buf, 1);
        return 0;
    }
    if (likely(!ret))
        return ETIMEDOUT;
    if (interrupted())
        goto retry;
    quitfrom(1, file, func, line, "Failed to sem_timedwait errno=%d cgsem=0x%p", errno, cgsem);
    /* We don't reach here */
    return 0;
}

/* Reset semaphore count back to zero */
void cgsem_reset(cgsem_t *cgsem)
{
    int ret, fd;
    fd_set rd;
    char buf;

    fd = cgsem->pipefd[0];
    FD_ZERO(&rd);
    FD_SET(fd, &rd);
    do
    {
        struct timeval timeout = {0, 0};

        ret = select(fd + 1, &rd, NULL, NULL, &timeout);
        if (ret > 0)
            ret = read(fd, &buf, 1);
        else if (unlikely(ret < 0 && interrupted()))
            ret = 1;
    }
    while (ret > 0);
}
#else
void _cgsem_init(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
    int ret;
    if ((ret = sem_init(cgsem, 0, 0)))
        quitfrom(1, file, func, line, "Failed to sem_init ret=%d errno=%d", ret, errno);
}

void _cgsem_post(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
    if (unlikely(sem_post(cgsem)))
        quitfrom(1, file, func, line, "Failed to sem_post errno=%d cgsem=0x%p", errno, cgsem);
}

void _cgsem_wait(cgsem_t *cgsem, const char *file, const char *func, const int line)
{
retry:
    if (unlikely(sem_wait(cgsem)))
    {
        if (interrupted())
            goto retry;
        quitfrom(1, file, func, line, "Failed to sem_wait errno=%d cgsem=0x%p", errno, cgsem);
    }
}

int _cgsem_mswait(cgsem_t *cgsem, int ms, const char *file, const char *func, const int line)
{
    struct timespec abs_timeout, ts_now;
    struct timeval tv_now;
    int ret;

    cgtime(&tv_now);
    timeval_to_spec(&ts_now, &tv_now);
    ms_to_timespec(&abs_timeout, (int64_t)ms);
retry:
    timeraddspec(&abs_timeout, &ts_now);
    ret = sem_timedwait(cgsem, &abs_timeout);

    if (ret)
    {
        if (likely(sock_timeout()))
            return ETIMEDOUT;
        if (interrupted())
            goto retry;
        quitfrom(1, file, func, line, "Failed to sem_timedwait errno=%d cgsem=0x%p", errno, cgsem);
    }
    return 0;
}

void cgsem_reset(cgsem_t *cgsem)
{
    int ret;

    do
    {
        ret = sem_trywait(cgsem);
        if (unlikely(ret < 0 && interrupted()))
            ret = 0;
    }
    while (!ret);
}

void cgsem_destroy(cgsem_t *cgsem)
{
    sem_destroy(cgsem);
}
#endif

/* Provide a completion_timeout helper function for unreliable functions that
 * may die due to driver issues etc that time out if the function fails and
 * can then reliably return. */
struct cg_completion
{
    cgsem_t cgsem;
    void (*fn)(void *fnarg);
    void *fnarg;
};

void *completion_thread(void *arg)
{
    struct cg_completion *cgc = (struct cg_completion *)arg;

    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    cgc->fn(cgc->fnarg);
    cgsem_post(&cgc->cgsem);

    return NULL;
}

bool cg_completion_timeout(void *fn, void *fnarg, int timeout)
{
    struct cg_completion *cgc;
    pthread_t pthread;
    bool ret = false;

    cgc = cgmalloc(sizeof(struct cg_completion));
    cgsem_init(&cgc->cgsem);
    cgc->fn = fn;
    cgc->fnarg = fnarg;

    pthread_create(&pthread, NULL, completion_thread, (void *)cgc);

    ret = cgsem_mswait(&cgc->cgsem, timeout);
    if (!ret)
    {
        pthread_join(pthread, NULL);
        free(cgc);
    }
    else
        pthread_cancel(pthread);
    return !ret;
}

void _cg_memcpy(void *dest, const void *src, unsigned int n, const char *file, const char *func, const int line)
{
    if (unlikely(n < 1 || n > (1ul << 31)))
    {
        applog(LOG_ERR, "ERR: Asked to memcpy %u bytes from %s %s():%d",
               n, file, func, line);
        return;
    }
    if (unlikely(!dest)) {
        applog(LOG_ERR, "ERR: Asked to memcpy %u bytes to NULL from %s %s():%d",
               n, file, func, line);
        return;
    }
    if (unlikely(!src)) {
        applog(LOG_ERR, "ERR: Asked to memcpy %u bytes from NULL from %s %s():%d",
               n, file, func, line);
        return;
    }
    memcpy(dest, src, n);
}

int cg_timeval_subtract(struct timeval* result, struct timeval* x, struct timeval* y)
{
    int nsec = 0;
    if(x->tv_sec > y->tv_sec)
        return -1;

    if((x->tv_sec == y->tv_sec) && (x->tv_usec > y->tv_usec))
        return -1;

    result->tv_sec = (y->tv_sec - x->tv_sec);
    result->tv_usec = (y->tv_usec - x->tv_usec);

    if(result->tv_usec < 0)
    {
        result->tv_sec--;
        result->tv_usec += 1000000;
    }
    return 0;
}

void rev(unsigned char *s, size_t l)
{
    size_t i, j;
    unsigned char t;

    for (i = 0, j = l - 1; i < j; i++, j--) {
        t = s[i];
        s[i] = s[j];
        s[j] = t;
    }
}

int check_asicnum(int asic_num, unsigned char nonce)
{
    switch(asic_num)
    {
        case 1:
            return 1;
        case 2:
            switch(nonce & 0x80)
            {
                case 0x80: return 2;
                default: return 1;
            }
        case 4:
            switch(nonce & 0xC0)
            {
                case 0xC0: return 4;
                case 0x80: return 3;
                case 0x40: return 2;
                default: return 1;
            }
        case 8:
            switch(nonce & 0xE0)
            {
                case 0xE0: return 8;
                case 0xC0: return 7;
                case 0xA0: return 6;
                case 0x80: return 5;
                case 0x60: return 4;
                case 0x40: return 3;
                case 0x20: return 2;
                default : return 1;
            }
        case 16:
            switch(nonce & 0xF0)
            {
                case 0xF0: return 16;
                case 0xE0: return 15;
                case 0xD0: return 14;
                case 0xC0: return 13;
                case 0xB0: return 12;
                case 0xA0: return 11;
                case 0x90: return 10;
                case 0x80: return 9;
                case 0x70: return 8;
                case 0x60: return 7;
                case 0x50: return 6;
                case 0x40: return 5;
                case 0x30: return 4;
                case 0x20: return 3;
                case 0x10: return 2;
                default : return 1;
            }
        case 32:
            switch(nonce & 0xF8)
            {
                case 0xF8: return 32;
                case 0xF0: return 31;
                case 0xE8: return 30;
                case 0xE0: return 29;
                case 0xD8: return 28;
                case 0xD0: return 27;
                case 0xC8: return 26;
                case 0xC0: return 25;
                case 0xB8: return 24;
                case 0xB0: return 23;
                case 0xA8: return 22;
                case 0xA0: return 21;
                case 0x98: return 20;
                case 0x90: return 19;
                case 0x88: return 18;
                case 0x80: return 17;
                case 0x78: return 16;
                case 0x70: return 15;
                case 0x68: return 14;
                case 0x60: return 13;
                case 0x58: return 12;
                case 0x50: return 11;
                case 0x48: return 10;
                case 0x40: return 9;
                case 0x38: return 8;
                case 0x30: return 7;
                case 0x28: return 6;
                case 0x20: return 5;
                case 0x18: return 4;
                case 0x10: return 3;
                case 0x08: return 2;
                default : return 1;
            }
        case 64:
            switch(nonce & 0xFC)
            {
                case 0xFC: return 64;
                case 0xF8: return 63;
                case 0xF4: return 62;
                case 0xF0: return 61;
                case 0xEC: return 60;
                case 0xE8: return 59;
                case 0xE4: return 58;
                case 0xE0: return 57;
                case 0xDC: return 56;
                case 0xD8: return 55;
                case 0xD4: return 54;
                case 0xD0: return 53;
                case 0xCC: return 52;
                case 0xC8: return 51;
                case 0xC4: return 50;
                case 0xC0: return 49;
                case 0xBC: return 48;
                case 0xB8: return 47;
                case 0xB4: return 46;
                case 0xB0: return 45;
                case 0xAC: return 44;
                case 0xA8: return 43;
                case 0xA4: return 42;
                case 0xA0: return 41;
                case 0x9C: return 40;
                case 0x98: return 39;
                case 0x94: return 38;
                case 0x90: return 37;
                case 0x8C: return 36;
                case 0x88: return 35;
                case 0x84: return 34;
                case 0x80: return 33;
                case 0x7C: return 32;
                case 0x78: return 31;
                case 0x74: return 30;
                case 0x70: return 29;
                case 0x6C: return 28;
                case 0x68: return 27;
                case 0x64: return 26;
                case 0x60: return 25;
                case 0x5C: return 24;
                case 0x58: return 23;
                case 0x54: return 22;
                case 0x50: return 21;
                case 0x4C: return 20;
                case 0x48: return 19;
                case 0x44: return 18;
                case 0x40: return 17;
                case 0x3C: return 16;
                case 0x38: return 15;
                case 0x34: return 14;
                case 0x30: return 13;
                case 0x2C: return 12;
                case 0x28: return 11;
                case 0x24: return 10;
                case 0x20: return 9;
                case 0x1C: return 8;
                case 0x18: return 7;
                case 0x14: return 6;
                case 0x10: return 5;
                case 0x0C: return 4;
                case 0x08: return 3;
                case 0x04: return 2;
                default : return 1;
            }
        default:
            return 0;
    }
}

void cg_logwork(struct work *work, unsigned char *nonce_bin, bool ok)
{
    if(opt_logwork_path)
    {
        char szmsg[1024] = {0};
        unsigned char midstate_tmp[32] = {0};
        unsigned char data_tmp[32] = {0};
        unsigned char hash_tmp[32] = {0};
        char * szworkdata = NULL;
        char * szmidstate = NULL;
        char * szdata = NULL;
        char * sznonce4 = NULL;
        char * sznonce5 = NULL;
        char * szhash = NULL;
        int asicnum = 0;
        uint64_t worksharediff = 0;
        memcpy(midstate_tmp, work->midstate, 32);
        memcpy(data_tmp, work->data+64, 12);
        memcpy(hash_tmp, work->hash, 32);
        rev((void *)midstate_tmp, (size_t)32);
        rev((void *)data_tmp, (size_t)12);
        rev((void *)hash_tmp, (size_t)32);
        szworkdata = bin2hex((void *)work->data, (size_t)128);
        szmidstate = bin2hex((void *)midstate_tmp, (size_t)32);
        szdata = bin2hex((void *)data_tmp, (size_t)12);
        sznonce4 = bin2hex((void *)nonce_bin, (size_t)4);
        sznonce5 = bin2hex((void *)nonce_bin, (size_t)5);
        szhash = bin2hex((void *)hash_tmp, (size_t)32);
        worksharediff = share_ndiff(work);
        sprintf(szmsg, "%s %08x midstate %s data %s nonce %s hash %s diff %I64d", ok?"o":"x", work->id, szmidstate, szdata, sznonce5, szhash, worksharediff);
        if(strcmp(opt_logwork_path, "screen") == 0)
        {
            applog(LOG_ERR, szmsg);
        }
        else
        {
            applog(LOG_ERR, szmsg);
            if(g_logwork_file)
            {
                sprintf(szmsg, "%s %08x work %s midstate %s data %s nonce %s hash %s diff %I64d", ok?"o":"x", work->id, szworkdata, szmidstate, szdata, sznonce5, szhash, worksharediff);

                fwrite(szmsg, strlen(szmsg), 1, g_logwork_file);
                fwrite("\n", 1, 1, g_logwork_file);
                fflush(g_logwork_file);

                if(ok)
                {
                    if(g_logwork_asicnum == 1)
                    {
                        sprintf(szmsg, "midstate %s data %s nonce %s hash %s", szmidstate, szdata, sznonce4, szhash);
                        fwrite(szmsg, strlen(szmsg), 1, g_logwork_files[0]);
                        fwrite("\n", 1, 1, g_logwork_files[0]);
                        fflush(g_logwork_files[0]);
                    }
                    else if(g_logwork_asicnum == 32 || g_logwork_asicnum == 64)
                    {
                        sprintf(szmsg, "midstate %s data %s nonce %s hash %s", szmidstate, szdata, sznonce4, szhash);
                        asicnum = check_asicnum(g_logwork_asicnum, nonce_bin[0]);
                        fwrite(szmsg, strlen(szmsg), 1, g_logwork_files[asicnum]);
                        fwrite("\n", 1, 1, g_logwork_files[asicnum]);
                        fflush(g_logwork_files[asicnum]);
                    }

                    if(opt_logwork_diff)
                    {
                        int diffnum = 0;
                        uint64_t difftmp = worksharediff;
                        while(1)
                        {
                            difftmp = difftmp >> 1;
                            if(difftmp > 0)
                            {
                                diffnum++;
                                if(diffnum >= 64)
                                {
                                    break;
                                }
                            }
                            else
                            {
                                break;
                            }
                        }
                        applog(LOG_DEBUG, "work diff %I64d diffnum %d", worksharediff, diffnum);
                        sprintf(szmsg, "midstate %s data %s nonce %s hash %s", szmidstate, szdata, sznonce4, szhash);
                        fwrite(szmsg, strlen(szmsg), 1, g_logwork_diffs[diffnum]);
                        fwrite("\n", 1, 1, g_logwork_diffs[diffnum]);
                        fflush(g_logwork_diffs[diffnum]);
                    }
                }
            }
        }
        if(szworkdata) free(szworkdata);
        if(szmidstate) free(szmidstate);
        if(szdata) free(szdata);
        if(sznonce4) free(sznonce4);
        if(sznonce5) free(sznonce5);
        if(szhash) free(szhash);
    }
}

void cg_logwork_uint32(struct work *work, uint32_t nonce, bool ok)
{
    if(opt_logwork_path)
    {
        unsigned char nonce_bin[5] = {0};
        memcpy(nonce_bin, &nonce, 4);
        cg_logwork(work, nonce_bin, ok);
    }
}

void
avg_dump(struct timed_avg *ta)
{
	printf("%p: fwin=%s fsamp=%s last=%lf sum=%lf start=%lf int=%lf\n",
		ta,
		ta->first_window ? "yes" : "no",
		ta->first_sample? "yes" : "no",
		ta->last, ta->sum,
		ta->t_start,
		ta->interval);
}

void
avg_reset(struct timed_avg *ta)
{
	ta->last = ta->sum = 0;
	ta->t_start = 0;
	ta->first_window = true;
	ta->first_sample = true;
}

void
avg_init(struct timed_avg *ta, double interval)
{
	avg_reset(ta);
	ta->interval = interval;
}

double
avg_getavg(struct timed_avg *ta, double now)
{
	/* not initialized? */
	if (ta->interval == 0)
		return 0;
	/* nothing measured yet? */
	if (ta->first_sample)
		return 0;
	/* are we in the first window? */
	if (ta->first_window && now > ta->t_start) {
		/* do not take into account previous (zero) window */
		return ta->sum / (now - ta->t_start);
	}

	/* we haven't been updated in a window's worth of time */
	if (now >= ta->t_start + ta->interval)
		return ta->sum / (now - ta->t_start);

	/* how far are we into current window, p < 1 */
	double p = (now - ta->t_start) / ta->interval;
	/* interpolate between this and previous window */
	return (ta->sum + ta->last * (1 - p)) / ta->interval;
}

void
avg_insert(struct timed_avg *ta, double now, double x)
{
#if 0
	printf("inserting sample %lf into %p\n", x, ta);
#endif
	if (ta->first_sample) {
		ta->t_start = now;
		ta->first_sample = false;
	}

	/* is window full? */
	double since_start = now - ta->t_start;

	if (ta->interval > 0 && since_start >= ta->interval) {
		ta->last = ta->sum / since_start * ta->interval;
		ta->sum = 0;
		ta->t_start = now;
		ta->first_window = false;
	}

	ta->sum += x;
}
