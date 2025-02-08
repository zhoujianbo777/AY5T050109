/* ===========================================================================
 * Copyright (c) 2017-2022 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
=========================================================================== */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/keycodes.h>
#include <time.h>
#include "screen.h"
#include <errno.h>
#include <pthread.h>
#include <fcntl.h>

#include "test_util.h"
#include "test_util_qnx.h"

#include "pmem.h"
#include "sys/siginfo.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <gpio_devctl.h>
#include "gpio_client.h"

#ifdef __cplusplus
}
#endif

# define SIGEV_NONE_INIT(__e)					\
	((__e)->sigev_notify = SIGEV_NONE)

# define SIGEV_SIGNAL_INIT(__e, __s)			\
	((__e)->sigev_notify = SIGEV_SIGNAL,		\
	 (__e)->sigev_signo = (__s))

# define SIGEV_SIGNAL_VALUE_INIT(__e, __s, __v)		\
	((__e)->sigev_notify = SIGEV_SIGNAL,			\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v))

# define SIGEV_SIGNAL_VALUE_PTR_INIT(__e, __s, __v)		\
	((__e)->sigev_notify = SIGEV_SIGNAL,			\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v))

# define SIGEV_SIGNAL_VALUE_INT_INIT(__e, __s, __v)		\
	((__e)->sigev_notify = SIGEV_SIGNAL | SIGEV_FLAG_SIVAL_INT,	\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_int = (int)(__v))

# define SIGEV_SIGNAL_CODE_INIT(__e, __s, __v, __c)	\
	((__e)->sigev_notify = SIGEV_SIGNAL_CODE,		\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),	\
	 (__e)->sigev_code = (__c))

# define SIGEV_SIGNAL_CODE_PTR_INIT(__e, __s, __v, __c)	\
	((__e)->sigev_notify = SIGEV_SIGNAL_CODE,		\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),	\
	 (__e)->sigev_code = (__c))

# define SIGEV_SIGNAL_CODE_INT_INIT(__e, __s, __v, __c)	\
	((__e)->sigev_notify = SIGEV_SIGNAL_CODE | SIGEV_FLAG_SIVAL_INT,\
	 (__e)->sigev_signo = (__s),					\
	 (__e)->sigev_value.sival_int = (int)(__v),	\
	 (__e)->sigev_code = (__c))

# define SIGEV_SIGNAL_THREAD_INIT(__e, __s, __v, __c)	\
	((__e)->sigev_notify = SIGEV_SIGNAL_THREAD,			\
	 (__e)->sigev_signo = (__s),						\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),		\
	 (__e)->sigev_code = (__c))

# define SIGEV_PULSE_INIT(__e, __f, __p, __c, __v)	\
	((__e)->sigev_notify = SIGEV_PULSE,				\
	 (__e)->sigev_coid = (__f),						\
	 (__e)->sigev_priority = (__p),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),	\
	 (__e)->sigev_code = (__c))

# define SIGEV_PULSE_INT_INIT(__e, __f, __p, __c, __v)	\
	((__e)->sigev_notify = SIGEV_PULSE | SIGEV_FLAG_SIVAL_INT,	\
	 (__e)->sigev_coid = (__f),						\
	 (__e)->sigev_priority = (__p),					\
	 (__e)->sigev_value.sival_int = (int)(__v),		\
	 (__e)->sigev_code = (__c))

# define SIGEV_PULSE_PTR_INIT(__e, __f, __p, __c, __v)	\
	((__e)->sigev_notify = SIGEV_PULSE,				\
	 (__e)->sigev_coid = (__f),						\
	 (__e)->sigev_priority = (__p),					\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),			\
	 (__e)->sigev_code = (__c))

# define SIGEV_UNBLOCK_INIT(__e)				\
	((__e)->sigev_notify = SIGEV_UNBLOCK)

# define SIGEV_INTR_INIT(__e)					\
	((__e)->sigev_notify = SIGEV_INTR)

# define SIGEV_THREAD_INIT(__e, __f, __v, __a)		\
	((__e)->sigev_notify = SIGEV_THREAD,			\
	 (__e)->sigev_notify_function = (__f),			\
	 (__e)->sigev_value.sival_ptr = (void *)(_Uintptrt)(__v),	\
	 (__e)->sigev_notify_attributes = (__a))

# define SIGEV_MEMORY_INIT(__e, __a, __v, __o)		\
	((__e)->sigev_notify = SIGEV_MEMORY,			\
	 (__e)->sigev_addr = (__a),			\
	 (__e)->sigev_value.sival_int = (int)(__v),		\
	 (__e)->sigev_memop = (__o))

# define SIGEV_TYPE_MASK		0x000000ff
# define SIGEV_FLAG_OVERDRIVE	0x00000200
# define SIGEV_FLAG_UPDATEABLE	0x00000400
# define SIGEV_FLAG_SIVAL_INT	0x00000800
# define SIGEV_FLAG_NOQUEUE		0x00001000 //do not queue the pulse if no server to handle it.
# define SIGEV_FLAG_HANDLE		0x00002000
# define SIGEV_FLAG_CODE_UPDATEABLE	0x00004000

#define SIGEV_INVALID_HANDLE	0xffffffffU

/* Note some of bits in .sigev_notify have hidden meaning. So after
 * initializing an event, do not directly assign to .sigev_notify,
 * instead, use this macro:
 */
# define SIGEV_SET_TYPE(__e, __new_type)								\
	((__e)->sigev_notify = (int)((unsigned)((__e)->sigev_notify) & ~SIGEV_TYPE_MASK) | (__new_type))


/* Also, to test the type of a sigevent, don't compare .SIGEV_NOTIFY,
 * as some hidden bits may be set, instead use SIGEV_GET_TYPE, example
 * if( SIGEV_GET_TYPE(my_event) == SIGEV_PULSE) ...
 */
# define SIGEV_GET_TYPE(__e)					\
	(int)((unsigned)((__e)->sigev_notify) & SIGEV_TYPE_MASK)


/* Mark an event probably requiring lots of CPU when it gets delivered
 * so any DVFS algorithm that might be running will know to bump the
 * system to the highest performing point.
 */
# define SIGEV_MAKE_OVERDRIVE(__e)				\
	((__e)->sigev_notify |= SIGEV_FLAG_OVERDRIVE)
# define SIGEV_CLEAR_OVERDRIVE(__e)				\
	((__e)->sigev_notify &= ~SIGEV_FLAG_OVERDRIVE)


/* Mark the event indicating that the sigev_value field can updated by
 * the server before the event is delivered.
 */
# define SIGEV_MAKE_UPDATEABLE(__e)				\
	((__e)->sigev_notify |= SIGEV_FLAG_UPDATEABLE)
# define SIGEV_CLEAR_UPDATEABLE(__e)			\
	((__e)->sigev_notify &= ~SIGEV_FLAG_UPDATEABLE)

/* Mark the event indicating that the if there is no thread to receive it
 * do not queue the pulse.
 */
# define SIGEV_MAKE_NOQUEUE(__e)				\
	((__e)->sigev_notify |= SIGEV_FLAG_NOQUEUE)
# define SIGEV_CLEAR_NOQUEUE(__e)			\
	((__e)->sigev_notify &= ~SIGEV_FLAG_NOQUEUE)

/*
 * Get the number of bytes in the sigevent
 */
# define SIGEV_GET_SIZE(__e)	(((__e)->sigev_notify & SIGEV_64BIT) ? sizeof(struct __sigevent64) : sizeof(struct __sigevent32))


/*
 * for SIGEV_PULSE don't modify the receiving threads priority
 * when the pulse is received
 */
# define SIGEV_PULSE_PRIO_INHERIT (-1)

/*
 * SIGEV_MEMORY operations.
 */
# define SIGEV_MEM_ASSIGN		1
# define SIGEV_MEM_ADD			2
# define SIGEV_MEM_SUB			3
# define SIGEV_MEM_BITSET		4
# define SIGEV_MEM_BITCLR		5
# define SIGEV_MEM_BITTOGGLE	6

typedef enum
{
    TEST_UTIL_PATTERN_BLACK = 0
}test_util_pattern_t;


static int g_aborted = 0;
static int g_fd_gpio;

static void test_util_fill_planes(qcarcam_buffer_t* p_buffer, qcarcam_color_fmt_t fmt)
{
    switch (fmt)
    {
    case QCARCAM_FMT_RGB_888:
        p_buffer->planes[0].stride = p_buffer->planes[0].width * 3;
        break;
    case QCARCAM_FMT_MIPIRAW_8:
        p_buffer->planes[0].stride = p_buffer->planes[0].width;
        break;
    case QCARCAM_FMT_MIPIRAW_10:
        if (0 == (p_buffer->planes[0].width % 4))
        {
            p_buffer->planes[0].stride = p_buffer->planes[0].width * 5 / 4;
        }
        break;
    case QCARCAM_FMT_MIPIRAW_12:
        if (0 == (p_buffer->planes[0].width % 2))
        {
            p_buffer->planes[0].stride = p_buffer->planes[0].width * 3 / 2;
        }
        break;
    case QCARCAM_FMT_UYVY_8:
        p_buffer->planes[0].stride = p_buffer->planes[0].width * 2;
        break;
    case QCARCAM_FMT_UYVY_10:
        p_buffer->planes[0].stride = p_buffer->planes[0].width * 5 / 4;
        break;
    case QCARCAM_FMT_UYVY_12:
        if (0 == (p_buffer->planes[0].width % 2))
        {
            p_buffer->planes[0].stride = p_buffer->planes[0].width * 2 * 3 / 2;
        }
        break;
    case QCARCAM_FMT_NV12:
    case QCARCAM_FMT_NV21:
        p_buffer->n_planes = 2;

        p_buffer->planes[0].stride = p_buffer->planes[0].width;

        //plane 2
        p_buffer->planes[1].width = p_buffer->planes[0].width;
        p_buffer->planes[1].height = p_buffer->planes[0].height / 2;
        p_buffer->planes[1].stride = p_buffer->planes[0].width;

        break;
    default:
        break;
    }

    p_buffer->planes[0].size = p_buffer->planes[0].stride * p_buffer->planes[0].height;
    p_buffer->planes[1].size = p_buffer->planes[1].stride * p_buffer->planes[1].height;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_init
///
/// @brief Initialize context that is to be used to display content on the screen.
///
/// @param ctxt   Pointer to context to be initialized
/// @param params Parameters to init ctxt
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_init(test_util_ctxt_t **pp_ctxt, test_util_ctxt_params_t *p_params)
{
    int rc, i;
    test_util_ctxt_t* pCtxt;

    *pp_ctxt = NULL;

    if (!p_params)
    {
        return QCARCAM_RET_BADPARAM;
    }

    pCtxt = (test_util_ctxt_t*)calloc(1, sizeof(struct test_util_ctxt_t));
    if (!pCtxt)
    {
        return QCARCAM_RET_NOMEM;
    }

    pCtxt->params = *p_params;

    if (pCtxt->params.enable_gpio_config)
    {
        g_fd_gpio = gpio_open(NULL);
        if (g_fd_gpio == -1)
        {
            QCARCAM_ERRORMSG("gpio_open() failed");
            goto fail;
        }
    }

    if(!pCtxt->params.disable_display)
    {
        rc = screen_create_context(&pCtxt->screen_ctxt, 0);
        if (rc)
        {
            perror("screen_context_create");
            goto fail;
        }

        /*query displays*/
        rc = screen_get_context_property_iv(pCtxt->screen_ctxt, SCREEN_PROPERTY_DISPLAY_COUNT,
                                            &pCtxt->screen_ndisplays);
        if (rc)
        {
            perror("screen_get_context_property_iv(SCREEN_PROPERTY_DISPLAY_COUNT)");
            goto fail;
        }

        pCtxt->screen_display = (screen_display_t*)calloc(pCtxt->screen_ndisplays, sizeof(*pCtxt->screen_display));
        if (pCtxt->screen_display == NULL)
        {
            perror("could not allocate memory for display list");
            goto fail;
        }

        pCtxt->display_property = (test_util_display_prop_t*)calloc(pCtxt->screen_ndisplays, sizeof(*pCtxt->display_property));
        if (pCtxt->display_property == NULL)
        {
            perror("could not allocate memory for display list");
            goto fail;
        }

        rc = screen_get_context_property_pv(pCtxt->screen_ctxt, SCREEN_PROPERTY_DISPLAYS,
                                            (void **)pCtxt->screen_display);
        if (rc)
        {
            perror("screen_get_context_property_ptr(SCREEN_PROPERTY_DISPLAYS)");
            free(pCtxt->screen_display);
            goto fail;
        }

        for (i = 0; i < pCtxt->screen_ndisplays; ++i)
        {
            rc = screen_get_display_property_iv(pCtxt->screen_display[i],
                                                SCREEN_PROPERTY_ID, &pCtxt->display_property[i].display_id);
            if (rc)
            {
                perror("screen_get_display_property_iv(SCREEN_PROPERTY_ID)");
                goto fail;
            }

            rc = screen_get_display_property_iv(pCtxt->screen_display[i],
                                                SCREEN_PROPERTY_SIZE, pCtxt->display_property[i].size);
            if (rc)
            {
                perror("screen_get_display_property_iv(SCREEN_PROPERTY_SIZE)");
                goto fail;
            }
        }
    }

    *pp_ctxt = pCtxt;

    return QCARCAM_RET_OK;

fail:
    free(pCtxt);
    return QCARCAM_RET_FAILED;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_init_window
///
/// @brief Initialize new window
///
/// @param ctxt             Pointer to util context
/// @param user_ctxt        Pointer to new window to be initialized
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_init_window(test_util_ctxt_t *p_ctxt, test_util_window_t **pp_window)
{
    *pp_window = NULL;

    test_util_window_t* p_window = (test_util_window_t*)calloc(1, sizeof(struct test_util_window_t));
    if (!p_window)
    {
        return QCARCAM_RET_NOMEM;
    }

    p_window->prev_post_idx = -1;

    *pp_window = p_window;

    return QCARCAM_RET_OK;
}

static int test_util_get_screen_format(test_util_color_fmt_t fmt)
{
    int ret;
    switch (fmt)
    {
    case TESTUTIL_FMT_UYVY_8:
        ret = SCREEN_FORMAT_UYVY;
        break;
    case TESTUTIL_FMT_RGB_888:
        ret = SCREEN_FORMAT_RGB888;
        break;
    case TESTUTIL_FMT_RGBX_8888:
        ret = SCREEN_FORMAT_RGBX8888;
        break;
    case TESTUTIL_FMT_NV12:
        ret = SCREEN_FORMAT_NV12;
        break;
    case TESTUTIL_FMT_RGB_565:
    default:
        ret = SCREEN_FORMAT_RGB565;
        break;
    }
    return ret;
}

#ifndef C2D_DISABLED
static int test_util_get_c2d_format(int fmt)
{
    int ret;
    switch (fmt)
    {
    case SCREEN_FORMAT_RGBX8888:
        ret = C2D_COLOR_FORMAT_8888_ARGB;
        break;
    case SCREEN_FORMAT_RGB888:
        ret = C2D_COLOR_FORMAT_888_RGB;
        break;
    default:
        ret = C2D_COLOR_FORMAT_565_RGB;
        break;
    }
    return ret;
}
#endif

static void test_util_fill_buffer(test_util_buffer_t* p_buffer, test_util_pattern_t pattern, test_util_color_fmt_t format)
{
    if (format == TESTUTIL_FMT_UYVY_8)
    {
        //grey
        memset(p_buffer->ptr[0], 0x80, p_buffer->size[0]);
    }
    else if (format == TESTUTIL_FMT_NV12)
    {
        //black
        memset(p_buffer->ptr[0], 0x0, p_buffer->size[0]);
        memset(p_buffer->ptr[1], 0x80, p_buffer->size[1]);
    }
    else
    {
        memset(p_buffer->ptr[0], 0x0, p_buffer->size[0]);
    }
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_init_window_buffers
///
/// @brief Initialize buffers for display
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param buffers          Pointer to qcarcam buffers
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_init_window_buffers(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, qcarcam_buffers_t *p_buffers)
{
    int i, rc;

    p_window->n_buffers = p_buffers->n_buffers;
    p_window->buffers = (test_util_buffer_t*)calloc(p_window->n_buffers, sizeof(*p_window->buffers));
    if (!p_window->buffers)
    {
        QCARCAM_ERRORMSG("Failed to allocate buffers structure");
        return QCARCAM_RET_FAILED;
    }

    p_window->buffer_size[0] = p_buffers->buffers[0].planes[0].width;
    p_window->buffer_size[1] = p_buffers->buffers[0].planes[0].height;

    //set flag for pmem v2
#ifdef USE_PMEM_V2
    p_buffers->flags |= QCARCAM_BUFFER_FLAG_OS_HNDL;
#endif

    if (!p_ctxt->params.disable_display && !p_window->is_offscreen)
    {
        rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_FORMAT, &p_window->screen_format);
        if (rc)
        {
            perror("screen_set_window_property_iv(SCREEN_PROPERTY_FORMAT)");
            return QCARCAM_RET_FAILED;
        }

        rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_BUFFER_SIZE, p_window->buffer_size);
        if (rc)
        {
            perror("screen_set_window_property_iv(SCREEN_PROPERTY_BUFFER_SIZE)");
            return QCARCAM_RET_FAILED;
        }

        rc = screen_create_window_buffers(p_window->screen_win, p_window->n_buffers);
        if (rc)
        {
            perror("screen_create_window_buffers");
            return QCARCAM_RET_FAILED;
        }

        rc = screen_get_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_RENDER_BUFFER_COUNT, &p_window->n_pointers);
        if (rc)
        {
            perror("screen_get_window_property_iv(SCREEN_PROPERTY_RENDER_BUFFER_COUNT)");
            return QCARCAM_RET_FAILED;
        }

        p_window->screen_bufs = (screen_buffer_t*)calloc(p_window->n_pointers, sizeof(*p_window->screen_bufs));
        if (!p_window->screen_bufs)
        {
            QCARCAM_ERRORMSG("Failed to allocate screen buffers structure");
            return QCARCAM_RET_FAILED;
        }

        rc = screen_get_window_property_pv(p_window->screen_win, SCREEN_PROPERTY_RENDER_BUFFERS, (void **)p_window->screen_bufs);
        if (rc)
        {
            perror("screen_get_window_property_pv(SCREEN_PROPERTY_RENDER_BUFFERS)");
            return QCARCAM_RET_FAILED;
        }

        rc = screen_get_buffer_property_iv(p_window->screen_bufs[0], SCREEN_PROPERTY_STRIDE, &p_window->stride[0]);
        if (rc)
        {
            perror("screen_get_buffer_property_iv(SCREEN_PROPERTY_STRIDE)");
            return QCARCAM_RET_FAILED;
        }

        // offset for each plane from start of buffer
        rc = screen_get_buffer_property_iv(p_window->screen_bufs[0], SCREEN_PROPERTY_PLANAR_OFFSETS, &p_window->offset[0]);
        if (rc)
        {
            perror("screen_get_buffer_property_iv(SCREEN_PROPERTY_PLANAR_OFFSETS)");
            return QCARCAM_RET_FAILED;
        }

        for (i = 0; i < p_window->n_pointers; i++)
        {
            // Get pmem handle from screen buffer
            rc = screen_get_buffer_property_pv(p_window->screen_bufs[i], SCREEN_PROPERTY_EGL_HANDLE, &p_window->buffers[i].mem_handle);
            if (rc)
            {
                perror("screen_get_window_property_pv(SCREEN_PROPERTY_EGL_HANDLE)");
                return QCARCAM_RET_FAILED;
            }

            // obtain the pointer of the buffers, for the capture use
            rc = screen_get_buffer_property_pv(p_window->screen_bufs[i], SCREEN_PROPERTY_POINTER, &p_window->buffers[i].ptr[0]);
            if (rc)
            {
                perror("screen_get_window_property_pv(SCREEN_PROPERTY_POINTER)");
                return QCARCAM_RET_FAILED;
            }
            else
            {
                QCARCAM_DBGMSG("screen pointer[%d] = 0x%p", i, p_window->buffers[i].ptr[0]);
            }

            rc = screen_get_buffer_property_llv(p_window->screen_bufs[i], SCREEN_PROPERTY_PHYSICAL_ADDRESS, &p_window->buffers[i].phys_addr);
            if (rc)
            {
                perror("screen_get_window_property_pv(SCREEN_PROPERTY_PHYSICAL_ADDRESS)");
                return QCARCAM_RET_FAILED;
            }

            // For V2 we us pmem handle, for V1 we use virtual address
#ifdef USE_PMEM_V2
            p_buffers->buffers[i].planes[0].p_buf = p_window->buffers[i].mem_handle;
#else
            p_buffers->buffers[i].planes[0].p_buf = p_window->buffers[i].ptr[0];
#endif
            QCARCAM_DBGMSG("allocate: [%d] 0x%p", i, p_buffers->buffers[i].planes[0].p_buf);

            p_buffers->buffers[i].n_planes = 1;
            p_buffers->buffers[i].planes[0].stride = p_window->stride[0];
            p_buffers->buffers[i].planes[0].size = p_window->stride[0] * p_buffers->buffers[i].planes[0].height;
            p_window->buffers[i].size[0] = p_buffers->buffers[i].planes[0].size;

            QCARCAM_DBGMSG("\t [0] 0x%p %dx%d %d, offset:0x%x",
                p_buffers->buffers[i].planes[0].p_buf,
                p_buffers->buffers[i].planes[0].width,
                p_buffers->buffers[i].planes[0].height,
                p_buffers->buffers[i].planes[0].stride,
                p_window->offset[0]);

            if (p_window->format == TESTUTIL_FMT_NV12)
            {
                // Offset[1] is number of bytes to get to second plane (i.e. size of first plane)
                p_buffers->buffers[i].n_planes = 2;
                p_buffers->buffers[i].planes[0].size = p_window->offset[1];
                p_window->buffers[i].size[0] = p_window->offset[1];

                p_window->buffers[i].ptr[1] = (void*)((uintptr_t)(p_window->buffers[i].ptr[0]) + p_window->offset[1]);
#ifndef USE_PMEM_V2
                p_buffers->buffers[i].planes[1].p_buf = p_window->buffers[i].ptr[1];
#endif
                p_buffers->buffers[i].planes[1].width = p_buffers->buffers[i].planes[0].width;
                p_buffers->buffers[i].planes[1].height = p_buffers->buffers[i].planes[0].height / 2;

                //stride is same as plane 1
                p_window->stride[1] = p_window->stride[0];
                p_buffers->buffers[i].planes[1].stride = p_window->stride[1];

                p_buffers->buffers[i].planes[1].size = p_window->stride[1] * p_buffers->buffers[i].planes[1].height;
                p_window->buffers[i].size[1] = p_buffers->buffers[i].planes[1].size;

                QCARCAM_DBGMSG("\t [1] 0x%p %dx%d %d, offset:0x%x",
                    p_buffers->buffers[i].planes[1].p_buf,
                    p_buffers->buffers[i].planes[1].width,
                    p_buffers->buffers[i].planes[1].height,
                    p_buffers->buffers[i].planes[1].stride,
                    p_window->offset[1]);
            }


        }

        // Fill last buffer with 0 pattern. Some applications allocate one extra buffer
        // to send it constantly to display in the event of loss of signal from the sensor
        // instead of the last captured frame.
        test_util_fill_buffer(&p_window->buffers[p_window->n_pointers-1], TEST_UTIL_PATTERN_BLACK, p_window->format);
    }
    else
    {
        for (i = 0; i < p_window->n_buffers; i++)
        {
            test_util_fill_planes(&p_buffers->buffers[i], p_buffers->color_fmt);
            p_window->stride[0] = p_buffers->buffers[i].planes[0].stride;
            p_window->stride[1] = p_buffers->buffers[i].planes[1].stride;
            p_window->buffers[i].size[0] = p_buffers->buffers[i].planes[0].size;
            p_window->buffers[i].size[1] = p_buffers->buffers[i].planes[1].size;
            p_window->offset[1] = p_buffers->buffers[i].planes[0].size;

#ifdef USE_PMEM_V2
            p_window->buffers[i].ptr[0] = pmem_malloc_ext_v2(
                    p_buffers->buffers[i].planes[0].size + p_buffers->buffers[i].planes[1].size,
                    PMEM_CAMERA_ID,
                    PMEM_FLAGS_SHMEM | PMEM_FLAGS_PHYS_NON_CONTIG | PMEM_FLAGS_CACHE_NONE,
                    PMEM_ALIGNMENT_4K,
                    0,
                    (pmem_handle_t*)&p_window->buffers[i].mem_handle,
                    NULL);

            if (p_window->buffers[i].ptr[0] == NULL)
            {
                QCARCAM_ERRORMSG("test_util_init_window_buffers: pmem_alloc failed");
                return QCARCAM_RET_FAILED;
            }
            p_buffers->buffers[i].planes[0].p_buf = p_window->buffers[i].mem_handle;
            QCARCAM_INFOMSG("ptr=0x%x pmem=0x%x", p_window->buffers[i].ptr[0], p_buffers->buffers[i].planes[0].p_buf);
#else //SMMU V1
            p_window->buffers[i].ptr[0] = pmem_malloc_ext(
                    p_buffers->buffers[i].planes[0].size,
                    PMEM_CAMERA_ID,
                    PMEM_FLAGS_PHYS_NON_CONTIG,
                    PMEM_ALIGNMENT_4K);
            if (p_window->buffers[i].ptr[0] == NULL)
            {
                QCARCAM_ERRORMSG("test_util_init_window_buffers: pmem_alloc failed");
                return QCARCAM_RET_FAILED;
            }
            p_window->buffers[i].ptr[1] = (void*)((uintptr_t)(p_window->buffers[i].ptr[0]) + p_window->offset[1]);
            p_buffers->buffers[i].planes[0].p_buf = p_window->buffers[i].ptr[0];
            p_buffers->buffers[i].planes[1].p_buf = p_window->buffers[i].ptr[1];
#endif

            p_window->buffers[i].phys_addr = (long long)pmem_get_phys_addr(p_window->buffers[i].ptr[0]);
        }
    }

    // Prefill all buffers as black
    for (i = 0; i < p_window->n_pointers; i++)
    {
        test_util_fill_buffer(&p_window->buffers[i], TEST_UTIL_PATTERN_BLACK, p_window->format);
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_post_window_buffer
///
/// @brief Send frame to display
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param idx              Frame ID number
/// @param p_rel_buf_idx    List to fill with buffers ready to release
/// @param field_type       Field type in current frame buffer if interlaced
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_post_window_buffer(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window,
        unsigned int idx, std::list<uint32>* p_rel_buf_idx, qcarcam_field_t field_type)
{
    if (!p_ctxt->params.disable_display && !p_window->is_offscreen)
    {
        int rc = 0;
        int rect[4] = {p_window->params.spos[0], p_window->params.spos[1], p_window->params.ssize[0], p_window->params.ssize[1]};

        QCARCAM_DBGMSG("%s:%d %d %d", __func__, __LINE__, p_ctxt->params.enable_di, field_type);

        if (p_ctxt->params.enable_di == SW_BOB_30FPS)
        {
            if (field_type == QCARCAM_FIELD_ODD_EVEN)
            {
                int param[2] = {p_window->params.ssize[0], (DE_INTERLACE_HEIGHT/2)};
                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_SIZE, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[0] = p_window->params.spos[0];
                param[1] = 13;

                rect[1] = 13;
                rect[3] = DE_INTERLACE_HEIGHT/2;

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
            else if (field_type == QCARCAM_FIELD_EVEN_ODD)
            {
                int param[2] = {p_window->params.ssize[0], (DE_INTERLACE_HEIGHT/2)};
                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_SIZE, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[0] = p_window->params.spos[0];
                param[1] = 14;

                rect[1] = 14;
                rect[3] = DE_INTERLACE_HEIGHT/2;

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
            else
            {
                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
        }
        else if (p_ctxt->params.enable_di == SW_BOB_60FPS)
        {
            if (field_type == QCARCAM_FIELD_ODD_EVEN)
            {
                int param[2] = {p_window->params.ssize[0], (DE_INTERLACE_HEIGHT/2)};
                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_SIZE, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[0] = p_window->params.spos[0];
                param[1] = 13;

                rect[1] = 13;
                rect[3] = DE_INTERLACE_HEIGHT/2;

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[1] = 13 + (DE_INTERLACE_HEIGHT/2) + 14;
                rect[1] = param[1];

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
            else if (field_type == QCARCAM_FIELD_EVEN_ODD)
            {
                int param[2] = {p_window->params.ssize[0], (DE_INTERLACE_HEIGHT/2)};
                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_SIZE, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[0] = p_window->params.spos[0];
                param[1] = 14;

                rect[1] = 14;
                rect[3] = DE_INTERLACE_HEIGHT/2;

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                param[1] = 14 + DE_INTERLACE_HEIGHT/2 + 13;
                rect[1] = param[1];

                rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, param);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }

                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
            else
            {
                rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
                if (rc)
                {
                    return QCARCAM_RET_FAILED;
                }
            }
        }
        else
        {
            rc = screen_post_window(p_window->screen_win, p_window->screen_bufs[idx], 1, rect, SCREEN_WAIT_IDLE);
            if (rc)
            {
                return QCARCAM_RET_FAILED;
            }
        }

        if (p_window->prev_post_idx != -1)
        {
            p_rel_buf_idx->push_back(p_window->prev_post_idx);
        }
        p_window->prev_post_idx = idx;
    }
    else
    {
        p_rel_buf_idx->push_back(idx);
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_dump_window_buffer
///
/// @brief Dump frame to a file
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param idx              Frame ID number
/// @param filename         Char pointer to file name to be dumped
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_dump_window_buffer(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, unsigned int idx, const char *filename)
{
    FILE *fp;
    size_t numBytesWritten = 0;
    size_t numByteToWrite = 0;

    fp = fopen(filename, "w+");

    QCARCAM_ERRORMSG("dumping qcarcam frame %s", filename);

    if (0 != fp)
    {
        test_util_buffer_t* buffer = &p_window->buffers[idx];
        char buf[8];

        memset(buf, 0, sizeof(buf));
        memcpy(buf, buffer->ptr[0], sizeof(buf));

        numByteToWrite = buffer->size[0];
        numBytesWritten = fwrite(buffer->ptr[0], 1, buffer->size[0], fp);
        QCARCAM_INFOMSG("ptr=0x%x numByteToWrite=%d numBytesWritten=%d", buffer->ptr[0], numByteToWrite, numBytesWritten);

        if (p_window->format == TESTUTIL_FMT_NV12)
        {
            numByteToWrite += buffer->size[1];
            numBytesWritten += fwrite(buffer->ptr[1], 1, buffer->size[1], fp);
        }

        if (numBytesWritten != numByteToWrite)
        {
            QCARCAM_ERRORMSG("error no data written to file");
        }

        fclose(fp);
    }
    else
    {
        QCARCAM_ERRORMSG("failed to open file");
        return QCARCAM_RET_FAILED;
    }
    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_get_buf_ptr
///
/// @brief Get buffer virtual address
///
/// @param p_window       window
/// @param p_buf          pointer to buffer structure to be filled
///
/// @return Void
///////////////////////////////////////////////////////////////////////////////
void test_util_get_buf_ptr(test_util_window_t *p_window, test_util_buf_ptr_t *p_buf)
{
    // Todo

    int idx = p_buf->buf_idx % p_window->n_buffers;
    QCARCAM_ERRORMSG("buf idx = %d, no. buffers = %d, idx = %d", p_buf->buf_idx, p_window->n_buffers, idx);
    p_buf->p_va = (unsigned char *)p_window->buffers[idx].ptr[0];
    p_buf->stride = p_window->stride[0];
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_allocate_input_buffers
///
/// @brief Allocate buffers for injection as input to qcarcam
///
/// @param buffers          Pointer to qcarcam buffers structure
/// @param size             size to allocate
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_allocate_input_buffers(qcarcam_buffers_t *p_buffers, uint32 size)
{
    uint32 i = 0;
    qcarcam_ret_t rc = QCARCAM_RET_OK;

    for (i = 0; i < p_buffers->n_buffers; i++)
    {
        p_buffers->buffers[i].planes[0].p_buf = pmem_malloc((size_t)(size), PMEM_CAMERA_ID);

        if (p_buffers->buffers[i].planes[0].p_buf)
        {
            p_buffers->buffers[i].planes[0].size = size;
        }
        else
        {
            QCARCAM_ERRORMSG("failed to allocate qcarcam input buffers");
            rc = QCARCAM_RET_FAILED;
            break;
        }
    }

    if (rc != QCARCAM_RET_OK)
    {
        /* allocation failed-free any partially allocated buffers*/
        for (i = 0; i < p_buffers->n_buffers; i++)
        {
            if (p_buffers->buffers[i].planes[0].p_buf)
            {
                pmem_free(p_buffers->buffers[i].planes[0].p_buf);
            }
        }
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_read_input_data
///
/// @brief Read input data into buffer
///
/// @param buffers          Pointer to qcarcam buffers structure
/// @param idx              Index of buffer to read data into
/// @param filename         Path to data file to be read
/// @param size             Size to read from file
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_read_input_data(qcarcam_buffers_t *p_buffers, unsigned int idx, const char *filename, uint32 size)
{
    FILE *fp;
    size_t numByteToRead = size;
    unsigned char *pBuf = 0;

    fp = fopen(filename, "r");

    QCARCAM_ERRORMSG("reading file %s numBytes %d", filename, size);

    if (0 != fp)
    {
        pBuf = (unsigned char *)p_buffers->buffers[idx].planes[0].p_buf;
        fread(pBuf, 1, numByteToRead, fp);
        fclose(fp);
    }
    else
    {
        QCARCAM_ERRORMSG("failed to open file");
        return QCARCAM_RET_FAILED;
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_deinit_window_buffer
///
/// @brief Destroy window buffers
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_deinit_window_buffer(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window)
{
    int i;

    if(p_ctxt == NULL || p_window == NULL)
    {
        return QCARCAM_RET_BADPARAM;
    }

    if(p_ctxt->params.disable_display || p_window->is_offscreen)
    {
        /* De allocate buffers if created directly from PMEM */
        for (i = 0; i < p_window->n_buffers; i++)
        {
            if (p_window->buffers[i].ptr[0])
            {
                int rc = 0;
                if (0 != (rc = pmem_free(p_window->buffers[i].ptr[0])))
                {
                    QCARCAM_ERRORMSG("pmem_free failed: rc=%d, n_buf=%d", rc, i);
                }
            }
        }
    }

    if(p_window->screen_bufs != NULL)
    {
        free(p_window->screen_bufs);
        p_window->screen_bufs = NULL;
    }

    if(p_window->buffers != NULL)
    {
#ifndef C2D_DISABLED
        if (p_ctxt->params.enable_c2d)
        {
            for (i = 0; i < p_window->n_pointers; i++)
            {
                c2dDestroySurface(p_window->buffers[i].c2d_surface_id);
                p_window->buffers[i].c2d_surface_id = 0;
            }
        }
#endif

        free(p_window->buffers);
        p_window->buffers = NULL;
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_deinit_window
///
/// @brief Destroy window
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_deinit_window(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window)
{
    (void)p_ctxt;

    if (p_window)
    {
        if (p_window->screen_win != NULL)
        {
            screen_destroy_window(p_window->screen_win);
            p_window->screen_win = NULL;
        }
        free(p_window);
    }
    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_deinit
///
/// @brief Destroy context and free memory.
///
/// @param ctxt   Pointer to context to be destroyed
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_deinit(test_util_ctxt_t *p_ctxt)
{
    qcarcam_ret_t rc = QCARCAM_RET_OK;

    g_aborted = 1;

    if (p_ctxt)
    {
        if (p_ctxt->screen_ctxt != NULL)
        {
            screen_destroy_context(p_ctxt->screen_ctxt);
            p_ctxt->screen_ctxt = NULL;
        }
        if (p_ctxt->screen_display)
        {
            free(p_ctxt->screen_display);
            p_ctxt->screen_display = NULL;
        }
        if (p_ctxt->display_property)
        {
            free(p_ctxt->display_property);
            p_ctxt->display_property = NULL;
        }

        if (p_ctxt->params.enable_gpio_config)
        {
            g_fd_gpio = gpio_close(NULL);
            if (g_fd_gpio == -1)
            {
                QCARCAM_ERRORMSG("gpio_close() failed!");
                rc = QCARCAM_RET_FAILED;
            }
        }

        free(p_ctxt);
    }

    return rc;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_create_c2d_surface
///
/// @brief Create a C2D surface
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param idx              Frame ID number
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_create_c2d_surface(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, unsigned int idx)
{
#ifndef C2D_DISABLED
    if(!p_ctxt->params.disable_display && !p_window->is_offscreen)
    {
        if (SCREEN_FORMAT_UYVY == p_window->screen_format)
        {
            C2D_STATUS c2d_status;
            C2D_YUV_SURFACE_DEF c2d_yuv_surface_def;
            c2d_yuv_surface_def.format = C2D_COLOR_FORMAT_422_UYVY;
            c2d_yuv_surface_def.width = p_window->buffer_size[0];
            c2d_yuv_surface_def.height = p_window->buffer_size[1];
            c2d_yuv_surface_def.stride0 = p_window->stride[0];
            c2d_yuv_surface_def.plane0 = p_window->buffers[idx].ptr[0];
            c2d_yuv_surface_def.phys0 = (void *)p_window->buffers[idx].phys_addr;
            c2d_yuv_surface_def.offset0 = 0;
            c2d_yuv_surface_def.offset1 = 0;
            c2d_yuv_surface_def.offset2 = 0;
            c2d_yuv_surface_def.handlep = NULL;
            c2d_status = c2dCreateSurface(&p_window->buffers[idx].c2d_surface_id,
                                        C2D_SOURCE | C2D_TARGET,
                                        (C2D_SURFACE_TYPE)(C2D_SURFACE_YUV_HOST | C2D_SURFACE_WITH_PHYS),
                                        &c2d_yuv_surface_def);
            if (c2d_status != C2D_STATUS_OK)
            {
                QCARCAM_ERRORMSG("c2dCreateSurface %d buf %d failed %d", 0, idx, c2d_status);
                return QCARCAM_RET_FAILED;
            }
        }
        else
        {
            C2D_STATUS c2d_status;
            C2D_RGB_SURFACE_DEF c2d_rgb_surface_def;
            c2d_rgb_surface_def.format = test_util_get_c2d_format(p_window->screen_format);
            c2d_rgb_surface_def.width = p_window->buffer_size[0];
            c2d_rgb_surface_def.height = p_window->buffer_size[1];
            c2d_rgb_surface_def.stride = p_window->stride[0];
            c2d_rgb_surface_def.buffer = p_window->buffers[idx].ptr[0];
            c2d_rgb_surface_def.phys = (void *)p_window->buffers[idx].phys_addr;
            c2d_status = c2dCreateSurface(&p_window->buffers[idx].c2d_surface_id,
                                        C2D_SOURCE | C2D_TARGET,
                                        (C2D_SURFACE_TYPE)(C2D_SURFACE_RGB_HOST | C2D_SURFACE_WITH_PHYS),
                                        &c2d_rgb_surface_def);

            if (c2d_status != C2D_STATUS_OK)
            {
                QCARCAM_ERRORMSG("c2dCreateSurface %d buf %d failed %d", 0, idx, c2d_status);
                return QCARCAM_RET_FAILED;
            }
        }
    }
#endif
    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_get_c2d_surface_id
///
/// @brief Get the ID from a C2D surface
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param idx              Frame ID number
/// @param surface_id       Pointer to C2D sruface ID number
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_get_c2d_surface_id(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, unsigned int idx, unsigned int *p_surface_id)
{
    if (!p_surface_id)
        return QCARCAM_RET_BADPARAM;

    *p_surface_id = p_window->buffers[idx].c2d_surface_id;

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_set_window_param
///
/// @brief Send window parameters to display
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param window_params    Pointer to structure with window properties
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_set_window_param(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, test_util_window_param_t *p_window_params)
{
    int rc;
    int disp_idx = p_window_params->display_id;
    int val = SCREEN_USAGE_WRITE | SCREEN_USAGE_VIDEO | SCREEN_USAGE_CAPTURE;

    p_window->format = p_window_params->format;
    p_window->is_offscreen = p_window_params->is_offscreen;

    if (!p_ctxt->params.disable_display && !p_window->is_offscreen)
    {
        rc = screen_create_window(&p_window->screen_win, p_ctxt->screen_ctxt);
        if (rc)
        {
            perror("screen_create_window");
            return QCARCAM_RET_FAILED;
        }

        if (disp_idx >= p_ctxt->screen_ndisplays)
        {
            QCARCAM_ERRORMSG("display idx %d exceeds max number of displays[%d]",
            disp_idx, p_ctxt->screen_ndisplays);
            return QCARCAM_RET_FAILED;
        }

        // Set ID string for debugging via /dev/screen.
        rc = screen_set_window_property_cv(p_window->screen_win, SCREEN_PROPERTY_ID_STRING,
                                        strlen(p_window_params->debug_name), p_window_params->debug_name);
        if (rc)
        {
            perror("screen_set_window_property_cv(SCREEN_PROPERTY_ID_STRING)");
            return QCARCAM_RET_FAILED;
        }

        if (p_window_params->pipeline_id != -1)
        {
            QCARCAM_ERRORMSG("USING SCREEN_USAGE_OVERLAY!!!!");
            val |= SCREEN_USAGE_OVERLAY;
        }

        rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_USAGE, &val);
        if (rc)
        {
            perror("screen_set_window_property_iv(SCREEN_PROPERTY_USAGE)");
            return QCARCAM_RET_FAILED;
        }

        p_window->screen_format = test_util_get_screen_format(p_window->format);

        p_window->buffer_size[0] = p_window_params->buffer_size[0];
        p_window->buffer_size[1] = p_window_params->buffer_size[1];

        p_window->params.size[0] = p_window_params->window_size[0] * p_ctxt->display_property[disp_idx].size[0];
        p_window->params.size[1] = p_window_params->window_size[1] * p_ctxt->display_property[disp_idx].size[1];
        p_window->params.pos[0] = p_window_params->window_pos[0] * p_ctxt->display_property[disp_idx].size[0];
        p_window->params.pos[1] = p_window_params->window_pos[1] * p_ctxt->display_property[disp_idx].size[1];
        p_window->params.ssize[0] = p_window_params->window_source_size[0] * p_window->buffer_size[0];
        p_window->params.ssize[1] = p_window_params->window_source_size[1] * p_window->buffer_size[1];
        p_window->params.spos[0] = p_window_params->window_source_pos[0] * p_window->buffer_size[0];
        p_window->params.spos[1] = p_window_params->window_source_pos[1] * p_window->buffer_size[1];

        p_window->params.visibility = p_window_params->visibility;

        /*associate window with display */
        rc = screen_set_window_property_pv(p_window->screen_win, SCREEN_PROPERTY_DISPLAY,
                                        (void **)&p_ctxt->screen_display[disp_idx]);
        if (rc)
        {
            perror("screen_set_window_property_ptr(SCREEN_PROPERTY_DISPLAY)");
            free(p_ctxt->screen_display);
            return QCARCAM_RET_FAILED;
        }
        rc = screen_set_window_property_pv(p_window->screen_win, SCREEN_PROPERTY_DISPLAY,
                                        (void **)&p_ctxt->screen_display[disp_idx]);
        if (rc)
        {
            perror("screen_set_window_property_ptr(SCREEN_PROPERTY_DISPLAY)");
            free(p_ctxt->screen_display);
            return QCARCAM_RET_FAILED;
        }

        /*display size*/
        if (p_window->params.size[0] == -1 || p_window->params.size[1] == -1)
        {
            rc = screen_get_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SIZE, p_window->params.size);
            if (rc)
            {
                perror("screen_get_window_property_iv(SCREEN_PROPERTY_SIZE)");
                return QCARCAM_RET_FAILED;
            }
        }

        rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SIZE, p_window->params.size);
        if (rc)
        {
            perror("screen_set_window_property_iv(SCREEN_PROPERTY_SIZE)");
            return QCARCAM_RET_FAILED;
        }

        /*display position*/
        if (p_window->params.pos[0] != 0 || p_window->params.pos[1] != 0)
        {
            rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_POSITION, p_window->params.pos);

            if (rc)
            {
                perror("screen_set_window_property_iv(SCREEN_PROPERTY_POSITION)");
                return QCARCAM_RET_FAILED;
            }
        }

        if (p_window_params->pipeline_id != -1)
        {
            rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_PIPELINE, &p_window_params->pipeline_id);
            if (rc)
            {
                perror("screen_set_window_property_iv(SCREEN_PROPERTY_PIPELINE)");
                return QCARCAM_RET_FAILED;
            }
        }

        if (p_window_params->zorder != -1)
        {
            rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_ZORDER, &p_window_params->zorder);
            if (rc)
            {
                perror("screen_set_window_property_iv(SCREEN_PROPERTY_ZORDER)");
                return QCARCAM_RET_FAILED;
            }
        }

        if (p_window->params.ssize[0] == -1 || p_window->params.ssize[1] == -1 ||
            (p_window->params.ssize[0] + p_window->params.spos[0]) > p_window->buffer_size[0] ||
            (p_window->params.ssize[1] + p_window->params.spos[1]) > p_window->buffer_size[1])
        {
            QCARCAM_INFOMSG("adjusting viewport size from %d x %d ", p_window->params.ssize[0], p_window->params.ssize[1]);

            p_window->params.ssize[0] = p_window->buffer_size[0] - p_window->params.spos[0];
            p_window->params.ssize[1] = p_window->buffer_size[1] - p_window->params.spos[1];

            QCARCAM_INFOMSG("to %d x %d\n", p_window->params.ssize[0], p_window->params.ssize[1]);
        }

        rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_SIZE, p_window->params.ssize);
        if (rc)
        {
            perror("screen_set_window_property_iv(SCREEN_PROPERTY_SOURCE_SIZE)");
            return QCARCAM_RET_FAILED;
        }

        QCARCAM_INFOMSG("window_source_position %d x %d\n", p_window->params.spos[0], p_window->params.spos[1]);

        if (p_window->params.spos[0] != 0 || p_window->params.spos[1] != 0)
        {
            rc = screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_SOURCE_POSITION, p_window->params.spos);
            if (rc)
            {
                perror("screen_set_window_property_iv(SCREEN_PROPERTY_SOURCE_POSITION)");
                return QCARCAM_RET_FAILED;
            }
        }
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_set_diag
///
/// @brief set the diagnostic structure to test_util_window_t
///
/// @param ctxt             Pointer to display context
/// @param user_ctxt        Pointer to structure containing window parameters
/// @param diag             diagnostic structure
///
/// @return Void
///////////////////////////////////////////////////////////////////////////////
void test_util_set_diag(test_util_ctxt_t *p_ctxt, test_util_window_t *p_window, test_util_diag_t* p_diag)
{
    (void)p_ctxt;
    (void)p_window;
    (void)p_diag;
}

///////////////////////////////////////////////////////////////////////////////
/// gpio_interrupt_thread
///
/// @brief thread which toggles the visibility when gpio interrupt detected
///
/// @param arguments                arguments for the thread to handle
///
/// @return 0 when thread has finished running
///////////////////////////////////////////////////////////////////////////////
static int gpio_interrupt_thread(void *p_data)
{
    pthread_detach(pthread_self());

    test_util_intr_thrd_args_t *p_args = (test_util_intr_thrd_args_t*)p_data;
    uint32_t irq = p_args->irq;

    struct sigevent int_event;

    // Attach Event ISR
    SIGEV_INTR_INIT(&int_event);
    int interrupt_id = InterruptAttachEvent(irq, &int_event, _NTO_INTR_FLAGS_TRK_MSK);

    if (interrupt_id == -1)
    {
        QCARCAM_ERRORMSG("InterruptAttach failed!");
        goto exit_interrupt_thread;
    }

    int status;
    while (!g_aborted)
    {
        status = InterruptWait_r(0, NULL);

        if (status != EOK)
        {
            QCARCAM_ERRORMSG("InterruptWait_r failed with error %d", status);
            break;
        }

        if (InterruptUnmask(irq, interrupt_id) == -1)
        {
            QCARCAM_ERRORMSG("InterruptUnmask failed!");
            break;
        }

        p_args->cb_func();
    }

exit_interrupt_thread:
    return 0;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_gpio_interrupt_config
///
/// @brief enable IO privileges, configure the gpio and set it up for interrupts
///
/// @param intr             Pointer for the IRQ to be stored
/// @param gpio_number      Specific gpio that is being utilized
/// @param trigger          Instance of the signal which shall causes the interrupt
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_gpio_interrupt_config(uint32_t *p_intr, int gpio_number, test_util_trigger_type_t trigger)
{
    if (ThreadCtl(_NTO_TCTL_IO, 0) == -1)
    {
        QCARCAM_ERRORMSG("Failed to get IO privileges!");
        return QCARCAM_RET_FAILED;
    }

    uint32_t cfg;
    uint32_t irq;

    if (gpio_number <= 0)
    {
        QCARCAM_ERRORMSG("Bad GPIO input param  gpio=%d", gpio_number);
        return QCARCAM_RET_FAILED;
    }

    cfg = 0x0;
    if (GPIO_SUCCESS != gpio_set_config(g_fd_gpio, gpio_number, 0, cfg))
    {
        QCARCAM_ERRORMSG("gpio_set_config failed for gpio gpio_number %d", gpio_number);
        return QCARCAM_RET_FAILED;
    }

    if (GPIO_SUCCESS != gpio_set_interrupt_cfg(g_fd_gpio, gpio_number, trigger, NULL))
    {
        QCARCAM_ERRORMSG("Failed to setup detect pin interrupt");
        return QCARCAM_RET_FAILED;
    }

    if (GPIO_SUCCESS != gpio_get_interrupt_cfg(g_fd_gpio, gpio_number, &irq))
    {
        QCARCAM_ERRORMSG("Failed to get irq corresponding to gpio %d", gpio_number);
        return QCARCAM_RET_FAILED;
    }
    else
    {
        *p_intr = irq;
        QCARCAM_INFOMSG("irq corresponding to gpio %d is %d", gpio_number, irq);
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_interrupt_attach
///
/// @brief create a thread to handle the interrupt
///
/// @param arguments    arguments to pass to the newly created thread
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_interrupt_attach(test_util_intr_thrd_args_t *p_args)
{
    char thread_name[64];
    CameraThread interrupt_thread_handle;
    int rc;

    snprintf(thread_name, sizeof(thread_name), "gpio_interrupt_thrd");
    rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &gpio_interrupt_thread, p_args, 0, thread_name, &interrupt_thread_handle);
    if (rc)
    {
        QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
        return QCARCAM_RET_FAILED;
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_interrupt_wait_and_unmask
///
/// @brief wait for a GPIO interrupt and then unmask it
///
/// @param irq              IRQ to unmask
/// @param interrupt_id     interrupt id to unmask
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_interrupt_wait_and_unmask(uint32_t irq, int interrupt_id)
{
    int status = InterruptWait_r(0, NULL);

    if (status != EOK)
    {
        QCARCAM_ERRORMSG("InterruptWait_r failed with error %d", status);
        return QCARCAM_RET_FAILED;
    }

    if (InterruptUnmask(irq, interrupt_id) == -1)
    {
        QCARCAM_ERRORMSG("InterruptUnmask failed!");
        return QCARCAM_RET_FAILED;
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_get_param
///
/// @brief get the value of the window parameter of the window
///
/// @param user_ctxt        window we want to use
/// @param param            window parameter you are trying to access
/// @param value            value of parameter will be stored here
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_get_param(test_util_window_t *p_window, test_util_params_t param, int *value)
{
    switch (param)
    {
    case TEST_UTIL_VISIBILITY:
        *value = p_window->params.visibility;
        break;
    default:
        QCARCAM_ERRORMSG("Param not supported");
        return QCARCAM_RET_FAILED;
    }

    return QCARCAM_RET_OK;
}

///////////////////////////////////////////////////////////////////////////////
/// test_util_set_param
///
/// @brief set the value of the window parameter
///
/// @param user_ctxt        window we want to use
/// @param param            window parameter you want to change
/// @param value            value you want to set the param to
///
/// @return QCARCAM_RET_OK if successful
///////////////////////////////////////////////////////////////////////////////
qcarcam_ret_t test_util_set_param(test_util_window_t *p_window, test_util_params_t param, int value)
{
    switch (param)
    {
    case TEST_UTIL_VISIBILITY:
        p_window->params.visibility = value;
        screen_set_window_property_iv(p_window->screen_win, SCREEN_PROPERTY_VISIBLE, &(value));
        break;
    default:
        QCARCAM_ERRORMSG("Param not supported");
        return QCARCAM_RET_FAILED;
    }

    return QCARCAM_RET_OK;
}
