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
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <sys/select.h>
#include <queue>

#ifndef C2D_DISABLED
#include "c2d2.h"
#endif

#ifdef USE_VENDOR_EXT_PARAMS
#include "vendor_ext_properties.h"
#endif

#include "qcarcam.h"

#include "test_util.h"
#include "qcarcam_diag_types.h"

#define NUM_MAX_CAMERAS 28
#define NUM_MAX_DISP_BUFS 3

/*1sec delay before restart */
#define PAUSE_RESUME_USLEEP 1000000
#define START_STOP_USLEEP 1000000

/*print input state as frozen if start and no frames after 1 sec*/
#define QCARCAMTEST_SOF_FREEZE_TIMEOUT 1.0f

#define QCARCAM_TEST_DEFAULT_GET_FRAME_TIMEOUT 500000000
#define DEFAULT_PRINT_DELAY_SEC 10
#define SIGNAL_CHECK_DELAY 33333;
#define CSI_ERR_CHECK_DELAY 100000;
#define NS_TO_MS 0.000001F

#define QCARCAM_TEST_INPUT_INJECTION 11
#define BUFSIZE 10

#define SIGWAIT_TIMEOUT_MS 100
#define TIMER_THREAD_USLEEP 1000000

#define USR_PROCESS_THREAD_USLEEP 1000
#define USR_PROCESS_WAIT_USLEEP 1000

#if defined(__INTEGRITY)
#ifndef CLOCK_MONOTONIC
#define CLOCK_MONOTONIC CLOCK_REALTIME
#endif
extern "C" const Value __PosixServerPriority = CAM_POSIXSERVER_PRIORITY;
#endif

#define SET_BIT(num, nbit)   ((num) |=  (0x1<<(nbit)))
#define CHECK_BIT(num, nbit) ((num) & (0x1<<(nbit)))

#define QCARCAMTEST_BUFFER_STALE_BIT  4

typedef enum
{
    QCARCAMTEST_BUFFER_STATE_INIT,
    QCARCAMTEST_BUFFER_STATE_QCARCAM,
    QCARCAMTEST_BUFFER_STATE_GET_FRAME,
    QCARCAMTEST_BUFFER_STATE_USR_PROCESS,
    QCARCAMTEST_BUFFER_STATE_USR_PROCESS_DONE,
    QCARCAMTEST_BUFFER_STATE_POST_DISPLAY,
    QCARCAMTEST_BUFFER_STATE_MAX = 0x7FFFFFF
}qcarcam_test_buffer_state_t;

typedef enum
{
    QCARCAM_TEST_BUFFERS_OUTPUT = 0,
    QCARCAM_TEST_BUFFERS_DISPLAY,
    QCARCAM_TEST_BUFFERS_INPUT,
    QCARCAM_TEST_BUFFERS_MAX
} qcarcam_test_buffers_t;

typedef struct
{
    qcarcam_buffers_t p_buffers;

    qcarcam_color_fmt_t format;
    unsigned int n_buffers;
    unsigned int width;
    unsigned int height;
} qcarcam_buffers_param_t;

typedef enum
{
    QCARCAMTEST_STATE_INVALID = 0,
    QCARCAMTEST_STATE_INIT,
    QCARCAMTEST_STATE_OPEN,
    QCARCAMTEST_STATE_START,
    QCARCAMTEST_STATE_STOP,
    QCARCAMTEST_STATE_PAUSE,
    QCARCAMTEST_STATE_PAUSE_STOP_PENDING,
    QCARCAMTEST_STATE_ERROR,
    QCARCAMTEST_STATE_CLOSED,
}qcarcamtest_state_t;

typedef struct
{
    uint32 buf_idx;
    void *p_data;
}timer_usr_data;

typedef struct
{
    timer_usr_data usr_data;
    CameraTimer ptimer;
}qcarcam_test_buf_timer;

typedef struct
{
    qcarcam_event_t event_id;
    qcarcam_event_payload_t payload;
} qcarcam_event_msg_t;

typedef struct
{
    CameraThread thread_handle;
    CameraThread process_cb_event_handle;
    CameraSignal m_eventHandlerSignal;

    unsigned int idx;
    int query_inputs_idx;

    pthread_mutex_t mutex;
    qcarcamtest_state_t state;
    bool is_fatal_error;

    /*qcarcam context*/
    qcarcam_hndl_t qcarcam_context;
    qcarcam_input_desc_t qcarcam_input_id;

    qcarcam_buffers_param_t buffers_param[QCARCAM_TEST_BUFFERS_MAX];

    qcarcam_buffers_t p_buffers_output;
    qcarcam_buffers_t p_buffers_disp;
    qcarcam_buffers_t p_buffers_input;

    qcarcam_res_t resolution;

    unsigned long long int frame_timeout;
    int use_event_callback;
    qcarcam_opmode_type op_mode;

    /* test util objects */
    test_util_ctxt_t *test_util_ctxt;
    test_util_window_t *qcarcam_window;
    test_util_window_t *display_window;
    test_util_window_param_t window_params;

    /* buffer management tracking */
    int get_frame_buf_idx;
    int buf_idx_qcarcam;
    std::list<uint32> release_buf_idx;

    int buf_idx_disp;
    int prev_buf_idx_disp;

    /* diag */
    int frameCnt;
    int releaseframeCnt;
    int prev_frameCnt;
    int prev_releaseframeCnt;
    bool is_first_start;
    bool is_injection;
    unsigned long long t_start; //start command
    unsigned long long t_start_success;
    unsigned long long t_firstFrame; //first frame time
    unsigned long long t_before;
    unsigned long long t_after;
    test_util_diag_t diag;
    bool dumpNextFrame;

    /* Exposure values */
    float exp_time;
    float gain;
    int manual_exposure;

    /* frame rate parameters */
    qcarcam_frame_rate_t frame_rate_config;

    qcarcam_test_buffer_state_t buf_state[QCARCAM_MAX_NUM_BUFFERS];

    bool skip_post_display;
    qcarcam_field_t field_type_previous;

    bool signal_lost;
    int fatal_err_cnt;

#ifdef ENABLE_CL_CONVERTER
    void* g_converter = NULL;
    ClConverter_surface_t source_surface;
    ClConverter_surface_t target_surface;
#endif

    /* subscription for changed setting events notification */
    uint64 subscribe_parameter_change;
    bool is_master;

    /* user process */
    uint32 delay_time;
    qcarcam_test_buf_timer buf_timer[QCARCAM_MAX_NUM_BUFFERS];

    std::list<uint32> usr_process_buf_idx;
    int usr_process_frameCnt;
    CameraThread usr_process_thread_handle;
    std::queue<qcarcam_event_msg_t> eventqueue;
    pthread_mutex_t queue_mutex;
} qcarcam_test_input_t;

typedef struct
{
    int numInputs;
    qcarcam_test_input_t inputs[NUM_MAX_CAMERAS];
    int opened_stream_cnt;

    /* 0 : post buffers directly to screen
     * 1 : blit buffers to display buffers through c2d
     */
    int enable_c2d;
#ifndef C2D_DISABLED
    pthread_mutex_t mutex_c2d;
#endif


    int dumpFrame;
    int enablePauseResume;
    int enableStartStop;
    int multithreaded;
    int enableStats;
    int enableMenuMode;
    int enableBridgeErrorDetect;
    int enableFatalErrorRecover;
    int enableIFEOverflowhandle;
    int enableRetry;
    int gpioNumber;
    int gpioMode;
    int disable_display;
    int enable_csc;

    int exitSeconds;

    int fps_print_delay;
    int vis_value;
    int check_buffer_state;

    /*abort condition*/
    pthread_mutex_t mutex_abort;
    pthread_cond_t cond_abort;
    pthread_mutex_t mutex_csi_err;
    pthread_mutex_t mutex_open_cnt;

    unsigned long long t_start; //program start
    int enable_deinterlace;
} qcarcam_test_ctxt_t;

typedef enum
{
    QCARCAM_TEST_MENU_FIRST_ITEM = 1,
    QCARCAM_TEST_MENU_STREAM_OPEN = QCARCAM_TEST_MENU_FIRST_ITEM,
    QCARCAM_TEST_MENU_STREAM_CLOSE,
    QCARCAM_TEST_MENU_STREAM_STOP,
    QCARCAM_TEST_MENU_STREAM_START,
    QCARCAM_TEST_MENU_STREAM_PAUSE,
    QCARCAM_TEST_MENU_STREAM_RESUME,
    QCARCAM_TEST_MENU_STREAM_STOP_ALL,
    QCARCAM_TEST_MENU_STREAM_START_ALL,
    QCARCAM_TEST_MENU_STREAM_PAUSE_ALL,
    QCARCAM_TEST_MENU_STREAM_RESUME_ALL,
    QCARCAM_TEST_MENU_STREAM_ENABLE_CALLBACK,
    QCARCAM_TEST_MENU_STREAM_DISABLE_CALLBACK,
    QCARCAM_TEST_MENU_STREAM_SET_FRAMERATE,
    QCARCAM_TEST_MENU_STREAM_SET_EXPOSURE,
    QCARCAM_TEST_MENU_STREAM_SET_COLOR_PARAM,
    QCARCAM_TEST_MENU_STREAM_GET_COLOR_PARAM,
    QCARCAM_TEST_MENU_STREAM_SET_GAMMA_PARAM,
    QCARCAM_TEST_MENU_STREAM_GET_GAMMA_PARAM,
    QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM,
    QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM,
    QCARCAM_TEST_MENU_DUMP_NEXT_FRAME,
    QCARCAM_TEST_MENU_CHECK_BUFFERS,
    QCARCAM_TEST_MENU_STREAM_SET_VENDOR_PARAM,
    QCARCAM_TEST_MENU_STREAM_GET_VENDOR_PARAM,
    QCARCAM_TEST_MENU_STREAM_SUBSCRIBE_CHANGE_EVENT,
    QCARCAM_TEST_MENU_STREAM_UNSUBSCRIBE_CHANGE_EVENT,
    QCARCAM_TEST_MENU_MASTER,
    QCARCAM_TEST_MENU_GET_SYSTEM_DIAGNOSTICS,
    QCARCAM_TEST_MENU_MAX
}qcarcam_test_menu_option_t;

typedef struct
{
    qcarcam_test_menu_option_t id;
    const char* str;
}qcarcam_test_menu_t;

typedef struct
{
    qcarcam_isp_param_t id;
    const char* str;
}qcarcam_isp_menu_t;

static qcarcam_isp_menu_t g_qcarcam_isp_menu[QCARCAM_ISP_PARAM_NUM] =
{
    {QCARCAM_CONTROL_AE_LOCK,  "AE Lock"},
    {QCARCAM_CONTROL_AE_MODE,  "AE Mode"},
    {QCARCAM_CONTROL_AWB_LOCK,  "AWB Lock"},
    {QCARCAM_CONTROL_AWB_MODE,  "AWB Mode"},
    {QCARCAM_CONTROL_EFFECT_MODE,  "Effect Mode"},
    {QCARCAM_CONTROL_MODE,  "Control Mode"},
    {QCARCAM_CONTROL_SCENE_MODE,  "Scene Mode"},
    {QCARCAM_CONTROL_AE_ANTIBANDING_MODE,  "AE antibanding Mode"},
};

static qcarcam_test_menu_t g_qcarcam_menu[QCARCAM_TEST_MENU_MAX] =
{
    {},
    {QCARCAM_TEST_MENU_STREAM_OPEN,  "Open a stream"},
    {QCARCAM_TEST_MENU_STREAM_CLOSE, "Close a stream"},
    {QCARCAM_TEST_MENU_STREAM_STOP,  "Stop a stream"},
    {QCARCAM_TEST_MENU_STREAM_START, "Start a stream"},
    {QCARCAM_TEST_MENU_STREAM_PAUSE, "Pause a stream"},
    {QCARCAM_TEST_MENU_STREAM_RESUME, "Resume a stream"},
    {QCARCAM_TEST_MENU_STREAM_STOP_ALL, "Stop all streams"},
    {QCARCAM_TEST_MENU_STREAM_START_ALL, "Start all streams"},
    {QCARCAM_TEST_MENU_STREAM_PAUSE_ALL, "Pause all streams"},
    {QCARCAM_TEST_MENU_STREAM_RESUME_ALL, "Resume all streams"},
    {QCARCAM_TEST_MENU_STREAM_ENABLE_CALLBACK, "Enable callback"},
    {QCARCAM_TEST_MENU_STREAM_DISABLE_CALLBACK, "Disable callback"},
    {QCARCAM_TEST_MENU_STREAM_SET_FRAMERATE, "Set frame rate control"},
    {QCARCAM_TEST_MENU_STREAM_SET_EXPOSURE, "Set exposure"},
    {QCARCAM_TEST_MENU_STREAM_SET_COLOR_PARAM, "Set color param"},
    {QCARCAM_TEST_MENU_STREAM_GET_COLOR_PARAM, "Get color param"},
    {QCARCAM_TEST_MENU_STREAM_SET_GAMMA_PARAM, "Set Gamma Table"},
    {QCARCAM_TEST_MENU_STREAM_GET_GAMMA_PARAM, "Get Gamma Table"},
    {QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM, "Set ISP settings"},
    {QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM, "Get ISP settings"},
    {QCARCAM_TEST_MENU_DUMP_NEXT_FRAME, "Dump Next Frame"},
    {QCARCAM_TEST_MENU_CHECK_BUFFERS, "Check Buffers"},
    {QCARCAM_TEST_MENU_STREAM_SET_VENDOR_PARAM, "Set Vendor Param"},
    {QCARCAM_TEST_MENU_STREAM_GET_VENDOR_PARAM, "Get Vendor Param"},
    {QCARCAM_TEST_MENU_STREAM_SUBSCRIBE_CHANGE_EVENT, "Subscribe for an event"},
    {QCARCAM_TEST_MENU_STREAM_UNSUBSCRIBE_CHANGE_EVENT, "Unsubscribe for an event"},
    {QCARCAM_TEST_MENU_MASTER, "Set/Release a client as master"},
    {QCARCAM_TEST_MENU_GET_SYSTEM_DIAGNOSTICS, "Get System diagnostic info"},
};

///////////////////////////////
/// STATICS
///////////////////////////////
static qcarcam_test_ctxt_t gCtxt = {};

static char g_filename[128] = "qcarcam_config.xml";

static volatile int g_aborted = 0;

static sigset_t g_sigset;

static const int exceptsigs[] = {
    SIGCHLD, SIGIO, SIGURG, SIGWINCH,
    SIGTTIN, SIGTTOU, SIGCONT, SIGSEGV,
    -1,
};

static void initialize_qcarcam_test_ctxt(void)
{
    gCtxt.numInputs = 0;
    gCtxt.opened_stream_cnt = 0;

    gCtxt.enable_c2d = 0;
#ifndef C2D_DISABLED
    pthread_mutex_init(&gCtxt.mutex_c2d, NULL);
#endif

    gCtxt.dumpFrame = 0;
    gCtxt.enablePauseResume = 0;
    gCtxt.enableStartStop = 0;
    gCtxt.multithreaded = 1;
    gCtxt.enableStats = 1;
    gCtxt.enableMenuMode = 1;
    gCtxt.enableBridgeErrorDetect = 1;
    gCtxt.enableFatalErrorRecover = 0;
    gCtxt.enableIFEOverflowhandle = 1;
    gCtxt.enableRetry = 0;
    gCtxt.disable_display = 0;
    gCtxt.enable_csc = 0;

    gCtxt.exitSeconds = 0;
    gCtxt.gpioNumber = 0;
    gCtxt.vis_value = 1;

    gCtxt.fps_print_delay = DEFAULT_PRINT_DELAY_SEC;
    gCtxt.check_buffer_state = 0;

    pthread_mutex_init(&gCtxt.mutex_abort, NULL);
    pthread_cond_init(&gCtxt.cond_abort, NULL);
    pthread_mutex_init(&gCtxt.mutex_csi_err, NULL);
    pthread_mutex_init(&gCtxt.mutex_open_cnt, NULL);

    gCtxt.t_start = 0;
    gCtxt.enable_deinterlace = 0;
}

static void display_valid_stream_ids()
{
    int i;
    qcarcam_test_input_t *input_ctxt = NULL;

    printf("Valid stream ids are\n");
    printf("========================\n");
    printf("Camera id   stream id\n");
    printf("========================\n");
    for (i = 0; i < gCtxt.numInputs; ++i)
    {
        input_ctxt = &gCtxt.inputs[i];
        if (input_ctxt->qcarcam_context)
        {
            printf("%d          %d\n", input_ctxt->qcarcam_input_id, i);
        }
    }
    printf("========================\n");
}

/**
 * Returns user entered stream idx
 */
static int get_input_stream_id()
{
    int stream_id = gCtxt.numInputs;
    char buf[BUFSIZE];
    char *p = NULL;

    display_valid_stream_ids();

    printf("Enter stream id\n");

    if (fgets(buf, sizeof(buf), stdin) != NULL)
    {
        stream_id = strtol(buf, &p, 10);
    }

    return stream_id;
}

static void get_input_exposure(qcarcam_exposure_config_t* exposure_config)
{
    if (exposure_config != NULL)
    {
        char buf[BUFSIZE];

        printf("Enter camera exposure time value\n");
        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            exposure_config->exposure_time = strtof(buf, NULL);
        }

        printf("Enter camera gain value\n");
        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            exposure_config->gain = strtof(buf, NULL);
        }
    }
}

static void get_input_hdr_exposure(qcarcam_hdr_exposure_config_t* exposure_config)
{
    char buf[BUFSIZE];

#if 0
    printf("Enter lux idx\n");
    if (fgets(buf, sizeof(buf), stdin) != NULL)
    {
        exposure_config->lux_index = strtof(buf, NULL);
    }
#else
    int i = 0;
    for (i = 0; i < 3; i++)
    {
        printf("exp%d\n", i);
        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            exposure_config->exposure_time[i] = strtof(buf, NULL);
        }

        printf("gain%d\n", i);
        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            exposure_config->gain[i] = strtof(buf, NULL);
        }
    }
#endif
}

static void get_input_framerate(qcarcam_frame_rate_t* frame_rate_config)
{
    if (frame_rate_config != NULL)
    {
        printf("\n ====================================================== \n");
        printf(" '0'...Keep all frames          '1'...Keep every 2 frames \n");
        printf(" '2'...Keep every 3 frames      '3'...Keep every 4 frames \n");
        printf(" '4'...Drop all frames          '5'...Raw period pattern mode \n");
        printf("\n ====================================================== \n");
        printf(" Enter camera framerate mode\n");

        int frame_drop_mode = 0;
        int frame_drop_period = 0;
        int frame_drop_pattern = 0;

        char buf[BUFSIZE];

        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            frame_drop_mode = strtol(buf, NULL, 10);
        }

        if (QCARCAM_FRAMEDROP_MANUAL == frame_drop_mode)
        {
            printf("Enter period value, maximal 32\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                frame_drop_period = strtol(buf, NULL, 10);
            }
            printf("Enter pattern hex value, e.g. 0x23\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                frame_drop_pattern = strtol(buf, NULL, 16);
            }
        }
        frame_rate_config->frame_drop_mode = (qcarcam_frame_drop_mode_t)frame_drop_mode;
        frame_rate_config->frame_drop_period = (unsigned char)frame_drop_period;
        frame_rate_config->frame_drop_pattern = frame_drop_pattern;
    }
}

static int get_input_set_flag()
{
    printf("Enter set[1] release[0]");
    char buf[BUFSIZE];
    int flag = 0;
    char *p = NULL;
    if (fgets(buf, sizeof(buf), stdin) != NULL)
    {
        flag = strtol(buf, &p, 10);
    }
    return flag;
}

static int qcarcam_test_get_time(unsigned long long *pTime)
{
    struct timespec time;
    unsigned long long msec;

    if (clock_gettime(CLOCK_MONOTONIC, &time) == -1)
    {
        QCARCAM_ERRORMSG("Clock gettime failed");
        return 1;
    }
    msec = ((unsigned long long)time.tv_sec * 1000) + (((unsigned long long)time.tv_nsec / 1000) / 1000);
    *pTime = msec;

    return 0;
}

static void qcarcam_test_clear_usr_process_list(qcarcam_test_input_t *input_ctxt)
{
    pthread_mutex_lock(&input_ctxt->mutex);
    input_ctxt->state = QCARCAMTEST_STATE_PAUSE_STOP_PENDING;

    /* Wait user process frame in progress to finish */
    while (input_ctxt->usr_process_frameCnt)
    {
        pthread_mutex_unlock(&input_ctxt->mutex);
        usleep(USR_PROCESS_WAIT_USLEEP);
        pthread_mutex_lock(&input_ctxt->mutex);
    }

    /* reclaim user process frames in list */
    while (!input_ctxt->usr_process_buf_idx.empty())
    {
        uint32 idx = input_ctxt->usr_process_buf_idx.front();
        input_ctxt->usr_process_buf_idx.pop_front();
        input_ctxt->buf_state[idx] = QCARCAMTEST_BUFFER_STATE_GET_FRAME;
        input_ctxt->release_buf_idx.push_back(idx);
    }

    pthread_mutex_unlock(&input_ctxt->mutex);
}

static qcarcam_ret_t qcarcam_input_start(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    qcarcam_test_get_time(&input_ctxt->t_start);

    ret = qcarcam_start(input_ctxt->qcarcam_context);
    if (ret == QCARCAM_RET_OK)
    {
        input_ctxt->state = QCARCAMTEST_STATE_START;

        input_ctxt->frameCnt = 0;
        input_ctxt->releaseframeCnt = 0;
        input_ctxt->prev_frameCnt = 0;
        input_ctxt->prev_releaseframeCnt = 0;
        input_ctxt->signal_lost = 0;
        qcarcam_test_get_time(&input_ctxt->t_start_success);

        QCARCAM_INFOMSG("Client %d Input %d qcarcam_start successfully", input_ctxt->idx, input_ctxt->qcarcam_input_id);
    }
    else
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
        QCARCAM_ERRORMSG("Client %d Input %d qcarcam_start failed: %d", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
    }

    return ret;
}

static qcarcam_ret_t qcarcam_input_stop(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    ret = qcarcam_stop(input_ctxt->qcarcam_context);
    if (ret == QCARCAM_RET_OK)
    {
        input_ctxt->state = QCARCAMTEST_STATE_STOP;

        input_ctxt->frameCnt = 0;
        input_ctxt->releaseframeCnt = 0;
        input_ctxt->prev_frameCnt = 0;
        input_ctxt->prev_releaseframeCnt = 0;
        input_ctxt->release_buf_idx.clear();
        input_ctxt->usr_process_buf_idx.clear();

        QCARCAM_INFOMSG("Client %d Input %d qcarcam_stop successfully", input_ctxt->idx, input_ctxt->qcarcam_input_id);
    }
    else
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
        QCARCAM_ERRORMSG("Client %d Input %d qcarcam_stop failed: %d", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
    }

    memset(&input_ctxt->buf_state, 0x0, sizeof(input_ctxt->buf_state));

    return ret;
}

static qcarcam_ret_t qcarcam_input_pause(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    ret = qcarcam_pause(input_ctxt->qcarcam_context);
    if (ret == QCARCAM_RET_OK)
    {
        input_ctxt->state = QCARCAMTEST_STATE_PAUSE;

        input_ctxt->frameCnt = 0;
        input_ctxt->prev_frameCnt = 0;
        input_ctxt->releaseframeCnt = 0;
        input_ctxt->prev_releaseframeCnt = 0;

        QCARCAM_INFOMSG("Client %d Input %d qcarcam_pause successfully", input_ctxt->idx, input_ctxt->qcarcam_input_id);
    }
    else
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
        QCARCAM_ERRORMSG("Client %d Input %d qcarcam_pause failed: %d", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
    }

    return ret;
}

static qcarcam_ret_t qcarcam_input_resume(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    qcarcam_test_get_time(&input_ctxt->t_start);

    ret = qcarcam_resume(input_ctxt->qcarcam_context);
    if (ret == QCARCAM_RET_OK)
    {
        input_ctxt->state = QCARCAMTEST_STATE_START;

        input_ctxt->frameCnt = 0;
        input_ctxt->releaseframeCnt = 0;
        input_ctxt->prev_frameCnt = 0;
        input_ctxt->prev_releaseframeCnt = 0;
        input_ctxt->signal_lost = 0;
        qcarcam_test_get_time(&input_ctxt->t_start_success);

        QCARCAM_INFOMSG("Client %d Input %d qcarcam_resume success", input_ctxt->idx, input_ctxt->qcarcam_input_id);
    }
    else
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
        QCARCAM_ERRORMSG("Client %d Input %d qcarcam_resume failed %d", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
    }

    return ret;
}


static void qcarcam_test_check_buffers(qcarcam_test_input_t *input_ctxt)
{
    unsigned int i;
    unsigned int num_buffers = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers;

    for (i = 0; i < num_buffers; i++)
    {
        printf("|    |     | %d  0x%02x  ", i, input_ctxt->buf_state[i]);
        if (CHECK_BIT(input_ctxt->buf_state[i], QCARCAMTEST_BUFFER_STALE_BIT))
        {
            printf(" [stale]");
        }
        else
        {
            printf("        ");
        }
        printf("          |\n");

        input_ctxt->buf_state[i] = (qcarcam_test_buffer_state_t)(input_ctxt->buf_state[i] | (1 << QCARCAMTEST_BUFFER_STALE_BIT));
    }

    fflush(stdout);
}

static void qcarcam_test_get_frame_rate(unsigned long long timer_prev)
{
    float average_fps, average_rel_fps;
    int input_idx;
    unsigned long long timer_now = 0;

    qcarcam_test_get_time(&timer_now);

    printf("--------FPS Report - %.1f sec-----------\n", ((float)(timer_now - gCtxt.t_start) / 1000));
    printf("| id | qid |  state (time)| fps  | rel  |\n");

    for (input_idx = 0; input_idx < gCtxt.numInputs; ++input_idx)
    {
        qcarcam_test_input_t* input_ctxt = &gCtxt.inputs[input_idx];

        pthread_mutex_lock(&input_ctxt->mutex);

        qcarcam_test_get_time(&timer_now);

        printf("| %2u | %2u  | ", input_ctxt->idx, (unsigned int)input_ctxt->qcarcam_input_id);

        if (input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            if (!input_ctxt->prev_frameCnt)
            {
                average_fps = 0.0f;
                average_rel_fps = 0.0f;

                //use first frame time for first report if got a frame
                if (input_ctxt->frameCnt > 1)
                {
                    printf("ok      (%3.1f)", ((float)(timer_now - input_ctxt->t_firstFrame) / 1000));
                    average_fps = (input_ctxt->frameCnt-1) / ((float)(timer_now - input_ctxt->t_firstFrame) / 1000);
                    if (input_ctxt->releaseframeCnt > 1)
                    {
                        average_rel_fps = (input_ctxt->releaseframeCnt-1) / ((float)(timer_now - input_ctxt->t_firstFrame) / 1000);
                    }
                }
                else
                {
                    float time_since_start = ((float)(timer_now - input_ctxt->t_start) / 1000);
                    float time_since_start_success = ((float)(timer_now - input_ctxt->t_start_success) / 1000);
                    if (time_since_start_success > QCARCAMTEST_SOF_FREEZE_TIMEOUT)
                    {
                        //TODO: maybe can add freeze detection here to try and restart?
                        printf("freeze  (%3.1f)", time_since_start);
                    }
                    else
                    {
                        printf("started (%3.1f)", time_since_start);
                    }
                }
            }
            else
            {
                int frames_counted;
                int release_frames_counted;

                frames_counted = input_ctxt->frameCnt - input_ctxt->prev_frameCnt;
                release_frames_counted = input_ctxt->releaseframeCnt - input_ctxt->prev_releaseframeCnt;
                average_fps = frames_counted / ((float)(timer_now - timer_prev) / 1000);
                average_rel_fps = release_frames_counted / ((float)(timer_now - timer_prev) / 1000);

                if (frames_counted)
                {
                    printf("ok           ");
                }
                else
                {
                    //TODO: maybe can add freeze detection here to try and restart?
                    printf("freeze       ");
                }
            }

            printf("| %4.1f | %4.1f |\n", average_fps, average_rel_fps);

            input_ctxt->prev_frameCnt = input_ctxt->frameCnt;
            input_ctxt->prev_releaseframeCnt = input_ctxt->releaseframeCnt;

            if (gCtxt.check_buffer_state)
            {
                qcarcam_test_check_buffers(input_ctxt);
            }
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_STOP)
        {
            printf("stop         |      |      |\n");
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_PAUSE)
        {
            printf("pause        |      |      |\n");
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_ERROR)
        {
            printf("ERROR        |      |      |\n");
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_INIT)
        {
            printf("init         |      |      |\n");
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_PAUSE_STOP_PENDING)
        {
            printf("pending      |      |      |\n");
        }
        else
        {
            printf("UNKNOWN(%d)  |      |      |\n", input_ctxt->state);
        }

        pthread_mutex_unlock(&input_ctxt->mutex);
    }
    printf("-----------------------------------------\n");
    fflush(stdout);
}

static int test_signal_loss(qcarcam_test_input_t *input_ctxt, bool *signal_lost_check)
{
    if (input_ctxt->signal_lost != *signal_lost_check)
    {
        // Check if signal status has changed
        *signal_lost_check = input_ctxt->signal_lost;
    }
    else if (input_ctxt->signal_lost == 1)
    {
        // wait 1 cycle then post empty frame to display
        test_util_post_window_buffer(input_ctxt->test_util_ctxt,
                input_ctxt->qcarcam_window,
                input_ctxt->p_buffers_output.n_buffers,
                &input_ctxt->release_buf_idx,
                input_ctxt->field_type_previous);
    }

    return 0;
}

static int check_signal_loss_thread(void *arg)
{
    pthread_detach(pthread_self());

    bool signal_lost_check[NUM_MAX_CAMERAS] = {};
    unsigned int signal_check_delay_us = SIGNAL_CHECK_DELAY; // 33 milliseconds

    while (!g_aborted)
    {
        // Check if signal status has changed
        for (int i = 0; i < gCtxt.numInputs; i++)
        {
            test_signal_loss(&gCtxt.inputs[i], &signal_lost_check[i]);
        }
        usleep(signal_check_delay_us);
    }
    return 0;
}

static void test_fatal_error_check(qcarcam_test_input_t *input_ctxt, volatile int *csi_err_cnt_prev)
{
    if (input_ctxt->fatal_err_cnt != *csi_err_cnt_prev)
    {
        // error happened. Set it and wait 1 more iteration before take recovery action
        *csi_err_cnt_prev = input_ctxt->fatal_err_cnt;
        input_ctxt->is_fatal_error = 1;
    }
    else if (input_ctxt->is_fatal_error)
    {
        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            QCARCAM_ERRORMSG("Input %d already running again, return", input_ctxt->qcarcam_input_id);
        }
        else
        {
            (void)qcarcam_input_start(input_ctxt);
            input_ctxt->is_fatal_error = 0;
        }

        pthread_mutex_unlock(&input_ctxt->mutex);
    }
}

static int check_error_thread(void *arg)
{
    pthread_detach(pthread_self());

    volatile int fatal_error_prev[NUM_MAX_CAMERAS] = { 0 };
    unsigned int error_check_delay_us = CSI_ERR_CHECK_DELAY; // 100 milliseconds

    while (!g_aborted)
    {
        // Check if csi error continues or not
        for (int i = 0; i < gCtxt.numInputs; i++)
        {
            test_fatal_error_check(&gCtxt.inputs[i], &fatal_error_prev[i]);
        }
        usleep(error_check_delay_us);
    }
    return 0;
}

static int framerate_thread(void *arg)
{
    pthread_detach(pthread_self());

    unsigned int fps_print_delay_us = gCtxt.fps_print_delay * 1000000;
    unsigned long long timer1;

    while (!g_aborted)
    {
        qcarcam_test_get_time(&timer1);
        usleep(fps_print_delay_us);
        qcarcam_test_get_frame_rate(timer1);
    }
    return 0;
}

static void abort_test(void)
{
    QCARCAM_ERRORMSG("Aborting test");
    pthread_mutex_lock(&gCtxt.mutex_abort);
    g_aborted = 1;
    pthread_cond_broadcast(&gCtxt.cond_abort);
    pthread_mutex_unlock(&gCtxt.mutex_abort);
}

static int timer_thread(void *arg)
{
    pthread_detach(pthread_self());

    unsigned long long timer_start = 0;
    unsigned long long timer_test = 0;
    qcarcam_test_get_time(&timer_start);
    while (!g_aborted)
    {
        qcarcam_test_get_time(&timer_test);
        if ((timer_test - timer_start) >= ((unsigned long long)gCtxt.exitSeconds * 1000))
        {
            QCARCAM_ALWZMSG("TEST Aborted after running for %d secs successfully!", gCtxt.exitSeconds);
            abort_test();
            break;
        }
        usleep(TIMER_THREAD_USLEEP);
    }
    return 0;
}

static int signal_thread(void *arg)
{
    struct timespec timeout;

    pthread_detach(pthread_self());

    timeout.tv_sec = 0;
    timeout.tv_nsec = SIGWAIT_TIMEOUT_MS * 1000000;

    while (!g_aborted)
    {
        if (sigtimedwait(&g_sigset, NULL, &timeout) > 0)
        {
            abort_test();
            break;
        }
    }
    return 0;
}

static void process_deinterlace(qcarcam_test_input_t *input_ctxt, qcarcam_field_t field_type, int di_method)
{
    test_util_sw_di_t di_info;

    di_info.qcarcam_window = input_ctxt->qcarcam_window;
    di_info.display_window = input_ctxt->display_window;
    di_info.source_buf_idx = input_ctxt->frameCnt;
    di_info.field_type = field_type;

    switch (di_method) {
    case SW_WEAVE_30FPS:
        /* sw weave 30fps method deinterlacing */
        test_util_di_sw_weave_30fps(&di_info);
        break;
    case SW_BOB_30FPS:
    case SW_BOB_60FPS:
        /* needn't process field into new buffer, display each field in post_window_buffer directly */
        break;
    default:
        /* unsupported deinterlacing method */
        QCARCAM_ERRORMSG("Unknown deinterlacing method");
        break;
    }
}

/**
 * Function to retrieve frame from qcarcam and increase frame_counter
 * @param input_ctxt
 * @return 0 on success, -1 on failure
 */
static int qcarcam_test_get_frame(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret;
    qcarcam_frame_info_t frame_info;
    ret = qcarcam_get_frame(input_ctxt->qcarcam_context, &frame_info, input_ctxt->frame_timeout, 0);
    if (ret == QCARCAM_RET_TIMEOUT)
    {
        QCARCAM_ERRORMSG("qcarcam_get_frame timeout context %p ret %d", input_ctxt->qcarcam_context, ret);
        input_ctxt->signal_lost = 1;
        return -1;
    }

    if (QCARCAM_RET_OK != ret)
    {
        QCARCAM_ERRORMSG("Get frame context %p ret %d", input_ctxt->qcarcam_context, ret);
        return -1;
    }

    if (frame_info.idx >= input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers)
    {
        QCARCAM_ERRORMSG("Get frame context %p ret invalid idx %d", input_ctxt->qcarcam_context, frame_info.idx);
        return -1;
    }

#if 0
    if(0 == input_ctxt->idx)
    {
    	static int front_num = 0;
    	front_num++;
    	if(2 == front_num) {
    		char buf[8];
			FILE *fp;
			size_t numBytesWritten = 0;
			size_t numByteToWrite = 0;
			void *data = NULL;

    		QCARCAM_INFOMSG("save uyvy to file<%d %d>", input_ctxt->idx, frame_info.idx);
			fp = fopen("/tmp/front.uyvy", "w+");
			if(fp)
			{
				memset(buf, 0, sizeof(buf));
				buf[0] = '5';
				buf[1] = 'a';
				buf[2] = 'c';

				numByteToWrite = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_info.idx].planes[0].size;
				//numByteToWrite = 8;
				data = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_info.idx].planes[0].p_buf;
				numBytesWritten = fwrite(data, numByteToWrite, 1, fp);
				//memcpy(buf, data, sizeof(buf));
				//numBytesWritten = fwrite(buf, 1, numByteToWrite, fp);
				fclose(fp);

				QCARCAM_INFOMSG("<1>data=0x%x numByteToWrite=%d numBytesWritten=%d", data, numByteToWrite, numBytesWritten);
			}
			else
			{
				QCARCAM_INFOMSG("open file failed");
			}
    	}
	}
#endif

    if (input_ctxt->frameCnt == 0)
    {
        qcarcam_test_get_time(&input_ctxt->t_firstFrame);

        if (input_ctxt->is_first_start)
        {
            ais_log_kpi(AIS_EVENT_KPI_CLIENT_FIRST_FRAME);

            printf("Success - First Frame [%d:%d]\n", input_ctxt->idx, input_ctxt->qcarcam_input_id);
            fflush(stdout);

            QCARCAM_ALWZMSG("[%u:%u] First Frame buf_idx %d after : %lu ms (field type: %d)",
                    input_ctxt->idx, (unsigned int)input_ctxt->qcarcam_input_id, frame_info.idx, (input_ctxt->t_firstFrame - gCtxt.t_start), frame_info.field_type);

            input_ctxt->is_first_start = FALSE;
        }
        else
        {
            QCARCAM_ALWZMSG("[%d:%d] restart took %llu ms",
                    input_ctxt->idx, input_ctxt->qcarcam_input_id, (input_ctxt->t_firstFrame - input_ctxt->t_start));
        }

        if (gCtxt.enable_deinterlace)
        {
            input_ctxt->field_type_previous = QCARCAM_FIELD_UNKNOWN;

            if (frame_info.field_type == QCARCAM_FIELD_UNKNOWN)
                input_ctxt->skip_post_display = 1;
            else
                input_ctxt->field_type_previous = frame_info.field_type;
        }
    }
    else
    {
        if (gCtxt.enable_deinterlace && (input_ctxt->field_type_previous != frame_info.field_type))
        {
            input_ctxt->skip_post_display = 0;
            QCARCAM_ERRORMSG("Field type changed: %d -> %d @frame_%d", input_ctxt->field_type_previous, frame_info.field_type, frame_info.seq_no);
            if (frame_info.field_type == QCARCAM_FIELD_UNKNOWN)
                frame_info.field_type = input_ctxt->field_type_previous;
            else
                input_ctxt->field_type_previous = frame_info.field_type;
        }
    }

    input_ctxt->get_frame_buf_idx = frame_info.idx;
    input_ctxt->buf_state[input_ctxt->get_frame_buf_idx] = QCARCAMTEST_BUFFER_STATE_GET_FRAME;

    input_ctxt->signal_lost = 0;

    input_ctxt->diag.frame_generate_time[TEST_PREV_BUFFER] = input_ctxt->diag.frame_generate_time[TEST_CUR_BUFFER];
    input_ctxt->diag.frame_generate_time[TEST_CUR_BUFFER] = frame_info.timestamp * NS_TO_MS;

    QCARCAM_DBGMSG("[%d] frameId:%d bufId:%d qtime:%llu", input_ctxt->idx, input_ctxt->frameCnt, frame_info.idx, frame_info.sof_qtimestamp);

    if (gCtxt.enable_deinterlace)
        process_deinterlace(input_ctxt, frame_info.field_type, gCtxt.enable_deinterlace);

    input_ctxt->frameCnt++;

    return 0;
}
/**
 * Function to post new frame to display. May also do color conversion and frame dumps.
 * @param input_ctxt
 * @return 0 on success, -1 on failure
 */
static int qcarcam_test_post_to_display(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret;
    /**********************
     * Composite to display
     ********************** */
    QCARCAM_DBGMSG("Post Frame before buf_idx %i %d", input_ctxt->buf_idx_qcarcam, input_ctxt->idx);
    /**********************
     * Dump raw if necessary
     ********************** */
    if (input_ctxt->dumpNextFrame ||
        (gCtxt.dumpFrame && (0 == input_ctxt->frameCnt % gCtxt.dumpFrame)))
    {
#if 0
    	if(0 == input_ctxt->idx)
		{
			int frame_idx = input_ctxt->get_frame_buf_idx;

			static int front_num = 0;
			front_num++;
			//if(2 == front_num)
			{
				char buf[8];
				FILE *fp;
				size_t numBytesWritten = 0;
				size_t numByteToWrite = 0;
				void *data = NULL;

				QCARCAM_INFOMSG("save uyvy to file<%d %d>", input_ctxt->idx, frame_idx);
				fp = fopen("/tmp/front.uyvy", "w+");
				if(fp)
				{
					memset(buf, 0, sizeof(buf));
					buf[0] = '5';
					buf[1] = 'a';
					buf[2] = 'c';

					numByteToWrite = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_idx].planes[0].size;
					//numByteToWrite = 8;
					data = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_idx].planes[0].p_buf;
					numBytesWritten = fwrite(data, numByteToWrite, 1, fp);
					//memcpy(buf, data, sizeof(buf));
					//numBytesWritten = fwrite(buf, 1, numByteToWrite, fp);
					fclose(fp);

					QCARCAM_INFOMSG("<3>data=0x%x numByteToWrite=%d numBytesWritten=%d", data, numByteToWrite, numBytesWritten);
				}
				else
				{
					QCARCAM_INFOMSG("open file failed");
				}
			}
		}
#endif
    	QCARCAM_INFOMSG("<3>pmem=0x%x", gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[input_ctxt->get_frame_buf_idx].planes[0].p_buf);

        snprintf(g_filename, sizeof(g_filename), DEFAULT_DUMP_LOCATION "frame_%d_%i.raw", input_ctxt->idx, input_ctxt->frameCnt);
        QCARCAM_DBGMSG("<1>call test_util_dump_window_buffer ...");
        test_util_dump_window_buffer(input_ctxt->test_util_ctxt, input_ctxt->qcarcam_window, input_ctxt->buf_idx_qcarcam, g_filename);

        input_ctxt->dumpNextFrame = FALSE;
    }

    /**********************
     * Color conversion if necessary
     ********************** */
    if (!(gCtxt.enable_c2d || gCtxt.enable_csc))
    {
        /**********************
         * Post to screen
         ********************** */
        QCARCAM_DBGMSG("Post Frame %d", input_ctxt->buf_idx_qcarcam);
        if (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))
        {
            ret = test_util_post_window_buffer(input_ctxt->test_util_ctxt, input_ctxt->display_window, input_ctxt->buf_idx_disp, NULL, input_ctxt->field_type_previous);
            input_ctxt->buf_idx_disp++;
            input_ctxt->buf_idx_disp %= input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers;
            input_ctxt->release_buf_idx.push_back(input_ctxt->buf_idx_qcarcam);
        }
        else
        {
            ret = test_util_post_window_buffer(input_ctxt->test_util_ctxt, input_ctxt->qcarcam_window, input_ctxt->buf_idx_qcarcam, &input_ctxt->release_buf_idx, input_ctxt->field_type_previous);
            input_ctxt->buf_state[input_ctxt->buf_idx_qcarcam] = QCARCAMTEST_BUFFER_STATE_POST_DISPLAY;
        }
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_post_window_buffer failed");
        }
    }
#ifndef C2D_DISABLED
    else
    {
        if (!(gCtxt.enable_csc))
        {
            //for now always go through c2d conversion instead of posting directly to  test since gles composition cannot handle
            // uyvy buffers for now.
            QCARCAM_DBGMSG("[%d] converting through c2d %d -> %d", input_ctxt->idx, input_ctxt->buf_idx_qcarcam, input_ctxt->buf_idx_disp);

            C2D_STATUS c2d_status;
            C2D_OBJECT c2dObject;
            memset(&c2dObject, 0x0, sizeof(C2D_OBJECT));
            unsigned int target_id;
            ret = test_util_get_c2d_surface_id(input_ctxt->test_util_ctxt, input_ctxt->qcarcam_window, input_ctxt->buf_idx_qcarcam, &c2dObject.surface_id);
            ret = test_util_get_c2d_surface_id(input_ctxt->test_util_ctxt, input_ctxt->display_window, input_ctxt->buf_idx_disp, &target_id);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_get_c2d_surface_id failed (%d)", ret);
            }

            pthread_mutex_lock(&gCtxt.mutex_c2d);
            c2d_status = c2dDraw(target_id, C2D_TARGET_ROTATE_0, 0x0, 0, 0, &c2dObject, 1);

            c2d_ts_handle c2d_timestamp;
            if (c2d_status == C2D_STATUS_OK)
            {
                c2d_status = c2dFlush(target_id, &c2d_timestamp);
            }
            pthread_mutex_unlock(&gCtxt.mutex_c2d);

            if (c2d_status == C2D_STATUS_OK)
            {
                c2d_status = c2dWaitTimestamp(c2d_timestamp);
            }

            QCARCAM_DBGMSG("c2d conversion finished");

            if (c2d_status != C2D_STATUS_OK)
            {
                QCARCAM_ERRORMSG("c2d conversion failed with error %d", c2d_status);
            }
        }
#ifdef ENABLE_CL_CONVERTER
        else
        {
            //Use open CL to convert
            csc_run(input_ctxt->g_converter, input_ctxt->buf_idx_qcarcam, input_ctxt->buf_idx_disp, NULL);
            csc_wait(input_ctxt->g_converter, 0, NULL);
        }
#endif

        /**********************
         * Dump if necessary
         ********************** */
        if (0 != gCtxt.dumpFrame)
        {
            if (0 == input_ctxt->frameCnt % gCtxt.dumpFrame)
            {
                snprintf(g_filename, sizeof(g_filename), DEFAULT_DUMP_LOCATION "frame_display_%d_%i.raw", input_ctxt->idx, input_ctxt->frameCnt);
                QCARCAM_DBGMSG("<2>call test_util_dump_window_buffer ...");
                test_util_dump_window_buffer(input_ctxt->test_util_ctxt, input_ctxt->display_window, input_ctxt->buf_idx_disp, g_filename);
            }
        }
        /**********************
         * Post to screen
         ********************** */
        QCARCAM_DBGMSG("Post Frame %d", input_ctxt->buf_idx_disp);
        ret = test_util_post_window_buffer(input_ctxt->test_util_ctxt,
                                           input_ctxt->display_window,
                                           input_ctxt->buf_idx_disp,
                                           &input_ctxt->release_buf_idx,
                                           input_ctxt->field_type_previous);

        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_post_window_buffer failed");
        }

        input_ctxt->buf_idx_disp++;
        input_ctxt->buf_idx_disp %= input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers;
    }
#endif
    QCARCAM_DBGMSG("Post Frame after buf_idx %i", input_ctxt->buf_idx_qcarcam);

    return 0;
}
/**
 * Release frame back to qcarcam
 * @param input_ctxt
 * @return 0 on success, -1 on failure
 */
static int qcarcam_test_release_frame(qcarcam_test_input_t *input_ctxt)
{
    qcarcam_ret_t ret;

    if (gCtxt.disable_display || input_ctxt->window_params.is_offscreen)
    {
        /* should release current buffer back to HW immediately if needn't display */
        ret = qcarcam_release_frame(input_ctxt->qcarcam_context, input_ctxt->buf_idx_qcarcam);
        if (QCARCAM_RET_OK != ret)
        {
            QCARCAM_ERRORMSG("qcarcam_release_frame(%d) failed %d", input_ctxt->buf_idx_qcarcam, ret);
            return -1;
        }

        input_ctxt->buf_state[input_ctxt->buf_idx_qcarcam] = QCARCAMTEST_BUFFER_STATE_QCARCAM;
        input_ctxt->releaseframeCnt++;
    }
    else
    {
        while(!input_ctxt->release_buf_idx.empty())
        {
            uint32 rel_idx = input_ctxt->release_buf_idx.front();
            input_ctxt->release_buf_idx.pop_front();

            if (rel_idx >= 0 &&
                rel_idx < input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers)
            {
                if (QCARCAMTEST_BUFFER_STATE_GET_FRAME == (input_ctxt->buf_state[rel_idx] & 0xF) ||
                    QCARCAMTEST_BUFFER_STATE_POST_DISPLAY == (input_ctxt->buf_state[rel_idx] & 0xF))
                {
                    ret = qcarcam_release_frame(input_ctxt->qcarcam_context, rel_idx);
                    if (QCARCAM_RET_OK != ret)
                    {
                        QCARCAM_ERRORMSG("qcarcam_release_frame(%d) failed %d", rel_idx, ret);
                        return -1;
                    }
                    input_ctxt->releaseframeCnt++;
                    input_ctxt->buf_state[rel_idx] = QCARCAMTEST_BUFFER_STATE_QCARCAM;
                }
                else
                {
                    QCARCAM_ERRORMSG("qcarcam_release_frame(%d) skipped since buffer bad state (%d)", rel_idx, input_ctxt->buf_state[rel_idx]);
                }
            }
            else
            {
                QCARCAM_ERRORMSG("qcarcam_release_frame(%d) skipped", rel_idx);
            }
        }
    }

#ifdef ENABLE_INJECTION_SUPPORT
    //Requeue input buffer here for now...
    if (input_ctxt->is_injection)
    {
        qcarcam_param_value_t param;
        param.uint_value = 0; //TODO: fix this to be appropriate buffer
        qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_INJECTION_START, &param);
    }
#endif

    return 0;
}

static void usr_process_cb(void *arg)
{
    timer_usr_data * usr_data = (timer_usr_data *)arg;
    qcarcam_test_input_t *input_ctxt = (qcarcam_test_input_t *)usr_data->p_data;
    uint32 buf_idx = usr_data->buf_idx;

    pthread_mutex_lock(&input_ctxt->mutex);
    if (input_ctxt->buf_state[buf_idx] != QCARCAMTEST_BUFFER_STATE_USR_PROCESS)
    {
        QCARCAM_ERRORMSG("buf (%d) in error state %d", buf_idx, input_ctxt->buf_state[buf_idx]);
    }

    input_ctxt->buf_state[buf_idx] = QCARCAMTEST_BUFFER_STATE_USR_PROCESS_DONE;
    input_ctxt->usr_process_buf_idx.push_back(buf_idx);
    input_ctxt->usr_process_frameCnt--;

    QCARCAM_DBGMSG("buf (%d) usr process done, usr_process_frameCnt %d", buf_idx, input_ctxt->usr_process_frameCnt);

    pthread_mutex_unlock(&input_ctxt->mutex);

}

/**
 * Function to handle routine of fetching, displaying, and releasing frames when one is available
 * @param input_ctxt
 * @return 0 on success, -1 on failure
 */
static int qcarcam_test_handle_new_frame(qcarcam_test_input_t *input_ctxt)
{
    if (qcarcam_test_get_frame(input_ctxt))
    {
        /*if we fail to get frame, we silently continue...*/
        return 0;
    }

    if (gCtxt.enablePauseResume && 0 == (input_ctxt->frameCnt % gCtxt.enablePauseResume))
    {
        qcarcam_ret_t ret = QCARCAM_RET_OK;

        if (input_ctxt->delay_time)
            qcarcam_test_clear_usr_process_list(input_ctxt);

        //release frame before pause
        qcarcam_test_release_frame(input_ctxt);

        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state != QCARCAMTEST_STATE_CLOSED)
        {
            QCARCAM_INFOMSG("pause...");
            ret = qcarcam_input_pause(input_ctxt);
            if (ret == QCARCAM_RET_OK)
            {
                pthread_mutex_unlock(&input_ctxt->mutex);

                usleep(PAUSE_RESUME_USLEEP);

                pthread_mutex_lock(&input_ctxt->mutex);
                if (input_ctxt->state != QCARCAMTEST_STATE_CLOSED)
                {
                    QCARCAM_INFOMSG("resume...");
                    ret = qcarcam_input_resume(input_ctxt);
                }
            }
        }
        pthread_mutex_unlock(&input_ctxt->mutex);

        return ret;
    }
    else if (gCtxt.enableStartStop && 0 == (input_ctxt->frameCnt % gCtxt.enableStartStop))
    {
        qcarcam_ret_t ret = QCARCAM_RET_OK;

        if (input_ctxt->delay_time)
            qcarcam_test_clear_usr_process_list(input_ctxt);

        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state != QCARCAMTEST_STATE_CLOSED)
        {
            QCARCAM_INFOMSG("stop...");
            ret = qcarcam_input_stop(input_ctxt);
            if (ret == QCARCAM_RET_OK)
            {
                pthread_mutex_unlock(&input_ctxt->mutex);

                usleep(START_STOP_USLEEP);

                pthread_mutex_lock(&input_ctxt->mutex);
                if (input_ctxt->state != QCARCAMTEST_STATE_CLOSED)
                {
                    QCARCAM_INFOMSG("re-start...");
                    ret = qcarcam_input_start(input_ctxt);
                }
            }
        }
        pthread_mutex_unlock(&input_ctxt->mutex);

        return ret;
    }
    if (input_ctxt->delay_time)
    {
        uint32 idx = input_ctxt->get_frame_buf_idx;
        if (!input_ctxt->buf_timer[idx].ptimer)
        {
            input_ctxt->buf_timer[idx].usr_data.buf_idx = idx;
            input_ctxt->buf_timer[idx].usr_data.p_data = input_ctxt;

            if (CameraCreateTimer(input_ctxt->delay_time, 0, usr_process_cb, &input_ctxt->buf_timer[idx].usr_data, &input_ctxt->buf_timer[idx].ptimer))
            {
                QCARCAM_ERRORMSG("CameraCreateTimer failed for buf %d", idx);
                return -1;
            }
        }
        else
        {
            if (CameraUpdateTimer(input_ctxt->buf_timer[idx].ptimer, input_ctxt->delay_time))
            {
                QCARCAM_ERRORMSG("CameraUpdateTimer failed for buf %d", idx);
                return -1;
            }
        }

        pthread_mutex_lock(&input_ctxt->mutex);
        input_ctxt->usr_process_frameCnt++;
        input_ctxt->buf_state[idx] = QCARCAMTEST_BUFFER_STATE_USR_PROCESS;
        pthread_mutex_unlock(&input_ctxt->mutex);

        return 0;
    }

    input_ctxt->buf_idx_qcarcam = input_ctxt->get_frame_buf_idx;
    if (!input_ctxt->skip_post_display)
    {
#if 0
    	if(0 == input_ctxt->idx)
		{
    		int frame_idx = input_ctxt->get_frame_buf_idx;

			static int front_num = 0;
			front_num++;
			if(2 == front_num) {
				char buf[8];
				FILE *fp;
				size_t numBytesWritten = 0;
				size_t numByteToWrite = 0;
				void *data = NULL;

				QCARCAM_INFOMSG("save uyvy to file<%d %d>", input_ctxt->idx, frame_idx);
				fp = fopen("/tmp/front.uyvy", "w+");
				if(fp)
				{
					memset(buf, 0, sizeof(buf));
					buf[0] = '5';
					buf[1] = 'a';
					buf[2] = 'c';

					numByteToWrite = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_idx].planes[0].size;
					//numByteToWrite = 8;
					data = gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_idx].planes[0].p_buf;
					numBytesWritten = fwrite(data, numByteToWrite, 1, fp);
					//memcpy(buf, data, sizeof(buf));
					//numBytesWritten = fwrite(buf, 1, numByteToWrite, fp);
					fclose(fp);

					QCARCAM_INFOMSG("<2>data=0x%x numByteToWrite=%d numBytesWritten=%d", data, numByteToWrite, numBytesWritten);
				}
				else
				{
					QCARCAM_INFOMSG("open file failed");
				}
			}
		}
#endif

    	qcarcam_test_post_to_display(input_ctxt);
    }

    if (qcarcam_test_release_frame(input_ctxt))
        return -1;

    return 0;
}

static int inpt_ctxt_usr_process_thread(void *arg)
{
    qcarcam_test_input_t *input_ctxt = (qcarcam_test_input_t *)arg;
    pthread_detach(pthread_self());

    while (!g_aborted)
    {
        pthread_mutex_lock(&input_ctxt->mutex);
        if(!input_ctxt->usr_process_buf_idx.empty() && input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            input_ctxt->buf_idx_qcarcam = input_ctxt->usr_process_buf_idx.front();
            input_ctxt->usr_process_buf_idx.pop_front();
            pthread_mutex_unlock(&input_ctxt->mutex);

            if (!input_ctxt->skip_post_display)
                qcarcam_test_post_to_display(input_ctxt);

            qcarcam_test_release_frame(input_ctxt);

            pthread_mutex_lock(&input_ctxt->mutex);
        }

        pthread_mutex_unlock(&input_ctxt->mutex);
        usleep(USR_PROCESS_THREAD_USLEEP);
    }
    return 0;
}

static int qcarcam_test_handle_input_signal(qcarcam_test_input_t *input_ctxt, qcarcam_input_signal_t signal_type)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    switch (signal_type) {
    case QCARCAM_INPUT_SIGNAL_LOST:
        QCARCAM_ERRORMSG("LOST: idx: %d, input: %d", input_ctxt->idx, input_ctxt->qcarcam_input_id);

        /*TODO: offload this to other thread to handle restart recovery*/
        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state == QCARCAMTEST_STATE_STOP) {
            QCARCAM_ERRORMSG("Input %d already stop, break", input_ctxt->qcarcam_input_id);
            pthread_mutex_unlock(&input_ctxt->mutex);
            break;
        }

        input_ctxt->signal_lost = 1;
        qcarcam_input_stop(input_ctxt);

        pthread_mutex_unlock(&input_ctxt->mutex);

        break;
    case QCARCAM_INPUT_SIGNAL_VALID:
        QCARCAM_ERRORMSG("VALID: idx: %d, input: %d", input_ctxt->idx, input_ctxt->qcarcam_input_id);

        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            QCARCAM_ERRORMSG("Input %d already running, break", input_ctxt->qcarcam_input_id);
            pthread_mutex_unlock(&input_ctxt->mutex);
            break;
        }

        ret = qcarcam_input_start(input_ctxt);

        pthread_mutex_unlock(&input_ctxt->mutex);

        break;
    default:
        QCARCAM_ERRORMSG("Unknown Event type: %d", signal_type);
        break;
    }

    return ret;
}

static int qcarcam_test_handle_fatal_error(qcarcam_test_input_t *input_ctxt, boolean recover)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    QCARCAM_ERRORMSG("Fatal error: client idx - %d, input id - %d", input_ctxt->idx, input_ctxt->qcarcam_input_id);

    pthread_mutex_lock(&input_ctxt->mutex);

    input_ctxt->signal_lost = 1;
    input_ctxt->fatal_err_cnt++;

    if (input_ctxt->state == QCARCAMTEST_STATE_ERROR || input_ctxt->state == QCARCAMTEST_STATE_STOP) {
        QCARCAM_ERRORMSG("Input %d already error state, return", input_ctxt->qcarcam_input_id);
        pthread_mutex_unlock(&input_ctxt->mutex);
        return ret;
    }

    if (recover)
    {
        ret = qcarcam_stop(input_ctxt->qcarcam_context);
        if (ret == QCARCAM_RET_OK) {
            input_ctxt->state = QCARCAMTEST_STATE_STOP;
            QCARCAM_INFOMSG("Client %d Input %d qcarcam_stop successfully", input_ctxt->idx, input_ctxt->qcarcam_input_id);
        }
        else
        {
            input_ctxt->state = QCARCAMTEST_STATE_ERROR;
            QCARCAM_ERRORMSG("Client %d Input %d qcarcam_stop failed: %d !", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
        }
    }
    else
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
    }

    pthread_mutex_unlock(&input_ctxt->mutex);

    return ret;
}

static int qcarcam_test_handle_set_event_notification(qcarcam_test_input_t *input_ctxt, qcarcam_param_t evnt_type)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    switch(evnt_type)
    {
    case QCARCAM_PARAM_EXPOSURE:
    case QCARCAM_PARAM_HDR_EXPOSURE:
    case QCARCAM_PARAM_SATURATION:
    case QCARCAM_PARAM_HUE:
    case QCARCAM_PARAM_FRAME_RATE:
    case QCARCAM_PARAM_GAMMA:
        QCARCAM_INFOMSG("Input_ctxt is notified for changed setting:%d input_id:%d client_id:%d",
            evnt_type, input_ctxt->qcarcam_input_id, input_ctxt->idx);
        break;
    case QCARCAM_PARAM_MASTER:
        QCARCAM_INFOMSG("Master released input_id:%d client_id:%d",
            input_ctxt->qcarcam_input_id, input_ctxt->idx);
        break;
    default:
        QCARCAM_ERRORMSG("Input_ctxt is notified with unknown event");
        break;
    }
    return ret;
}

/**
 * Qcarcam event callback function
 * @param hndl
 * @param event_id
 * @param p_payload
 */
static void qcarcam_test_event_cb(qcarcam_hndl_t hndl, qcarcam_event_t event_id, qcarcam_event_payload_t *p_payload)
{
    int i = 0;
    int result = 0;
    qcarcam_test_input_t *input_ctxt = NULL;
    qcarcam_event_msg_t event_msg;

    for (i = 0; i < gCtxt.numInputs; i++)
    {
        if (hndl == gCtxt.inputs[i].qcarcam_context)
        {
            input_ctxt = &gCtxt.inputs[i];
            break;
        }
    }

    if (!input_ctxt)
    {
        QCARCAM_ERRORMSG("event_cb called with invalid qcarcam handle %p", hndl);
        return;
    }

    if (g_aborted)
    {
        QCARCAM_ERRORMSG("Test aborted");
        return;
    }

    event_msg.event_id = event_id;

    memcpy(&event_msg.payload, p_payload, sizeof(qcarcam_event_payload_t));

    pthread_mutex_lock(&input_ctxt->queue_mutex);
    input_ctxt->eventqueue.push(event_msg);
    pthread_mutex_unlock(&input_ctxt->queue_mutex);

    result = CameraSetSignal(input_ctxt->m_eventHandlerSignal);
    if (result)
    {
        QCARCAM_ERRORMSG("Failed to signal event %d (%d)", event_id, result);
    }
}

static int process_cb_event_thread(void *arg)
{
    qcarcam_test_input_t *input_ctxt = (qcarcam_test_input_t *)arg;
    qcarcam_event_msg_t event_msg;

    while (!g_aborted)
    {
        CameraWaitOnSignal(input_ctxt->m_eventHandlerSignal, CAM_SIGNAL_WAIT_NO_TIMEOUT);

        pthread_mutex_lock(&input_ctxt->queue_mutex);
        if (!input_ctxt->eventqueue.empty())
        {
            event_msg = input_ctxt->eventqueue.front();
            input_ctxt->eventqueue.pop();
        }
        else
        {
            QCARCAM_ERRORMSG("event queue is empty");
            pthread_mutex_unlock(&input_ctxt->queue_mutex);
            continue;
        }

        pthread_mutex_unlock(&input_ctxt->queue_mutex);

        switch (event_msg.event_id)
        {
        case QCARCAM_EVENT_FRAME_READY:
        {
            if (input_ctxt->state == QCARCAMTEST_STATE_START)
            {
                QCARCAM_DBGMSG("%d received QCARCAM_EVENT_FRAME_READY ...", input_ctxt->idx);
                qcarcam_test_handle_new_frame(input_ctxt);
                QCARCAM_DBGMSG("%d received QCARCAM_EVENT_FRAME_READY done", input_ctxt->idx);
            }
            break;
        }
        case QCARCAM_EVENT_INPUT_SIGNAL:
        {
            if (gCtxt.enableBridgeErrorDetect)
            {
                qcarcam_test_handle_input_signal(input_ctxt, (qcarcam_input_signal_t)event_msg.payload.uint_payload);
            }
            break;
        }
        case QCARCAM_EVENT_ERROR:
        {
            switch (event_msg.payload.uint_payload)
            {
            case QCARCAM_CONN_ERROR:
                QCARCAM_ERRORMSG("Connetion to server lost. id:%d qid:%d",
                                 input_ctxt->idx, input_ctxt->qcarcam_input_id);

                abort_test();
                break;
            case QCARCAM_FATAL_ERROR:
                QCARCAM_ERRORMSG("fatal error %d on id:%d qid:%d",
                                 event_msg.payload.uint_payload, input_ctxt->idx, input_ctxt->qcarcam_input_id);

                qcarcam_test_handle_fatal_error(input_ctxt, gCtxt.enableFatalErrorRecover);
                break;
            case QCARCAM_FRAMESYNC_ERROR:
                QCARCAM_ERRORMSG("fatal error %d on id:%d qid:%d",
                                 event_msg.payload.uint_payload, input_ctxt->idx, input_ctxt->qcarcam_input_id);

                qcarcam_test_handle_fatal_error(input_ctxt, TRUE);

                break;
            case QCARCAM_IFE_OVERFLOW_ERROR:
                QCARCAM_ERRORMSG("output overflow on id:%d qid:%d",
                                 input_ctxt->idx, input_ctxt->qcarcam_input_id);

                qcarcam_test_handle_fatal_error(input_ctxt, gCtxt.enableIFEOverflowhandle);
                break;
            default:
                break;
            }

            break;
        }
        case QCARCAM_EVENT_VENDOR:
        {
            QCARCAM_INFOMSG("receive QCARCAM_EVENT_VENDOR data[0]=%u data[1]=%u",
                            event_msg.payload.array[0], event_msg.payload.array[1]);
            break;
        }
        case QCARCAM_EVENT_PROPERTY_NOTIFY:
            qcarcam_test_handle_set_event_notification(input_ctxt, (qcarcam_param_t)event_msg.payload.uint_payload);
            break;
        case QCARCAM_EVENT_FRAME_FREEZE:
        {
            ais_log_kpi(AIS_EVENT_KPI_CLIENT_FRAME_FREEZE);
            QCARCAM_ERRORMSG("Detected QCARCAM_EVENT_FRAME_FREEZE, Impacted Frame id = %d",
                             event_msg.payload.frame_freeze.seq_no);
            break;
        }
        default:
            QCARCAM_INFOMSG("%d received unsupported event %d", input_ctxt->idx, event_msg.event_id);
            break;
        }
    }
    return 0;
}

/**
 * Qcarcam gpio interrupt callback function for visibility mode
 */
static void qcarcam_test_gpio_interrupt_cb()
{
    int idx;
    unsigned long long t_before = 0;
    unsigned long long t_after = 0;

    gCtxt.vis_value = !gCtxt.vis_value;

    qcarcam_test_get_time(&t_before);

    for (idx = 0; idx < gCtxt.numInputs; ++idx)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[idx];

        if (input_ctxt->qcarcam_window != NULL)
        {
            test_util_set_param(input_ctxt->qcarcam_window, TEST_UTIL_VISIBILITY, gCtxt.vis_value);
        }

        if (input_ctxt->display_window != NULL)
        {
            test_util_set_param(input_ctxt->display_window, TEST_UTIL_VISIBILITY, gCtxt.vis_value);
        }

        input_ctxt->window_params.visibility = gCtxt.vis_value;
    }

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "Time for visbility toggle : %lu ms", (t_after - t_before));
}

/**
 * Qcarcam gpio interrupt callback function for play/pause mode
 */
static void qcarcam_test_gpio_play_pause_cb()
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;
    int stream_id;
    unsigned long long t_before = 0;
    unsigned long long t_after = 0;

    qcarcam_test_get_time(&t_before);

    for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[stream_id];

        pthread_mutex_lock(&input_ctxt->mutex);

        if (input_ctxt->state == QCARCAMTEST_STATE_PAUSE)
        {
            ret = qcarcam_input_resume(input_ctxt);
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            ret = qcarcam_input_pause(input_ctxt);
        }
        else
        {
            QCARCAM_ERRORMSG("bad state %d", input_ctxt->state);
            ret = QCARCAM_RET_BADSTATE;
        }

        pthread_mutex_unlock(&input_ctxt->mutex);

        if (ret)
        {
            QCARCAM_ERRORMSG("failed gpio toggle (%d)", input_ctxt->state);
        }
    }

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "Time for play/pause toggle : %lu ms",  (t_after - t_before));
}

/**
 * Qcarcam gpio interrupt callback function for stop/start mode
 */
static void qcarcam_test_gpio_start_stop_cb()
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;
    int stream_id;
    unsigned long long t_before = 0;
    unsigned long long t_after = 0;

    qcarcam_test_get_time(&t_before);

    for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[stream_id];

        pthread_mutex_lock(&input_ctxt->mutex);

        if (input_ctxt->state == QCARCAMTEST_STATE_STOP)
        {
            ret = qcarcam_input_start(input_ctxt);
        }
        else if (input_ctxt->state == QCARCAMTEST_STATE_START)
        {
            ret = qcarcam_input_stop(input_ctxt);
        }
        else
        {
            QCARCAM_ERRORMSG("bad state %d", input_ctxt->state);
            ret = QCARCAM_RET_BADSTATE;
        }

        pthread_mutex_unlock(&input_ctxt->mutex);

        if (ret)
        {
            QCARCAM_ERRORMSG("failed gpio toggle (%d)", input_ctxt->state);
        }
    }

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "Time for start/stop toggle : %lu ms", (t_after - t_before));
}

/**
 * API to set/release a client as master
 *
 *
 * @param qcarcam_test_input_t* input_ctxt
 * @param unsigned int flag to set(1)/release(0) master
 */

static void qcarcam_test_set_master(qcarcam_test_input_t *input_ctxt, unsigned int flag)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    qcarcam_param_value_t param;
    param.uint_value = flag;
    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_MASTER, &param);
    if (ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("This client can't be set(1)/release(0) master:%d, it's not fatal", flag);
    }
    else
    {
        if (flag == 1)
        {
            input_ctxt->is_master = true;
            printf("Client [Stream ID %u  Input ID %u] has been set as master \n",
                    input_ctxt->idx, (unsigned int)input_ctxt->qcarcam_input_id);
        }
        else
        {
            input_ctxt->is_master = false;
            printf("Client [Stream ID %u  Input ID %u] has been released \n",
                    input_ctxt->idx, (unsigned int)input_ctxt->qcarcam_input_id);
        }
    }
}

/**
 * API to subscribe for events to get notifications
 *
 *
 * @param qcarcam_test_input_t* input_ctxt
 * @param change_events  Bitmask of events
 */

static int qcarcam_test_subscribe_input_params_change(qcarcam_test_input_t *input_ctxt, uint64 change_events)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    qcarcam_param_value_t param;
    param.uint64_value = change_events;

    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_CHANGE_SUBSCRIBE, &param);
    if(ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_EVENT_SUBSCRIBE) failed for events:%llu with ret:%d", change_events, ret);
        return -1;
    }
    return 0;
}

/**
 * API to unsubscribe for events to get notifications
 *
 *
 * @param qcarcam_test_input_t* input_ctxt
 * @param change_events  Bitmask of events
 */

static int qcarcam_test_unsubscribe_input_params_change(qcarcam_test_input_t *input_ctxt, uint64 change_events)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    qcarcam_param_value_t param;
    param.uint64_value = change_events;

    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_CHANGE_UNSUBSCRIBE, &param);
    if(ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_EVENT_UNSUBSCRIBE) failed");
        return -1;
    }
    return 0;
}

/**
 * Thread to setup and run qcarcam based on test input context
 *
 * @note For single threaded operation, this function only sets up qcarcam context.
 *      qcarcam_start and handling of frames is not executed.
 *
 * @param arg qcarcam_test_input_t* input_ctxt
 */
static int qcarcam_test_setup_input_ctxt_thread(void *arg)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;
    qcarcam_test_input_t *input_ctxt = (qcarcam_test_input_t *)arg;
    unsigned long long t_before = 0;
    unsigned long long t_after = 0;

    if (!input_ctxt)
        return -1;

    QCARCAM_INFOMSG("setup_input_ctxt_thread idx = %d, input_desc=%d", input_ctxt->idx, input_ctxt->qcarcam_input_id);

    qcarcam_test_get_time(&t_before);

    input_ctxt->qcarcam_context = qcarcam_open(input_ctxt->qcarcam_input_id);
    pthread_mutex_lock(&gCtxt.mutex_open_cnt);
    gCtxt.opened_stream_cnt++;
    pthread_mutex_unlock(&gCtxt.mutex_open_cnt);
    if (input_ctxt->qcarcam_context == 0)
    {
        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
        QCARCAM_ERRORMSG("qcarcam_open() failed");
        goto qcarcam_thread_fail;
    }

    input_ctxt->state = QCARCAMTEST_STATE_OPEN;

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_open (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
    t_before = t_after;

    QCARCAM_INFOMSG("render_thread idx = %d, input_desc=%d context=%p",
            input_ctxt->idx, input_ctxt->qcarcam_input_id, input_ctxt->qcarcam_context);

    // For HDMI/CVBS Input
    // NOTE: set HDMI IN resolution in qcarcam_config_single_hdmi.xml before
    // running the test
    if (input_ctxt->qcarcam_input_id == QCARCAM_INPUT_TYPE_DIGITAL_MEDIA)
    {
        qcarcam_param_value_t param;
        param.res_value.width = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].width;
        param.res_value.height = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].height;

        ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_RESOLUTION, &param);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_param resolution() failed");
            goto qcarcam_thread_fail;
        }
        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_s_param resolution (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;
    }

    ret = qcarcam_s_buffers(input_ctxt->qcarcam_context, &input_ctxt->p_buffers_output);
    if (ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("qcarcam_s_buffers() failed");
        goto qcarcam_thread_fail;
    }

#ifdef ENABLE_INJECTION_SUPPORT
    if (input_ctxt->is_injection)
    {
        QCARCAM_ERRORMSG("read from input_file - set input buffers");

        ret = qcarcam_s_input_buffers(input_ctxt->qcarcam_context, &input_ctxt->p_buffers_input);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_input_buffers() failed");
            goto qcarcam_thread_fail;
        }
    }
#endif

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_s_buffers (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
    t_before = t_after;

    QCARCAM_INFOMSG("qcarcam_s_buffers done, qcarcam_start ...");

    if (input_ctxt->use_event_callback)
    {
        qcarcam_param_value_t param;
        param.ptr_value = (void *)qcarcam_test_event_cb;
        ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_CB, &param);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_EVENT_CB) failed");
            goto qcarcam_thread_fail;
        }

        param.uint_value = QCARCAM_EVENT_FRAME_READY |
                QCARCAM_EVENT_INPUT_SIGNAL |
                QCARCAM_EVENT_ERROR | QCARCAM_EVENT_VENDOR | QCARCAM_EVENT_FRAME_FREEZE;
        ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_MASK, &param);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_EVENT_MASK) failed");
            goto qcarcam_thread_fail;
        }

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_s_param (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;
    }

    if (input_ctxt->frame_rate_config.frame_drop_mode != QCARCAM_KEEP_ALL_FRAMES)
    {
        qcarcam_param_value_t param;

        param.frame_rate_config = input_ctxt->frame_rate_config;

        ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_FRAME_RATE, &param);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_FRAME_RATE) failed");
            goto qcarcam_thread_fail;
        }

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_s_param fps (idx %u): %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;

        QCARCAM_INFOMSG("Provided QCARCAM_PARAM_FRAME_RATE: mode = %d, period = %d, pattern = 0x%x",
            param.frame_rate_config.frame_drop_mode,
            param.frame_rate_config.frame_drop_period,
            param.frame_rate_config.frame_drop_pattern);
    }

    if (input_ctxt->op_mode != QCARCAM_OPMODE_MAX)
    {
        qcarcam_param_value_t param;
        param.uint_value = input_ctxt->op_mode;
        ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_OPMODE, &param);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_s_param(QCARCAM_PARAM_OPMODE) failed");
            goto qcarcam_thread_fail;
        }

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_s_param opmode (idx %u): %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;
    }

    /*single threaded handles frames outside this function*/
    if (gCtxt.multithreaded)
    {
        pthread_mutex_lock(&input_ctxt->mutex);

        input_ctxt->is_first_start = TRUE;
        ret = qcarcam_input_start(input_ctxt);

        pthread_mutex_unlock(&input_ctxt->mutex);

#ifdef ENABLE_INJECTION_SUPPORT
        if (input_ctxt->is_injection)
        {
            qcarcam_param_value_t param;
            param.uint_value = 0; //TODO: fix this to be appropriate buffer
            qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_INJECTION_START, &param);
        }
#endif

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam_start (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;

        if (input_ctxt->manual_exposure != -1)
        {
            /* Set exposure configuration */
            qcarcam_param_value_t param;
            param.exposure_config.exposure_time = input_ctxt->exp_time;
            param.exposure_config.gain = input_ctxt->gain;
            param.exposure_config.exposure_mode_type = input_ctxt->manual_exposure ? QCARCAM_EXPOSURE_MANUAL : QCARCAM_EXPOSURE_AUTO;

            if (input_ctxt->manual_exposure)
            {
                QCARCAM_INFOMSG("Provided QCARCAM_PARAM_MANUAL_EXPOSURE : time =%f ms, gain = %f", input_ctxt->exp_time, input_ctxt->gain);
            }
            else
            {
                QCARCAM_INFOMSG("AUTO EXPOSURE MODE");
            }

            qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EXPOSURE, &param);
        }
        else
        {
            QCARCAM_INFOMSG("Exposure is not configured, use default setting");
        }

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam setexposure (idx %u) : %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;

        if (!input_ctxt->use_event_callback)
        {
            while (!g_aborted)
            {
                if (qcarcam_test_handle_new_frame(input_ctxt))
                    break;
            }
        }
        else
        {
            pthread_mutex_lock(&gCtxt.mutex_abort);
            if (!g_aborted)
            {
                pthread_cond_wait(&gCtxt.cond_abort, &gCtxt.mutex_abort);
            }
            pthread_mutex_unlock(&gCtxt.mutex_abort);
        }
    }

    QCARCAM_INFOMSG("exit setup_input_ctxt_thread idx = %d", input_ctxt->idx);
    return 0;

qcarcam_thread_fail:
    if (input_ctxt->qcarcam_context)
    {
        qcarcam_close(input_ctxt->qcarcam_context);
        input_ctxt->qcarcam_context = NULL;

        input_ctxt->state = QCARCAMTEST_STATE_ERROR;
    }

    return -1;
}

static void display_isp_settings(qcarcam_param_value_t *param)
{
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_AE_MODE))
        printf("AE_MODE = %d\t",param->isp_ctrls.ae_mode);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_AE_LOCK))
        printf("AE_LOCK = %d\n",param->isp_ctrls.ae_lock);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_AWB_MODE))
        printf("AWB_MODE = %d\t",param->isp_ctrls.awb_mode);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_AWB_LOCK))
        printf("AWB_LOCK = %d\n",param->isp_ctrls.awb_lock);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_EFFECT_MODE))
        printf("EFFECT_MODE = %d\t",param->isp_ctrls.effect_mode);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_MODE))
        printf("CONTROL_MODE = %d\n",param->isp_ctrls.ctrl_mode);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_SCENE_MODE))
        printf("SCENE_MODE = %d\t",param->isp_ctrls.scene_mode);
    if (CHECK_BIT(param->isp_ctrls.param_mask, QCARCAM_CONTROL_AE_ANTIBANDING_MODE))
        printf("AE_ANTIBANDING_MODE = %d\n",param->isp_ctrls.ae_antibanding_mode);
}

static void input_isp_param(qcarcam_isp_param_t ch, qcarcam_param_value_t *param)
{
    int sub_ch = 0;
    char buf[BUFSIZE];
    char *p = NULL;
    SET_BIT(param->isp_ctrls.param_mask, ch);
    switch(ch)
    {
        case QCARCAM_CONTROL_AE_LOCK:
            printf("'%d'....QCARCAM_CONTROL_AE_LOCK_OFF\n", QCARCAM_CONTROL_AE_LOCK_OFF);
            printf("'%d'....QCARCAM_CONTROL_AE_LOCK_ON\n", QCARCAM_CONTROL_AE_LOCK_ON);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_AE_LOCK_OFF && sub_ch<=QCARCAM_CONTROL_AE_LOCK_ON)
                {
                    param->isp_ctrls.ae_lock = (qcarcam_ctrl_ae_lock_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_AE_MODE:
            printf("'%d'....QCARCAM_CONTROL_AE_MODE_MANUAL\n", QCARCAM_CONTROL_AE_MODE_MANUAL);
            printf("'%d'....QCARCAM_CONTROL_AE_MODE_AUTO\n", QCARCAM_CONTROL_AE_MODE_AUTO);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_AE_MODE_MANUAL && sub_ch<=QCARCAM_CONTROL_AE_MODE_AUTO)
                {
                    param->isp_ctrls.ae_mode = (qcarcam_ctrl_ae_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_AWB_LOCK:
            printf("'%d'....QCARCAM_CONTROL_AWB_LOCK_OFF\n", QCARCAM_CONTROL_AWB_LOCK_OFF);
            printf("'%d'....QCARCAM_CONTROL_AWB_LOCK_ON\n", QCARCAM_CONTROL_AWB_LOCK_ON);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_AWB_LOCK_OFF && sub_ch<=QCARCAM_CONTROL_AWB_LOCK_ON)
                {
                    param->isp_ctrls.awb_lock = (qcarcam_ctrl_awb_lock_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_AWB_MODE:
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_OFF\n", QCARCAM_CONTROL_AWB_MODE_OFF);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_AUTO\n", QCARCAM_CONTROL_AWB_MODE_AUTO);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_INCANDESCENT\n", QCARCAM_CONTROL_AWB_MODE_INCANDESCENT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_FLUORESCENT\n", QCARCAM_CONTROL_AWB_MODE_FLUORESCENT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_WARM_FLUORESCENT\n", QCARCAM_CONTROL_AWB_MODE_WARM_FLUORESCENT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_DAYLIGHT\n", QCARCAM_CONTROL_AWB_MODE_DAYLIGHT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT\n", QCARCAM_CONTROL_AWB_MODE_CLOUDY_DAYLIGHT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_TWILIGHT\n", QCARCAM_CONTROL_AWB_MODE_TWILIGHT);
            printf("'%d'....QCARCAM_CONTROL_AWB_MODE_SHADE\n", QCARCAM_CONTROL_AWB_MODE_SHADE);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_AWB_MODE_OFF && sub_ch<=QCARCAM_CONTROL_AWB_MODE_SHADE)
                {
                    param->isp_ctrls.awb_mode = (qcarcam_ctrl_awb_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_EFFECT_MODE:
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_OFF\n", QCARCAM_CONTROL_EFFECT_MODE_OFF);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_MONO\n", QCARCAM_CONTROL_EFFECT_MODE_MONO);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_NEGATIVE\n", QCARCAM_CONTROL_EFFECT_MODE_NEGATIVE);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_SOLARIZE\n", QCARCAM_CONTROL_EFFECT_MODE_SOLARIZE);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_SEPIA\n", QCARCAM_CONTROL_EFFECT_MODE_SEPIA);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_POSTERIZE\n", QCARCAM_CONTROL_EFFECT_MODE_POSTERIZE);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_WHITEBOARD\n", QCARCAM_CONTROL_EFFECT_MODE_WHITEBOARD);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_BLACKBOARD\n", QCARCAM_CONTROL_EFFECT_MODE_BLACKBOARD);
            printf("'%d'....QCARCAM_CONTROL_EFFECT_MODE_AQUA\n", QCARCAM_CONTROL_EFFECT_MODE_AQUA);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_EFFECT_MODE_OFF && sub_ch<=QCARCAM_CONTROL_EFFECT_MODE_AQUA)
                {
                    param->isp_ctrls.effect_mode = (qcarcam_ctrl_control_effect_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_MODE:
            printf("'%d'....QCARCAM_CONTROL_MODE_OFF\n", QCARCAM_CONTROL_MODE_OFF);
            printf("'%d'....QCARCAM_CONTROL_MODE_AUTO\n", QCARCAM_CONTROL_MODE_AUTO);
            printf("'%d'....QCARCAM_CONTROL_MODE_USE_SCENE_MODE\n", QCARCAM_CONTROL_MODE_USE_SCENE_MODE);
            printf("'%d'....QCARCAM_CONTROL_MODE_OFF_KEEP_STATE\n", QCARCAM_CONTROL_MODE_OFF_KEEP_STATE);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_MODE_OFF && sub_ch<=QCARCAM_CONTROL_MODE_OFF_KEEP_STATE)
                {
                    param->isp_ctrls.ctrl_mode = (qcarcam_ctrl_control_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_SCENE_MODE:
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_DISABLED\n", QCARCAM_CONTROL_SCENE_MODE_DISABLED);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_FACE_PRIORITY\n", QCARCAM_CONTROL_SCENE_MODE_FACE_PRIORITY);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_ACTION\n", QCARCAM_CONTROL_SCENE_MODE_ACTION);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_PORTRAIT\n", QCARCAM_CONTROL_SCENE_MODE_PORTRAIT);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_LANDSCAPE\n", QCARCAM_CONTROL_SCENE_MODE_LANDSCAPE);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_NIGHT\n", QCARCAM_CONTROL_SCENE_MODE_NIGHT);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_NIGHT_PORTRAIT\n", QCARCAM_CONTROL_SCENE_MODE_NIGHT_PORTRAIT);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_THEATRE\n", QCARCAM_CONTROL_SCENE_MODE_THEATRE);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_BEACH\n", QCARCAM_CONTROL_SCENE_MODE_BEACH);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_SNOW\n", QCARCAM_CONTROL_SCENE_MODE_SNOW);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_SUNSET\n", QCARCAM_CONTROL_SCENE_MODE_SUNSET);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_STEADYPHOTO\n", QCARCAM_CONTROL_SCENE_MODE_STEADYPHOTO);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_FIREWORKS\n", QCARCAM_CONTROL_SCENE_MODE_FIREWORKS);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_SPORTS\n", QCARCAM_CONTROL_SCENE_MODE_SPORTS);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_PARTY\n", QCARCAM_CONTROL_SCENE_MODE_PARTY);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_CANDLELIGHT\n", QCARCAM_CONTROL_SCENE_MODE_CANDLELIGHT);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_BARCODE\n", QCARCAM_CONTROL_SCENE_MODE_BARCODE);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_HIGH_SPEED_VIDEO\n", QCARCAM_CONTROL_SCENE_MODE_HIGH_SPEED_VIDEO);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_HDR\n", QCARCAM_CONTROL_SCENE_MODE_HDR);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_FACE_PRIORITY_LOW_LIGHT\n", QCARCAM_CONTROL_SCENE_MODE_FACE_PRIORITY_LOW_LIGHT);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_DEVICE_CUSTOM_START\n", QCARCAM_CONTROL_SCENE_MODE_DEVICE_CUSTOM_START);
            printf("'%d'....QCARCAM_CONTROL_SCENE_MODE_DEVICE_CUSTOM_END\n", QCARCAM_CONTROL_SCENE_MODE_DEVICE_CUSTOM_END);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_SCENE_MODE_DISABLED && sub_ch<=QCARCAM_CONTROL_SCENE_MODE_DEVICE_CUSTOM_END)
                {
                    param->isp_ctrls.scene_mode = (qcarcam_ctrl_control_scene_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;

        case QCARCAM_CONTROL_AE_ANTIBANDING_MODE:
            printf("'%d'....QCARCAM_CONTROL_AE_ANTIBANDING_MODE_OFF\n", QCARCAM_CONTROL_AE_ANTIBANDING_MODE_OFF);
            printf("'%d'....QCARCAM_CONTROL_AE_ANTIBANDING_MODE_50HZ\n", QCARCAM_CONTROL_AE_ANTIBANDING_MODE_50HZ);
            printf("'%d'....QCARCAM_CONTROL_AE_ANTIBANDING_MODE_60HZ\n", QCARCAM_CONTROL_AE_ANTIBANDING_MODE_60HZ);
            printf("'%d'....QCARCAM_CONTROL_AE_ANTIBANDING_MODE_AUTO\n", QCARCAM_CONTROL_AE_ANTIBANDING_MODE_AUTO);
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                sub_ch = strtol(buf, &p, 10);
                if (sub_ch>=QCARCAM_CONTROL_AE_ANTIBANDING_MODE_OFF && sub_ch<=QCARCAM_CONTROL_AE_ANTIBANDING_MODE_AUTO)
                {
                    param->isp_ctrls.ae_antibanding_mode = (qcarcam_ctrl_ae_antibanding_mode_t)sub_ch;
                }
                else
                {
                    printf("Invalid input\n");
                }
            }
            else
            {
                printf("failed to get input\n");
            }
            break;
        default: printf("Invalid choice\n");
    }
}

static void input_isp_settings(qcarcam_param_value_t *param, qcarcam_test_menu_option_t option)
{
    int ch = 0, i;
    char buf[BUFSIZE];
    char *p = NULL;
    if (option == QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM)
    {
        while(1)
        {
            printf("\n ========================================================== \n");
            for (i = 0; i < QCARCAM_ISP_PARAM_NUM; i++)
            {
                printf("'%d'....%s \t\t", g_qcarcam_isp_menu[i].id, g_qcarcam_isp_menu[i].str);
                if (i%2) printf("\n");
            }
            printf("\n ========================================================== \n");
            printf("'%d'.... Apply and Exit ISP settings", QCARCAM_ISP_PARAM_NUM);
            printf("\n ========================================================== \n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                ch = strtol(buf, &p, 10);
                if (ch < QCARCAM_ISP_PARAM_NUM)
                {
                    input_isp_param((qcarcam_isp_param_t)ch, param);
                }
                else
                {
                    printf("Invalid input\n");
                    break;
                }
            }
            else
            {
                printf("failed to get input\n");
                break;
            }

        }
    }
    else if (option == QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM)
    {
        printf("\n ========================================================== \n");
        for (i = 0; i < QCARCAM_ISP_PARAM_NUM; i++)
        {
            printf("'%d'....%s \t\t", g_qcarcam_isp_menu[i].id, g_qcarcam_isp_menu[i].str);
            if (i%2) printf("\n");
        }
        printf("'%d'....Get all settings", QCARCAM_ISP_PARAM_NUM);
        printf("\n ========================================================== \n");
        if (fgets(buf, sizeof(buf), stdin) != NULL)
        {
            /* 10 indicates decimal number*/
            ch = strtol(buf, &p, 10);
            if (ch < QCARCAM_ISP_PARAM_NUM)
            {
                SET_BIT(param->isp_ctrls.param_mask, ch);
            }
            else
            {
                param->isp_ctrls.param_mask = ~0x0; // Set all bits
            }
        }
        else
        {
            printf("failed to get input\n");
        }
    }
}

static void display_menu()
{
    int i = 0;

    printf("\n ========================================================== \n");

    for (i = QCARCAM_TEST_MENU_FIRST_ITEM; i < QCARCAM_TEST_MENU_MAX; i++)
    {
        printf("'%d'....%s \t\t", g_qcarcam_menu[i].id, g_qcarcam_menu[i].str);

        i++;
        if (i < QCARCAM_TEST_MENU_MAX)
        {
            printf("'%d'....%s\n", g_qcarcam_menu[i].id, g_qcarcam_menu[i].str);
        }
        else
        {
            printf("\n");
            break;
        }
    }

    printf(" 'h'.....display menu \n");
    printf(" 's'.....Dump Image \n");
    printf(" 'e'.....Exit \n");
    printf("\n =========================================================== \n");
    printf(" Enter your choice\n");
}

static qcarcam_test_input_t* get_input_ctxt(int stream_id)
{
    if (stream_id >= 0 && stream_id < gCtxt.numInputs)
    {
        return &gCtxt.inputs[stream_id];
    }

    printf("Wrong stream id entered, please check the input xml\n");

    return NULL;
}

void parse_diagnostics(AisDiagnosticInfo* diagnosticInfo)
{
    if(!diagnosticInfo)
        return;
    //parsing client Diagnostic Info
    AisDiagClientInfo* pDiagClientInfo = diagnosticInfo->aisDiagClientInfo;
    for (uint32 i = 0; i < MAX_USR_CLIENTS; i++)
    {
        qcarcam_test_input_t *input_ctxt = NULL;
        if (!pDiagClientInfo[i].usrHdl)
        {
            break;
        }
        for (uint32 j = 0; j < (uint32)gCtxt.numInputs; j++)
        {
            if (pDiagClientInfo[i].usrHdl == gCtxt.inputs[j].qcarcam_context)
            {
                input_ctxt = &gCtxt.inputs[j];
                break;
            }
        }

        if (!input_ctxt)
        {
            QCARCAM_INFOMSG("This handle is invalid %p", pDiagClientInfo[i].usrHdl);
        }
        else
        {
            AisDiagClientInfo *usrInfo = &pDiagClientInfo[i];

            QCARCAM_INFOMSG("usrHdl:%p usr_state:%d inputId:%d opMode:%d inputDevId:%d"
                "csiPhyDevId:%d ifeDevId:%d rdiId:%d timeStampStart:%llu sofCounter:%llu frameCounter:%llu",
                usrInfo->usrHdl, usrInfo->state, usrInfo->inputId, usrInfo->opMode,
                usrInfo->inputDevId, usrInfo->csiphyDevId, usrInfo->ifeDevId, usrInfo->rdiId,
                usrInfo->timeStampStart, usrInfo->sofCounter, usrInfo->frameCounter);
            for (int j = 0; j < QCARCAM_MAX_NUM_BUFFERS; j++)
            {
                BufferInfo *bufInfo = &usrInfo->bufInfo[j];
                QCARCAM_INFOMSG("bufId:%d bufStatus:%d", bufInfo->bufId, bufInfo->bufStatus);
            }
        }
    }

    //parsing Input device statistics info
    AisDiagInputDeviceInfo* pDiagInputInfo = diagnosticInfo->aisDiagInputDevInfo;
    for(uint32 i = 0; i < MAX_NUM_INPUT_DEVICES; i++)
    {
        AisDiagInputDeviceInfo* inputInfo = &pDiagInputInfo[i];

        QCARCAM_INFOMSG("DevId:%d numSensors:%d cciDevId:%d cciPortId:%d state:%d srcIdEnableMask:%d", inputInfo->inputDevId, inputInfo->numSensors,
            inputInfo->cciMap.cciDevId, inputInfo->cciMap.cciPortId, inputInfo->state, inputInfo->srcIdEnableMask);

        for (uint32 j = 0; j < (uint32)inputInfo->numSensors; j++)
        {
            InputSourceInfo* sourceInfo = &inputInfo->inputSourceInfo[j];

            QCARCAM_INFOMSG("inputSrcId:%d status:%d fps:%.2f width:%d height:%d format:%d",
                sourceInfo->inputSrcId, sourceInfo->status,
                sourceInfo->sensorMode.res.fps, sourceInfo->sensorMode.res.width,
                sourceInfo->sensorMode.res.height, sourceInfo->sensorMode.fmt);
        }
    }

    //parse csiphy device info
    AisDiagCsiDeviceInfo* pDiagCsiInfo = diagnosticInfo->aisDiagCsiDevInfo;
    for(uint32 i = 0; i < MAX_NUM_CSIPHY_DEVICES; i++)
    {
        AisDiagCsiDeviceInfo* csiInfo = &pDiagCsiInfo[i];

        QCARCAM_INFOMSG("DevId:%d laneMapping:%x numIfeMap:%d ifeMap:%x", csiInfo->csiphyDevId, csiInfo->csiLaneMapping,
            csiInfo->numIfeMap, csiInfo->ifeMap);
    }

    //parse ife device info
    AisDiagIfeDeviceInfo* pDiagIfeInfo = diagnosticInfo->aisDiagIfeDevInfo;
    for (uint32 i = 0; i < MAX_NUM_IFE_DEVICES; i++)
    {
        AisDiagIfeDeviceInfo* ifeInfo = &pDiagIfeInfo[i];

        QCARCAM_INFOMSG("DevId:%d csiDevId:%d numRdi:%d csidPktsReceived:%llu", ifeInfo->ifeDevId, ifeInfo->csiDevId,
            ifeInfo->numRdi, ifeInfo->csidPktsRcvd);

        for(uint32 j = 0; j < (uint32)ifeInfo->numRdi; j++)
        {
            RdiInfo *rdiInfo = &ifeInfo->rdiInfo[j];

            QCARCAM_INFOMSG("rdiId:%d rdiStatus:%d", rdiInfo->rdiId, rdiInfo->rdiStatus);
        }
    }

    //parse Error Info
    AisDiagErrorInfo* pDiagErrorInfo = diagnosticInfo->aisDiagErrInfo;
    for(uint32 i = 0; i < MAX_ERR_QUEUE_SIZE; i++)
    {
        AisDiagErrorInfo* errorInfo = &pDiagErrorInfo[i];
        QCARCAM_INFOMSG("errorType:%d errorStatus:%d usrHdl:%p inputSrcId:%d inputDevId:%d csiphyId:%d"
            "ifeDevId:%d rdiId:%d errorTimeStamp:%llu", errorInfo->errorType, errorInfo->payload[0],
            errorInfo->usrHdl, errorInfo->inputSrcId, errorInfo->inputDevId, errorInfo->csiphyDevId,
            errorInfo->ifeDevId, errorInfo->rdiId, errorInfo->errorTimeStamp);
    }
}

static void process_cmds(uint32 option)
{
    int ret = 0;
    int stream_id;
    qcarcam_test_input_t *input_ctxt = NULL;

    switch (option)
    {
    case QCARCAM_TEST_MENU_STREAM_OPEN:
    {
        printf("This feature is not enabled yet!\n");
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_CLOSE:
    {
        printf("This feature is not enabled yet!!!!!!\n");
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_START:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt)
        {
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_STOP)
            {
                ret = qcarcam_input_start(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_START_ALL:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_STOP)
            {
                ret = qcarcam_input_start(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_STOP:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt)
        {
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_START ||
                input_ctxt->state == QCARCAMTEST_STATE_ERROR)
            {
                ret = qcarcam_input_stop(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_STOP_ALL:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];

            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_START ||
                input_ctxt->state == QCARCAMTEST_STATE_ERROR)
            {
                ret = qcarcam_input_stop(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_PAUSE:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt)
        {
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_START)
            {
                ret = qcarcam_input_pause(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_PAUSE_ALL:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];

            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_START)
            {
                ret = qcarcam_input_pause(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_RESUME:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt)
        {
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_PAUSE)
            {
                ret = qcarcam_input_resume(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_RESUME_ALL:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];
            pthread_mutex_lock(&input_ctxt->mutex);
            if (input_ctxt->state == QCARCAMTEST_STATE_PAUSE)
            {
                ret = qcarcam_input_resume(input_ctxt);
            }
            pthread_mutex_unlock(&input_ctxt->mutex);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SET_EXPOSURE:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
            char buf[BUFSIZE];
            char *p = NULL;
            int param_option = 0;

            printf("Select exposure type: [0]Single Manual, [1]Single Auto, [2]HDR Manual, [3]HDR Auto \n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                param_option = strtol(buf, &p, 10);
            }

            switch (param_option)
            {
                case 0:
                {
                    get_input_exposure(&param.exposure_config);
                    param.exposure_config.exposure_mode_type = QCARCAM_EXPOSURE_MANUAL;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EXPOSURE, &param);
                    break;
                }
                case 1:
                {
                    param.exposure_config.exposure_mode_type = QCARCAM_EXPOSURE_AUTO;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EXPOSURE, &param);
                    break;
                }
                case 2:
                {
                    get_input_hdr_exposure(&param.hdr_exposure_config);
                    param.hdr_exposure_config.exposure_mode_type = QCARCAM_EXPOSURE_MANUAL;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_HDR_EXPOSURE, &param);
                    break;
                }
                case 3:
                {
                    param.hdr_exposure_config.exposure_mode_type = QCARCAM_EXPOSURE_AUTO;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_HDR_EXPOSURE, &param);
                    break;
                }
                default:
                {
                    printf("Invalid input");
                    break;
                }
            }

            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("qcarcam_s_param (%d) failed for stream id %d, ret = %d", param_option, input_ctxt->qcarcam_input_id, ret);
            }
            else
            {
                QCARCAM_INFOMSG("qcarcam_s_param (%d) success for stream id %d", param_option, input_ctxt->qcarcam_input_id);
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
            memset(&param.isp_ctrls, 0x0, sizeof(qcarcam_param_isp_ctrls_t));
            input_isp_settings(&param, QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM);
            ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_ISP_CTRLS, &param);

            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("qcarcam_s_param (%d) failed for stream id %d, ret = %d", (int)QCARCAM_TEST_MENU_STREAM_SET_ISP_PARAM, input_ctxt->qcarcam_input_id, ret);
            }
        }
        display_menu();
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
            memset(&param.isp_ctrls, 0x0, sizeof(qcarcam_param_isp_ctrls_t));
            input_isp_settings(&param, QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM);
            ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_ISP_CTRLS, &param);

            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("qcarcam_g_param (%d) failed for stream id %d, ret = %d", (int)QCARCAM_TEST_MENU_STREAM_GET_ISP_PARAM, input_ctxt->qcarcam_input_id, ret);
            }
            else
                display_isp_settings(&param);
        }
        display_menu();
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_ENABLE_CALLBACK:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];
            if (input_ctxt->qcarcam_context && input_ctxt->use_event_callback)
            {
                qcarcam_param_value_t param;
                param.ptr_value = (void *)qcarcam_test_event_cb;
                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_CB, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d, ret = %d",input_ctxt->qcarcam_input_id, ret);
                    break;
                }

                param.uint_value = QCARCAM_EVENT_FRAME_READY | QCARCAM_EVENT_INPUT_SIGNAL | QCARCAM_EVENT_ERROR;
                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_MASK, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d, ret = %d",input_ctxt->qcarcam_input_id, ret);
                }
                else
                {
                    QCARCAM_INFOMSG("qcarcam_s_param success for stream id %d",input_ctxt->qcarcam_input_id);
                }
            }
            else
            {
                QCARCAM_ERRORMSG("Callback is disabled in xml, please check the input xml");
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_DISABLE_CALLBACK:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];
            if (input_ctxt->qcarcam_context && input_ctxt->use_event_callback)
            {
                qcarcam_param_value_t param;
                param.ptr_value = NULL;
                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_CB, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d, ret = %d",input_ctxt->qcarcam_input_id, ret);
                    break;
                }

                param.uint_value = 0x0;
                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EVENT_MASK, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d, ret = %d",input_ctxt->qcarcam_input_id, ret);
                }
                else
                {
                    QCARCAM_INFOMSG("qcarcam_s_param success for stream id %d",input_ctxt->qcarcam_input_id);
                }
            }
            else
            {
                QCARCAM_ERRORMSG("Callback is disabled in xml, please check the input xml");
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SET_FRAMERATE:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
            get_input_framerate(&param.frame_rate_config);
            ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_FRAME_RATE, &param);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("qcarcam_s_param framerate failed for stream id %d, ret = %d",input_ctxt->qcarcam_input_id, ret);
            }
            else
            {
                QCARCAM_INFOMSG("qcarcam_s_param framerate success for stream id %d",input_ctxt->qcarcam_input_id);
            }
            QCARCAM_INFOMSG("Provided QCARCAM_PARAM_FRAME_RATE: mode = %d, period = %d, pattern = 0x%x",
                param.frame_rate_config.frame_drop_mode,
                param.frame_rate_config.frame_drop_period,
                param.frame_rate_config.frame_drop_pattern);
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SET_COLOR_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            int param_option = 0;
            float param_value = 0.0;
            qcarcam_param_value_t param;

            printf("Select param: [0] Saturation, [1] Hue.\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                param_option = strtol(buf, &p, 10);
            }

            switch (param_option)
            {
                case 0:
                    printf("Enter saturation value. Valid range (-1.0 to 1.0)\n");
                    if (fgets(buf, sizeof(buf), stdin) != NULL)
                    {
                        param_value = strtof(buf, NULL);
                    }

                    param.float_value = param_value;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_SATURATION, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d! ret = %d\n",
                        input_ctxt->qcarcam_input_id, ret);
                    }
                    break;
                case 1:
                    printf("Enter hue value. Valid range (-1.0 to 1.0)\n");
                    if (fgets(buf, sizeof(buf), stdin) != NULL)
                    {
                        param_value = strtof(buf, NULL);
                    }

                    param.float_value = param_value;
                    ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_HUE, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d! ret = %d\n",
                        input_ctxt->qcarcam_input_id, ret);
                    }
                    break;
                default:
                    printf("Param not available\n");
                    break;
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_GET_COLOR_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            int param_option = 0;
            qcarcam_param_value_t param;

            printf("Select param: [0] Saturation, [1] Hue.\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                param_option = strtol(buf, &p, 10);
            }

            switch (param_option)
            {
                case 0:
                    ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_SATURATION, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_g_param failed for stream id %d! ret = %d",
                        input_ctxt->qcarcam_input_id, ret);
                    }
                    break;
                case 1:
                    ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_HUE, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_g_param failed for stream id %d! ret = %d",
                        input_ctxt->qcarcam_input_id, ret);
                    }
                    break;
                default:
                    printf("Param not available\n");
                    ret = -1;
                    break;
            }

            if (ret == 0)
            {
                printf("Param value returned = %.2f\n", param.float_value);
                QCARCAM_ALWZMSG("qcarcam_g_param value returned = %.2f", param.float_value);
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SET_GAMMA_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            int param_option = 0;
            float param_value = 0.0;
            qcarcam_param_value_t param;
            unsigned int len = 0, index = 0;
            unsigned int* p_value;
            FILE* fp = NULL;
            char file_name[128];

            printf("Select gamma config type: [0] common, [1] user defined.\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                param_option = strtol(buf, &p, 10);
            }

            switch (param_option)
            {
            case 0:
            {
                printf("Enter gamma value. Value should be positive\n");
                if (fgets(buf, sizeof(buf), stdin) != NULL)
                {
                    param_value = strtof(buf, NULL);
                }

                if (param_value < 0) {
                    printf("invalid gamma value\n");
                    break;
                }

                param.gamma_config.config_type = QCARCAM_GAMMA_EXPONENT;
                param.gamma_config.gamma.f_value = param_value;

                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_GAMMA, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d! ret = %d",
                    input_ctxt->qcarcam_input_id, ret);
                }

                break;
            }
            case 1:
            {
                printf("Enter table length\n");
                if (fgets(buf, sizeof(buf), stdin) != NULL)
                {
                    /* 10 indicates decimal number*/
                    len = strtol(buf, &p, 10);
                }

                if (len < 0 || len > QCARCAM_MAX_GAMMA_TABLE)
                {
                    QCARCAM_ERRORMSG("invalid length");
                    break;
                }

                printf("Enter table file\n");
                if (fgets(file_name, sizeof(file_name), stdin) != NULL)
                {
                    if (file_name[strlen(file_name) - 1] == '\n')
                    {
                        file_name[strlen(file_name) - 1] = '\0';
                    }
                    fp = fopen(file_name, "rb");
                }

                if (NULL == fp)
                {
                    QCARCAM_ERRORMSG("open file %s failed", file_name);
                    break;
                }

                p_value = (unsigned int *)calloc(len, sizeof(*p_value));
                if (NULL == p_value)
                {
                    QCARCAM_ERRORMSG("Failed to allocate table of size %d", len);
                    fclose(fp);
                    break;
                }

                index = 0;
                while (!feof(fp))
                {
                    if (fgets(buf, sizeof(buf), fp) == NULL)
                    {
                        break;
                    }

                    p_value[index] = strtol(buf, &p, 0);
                    index++;

                    if (index == len)
                    {
                        break;
                    }
                }
                fclose(fp);

                if (index != len)
                {
                    QCARCAM_ERRORMSG("gamma table length does not match");
                    free(p_value);
                    break;
                }

                param.gamma_config.config_type = QCARCAM_GAMMA_KNEEPOINTS;
                param.gamma_config.gamma.table.p_value = p_value;
                param.gamma_config.gamma.table.length = len;
                ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_GAMMA, &param);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("qcarcam_s_param failed for stream id %d! ret = %d",
                        input_ctxt->qcarcam_input_id, ret);
                }

                free(p_value);
                break;
            }
            default:
            {
                printf("Param not available\n");
                break;
            }
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_GET_GAMMA_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            int param_option = 0;
            qcarcam_param_value_t param;
            unsigned int len = 0, index = 0;
            unsigned int* p_value = NULL;
            FILE* fp = NULL;
            char file_name[128];

            printf("Select gamma config type: [0] common, [1] user defined.\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                /* 10 indicates decimal number*/
                param_option = strtol(buf, &p, 10);
            }

            switch (param_option)
            {
                case 0:
                    param.gamma_config.config_type = QCARCAM_GAMMA_EXPONENT;

                    ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_GAMMA, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_g_param failed for stream id %d! ret = %d",
                        input_ctxt->qcarcam_input_id, ret);
                    }
                    else
                    {
                        printf("Returned gamma value: %.2f\n", param.gamma_config.gamma.f_value);
                    }
                    break;
                case 1:
                    printf("Enter table length\n");
                    if (fgets(buf, sizeof(buf), stdin) != NULL)
                    {
                        /* 10 indicates decimal number*/
                        len = strtol(buf, &p, 10);
                    }

                    if (len < 0 || len > QCARCAM_MAX_GAMMA_TABLE)
                    {
                        QCARCAM_ERRORMSG("invalid length");
                        break;
                    }

                    printf("Enter output file_name\n");
                    if (fgets(file_name, sizeof(file_name), stdin) == NULL)
                    {
                        QCARCAM_ERRORMSG("invalid file name");
                        break;
                    }

                    if (file_name[strlen(file_name) - 1] == '\n')
                    {
                        file_name[strlen(file_name) - 1] = '\0';
                    }

                    p_value = (unsigned int *)calloc(len, sizeof(*p_value));
                    if (!p_value)
                    {
                        QCARCAM_ERRORMSG("No enough memory");
                        break;
                    }

                    param.gamma_config.config_type = QCARCAM_GAMMA_KNEEPOINTS;
                    param.gamma_config.gamma.table.p_value = p_value;
                    param.gamma_config.gamma.table.length = len;
                    ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_GAMMA, &param);
                    if (ret != QCARCAM_RET_OK)
                    {
                        QCARCAM_ERRORMSG("qcarcam_g_param failed for stream id %d! ret = %d",
                            input_ctxt->qcarcam_input_id, ret);
                    }
                    else
                    {
                        fp = fopen(file_name, "wb");
                        if (NULL == fp)
                        {
                            QCARCAM_ERRORMSG("open file %s failed:errno=%d", file_name, errno);
                            free(p_value);
                            break;
                        }
                        for (index = 0; index != len; ++index)
                        {
                            fprintf(fp, "0x%x\n", p_value[index]);
                        }
                        fclose(fp);
                    }
                    free(p_value);
                    break;
                default:
                    printf("Param not available\n");
                    break;
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_DUMP_NEXT_FRAME:
    {
        for (stream_id = 0; stream_id < gCtxt.numInputs; ++stream_id)
        {
            input_ctxt = &gCtxt.inputs[stream_id];

            input_ctxt->dumpNextFrame = TRUE;
        }
        break;
    }
    case QCARCAM_TEST_MENU_CHECK_BUFFERS:
        gCtxt.check_buffer_state = !gCtxt.check_buffer_state;
        break;
    case QCARCAM_TEST_MENU_STREAM_SET_VENDOR_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
#ifndef USE_VENDOR_EXT_PARAMS
            param.vendor_param.data[0] = 0x12345678;
            param.vendor_param.data[1] = 0xfedcba98;
#else
            vendor_ext_property_t *p_isp_prop = (vendor_ext_property_t*)&(param.vendor_param.data[0]);
            p_isp_prop->type = VENDOR_EXT_PROP_TEST;
            p_isp_prop->value.uint_val = 0x12345678;
#endif

            ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_VENDOR, &param);
            if (QCARCAM_RET_OK != ret)
            {
                QCARCAM_ERRORMSG("qcarcam_s_param QCARCAM_PARAM_VENDOR failed for stream id %d! ret = %d",
                    input_ctxt->qcarcam_input_id, ret);
            }
            else
            {
#ifndef USE_VENDOR_EXT_PARAMS
                QCARCAM_INFOMSG("qcarcam_s_param QCARCAM_PARAM_VENDOR succeed for stream id %d: data[0]=%u data[1]=%u",
                    input_ctxt->qcarcam_input_id, param.vendor_param.data[0],
                    param.vendor_param.data[1]);
#else
                QCARCAM_INFOMSG("qcarcam_s_param QCARCAM_PARAM_VENDOR succeed for stream id %d: isp prop type=%u,val=%u",
                    input_ctxt->qcarcam_input_id, p_isp_prop->type ,
                    p_isp_prop->value.uint_val);
#endif
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_GET_VENDOR_PARAM:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if (input_ctxt && input_ctxt->qcarcam_context)
        {
            qcarcam_param_value_t param;
#ifndef USE_VENDOR_EXT_PARAMS
            param.vendor_param.data[0] = 0;
#else
            vendor_ext_property_t *p_isp_prop = (vendor_ext_property_t*)&(param.vendor_param.data[0]);
            p_isp_prop->type =  VENDOR_EXT_PROP_TEST;
#endif
            ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_VENDOR, &param);
            if (QCARCAM_RET_OK != ret)
            {
                QCARCAM_ERRORMSG("qcarcam_g_param QCARCAM_PARAM_VENDOR failed for stream id %d! ret = %d",
                    input_ctxt->qcarcam_input_id, ret);
            }
            else
            {
                QCARCAM_INFOMSG("qcarcam_g_param QCARCAM_PARAM_VENDOR succeeds");
#ifdef USE_VENDOR_EXT_PARAMS
                QCARCAM_INFOMSG("qcarcam_g_param QCARCAM_PARAM_VENDOR succeed for stream id %d: isp prop type=%u,val=%u",
                    input_ctxt->qcarcam_input_id, p_isp_prop->type,
                    p_isp_prop->value.uint_val);
                printf("ISP param type = %u, value returned = %u\n", p_isp_prop->type,
                    p_isp_prop->value.uint_val);
#endif
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_SUBSCRIBE_CHANGE_EVENT:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);

        if(input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            uint64 param_option = 0;
            printf("Select setting in hexadecimal (1<<qcarcam_param_t)\n"
                "EXPOSURE [(1<<17) -- 0x20000] HDR EXPOSURE [1<<20 -- 0x100000]\n"
                "SATURATION [1<<19 -- 0x80000] HUE [1<<18 -- 0x40000]\n"
                "FRAME RATE [1<<9 -- 0x200] GAMMA [1<<21 -- 0x200000]\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                param_option = strtoul(buf, &p, 0);
                qcarcam_test_subscribe_input_params_change(input_ctxt, param_option);
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_STREAM_UNSUBSCRIBE_CHANGE_EVENT:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);

        if(input_ctxt && input_ctxt->qcarcam_context)
        {
            char buf[BUFSIZE];
            char *p = NULL;
            uint64 param_option = 0;
            printf("Select setting in hexadecimal (1<<qcarcam_param_t)\n"
                "EXPOSURE [(1<<17) -- 0x20000] HDR EXPOSURE [1<<20 -- 0x100000]\n"
                "SATURATION [1<<19 -- 0x80000] HUE [1<<18 -- 0x40000]\n"
                "FRAME RATE [1<<9 -- 0x200] GAMMA [1<<21 -- 0x200000]\n");
            if (fgets(buf, sizeof(buf), stdin) != NULL)
            {
                param_option = strtoul(buf, &p, 0);
                qcarcam_test_unsubscribe_input_params_change(input_ctxt, param_option);
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_MASTER:
    {
        stream_id = get_input_stream_id();
        input_ctxt = get_input_ctxt(stream_id);
        if(input_ctxt && input_ctxt->qcarcam_context)
        {
            unsigned int flag = get_input_set_flag();
            if (flag == 1)
            {
                // to prevent setting a master stream as master again
                if(input_ctxt->is_master)
                {
                    QCARCAM_INFOMSG("This client is already set as the master");
                }
                else
                {
                    qcarcam_test_set_master(input_ctxt, flag);
                    if(!input_ctxt->is_master)
                    {
                        QCARCAM_INFOMSG("There is already another client set as master");
                    }
                }

            }
            else if (flag == 0)
            {
                if(!input_ctxt->is_master)
                {
                    QCARCAM_INFOMSG("This client is not master in order to release, stream_id:%d", stream_id);
                }
                else
                {
                    qcarcam_test_set_master(input_ctxt, flag);
                }
            }
            else
            {
                QCARCAM_ERRORMSG("Invalid option (%d)", flag);
            }
        }
        break;
    }
    case QCARCAM_TEST_MENU_GET_SYSTEM_DIAGNOSTICS:
    {
        AisDiagnosticInfo* diagnosticInfo = (AisDiagnosticInfo*)calloc(1, sizeof(AisDiagnosticInfo));
        if (!diagnosticInfo)
        {
            QCARCAM_ERRORMSG("Failed to allocate diagnosticInfo");
            break;
        }
        memset(diagnosticInfo, 0x0, sizeof(AisDiagnosticInfo));
        ret = qcarcam_query_diagnostics((void *)diagnosticInfo, sizeof(AisDiagnosticInfo));
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("qcarcam_query_diagnostics failed with ret = %d", ret);
        }
        else
        {
            parse_diagnostics(diagnosticInfo);
            QCARCAM_INFOMSG("qcarcam_query_diagnostics is success and parsed");
        }

        free(diagnosticInfo);
        break;
    }
    default:
        QCARCAM_ERRORMSG("Invalid option (%d)", option);
        break;
    }
}

int main(int argc, char **argv)
{
#if defined(__INTEGRITY)
    // Set priority of main thread
    // Task needs to be linked to an object defined in corresponding INT file
    Task mainTask = TaskObjectNumber(QCARCAM_OBJ_NUM);
    SetTaskPriority(mainTask, CameraTranslateThreadPriority(QCARCAM_THRD_PRIO), false);
#endif

    initialize_qcarcam_test_ctxt();

    unsigned long long t_before = 0;
    unsigned long long t_after = 0;

    qcarcam_test_get_time(&gCtxt.t_start);

    const char *tok = NULL;
    qcarcam_ret_t ret = QCARCAM_RET_OK;

    int rval = EXIT_FAILURE;
    unsigned int i = 0;
    int rc = 0;
    int input_idx = 0;

    CameraThread timer_thread_handle = NULL;
    CameraThread signal_thread_handle = NULL;
    CameraThread fps_thread_handle = NULL;
    CameraThread signal_loss_thread_handle = NULL;
    CameraThread csi_error_thread_handle = NULL;
    char thread_name[64];
    int bprint_diag = 0;

    test_util_intr_thrd_args_t intr_thrd_args;

    int remote_attach_loop = 0; // no looping for remote process attach

    /* Should be placed before any forked thread */
    sigfillset(&g_sigset);
    for (i = 0; exceptsigs[i] != -1; i++)
    {
        sigdelset(&g_sigset, exceptsigs[i]);
    }
    pthread_sigmask(SIG_BLOCK, &g_sigset, NULL);

    qcarcam_init_t qcarcam_init = {0};
    qcarcam_init.version = QCARCAM_VERSION;
    qcarcam_init.debug_tag = (char *)"qcarcam_test";

    ret = qcarcam_initialize(&qcarcam_init);
    if (ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("qcarcam_initialize failed %d", ret);
        exit(-1);
    }

    test_util_ctxt_t* test_util_ctxt = NULL;
    test_util_ctxt_params_t test_util_ctxt_params = {};

    QCARCAM_DBGMSG("Arg parse Begin");

    for (i = 1; (int)i < argc; i++)
    {
        if (!strncmp(argv[i], "-config=", strlen("-config=")))
        {
            tok = argv[i] + strlen("-config=");
            snprintf(g_filename, sizeof(g_filename), "%s", tok);
        }
        else if (!strncmp(argv[i], "-dumpFrame=", strlen("-dumpFrame=")))
        {
            tok = argv[i] + strlen("-dumpFrame=");
            gCtxt.dumpFrame = atoi(tok);
        }
        else if (!strncmp(argv[i], "-startStop=", strlen("-startStop=")))
        {
            tok = argv[i] + strlen("-startStop=");
            gCtxt.enableStartStop = atoi(tok);
        }
        else if (!strncmp(argv[i], "-pauseResume=", strlen("-pauseResume=")))
        {
            tok = argv[i] + strlen("-pauseResume=");
            gCtxt.enablePauseResume = atoi(tok);
        }
        else if (!strncmp(argv[i], "-noDisplay", strlen("-noDisplay")))
        {
            gCtxt.disable_display = 1;
            test_util_ctxt_params.disable_display = 1;
        }
        else if (!strncmp(argv[i], "-singlethread", strlen("-singlethread")))
        {
            gCtxt.multithreaded = 0;
        }
        else if (!strncmp(argv[i], "-printfps", strlen("-printfps")))
        {
            tok = argv[i] + strlen("-printfps=");
            gCtxt.fps_print_delay = atoi(tok);
        }
        else if (!strncmp(argv[i], "-checkbuf", strlen("-checkbuf")))
        {
            tok = argv[i] + strlen("-checkbuf=");
            gCtxt.check_buffer_state = atoi(tok);
        }
        else if (!strncmp(argv[i], "-showStats=", strlen("-showStats=")))
        {
            tok = argv[i] + strlen("-showStats=");
            gCtxt.enableStats = atoi(tok);
        }
        else if (!strncmp(argv[i], "-c2d=", strlen("-c2d=")))
        {
            tok = argv[i] + strlen("-c2d=");
            gCtxt.enable_c2d = atoi(tok);
            test_util_ctxt_params.enable_c2d = gCtxt.enable_c2d;
        }
#ifdef ENABLE_CL_CONVERTER
        else if (!strncmp(argv[i], "-csc=", strlen("-csc=")))
        {
            tok = argv[i] + strlen("-csc=");
            gCtxt.enable_csc = atoi(tok);
            test_util_ctxt_params.enable_csc = gCtxt.enable_csc;
        }
#endif
        else if (!strncmp(argv[i], "-di=", strlen("-di=")))
        {
            tok = argv[i] + strlen("-di=");
            gCtxt.enable_deinterlace = atoi(tok);
            test_util_ctxt_params.enable_di = gCtxt.enable_deinterlace;
        }
        else if (!strncmp(argv[i], "-nonInteractive", strlen("-nonInteractive")) ||
                !strncmp(argv[i], "-nomenu", strlen("-nomenu")))
        {
            gCtxt.enableMenuMode = 0;
        }
        else if (!strncmp(argv[i], "-fatalErrorRecover", strlen("-fatalErrorRecover")))
        {
            /* only one of them enabled in real scenario */
            gCtxt.enableBridgeErrorDetect = 0;
            gCtxt.enableFatalErrorRecover = 1;
        }
        else if (!strncmp(argv[i], "-printlatency", strlen("-printlatency")))
        {
            bprint_diag = 1;
        }
        else if (!strncmp(argv[i], "-seconds=", strlen("-seconds=")))
        {
            tok = argv[i] + strlen("-seconds=");
            gCtxt.exitSeconds = atoi(tok);
        }
        else if (!strncmp(argv[i], "-retry", strlen("-retry")))
        {
            /* Used to wait for query inputs to handoff early RVC */
            gCtxt.enableRetry = 1;
        }
        else if (!strncmp(argv[i], "-gpioNumber=", strlen("-gpioNumber=")))
        {
            tok = argv[i]+ strlen("-gpioNumber=");
            gCtxt.gpioNumber = atoi(tok);
            test_util_ctxt_params.enable_gpio_config = 1;
        }
        else if (!strncmp(argv[i], "-gpioMode=", strlen("-gpioMode=")))
        {
            /**
            *   Mode 0 : Visiblity toggle
            *   Mode 1 : Play/Pause toggle
            *   Mode 2 : Start/Stop toggle
            */
            tok = argv[i] + strlen("-gpioMode=");
            gCtxt.gpioMode = atoi(tok);
        }
        else
        {
            QCARCAM_ERRORMSG("Invalid argument, argv[%d]=%s", i, argv[i]);
            exit(-1);
        }
    }

    QCARCAM_DBGMSG("Arg parse End");

    qcarcam_test_get_time(&t_before);

    QCARCAM_DBGMSG("test_util_init");
    ret = test_util_init(&test_util_ctxt, &test_util_ctxt_params);
    if (ret != QCARCAM_RET_OK)
    {
        QCARCAM_ERRORMSG("test_util_init failed");
        exit(-1);
    }

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "test_util_init : %lu ms", (t_after - t_before));
    t_before = t_after;

    /*parse xml*/
    test_util_xml_input_t *xml_inputs = (test_util_xml_input_t *)calloc(NUM_MAX_CAMERAS, sizeof(test_util_xml_input_t));
    if (!xml_inputs)
    {
        QCARCAM_ERRORMSG("Failed to allocate xml input struct");
        exit(-1);
    }

    /*If exposure is not set in XML file, set manual_exp to -1 to use default exposure setting*/
    for (i = 0; i < NUM_MAX_CAMERAS; i++)
    {
        xml_inputs[i].exp_params.manual_exp = -1;
    }

    gCtxt.numInputs = test_util_parse_xml_config_file(g_filename, xml_inputs, NUM_MAX_CAMERAS);
    if (gCtxt.numInputs <= 0)
    {
        QCARCAM_ERRORMSG("Failed to parse config file!");
        exit(-1);
    }

    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "xml parsing : %lu ms", (t_after - t_before));
    t_before = t_after;


    /*query qcarcam*/
    qcarcam_input_t *pInputs;
    unsigned int queryNumInputs = 0, queryFilled = 0;
    /*retry used for early camera handoff for RVC*/
    do {
        ret = qcarcam_query_inputs(NULL, 0, &queryNumInputs);
        if (QCARCAM_RET_OK != ret || queryNumInputs == 0)
        {
            QCARCAM_ERRORMSG("Failed qcarcam_query_inputs number of inputs with ret %d", ret);
            if (gCtxt.enableRetry == 0)
            {
                exit(-1);
            }
        }
        else
        {
            break;
        }
        if (gCtxt.enableRetry == 1)
        {
            usleep(500000);
        }
    } while (gCtxt.enableRetry == 1);

    pInputs = (qcarcam_input_t *)calloc(queryNumInputs, sizeof(*pInputs));
    if (!pInputs)
    {
        QCARCAM_ERRORMSG("Failed to allocate pInputs");
        exit(-1);
    }

    ret = qcarcam_query_inputs(pInputs, queryNumInputs, &queryFilled);
    if (QCARCAM_RET_OK != ret || queryFilled != queryNumInputs)
    {
        QCARCAM_ERRORMSG("Failed qcarcam_query_inputs with ret %d %d %d", ret, queryFilled, queryNumInputs);
        exit(-1);
    }

    QCARCAM_INFOMSG("--- QCarCam Queried Inputs ----");
    for (i = 0; i < queryFilled; i++)
    {
        QCARCAM_INFOMSG("%d: input_id=%d, res=%dx%d fmt=0x%08x fps=%.2f flags=0x%x",
            i, pInputs[i].desc,
            pInputs[i].res[0].width, pInputs[i].res[0].height,
            pInputs[i].color_fmt[0], pInputs[i].res[0].fps, pInputs[i].flags);
    }
    qcarcam_test_get_time(&t_after);
    QCARCAM_PERFMSG(gCtxt.enableStats, "query inputs : %lu ms", (t_after - t_before));
    QCARCAM_INFOMSG("-------------------------------");

    while (remote_attach_loop)
    {
        volatile int i;
        for (i = 0; i < 1000; i++)
            ;
    }

#ifndef C2D_DISABLED
    if (gCtxt.enable_c2d || gCtxt.enable_csc)
    {
        QCARCAM_DBGMSG("C2D Setup Begin");

        /*init C2D*/
        C2D_DRIVER_SETUP_INFO set_driver_op = {
            .max_surface_template_needed = (NUM_MAX_DISP_BUFS + QCARCAM_MAX_NUM_BUFFERS) * NUM_MAX_CAMERAS,
        };
        C2D_STATUS c2d_status = c2dDriverInit(&set_driver_op);
        if (c2d_status != C2D_STATUS_OK)
        {
            QCARCAM_ERRORMSG("c2dDriverInit failed");
            goto fail;
        }
    }
#endif

    /*signal handler thread*/
    snprintf(thread_name, sizeof(thread_name), "signal_thrd");
    rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &signal_thread, 0, 0, thread_name, &signal_thread_handle);
    if (rc)
    {
        QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
        goto fail;
    }

#ifdef __QNXNTO__
    snprintf(thread_name, sizeof(thread_name), "signal_loss_thrd");
    rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &check_signal_loss_thread, 0, 0, thread_name, &signal_loss_thread_handle);
    if (rc)
    {
        QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
        goto fail;
    }
#endif

    if (gCtxt.enableFatalErrorRecover)
    {
        snprintf(thread_name, sizeof(thread_name), "check_error_thread");
        rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &check_error_thread, 0, 0, thread_name, &csi_error_thread_handle);
        if (rc)
        {
            QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
            goto fail;
        }
    }

    /*if GPIO interrupts have been enabled*/
    if (gCtxt.gpioNumber > 0)
    {
        uint32_t irq;
        test_util_trigger_type_t trigger;

        switch (gCtxt.gpioMode)
        {
        case TESTUTIL_GPIO_MODE_VISIBILITY:
            // gpioMode 0: visibility toggle
            QCARCAM_INFOMSG("Selected GPIO Mode: Visibility toggle");
            trigger = TESTUTIL_GPIO_INTERRUPT_TRIGGER_EDGE;
            intr_thrd_args.cb_func = qcarcam_test_gpio_interrupt_cb;
            break;
        case TESTUTIL_GPIO_MODE_PLAYPAUSE:
            // gpioMode 1: play/pause toggle
            QCARCAM_INFOMSG("Selected GPIO Mode: Play/Pause toggle");
            trigger = TESTUTIL_GPIO_INTERRUPT_TRIGGER_FALLING;
            intr_thrd_args.cb_func = qcarcam_test_gpio_play_pause_cb;
            break;
        case TESTUTIL_GPIO_MODE_STARTSTOP:
            // gpioMode 2: start/stop toggle
            QCARCAM_INFOMSG("Selected GPIO Mode: Start/Stop toggle");
            trigger = TESTUTIL_GPIO_INTERRUPT_TRIGGER_FALLING;
            intr_thrd_args.cb_func = qcarcam_test_gpio_start_stop_cb;
            break;
        default:
            QCARCAM_ERRORMSG("Invalid mode");
            goto fail;
        }

        if (test_util_gpio_interrupt_config(&irq, gCtxt.gpioNumber, trigger) != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_gpio_interrupt_config failed");
            goto fail;
        }

        intr_thrd_args.irq = irq;

        if (test_util_interrupt_attach(&intr_thrd_args) != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_interrupt_attach failed");
            goto fail;
        }
    }

    /*thread used frame rate measurement*/
    if (gCtxt.fps_print_delay)
    {
        snprintf(thread_name, sizeof(thread_name), "framerate_thrd");
        rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &framerate_thread, 0, 0, thread_name, &fps_thread_handle);
        if (rc)
        {
            QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
            goto fail;
        }
    }

    for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
    {
        qcarcam_test_input_t* input_ctxt = &gCtxt.inputs[input_idx];
        test_util_xml_input_t* p_xml_input = &xml_inputs[input_idx];

        pthread_mutex_init(&input_ctxt->mutex, NULL);
        pthread_mutex_init(&input_ctxt->queue_mutex, NULL);
        input_ctxt->state = QCARCAMTEST_STATE_INIT;

        input_ctxt->qcarcam_input_id = p_xml_input->properties.qcarcam_input_id;
        input_ctxt->op_mode = p_xml_input->properties.op_mode;
        input_ctxt->use_event_callback = p_xml_input->properties.use_event_callback;
        input_ctxt->subscribe_parameter_change = p_xml_input->properties.subscribe_parameter_change;

        /*no timeout when using event callback as frame is ready as soon as we get callback*/
        if (input_ctxt->use_event_callback)
        {
            input_ctxt->frame_timeout = 0;
        }
        else
        {
            input_ctxt->frame_timeout = (p_xml_input->properties.frame_timeout == -1) ?
                QCARCAM_TEST_DEFAULT_GET_FRAME_TIMEOUT :
                p_xml_input->properties.frame_timeout * 1000 * 1000;
        }


        for (i = 0; i < queryFilled; i++)
        {
            if (pInputs[i].desc == input_ctxt->qcarcam_input_id)
            {
                input_ctxt->query_inputs_idx = i;
            }
        }

        //Set output buffer params
        input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers = p_xml_input->output_params.n_buffers;
        if (p_xml_input->output_params.width == -1 || p_xml_input->output_params.height == -1)
        {
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].width = pInputs[input_ctxt->query_inputs_idx].res[0].width;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].height = pInputs[input_ctxt->query_inputs_idx].res[0].height;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].width = pInputs[input_ctxt->query_inputs_idx].res[0].width;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].height = pInputs[input_ctxt->query_inputs_idx].res[0].height;
        }
        else
        {
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].width = p_xml_input->output_params.width;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].height = p_xml_input->output_params.height;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].width = p_xml_input->output_params.width;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].height = p_xml_input->output_params.height;
        }

        if (p_xml_input->output_params.format == -1)
        {
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].format =
                pInputs[input_ctxt->query_inputs_idx].color_fmt[0];
        }
        else
        {
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].format =
                p_xml_input->output_params.format;
        }

        input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers = p_xml_input->window_params.n_buffers_display;
        input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].format = QCARCAM_FMT_RGB_888;

        input_ctxt->window_params = p_xml_input->window_params;
        input_ctxt->window_params.buffer_size[0] = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].width;
        input_ctxt->window_params.buffer_size[1] = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].height;
        input_ctxt->window_params.visibility = 1;
        //set window debug name to XML input index
        snprintf(input_ctxt->window_params.debug_name, sizeof(input_ctxt->window_params.debug_name), "qcarcam_%d", input_idx);

        input_ctxt->window_params.flags = pInputs[input_ctxt->query_inputs_idx].flags;

        //Capture exposure params if any
        input_ctxt->exp_time = p_xml_input->exp_params.exposure_time;
        input_ctxt->gain = p_xml_input->exp_params.gain;
        input_ctxt->manual_exposure = p_xml_input->exp_params.manual_exp;

        //Capture frame rate params if any
        input_ctxt->frame_rate_config = p_xml_input->output_params.frame_rate_config;

        if (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))
        {
            if (gCtxt.enable_c2d || gCtxt.enable_csc)
            {
                QCARCAM_ERRORMSG("c2d & deinterlace can't be used at the same time");
                goto fail;
            }

            input_ctxt->window_params.buffer_size[1] = DE_INTERLACE_HEIGHT;
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].height = DE_INTERLACE_HEIGHT;
        }

        //Use input 11 for injection
        if (input_ctxt->qcarcam_input_id == QCARCAM_TEST_INPUT_INJECTION)
        {
#ifdef ENABLE_INJECTION_SUPPORT
            size_t size;

            input_ctxt->is_injection = 1;

            input_ctxt->p_buffers_input.n_buffers = 1;
            input_ctxt->p_buffers_input.color_fmt = (qcarcam_color_fmt_t)QCARCAM_COLOR_FMT(p_xml_input->inject_params.pattern, p_xml_input->inject_params.bitdepth, p_xml_input->inject_params.pack);
            input_ctxt->p_buffers_input.buffers = (qcarcam_buffer_t *)calloc(input_ctxt->p_buffers_input.n_buffers, sizeof(*input_ctxt->p_buffers_input.buffers));

            if (input_ctxt->p_buffers_input.buffers == 0)
            {
                QCARCAM_ERRORMSG("alloc qcarcam_buffer input buffers failed");
                goto fail;
            }

            input_ctxt->p_buffers_input.buffers[0].n_planes = 1;
            input_ctxt->p_buffers_input.buffers[0].planes[0].width = p_xml_input->inject_params.buffer_size[0];
            input_ctxt->p_buffers_input.buffers[0].planes[0].height = p_xml_input->inject_params.buffer_size[1];
            input_ctxt->p_buffers_input.buffers[0].planes[0].stride = p_xml_input->inject_params.stride;

            size = input_ctxt->p_buffers_input.buffers[0].planes[0].stride * input_ctxt->p_buffers_input.buffers[0].planes[0].height;

            ret = test_util_allocate_input_buffers(&input_ctxt->p_buffers_input, size);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_alloc_input_buffers failed");
                goto fail;
            }

            //TODO: this only works for single buffer for now...
            ret = test_util_read_input_data(&input_ctxt->p_buffers_input, 0, p_xml_input->inject_params.filename, size);
#else
            QCARCAM_ERRORMSG("Injection not supported!");
            goto fail;
#endif
        }
    }

    QCARCAM_DBGMSG("Buffer Setup Begin");

    for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];
        input_ctxt->test_util_ctxt = test_util_ctxt;
        unsigned int output_n_buffers = 0;

        // Allocate an additional buffer to be shown in case of signal loss
        output_n_buffers = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers + 1;
        input_ctxt->p_buffers_output.n_buffers = output_n_buffers;

        input_ctxt->p_buffers_output.color_fmt = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].format;
        input_ctxt->p_buffers_output.buffers = (qcarcam_buffer_t *)calloc(input_ctxt->p_buffers_output.n_buffers, sizeof(*input_ctxt->p_buffers_output.buffers));
        input_ctxt->diag.bprint = bprint_diag;
        if (input_ctxt->p_buffers_output.buffers == 0)
        {
            QCARCAM_ERRORMSG("alloc qcarcam_buffer failed");
            goto fail;
        }

        qcarcam_test_get_time(&t_before);

        if (input_ctxt->window_params.flags & QCARCAM_INPUT_FLAG_CONTENT_PROTECTED)
        {
            input_ctxt->p_buffers_output.flags = QCARCAM_BUFFER_FLAG_SECURE;
        }

        for (i = 0; i < output_n_buffers; ++i)
        {
            input_ctxt->p_buffers_output.buffers[i].n_planes = 1;
            input_ctxt->p_buffers_output.buffers[i].planes[0].width = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].width;
            input_ctxt->p_buffers_output.buffers[i].planes[0].height = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].height;
        }

        ret = test_util_init_window(test_util_ctxt, &input_ctxt->qcarcam_window);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_init_window failed for qcarcam_window");
            goto fail;
        }

        test_util_set_diag(test_util_ctxt, input_ctxt->qcarcam_window, &input_ctxt->diag);

        if (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))
        {
            ret = test_util_init_window(test_util_ctxt, &input_ctxt->display_window);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_init_window failed for display_window");
                goto fail;
            }
            test_util_set_diag(test_util_ctxt, input_ctxt->display_window, &input_ctxt->diag);
        }

        if (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))
        {
            ret = test_util_set_window_param(test_util_ctxt, input_ctxt->display_window, &input_ctxt->window_params);
        }
        else
        {
            ret = test_util_set_window_param(test_util_ctxt, input_ctxt->qcarcam_window, &input_ctxt->window_params);
        }

        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_set_window_param failed");
            goto fail;
        }

        ret = test_util_init_window_buffers(test_util_ctxt, input_ctxt->qcarcam_window, &input_ctxt->p_buffers_output);
        if (ret != QCARCAM_RET_OK)
        {
            QCARCAM_ERRORMSG("test_util_init_window_buffers failed");
            goto fail;
        }

        if (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))
        {
            input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].format = QCARCAM_FMT_UYVY_8;
            input_ctxt->p_buffers_disp.color_fmt = (qcarcam_color_fmt_t)input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].format;
            input_ctxt->p_buffers_disp.n_buffers = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers;
            input_ctxt->p_buffers_disp.buffers = (qcarcam_buffer_t *)calloc(input_ctxt->p_buffers_disp.n_buffers, sizeof(*input_ctxt->p_buffers_disp.buffers));
            if (input_ctxt->p_buffers_disp.buffers == 0)
            {
                QCARCAM_ERRORMSG("alloc qcarcam_buffer failed");
                goto fail;
            }

            for (i = 0; i < input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers; ++i)
            {
                input_ctxt->p_buffers_disp.buffers[i].n_planes = 1;
                input_ctxt->p_buffers_disp.buffers[i].planes[0].width = input_ctxt->window_params.buffer_size[0];
                input_ctxt->p_buffers_disp.buffers[i].planes[0].height = input_ctxt->window_params.buffer_size[1];
            }

            ret = test_util_init_window_buffers(test_util_ctxt, input_ctxt->display_window, &input_ctxt->p_buffers_disp);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_init_window_buffers failed");
                goto fail;
            }
        }

        qcarcam_test_get_time(&t_after);
        QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam window init (idx %d) : %lu ms", input_ctxt->idx, (t_after - t_before));
        t_before = t_after;

        input_ctxt->p_buffers_output.n_buffers = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers;

        if (gCtxt.enable_c2d || gCtxt.enable_csc)
        {
            for (i = 0; i < input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_OUTPUT].n_buffers; ++i)
            {
                test_util_create_c2d_surface(input_ctxt->test_util_ctxt, input_ctxt->qcarcam_window, i);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("test_util_create_c2d_surface failed");
                    goto fail;
                }
            }

            qcarcam_test_get_time(&t_after);
            QCARCAM_PERFMSG(gCtxt.enableStats, "qcarcam create_c2d_surface (idx %d) : %lu ms", input_ctxt->idx, (t_after - t_before));
            t_before = t_after;

            /*display window*/
            input_ctxt->p_buffers_disp.n_buffers = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers;
            input_ctxt->p_buffers_disp.color_fmt = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].format;
            input_ctxt->p_buffers_disp.buffers = (qcarcam_buffer_t *)calloc(input_ctxt->p_buffers_disp.n_buffers, sizeof(*input_ctxt->p_buffers_disp.buffers));
            if (input_ctxt->p_buffers_disp.buffers == 0)
            {
                QCARCAM_ERRORMSG("alloc qcarcam_buffer failed");
                goto fail;
            }

            for (i = 0; i < input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers; ++i)
            {
                input_ctxt->p_buffers_disp.buffers[i].n_planes = 1;
                input_ctxt->p_buffers_disp.buffers[i].planes[0].width = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].width;
                input_ctxt->p_buffers_disp.buffers[i].planes[0].height = input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].height;
            }

            ret = test_util_init_window(test_util_ctxt, &input_ctxt->display_window);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_init_window failed");
                goto fail;
            }

            test_util_set_diag(test_util_ctxt, input_ctxt->display_window, &input_ctxt->diag);

            ret = test_util_set_window_param(test_util_ctxt, input_ctxt->display_window, &input_ctxt->window_params);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_set_window_param failed");
                goto fail;
            }

            ret = test_util_init_window_buffers(test_util_ctxt, input_ctxt->display_window, &input_ctxt->p_buffers_disp);
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("test_util_init_window_buffers failed");
                goto fail;
            }

            qcarcam_test_get_time(&t_after);
            QCARCAM_PERFMSG(gCtxt.enableStats, "display window init (idx %d) : %lu ms", input_ctxt->idx, (t_after - t_before));
            t_before = t_after;

            for (i = 0; i < input_ctxt->buffers_param[QCARCAM_TEST_BUFFERS_DISPLAY].n_buffers; ++i)
            {
                test_util_create_c2d_surface(input_ctxt->test_util_ctxt, input_ctxt->display_window, i);
                if (ret != QCARCAM_RET_OK)
                {
                    QCARCAM_ERRORMSG("test_util_create_c2d_surface failed");
                    goto fail;
                }
            }

            qcarcam_test_get_time(&t_after);
            QCARCAM_PERFMSG(gCtxt.enableStats, "display create_c2d_surface (idx %d) : %lu ms", input_ctxt->idx, (t_after - t_before));
            t_before = t_after;

#ifdef ENABLE_CL_CONVERTER
            input_ctxt->g_converter = csc_create();

            ret = test_util_init_cl_converter(test_util_ctxt, input_ctxt->qcarcam_window, input_ctxt->display_window, input_ctxt->g_converter, &(input_ctxt->source_surface), &(input_ctxt->target_surface));
            if (ret != QCARCAM_RET_OK)
            {
                QCARCAM_ERRORMSG("cl converter initialization failed");
                goto fail;
            }
            qcarcam_test_get_time(&t_after);
            QCARCAM_PERFMSG(gCtxt.enableStats, "init cl convert : %lu ms", (t_after - t_before));
            t_before = t_after;
#endif

        }
    }


    QCARCAM_DBGMSG("Buffer Setup End");
    QCARCAM_DBGMSG("Create qcarcam_context Begin");

    /*launch threads to do the work for multithreaded*/
    if (gCtxt.multithreaded)
    {
        for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
        {
            qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

            input_ctxt->idx = input_idx;
            snprintf(thread_name, sizeof(thread_name), "inpt_ctxt_%d", input_idx);

            rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &qcarcam_test_setup_input_ctxt_thread, &gCtxt.inputs[input_idx], 0, thread_name, &input_ctxt->thread_handle);
            if (rc)
            {
                QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
                goto fail;
            }

            rc = CameraCreateSignal(&gCtxt.inputs[input_idx].m_eventHandlerSignal);
            if (rc)
            {
                QCARCAM_ERRORMSG("CameraCreateSignal failed, rc=%d", rc);
                goto fail;
            }

            snprintf(thread_name, sizeof(thread_name), "process_cb_event_%d", input_idx);
            rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &process_cb_event_thread, &gCtxt.inputs[input_idx], 0, thread_name, &input_ctxt->process_cb_event_handle);
            if (rc)
            {
                QCARCAM_ERRORMSG("Create cb event process thread failed, rc=%d", rc);
                goto fail;
            }

            if (input_ctxt->delay_time)
            {
                snprintf(thread_name, sizeof(thread_name), "inpt_ctxt_usr_process_%d", input_idx);
                rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &inpt_ctxt_usr_process_thread, &gCtxt.inputs[input_idx], 0, thread_name, &input_ctxt->usr_process_thread_handle);
                if (rc)
                {
                    QCARCAM_ERRORMSG("CameraCreateThread failed : %s", thread_name);
                    goto fail;
                }
            }
        }

        if (gCtxt.exitSeconds)
        {
            snprintf(thread_name, sizeof(thread_name), "timer_thread");
            rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &timer_thread, 0, 0, thread_name, &timer_thread_handle);
            if (rc)
            {
                QCARCAM_ERRORMSG("CameraCreateThread failed, rc=%d", rc);
                goto fail;
            }
        }

        if (gCtxt.enableMenuMode)
        {
            while (!g_aborted)
            {
                if (gCtxt.opened_stream_cnt < gCtxt.numInputs)
                {
                    // Wait till all streams starts
                    usleep(10000);
                    continue;
                }
                fflush(stdin);
                display_menu();

                while (!g_aborted)
                {
                    uint32 option = 0;
                    char buf[BUFSIZE];
                    fd_set readfds;
                    struct timeval tv;
                    FD_ZERO(&readfds);
                    FD_SET(fileno(stdin), &readfds);

                    tv.tv_sec = 0;
                    tv.tv_usec = 100000;

                    int num = select(1, &readfds, NULL, NULL, &tv);
                    if (num > 0)
                    {
                        if (fgets(buf, sizeof(buf), stdin) != NULL)
                        {
                            option = buf[0];
                            if ('e' == option)
                            {
                                abort_test();
                            }
                            else if ('s' == option)
                            {
                                process_cmds(QCARCAM_TEST_MENU_DUMP_NEXT_FRAME);
                            }
                            else if ('h' == option)
                            {
                                display_menu();
                            }
                            else
                            {
                                option = strtoul(buf, NULL, 0);
                                process_cmds(option);
                            }
                        }
                    }
                }
            }
        }

        /*Wait on all of them to join*/
        for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
        {
            qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

            CameraJoinThread(input_ctxt->thread_handle, NULL);

            CameraSetSignal(input_ctxt->m_eventHandlerSignal);

            CameraJoinThread(input_ctxt->process_cb_event_handle, NULL);

            CameraDestroySignal(input_ctxt->m_eventHandlerSignal);

            input_ctxt->m_eventHandlerSignal = NULL;
        }
    }
    else
    {
        /*singlethreaded - init and start each camera sequentially*/
        for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
        {
            qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

            input_ctxt->idx = input_idx;
            if (0 != qcarcam_test_setup_input_ctxt_thread(&gCtxt.inputs[input_idx]))
            {
                QCARCAM_ERRORMSG("qcarcam_test_setup_input_ctxt_thread failed");
                goto fail;
            }
        }

        for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
        {
            qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

            input_ctxt->is_first_start = TRUE;
            ret = qcarcam_input_start(input_ctxt);
            if (ret != QCARCAM_RET_OK)
            {
                qcarcam_close(input_ctxt->qcarcam_context);
                input_ctxt->qcarcam_context = NULL;
            }

            rc = CameraCreateSignal(&gCtxt.inputs[input_idx].m_eventHandlerSignal);
            if (rc)
            {
                QCARCAM_ERRORMSG("CameraCreateSignal failed, rc=%d", rc);
                goto fail;
            }

            snprintf(thread_name, sizeof(thread_name), "process_cb_event_%d", input_idx);
            rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &process_cb_event_thread, &gCtxt.inputs[input_idx], 0, thread_name, &input_ctxt->process_cb_event_handle);
            if (rc)
            {
                QCARCAM_ERRORMSG("Create cb event process thread failed, rc=%d", rc);
                goto fail;
            }

        }

        if (gCtxt.exitSeconds)
        {
            snprintf(thread_name, sizeof(thread_name), "timer_thread");
            rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &timer_thread, 0, 0, thread_name, &timer_thread_handle);
            if (rc)
            {
                QCARCAM_ERRORMSG("CameraCreateThread failed, rc=%d", rc);
                goto fail;
            }
        }

        while (!g_aborted)
        {
            for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
            {
                qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

                if (!input_ctxt->use_event_callback)
                {
                    if (qcarcam_test_handle_new_frame(input_ctxt))
                        break;
                }
                else
                {
                    sched_yield();
                }
            }
        }
    }

fail:

    /*Wait on all of them to join*/
    for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

        if (input_ctxt->thread_handle)
            CameraJoinThread(input_ctxt->thread_handle, NULL);

        if (input_ctxt->m_eventHandlerSignal)
            CameraSetSignal(input_ctxt->m_eventHandlerSignal);

        if (input_ctxt->process_cb_event_handle)
            CameraJoinThread(input_ctxt->process_cb_event_handle, NULL);

        if (input_ctxt->m_eventHandlerSignal)
            CameraDestroySignal(input_ctxt->m_eventHandlerSignal);

        input_ctxt->m_eventHandlerSignal = NULL;
    }

    // cleanup
    for (input_idx = 0; input_idx < gCtxt.numInputs; input_idx++)
    {
        qcarcam_test_input_t *input_ctxt = &gCtxt.inputs[input_idx];

        if (input_ctxt->qcarcam_context != NULL)
        {
            pthread_mutex_lock(&input_ctxt->mutex);
            (void)qcarcam_stop(input_ctxt->qcarcam_context);
            (void)qcarcam_close(input_ctxt->qcarcam_context);
            input_ctxt->state = QCARCAMTEST_STATE_CLOSED;
            pthread_mutex_unlock(&input_ctxt->mutex);
            input_ctxt->qcarcam_context = NULL;
        }

        if (input_ctxt->qcarcam_window)
        {
            test_util_deinit_window_buffer(test_util_ctxt, input_ctxt->qcarcam_window);
            test_util_deinit_window(test_util_ctxt, input_ctxt->qcarcam_window);
        }

        if (input_ctxt->delay_time)
        {
            for (unsigned int idx = 0; idx < input_ctxt->p_buffers_output.n_buffers; idx++)
            {
                 if (input_ctxt->buf_timer[idx].ptimer)
                 {
                     CameraReleaseTimer(input_ctxt->buf_timer[idx].ptimer);
                 }
            }
        }

        if ((gCtxt.enable_csc) || (gCtxt.enable_c2d || (gCtxt.enable_deinterlace && (gCtxt.enable_deinterlace != SW_BOB_30FPS && gCtxt.enable_deinterlace != SW_BOB_60FPS))))
        {
            if (input_ctxt->display_window)
            {
                test_util_deinit_window_buffer(test_util_ctxt, input_ctxt->display_window);
                test_util_deinit_window(test_util_ctxt, input_ctxt->display_window);
            }
        }
        if (input_ctxt->p_buffers_output.buffers)
        {
            free(input_ctxt->p_buffers_output.buffers);
            input_ctxt->p_buffers_output.buffers = NULL;
        }

        if (input_ctxt->p_buffers_disp.buffers)
        {
            free(input_ctxt->p_buffers_disp.buffers);
            input_ctxt->p_buffers_disp.buffers = NULL;
        }

#ifdef ENABLE_CL_CONVERTER
        if (gCtxt.enable_csc)
        {
            test_util_deinit_cl_converter(input_ctxt->g_converter, &(input_ctxt->source_surface), &(input_ctxt->target_surface));
        }
#endif
    }

    if (pInputs)
    {
        free(pInputs);
        pInputs = NULL;
    }

    if (xml_inputs)
    {
        free(xml_inputs);
        xml_inputs = NULL;
    }

    if (!g_aborted)
    {
        abort_test();
    }

    test_util_deinit(test_util_ctxt);

    qcarcam_uninitialize();

    return rval;
}
