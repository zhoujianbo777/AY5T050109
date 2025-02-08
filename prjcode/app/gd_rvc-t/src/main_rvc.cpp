/**
 ** OpenGL ES 2.0 Sample
 **
 ** Sample app that draws a triangle on an EGL surface
 **/

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <queue>
#include <sys/pps.h>
#include <fcntl.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#include <screen/screen.h>

#include <qcarcam.h>
#include <CameraOSServices.h>
#include <pmem.h>

#include "bv_log.h"
#include "libAVM.h"
#include "bv_qcarcam.h"
#include "test_util_qnx.h"
#include "qcarcam_types.h"
// pm register
#include "dcmd_pm.h"
#include "pm_client_lib.h"
// slog
#include "sv_log.h"

//#define QNX_GLES_SCREEN_TRIANGLE

#define QCARCAM_THRD_PRIO CAMERA_THREAD_PRIO_DEFAULT

#define BVAVM_QCAM_WIDTH		1280
#define BVAVM_QCAM_HEIGHT		800
#define BVAVM_QCAM_SIZE			(BVAVM_QCAM_WIDTH*BVAVM_QCAM_HEIGHT*2)  //UYVY

#define BVAVM_QCAM_NUM		4
#define BVAVM_QCAM_BUF_NUM		5

#define BVAVM_IMG_FILE_ROOT		"/tmp/"

#define PPS_BUFFER_SIZE_MAX		256
#define PPS_SERVICE_PATH		"/tmp/pps/spi_service/a032"
#define PPS_NOTIFYRVC_PATH		"/tmp/pps/ivi_service/8212"

typedef enum {
	BV_QCARCAM_STATE_INVALID = 0,
	BV_QCARCAM_STATE_INIT,
	BV_QCARCAM_STATE_OPEN,
	BV_QCARCAM_STATE_START,
	BV_QCARCAM_STATE_STOP,
	BV_QCARCAM_STATE_PAUSE,
	BV_QCARCAM_STATE_PAUSE_STOP_PENDING,
	BV_QCARCAM_STATE_ERROR,
	BV_QCARCAM_STATE_CLOSED,
} bv_qcarcam_state_t;

typedef enum {
	RVC_UKN_GEAR = 0,
	RVC_D_GEAR = 1,
	RVC_N_GEAR = 2,
	RVC_R_GEAR = 3,
	RVC_P_GEAR = 4
} RVC_GEAR;

typedef enum {
	RVC_DEINIT_ST = 0, RVC_SHOW_ST = 1, RVC_HIDE_ST = 2
} RVC_ST;

typedef enum {
	SCREEN_DEINIT_ST = 0, SCREEN_SHOW_ST = 1, SCREEN_HIDE_ST = 2
} SCREEN_ST;

typedef struct {
	qcarcam_event_t event_id;
	qcarcam_event_payload_t payload;
} bv_qcarcam_event_msg_t;

typedef struct {
	CameraThread thread_handle;
	CameraThread process_cb_event_handle;
	CameraSignal m_eventHandlerSignal;

	unsigned int idx;

	qcarcam_hndl_t qcarcam_context;
	qcarcam_input_desc_t qcarcam_input_id;

	pthread_mutex_t mutex;
	bv_qcarcam_state_t state;
	
	qcarcam_buffers_t p_buffers_output;
	qcarcam_buffers_t p_buffers_disp;
	qcarcam_buffers_t p_buffers_input;

	bool is_first_start;
	bool signal_lost;
	int fatal_err_cnt;
	int use_event_callback;
	
	std::queue<bv_qcarcam_event_msg_t> eventqueue;
	pthread_mutex_t queue_mutex;
} bv_qcarcam_input_t;
;

typedef struct {
	bv_qcarcam_input_t inputs[4];
	int opened_stream_cnt;
	
	int enableBridgeErrorDetect;
	int enableFatalErrorRecover;

	pthread_mutex_t mutex_abort;
	pthread_cond_t cond_abort;
	pthread_mutex_t mutex_open_cnt;

	pthread_mutex_t mutex_bvavm;
} bv_qcarcam_ctxt_t;

static bv_qcarcam_ctxt_t gCtxt = { };
static volatile int g_aborted = 0;

unsigned char *yuvData[4];
qcarcam_input_t *g_pInputs = NULL;
void *g_ptr[BVAVM_QCAM_NUM][BVAVM_QCAM_BUF_NUM];
void *g_mem[BVAVM_QCAM_NUM][BVAVM_QCAM_BUF_NUM];

static sigset_t g_sigset;

static const int exceptsigs[] = { SIGCHLD, SIGIO, SIGURG, SIGWINCH, SIGTTIN,
		SIGTTOU, SIGCONT, SIGSEGV, -1, };

static int rvc_exit = 0;
static RVC_GEAR rvc_gear = RVC_UKN_GEAR;
static RVC_ST rvc_st = RVC_DEINIT_ST;
static SCREEN_ST screen_st = SCREEN_DEINIT_ST;

#ifdef QNX_GLES_SCREEN_TRIANGLE
typedef struct
{
	int display_id;
	int size[2];
}display_prop_t;

screen_context_t screen_ctx;
screen_window_t screen_win;

screen_display_t *screen_display;
display_prop_t *display_property;
int screen_ndisplays;
int disp_idx = 1;

EGLDisplay egl_display;
EGLContext egl_ctx;
EGLSurface egl_surface;

static int initScreen()
{
	int rc;
	int i;

	//Create the screen context
	rc = screen_create_context(&screen_ctx, SCREEN_APPLICATION_CONTEXT);
	if (rc) {
		LOGINF("screen_create_window");
		return EXIT_FAILURE;
	}

	/*query displays*/
	rc = screen_get_context_property_iv(screen_ctx, SCREEN_PROPERTY_DISPLAY_COUNT,
			&screen_ndisplays);
	if (rc)
	{
		LOGINF("screen_get_context_property_iv(SCREEN_PROPERTY_DISPLAY_COUNT)");
		return EXIT_FAILURE;
	}
	else
	{
		printf("screen_ndisplays = %d\n", screen_ndisplays);
	}

	screen_display = (screen_display_t*)calloc(screen_ndisplays, sizeof(*screen_display));
	if (screen_display == NULL)
	{
		LOGINF("could not allocate memory for display list");
		return EXIT_FAILURE;
	}

	display_property = (display_prop_t *)calloc(screen_ndisplays, sizeof(*display_property));
	if (display_property == NULL)
	{
		LOGINF("could not allocate memory for display list");
		return EXIT_FAILURE;
	}

	rc = screen_get_context_property_pv(screen_ctx, SCREEN_PROPERTY_DISPLAYS,
			(void **)screen_display);
	if (rc)
	{
		LOGINF("screen_get_context_property_ptr(SCREEN_PROPERTY_DISPLAYS)");
		free(screen_display);
		return EXIT_FAILURE;
	}

	for (i = 0; i < screen_ndisplays; ++i)
	{
		rc = screen_get_display_property_iv(screen_display[i],
				SCREEN_PROPERTY_ID, &display_property[i].display_id);
		if (rc)
		{
			LOGINF("screen_get_display_property_iv(SCREEN_PROPERTY_ID)");
			return EXIT_FAILURE;
		}

		rc = screen_get_display_property_iv(screen_display[i],
				SCREEN_PROPERTY_SIZE, display_property[i].size);
		if (rc)
		{
			LOGINF("screen_get_display_property_iv(SCREEN_PROPERTY_SIZE)");
			return EXIT_FAILURE;
		}
	}

	//Create the screen window that will be render onto
	rc = screen_create_window(&screen_win, screen_ctx);
	if (rc) {
		LOGINF("screen_create_window");
		return EXIT_FAILURE;
	}

	if (screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_SIZE, (const int[]) {800, 600}) < 0) {
		LOGINF("screen_set_window_property_iv");
	}

	screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_FORMAT, (const int[]) {SCREEN_FORMAT_RGBX8888});
	screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_USAGE, (const int[]) {SCREEN_USAGE_OPENGL_ES2});

	/*associate window with display */
	rc = screen_set_window_property_pv(screen_win, SCREEN_PROPERTY_DISPLAY,
			(void **)&screen_display[disp_idx]);
	if (rc)
	{
		LOGINF("screen_set_window_property_ptr(SCREEN_PROPERTY_DISPLAY)");
		free(screen_display);
		return EXIT_FAILURE;
	}

	rc = screen_create_window_buffers(screen_win, 2);
	if (rc) {
		LOGINF("screen_create_window_buffers");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

static int initEGL(void)
{
	EGLBoolean rc;
	EGLConfig egl_conf = (EGLConfig)0;
	EGLint num_confs = 0;
	const EGLint egl_ctx_attr[] = {
		EGL_CONTEXT_CLIENT_VERSION, 2,
		EGL_NONE
	};

	const EGLint egl_attrib_list[] = {
		EGL_RED_SIZE, 8,
		EGL_GREEN_SIZE, 8,
		EGL_BLUE_SIZE, 8,
		EGL_ALPHA_SIZE, 0,
		EGL_DEPTH_SIZE, 0,
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_SAMPLE_BUFFERS, 1,	// Added for anti-aliased lines
		EGL_NONE
	};

	egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
	if (egl_display == EGL_NO_DISPLAY) {
		LOGERR("eglGetDisplay failed\n");
		return EXIT_FAILURE;
	}

	rc = eglInitialize(egl_display, NULL, NULL);
	if (rc != EGL_TRUE) {
		LOGERR("eglInitialize failed\n");
		return EXIT_FAILURE;
	}

	rc = eglChooseConfig(egl_display, egl_attrib_list, &egl_conf, 1, &num_confs);
	if ((rc != EGL_TRUE) || (num_confs == 0)) {
		LOGERR("eglChooseConfig failed\n");
		return EXIT_FAILURE;
	}

	egl_ctx = eglCreateContext(egl_display, egl_conf, EGL_NO_CONTEXT, (EGLint*)&egl_ctx_attr);
	if (egl_ctx == EGL_NO_CONTEXT) {
		LOGERR("eglCreateContext failed\n");
		return EXIT_FAILURE;
	}

	//Create the EGL surface from the screen window
	egl_surface = eglCreateWindowSurface(egl_display, egl_conf, screen_win, NULL);
	if (egl_surface == EGL_NO_SURFACE) {
		LOGERR("eglCreateWindowSurface failed\n");
		return EXIT_FAILURE;
	}

	rc = eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_ctx);
	if (rc != EGL_TRUE) {
		LOGERR("eglMakeCurrent failed\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

///
// Create a shader object, load the shader source, and
// compile the shader.
//
GLuint LoadShader ( EGLenum type, const char *shaderSrc )
{
	GLuint shader = 0;
	GLint compiled;

	// Create the shader object
	shader = glCreateShader ( type );

	if ( !shader ) {
		return 0;
	}

	// Load the shader source
	glShaderSource( shader, 1, &shaderSrc, 0 );

	// Compile the shader
	glCompileShader( shader );

	// Check the compile status
	glGetShaderiv( shader, GL_COMPILE_STATUS, &compiled );
	if ( !compiled ) {
		GLint infoLen = 0;

		glGetShaderiv ( shader, GL_INFO_LOG_LENGTH, &infoLen );

		if ( infoLen > 1 ) {
			char *infoLog = (char *)malloc ( sizeof ( char ) * infoLen );
			glGetShaderInfoLog ( shader, infoLen, NULL, infoLog );
			fprintf(stdout, "%s\n", infoLog);
			free ( infoLog );
		}

		glDeleteShader ( shader );
		return 0;
	}

	return shader;
}

static int initOpenGL(void)
{
	char vShaderStr[] =
	"attribute vec4 vPosition;                \n"
	"void main()                              \n"
	"{                                        \n"
	"   gl_Position = vPosition;              \n"
	"}                                        \n";

	char fShaderStr[] =
	"precision mediump float;                        \n"
	"void main()                                     \n"
	"{                                               \n"
	"   gl_FragColor = vec4 ( 1.0, 0.0, 0.0, 1.0 );  \n"
	"}                                               \n";

	GLuint vertexShader;
	GLuint fragmentShader;
	GLuint programObject;
	GLint linked;

	// Load the vertex/fragment shaders
	vertexShader = LoadShader ( GL_VERTEX_SHADER, vShaderStr );
	fragmentShader = LoadShader ( GL_FRAGMENT_SHADER, fShaderStr );

	// Create the program object
	programObject = glCreateProgram ( );

	if ( programObject == 0 ) {
		return 0;
	}

	glAttachShader ( programObject, vertexShader );
	glAttachShader ( programObject, fragmentShader );

	// Link the program
	glLinkProgram ( programObject );

	// Check the link status
	glGetProgramiv ( programObject, GL_LINK_STATUS, &linked );
	if ( !linked ) {
		GLint infoLen = 0;

		glGetProgramiv ( programObject, GL_INFO_LOG_LENGTH, &infoLen );
		if ( infoLen > 1 ) {
			char *infoLog = (char *)malloc ( sizeof ( char ) * infoLen );
			glGetProgramInfoLog ( programObject, infoLen, NULL, infoLog );
			fprintf(stdout, "[glError] %s\n", infoLog);
			free ( infoLog );
		}

		glDeleteProgram ( programObject );

		return EXIT_FAILURE;
	}

	// Store the program object
	glUseProgram (programObject);

	glClearColor ( 1.0f, 1.0f, 1.0f, 1.0f );

	// We don't need the shaders anymore
	glDeleteShader(fragmentShader);
	glDeleteShader(vertexShader);

	return EXIT_SUCCESS;
}

void render()
{
	EGLint surface_width;
	EGLint surface_height;

	GLfloat vTriangle[] = {
		-0.2f, -0.2f, 0.0f,
		0.2f, -0.2f, 0.0f,
		0.0f, 0.2f, 0.0f
	};

	eglQuerySurface(egl_display, egl_surface, EGL_WIDTH, &surface_width);
	eglQuerySurface(egl_display, egl_surface, EGL_HEIGHT, &surface_height);
	LOGINF("surface_width=%d surface_height=%d", surface_width, surface_height);

	// This is a temporary fix to keep aspect ratio
//	surface_width = 800;
//	surface_height = 800;

	// Set the viewport
	glViewport ( 0, 0, surface_width, surface_height );

	// Clear the color buffer
	glClear ( GL_COLOR_BUFFER_BIT );

	// Load the vertex data
	glVertexAttribPointer (0, 3, GL_FLOAT, GL_FALSE, 0, vTriangle);
	glEnableVertexAttribArray ( 0 );

	glDrawArrays ( GL_TRIANGLES, 0, 3 );
}
#endif

static int bv_get_time(unsigned long long *pTime) {
	struct timespec time;
	unsigned long long msec;

	if (clock_gettime(CLOCK_MONOTONIC, &time) == -1) {
		LOGERR("Clock gettime failed");
		return 1;
	}
	msec = ((unsigned long long) time.tv_sec * 1000)
			+ (((unsigned long long) time.tv_nsec / 1000) / 1000);
	*pTime = msec;

	return 0;
}

static qcarcam_ret_t qcarcam_input_start(bv_qcarcam_input_t *input_ctxt) {
	qcarcam_ret_t ret = QCARCAM_RET_OK;

	//qcarcam_test_get_time(&input_ctxt->t_start);
	if(input_ctxt->state != BV_QCARCAM_STATE_START){
		ret = qcarcam_start(input_ctxt->qcarcam_context);
		if (ret == QCARCAM_RET_OK) {
			input_ctxt->state = BV_QCARCAM_STATE_START;
#if 0
			input_ctxt->frameCnt = 0;
			input_ctxt->releaseframeCnt = 0;
			input_ctxt->prev_frameCnt = 0;
			input_ctxt->prev_releaseframeCnt = 0;
			input_ctxt->signal_lost = 0;
			qcarcam_test_get_time(&input_ctxt->t_start_success);
#endif
			LOGINF("Client %d Input %d qcarcam_start successfully",
					input_ctxt->idx, input_ctxt->qcarcam_input_id);
		} else {
			input_ctxt->state = BV_QCARCAM_STATE_ERROR;
			LOGINF("Client %d Input %d qcarcam_start failed: %d",
					input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
		}
	}
	else{
		LOGINF("%d %d is start(%d)!", 
				input_ctxt->idx, input_ctxt->qcarcam_input_id, input_ctxt->state);
	}
	return ret;
}

static qcarcam_ret_t qcarcam_input_stop(bv_qcarcam_input_t *input_ctxt) {
	qcarcam_ret_t ret = QCARCAM_RET_OK;
	LOGINF("enter %s", __func__);
	if(input_ctxt->state != BV_QCARCAM_STATE_STOP){
		ret = qcarcam_stop(input_ctxt->qcarcam_context);
		if (ret == QCARCAM_RET_OK) {
			input_ctxt->state = BV_QCARCAM_STATE_STOP;

		ret = qcarcam_close(input_ctxt->qcarcam_context);
		if (ret != QCARCAM_RET_OK) {
			LOGINF("qcarcam_close failed: %d", ret);
		}
		else if(ret == QCARCAM_RET_OK){
			LOGINF("qcarcam_close successfully: %d", ret);
		}
		input_ctxt->state = BV_QCARCAM_STATE_CLOSED;
		input_ctxt->qcarcam_context = NULL;
#if 0
			input_ctxt->frameCnt = 0;
			input_ctxt->releaseframeCnt = 0;
			input_ctxt->prev_frameCnt = 0;
			input_ctxt->prev_releaseframeCnt = 0;
			input_ctxt->release_buf_idx.clear();
			input_ctxt->usr_process_buf_idx.clear();
#endif

			LOGINF("Client %d Input %d qcarcam_stop successfully",
					input_ctxt->idx, input_ctxt->qcarcam_input_id);
		} else {
			input_ctxt->state = BV_QCARCAM_STATE_ERROR;
			LOGINF("Client %d Input %d qcarcam_stop failed: %d",
					input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
		}
	}
	else{
		LOGINF("%d %d is stop(%d)!", 
				input_ctxt->idx, input_ctxt->qcarcam_input_id, input_ctxt->state);
	}

	//memset(&input_ctxt->buf_state, 0x0, sizeof(input_ctxt->buf_state));
	LOGINF("leave %s", __func__);
	return ret;
}

void bv_init_context() {
	int i, j;

	gCtxt.opened_stream_cnt = 0;
	gCtxt.enableBridgeErrorDetect = 1;
	gCtxt.enableFatalErrorRecover = 1;

	LOGINF("enter %s", __func__);
	pthread_mutex_init(&gCtxt.mutex_abort, NULL);
	pthread_mutex_init(&gCtxt.cond_abort, NULL);
	pthread_mutex_init(&gCtxt.mutex_open_cnt, NULL);
	pthread_mutex_init(&gCtxt.mutex_bvavm, NULL);

	for (i = 0; i < BVAVM_QCAM_NUM; i++) {
		gCtxt.inputs[i].idx = i;
		if (3 == gCtxt.inputs[i].idx)
			gCtxt.inputs[i].qcarcam_input_id = QCARCAM_INPUT_TYPE_DRIVER;
		else if (1 == gCtxt.inputs[i].idx)
			gCtxt.inputs[i].qcarcam_input_id = QCARCAM_INPUT_TYPE_LANE_WATCH;
		else if (2 == gCtxt.inputs[i].idx)
			gCtxt.inputs[i].qcarcam_input_id = QCARCAM_INPUT_TYPE_GESTURE;
		else if (0 == gCtxt.inputs[i].idx)
			gCtxt.inputs[i].qcarcam_input_id = QCARCAM_INPUT_TYPE_IRIS;

		gCtxt.inputs[i].p_buffers_output.n_buffers = 5;
		gCtxt.inputs[i].p_buffers_output.flags = QCARCAM_BUFFER_FLAG_OS_HNDL;
		gCtxt.inputs[i].p_buffers_output.color_fmt = QCARCAM_FMT_UYVY_8;
		gCtxt.inputs[i].p_buffers_output.buffers = (qcarcam_buffer_t *) calloc(
				gCtxt.inputs[i].p_buffers_output.n_buffers,
				sizeof(*gCtxt.inputs[i].p_buffers_output.buffers));
		if (gCtxt.inputs[i].p_buffers_output.buffers == 0) {
			LOGINF("alloc qcarcam_buffer failed");
		}
		for (j = 0; j < BVAVM_QCAM_BUF_NUM; j++) {
			gCtxt.inputs[i].p_buffers_output.buffers[j].n_planes = 1;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].width =
			BVAVM_QCAM_WIDTH;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].height =
			BVAVM_QCAM_HEIGHT;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].stride =
					gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].width
							* 2;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[1].stride = 0;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].size =
					gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].stride
							* gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].height;
			gCtxt.inputs[i].p_buffers_output.buffers[j].planes[1].size = 0;

			g_ptr[i][j] =
					pmem_malloc_ext_v2(
							gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].size
									+ gCtxt.inputs[i].p_buffers_output.buffers[j].planes[1].size,
							PMEM_CAMERA_ID,
							PMEM_FLAGS_SHMEM | PMEM_FLAGS_PHYS_NON_CONTIG
									| PMEM_FLAGS_CACHE_NONE,
							PMEM_ALIGNMENT_4K, 0,
							(pmem_handle_t*) &(g_mem[i][j]),
							NULL);

			if (g_ptr[i][j] == NULL) {
				LOGINF(
						"test_util_init_window_buffers: pmem_alloc failed");
				//return QCARCAM_RET_FAILED;
				gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].p_buf =
				NULL;
			} else {
				gCtxt.inputs[i].p_buffers_output.buffers[j].planes[0].p_buf =
						g_mem[i][j];
				//LOGINF("ptr=0x%x pmem=0x%x", g_ptr[i][j], g_mem[i][j]);
			}
		}

		pthread_mutex_init(&(gCtxt.inputs[i].mutex), NULL);
		pthread_mutex_init(&(gCtxt.inputs[i].queue_mutex), NULL);
	}
	LOGINF("leave %s", __func__);
}

void bv_deinit_context() {
	int i, j;
	LOGINF("enter %s", __func__);
	pthread_mutex_destroy(&gCtxt.mutex_abort);
	pthread_mutex_destroy(&gCtxt.cond_abort);
	pthread_mutex_destroy(&gCtxt.mutex_open_cnt);
	pthread_mutex_destroy(&gCtxt.mutex_bvavm);

	for (i = 0; i < BVAVM_QCAM_NUM; i++) {
		qcarcam_input_stop(&(gCtxt.inputs[i]));

		pthread_mutex_destroy(&(gCtxt.inputs[i].mutex));
		pthread_mutex_destroy(&(gCtxt.inputs[i].queue_mutex));

		for (j = 0; j < BVAVM_QCAM_BUF_NUM; j++) {
			if (g_ptr[i][j]) {
				//printf("pmem_free i:%d j:%d\n", i, j);
				pmem_free(g_ptr[i][j]);
				g_ptr[i][j] = NULL;
			}
		}
	}
	LOGINF("leave %s", __func__);
}

void bv_init_avm_buf(void) {
	int i;
	int w = BVAVM_QCAM_WIDTH, h = BVAVM_QCAM_HEIGHT;
	FILE *fp[4];
	char fileName[256];

	LOGINF("enter %s", __func__);
	for (i = 0; i < 4; i++) {
		yuvData[i] = (unsigned char *) malloc(BVAVM_QCAM_SIZE);
		if (!yuvData[i]) {
			LOGERR("Failed to allocate yuv buffer for camera %d", i);
			// Clean up previously allocated buffers
			while (--i >= 0) {
				free(yuvData[i]);
				yuvData[i] = nullptr;
			}
			return;
		}
		memset(yuvData[i], 128, BVAVM_QCAM_SIZE);
		//yuvCamData[i] = (unsigned char *)malloc(w*h*3/2);
		//memset(yuvCamData[i], 0, w*h*3/2);
#if 0
		fp[i] = fopen(fileName, "rb");
		if(fp[i]) {
			printf("%s = %p", fileName, fp[i]);
			fread(yuvData[i], 1, w * h * 3 / 2, fp[i]);
			fclose(fp[i]);
			fp[i] = NULL;
		}
#endif
	}
	LOGINF("leave %s", __func__);
}

void bv_exit_avm_buf(void) {
	int i;

	LOGINF("enter %s", __func__);
	for (i = 0; i < 4; i++) {
		//LOGINF("******%d", i);
		if (yuvData[i]) {
			free(yuvData[i]);
			yuvData[i] = NULL;
		}
	}
	LOGINF("leave %s", __func__);
}

qcarcam_ret_t bv_util_deinit(test_util_ctxt_t *p_ctxt) {

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
#if 0
        if (p_ctxt->params.enable_gpio_config)
        {
            g_fd_gpio = gpio_close(NULL);
            if (g_fd_gpio == -1)
            {
                QCARCAM_ERRORMSG("gpio_close() failed!");
                rc = QCARCAM_RET_FAILED;
            }
        }
#endif
        free(p_ctxt);
    }

    return rc;
}

int bv_save_image_to_file(bv_qcarcam_input_t *input_ctxt, int frame_idx) {
	int ret = 0;
	FILE *fp;
	size_t numBytesWritten = 0;
	size_t numByteToWrite = 0;
	char file_name[128];

	if (!input_ctxt
			|| (input_ctxt->idx < 0 || input_ctxt->idx >= BVAVM_QCAM_NUM)
			|| (frame_idx < 0 || frame_idx >= BVAVM_QCAM_BUF_NUM)) {
		LOGERR("param is error");
		return -1;
	}

	if (0 == input_ctxt->idx)
		sprintf(file_name, BVAVM_IMG_FILE_ROOT"front.raw");
	else if (1 == input_ctxt->idx)
		sprintf(file_name, BVAVM_IMG_FILE_ROOT"rear.raw");
	else if (2 == input_ctxt->idx)
		sprintf(file_name, BVAVM_IMG_FILE_ROOT"left.raw");
	else if (3 == input_ctxt->idx)
		sprintf(file_name, BVAVM_IMG_FILE_ROOT"right.raw");

	remove(file_name);
	fp = fopen(file_name, "w+");
	if (fp) {
		numByteToWrite =
				gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_idx].planes[0].size;
		numBytesWritten = fwrite(g_ptr[input_ctxt->idx][frame_idx], 1,
				numByteToWrite, fp);
		if (numBytesWritten == numByteToWrite) {
			LOGINF("save file <%s> OK", file_name);
		} else {
			LOGERR("save file <%s> failed", file_name);
			ret = -1;
		}
		fclose(fp);

		LOGINF("ptr=0x%x numByteToWrite=%d numBytesWritten=%d",
				g_ptr[input_ctxt->idx][frame_idx], numByteToWrite,
				numBytesWritten);
	} else {
		LOGERR("open file failed");
		ret = -1;
	}

	return ret;
}

static void bv_qcarcam_event_cb(qcarcam_hndl_t hndl, qcarcam_event_t event_id,
		qcarcam_event_payload_t *p_payload) {
	int i = 0;
	int result = 0;
	bv_qcarcam_input_t *input_ctxt = NULL;
	bv_qcarcam_event_msg_t event_msg;

	//printf("enter %s\n", __func__);
	for (i = 0; i < 4; i++) {
		if (hndl == gCtxt.inputs[i].qcarcam_context) {
			input_ctxt = &gCtxt.inputs[i];
			break;
		}
	}

	if (!input_ctxt) {
		LOGERR("event_cb called with invalid qcarcam handle %p", hndl);
		return;
	}

	if (g_aborted) {
		LOGERR("stop push");
		return;
	}

	event_msg.event_id = event_id;

	memcpy(&event_msg.payload, p_payload, sizeof(qcarcam_event_payload_t));

	pthread_mutex_lock(&input_ctxt->queue_mutex);
	input_ctxt->eventqueue.push(event_msg);
	pthread_mutex_unlock(&input_ctxt->queue_mutex);

	result = CameraSetSignal(input_ctxt->m_eventHandlerSignal);
	if (result) {
		LOGINF("Failed to signal event %d (%d)", event_id, result);
	}
	//LOGINF("******event_msg.event_id=%d\n", event_msg.event_id);
	//printf("leave %s\n", __func__);
}

static int qcarcam_handle_input_signal(bv_qcarcam_input_t *input_ctxt, qcarcam_input_signal_t signal_type)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;

	LOGINF("enter %s\n", __func__);
    switch (signal_type) {
    case QCARCAM_INPUT_SIGNAL_LOST:
        LOGINF("LOST: idx: %d, input: %d\n", input_ctxt->idx, input_ctxt->qcarcam_input_id);

        /*TODO: offload this to other thread to handle restart recovery*/
        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state == BV_QCARCAM_STATE_STOP) {
            printf("Input %d already stop, break\n", input_ctxt->qcarcam_input_id);
            pthread_mutex_unlock(&input_ctxt->mutex);
            break;
        }

        input_ctxt->signal_lost = 1;
        //qcarcam_input_stop(input_ctxt);
        ret = qcarcam_stop(input_ctxt->qcarcam_context);
		if (ret == QCARCAM_RET_OK) {
			input_ctxt->state = BV_QCARCAM_STATE_STOP;
			LOGINF("Client %d Input %d qcarcam_stop successfully",
						input_ctxt->idx, input_ctxt->qcarcam_input_id);
		} else {
			input_ctxt->state = BV_QCARCAM_STATE_ERROR;
			LOGINF("Client %d Input %d qcarcam_stop failed: %d",
					input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
		}

        pthread_mutex_unlock(&input_ctxt->mutex);

        break;
    case QCARCAM_INPUT_SIGNAL_VALID:
        LOGINF("VALID: idx: %d, input: %d\n", input_ctxt->idx, input_ctxt->qcarcam_input_id);

        pthread_mutex_lock(&input_ctxt->mutex);
        if (input_ctxt->state == BV_QCARCAM_STATE_START)
        {
            LOGINF("Input %d already running, break\n", input_ctxt->qcarcam_input_id);
            pthread_mutex_unlock(&input_ctxt->mutex);
            break;
        }

        ret = qcarcam_input_start(input_ctxt);

        pthread_mutex_unlock(&input_ctxt->mutex);

        break;
    default:
        LOGINF("Unknown Event type: %d\n", signal_type);
        break;
    }
	LOGINF("leave %s\n", __func__);
    return ret;
}

static int qcarcam_handle_fatal_error(bv_qcarcam_input_t *input_ctxt, boolean recover)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;
	LOGINF("enter %s\n", __func__);
    LOGINF("Fatal error: client idx - %d, input id - %d", input_ctxt->idx, input_ctxt->qcarcam_input_id);

    pthread_mutex_lock(&input_ctxt->mutex);

    input_ctxt->signal_lost = 1;
    input_ctxt->fatal_err_cnt++;

    if (input_ctxt->state == BV_QCARCAM_STATE_ERROR || input_ctxt->state == BV_QCARCAM_STATE_STOP) {
        LOGINF("Input %d already error state, return", input_ctxt->qcarcam_input_id);
        pthread_mutex_unlock(&input_ctxt->mutex);
        return ret;
    }

    if (recover)
    {
        ret = qcarcam_stop(input_ctxt->qcarcam_context);
        if (ret == QCARCAM_RET_OK) {
            input_ctxt->state = BV_QCARCAM_STATE_STOP;
            LOGINF("Client %d Input %d qcarcam_stop successfully", input_ctxt->idx, input_ctxt->qcarcam_input_id);
        }
        else
        {
            input_ctxt->state = BV_QCARCAM_STATE_ERROR;
            LOGINF("Client %d Input %d qcarcam_stop failed: %d !", input_ctxt->idx, input_ctxt->qcarcam_input_id, ret);
        }
    }
    else
    {
        input_ctxt->state = BV_QCARCAM_STATE_ERROR;
    }

    pthread_mutex_unlock(&input_ctxt->mutex);
	LOGINF("leave %s\n", __func__);
    return ret;
}

int qcarcam_0_setup_input_flag = 0;
int qcarcam_1_setup_input_flag = 0;
int qcarcam_2_setup_input_flag = 0;
int qcarcam_3_setup_input_flag = 0;
static int process_cb_event_thread(void *arg) {
	int nret = 0;
	unsigned long long start_time;
	unsigned long long end_time;
	start_time = 0;
	end_time = 0;
	bv_qcarcam_input_t *input_ctxt = (bv_qcarcam_input_t *) arg;
	bv_qcarcam_event_msg_t event_msg;
	int count = 0;
	//char fileName[256];
	
	LOGINF("enter %s %d", __func__, g_aborted);

	while (!g_aborted) {
		CameraWaitOnSignal(input_ctxt->m_eventHandlerSignal,
		CAM_SIGNAL_WAIT_NO_TIMEOUT);

		pthread_mutex_lock(&input_ctxt->queue_mutex);
		if (!input_ctxt->eventqueue.empty()) {
			event_msg = input_ctxt->eventqueue.front();
			input_ctxt->eventqueue.pop();
		} else {
			LOGINF("event queue is empty");
			pthread_mutex_unlock(&input_ctxt->queue_mutex);
			continue;
		}

		pthread_mutex_unlock(&input_ctxt->queue_mutex);
		//LOGINF("event_msg.event_id=%d, %d\n", event_msg.event_id, input_ctxt->idx);
		#if 1
		switch (event_msg.event_id) {
		case QCARCAM_EVENT_FRAME_READY: {
			if (input_ctxt->state == BV_QCARCAM_STATE_START) {
				static int front_num = 0;
				static int rear_num = 0;
				static int left_num = 0;
				static int right_num = 0;
				//static int frameCount_f = 0;
				//static int frameCount_b = 0;
				//static int frameCount_l = 0;
				//static int frameCount_r = 0;

				qcarcam_ret_t ret;
				qcarcam_frame_info_t frame_info;
				ret = qcarcam_get_frame(input_ctxt->qcarcam_context,
						&frame_info, 0, 0);
				if (ret == QCARCAM_RET_TIMEOUT) {
					LOGERR(
							"qcarcam_get_frame timeout context %p ret %d",
							input_ctxt->qcarcam_context, ret);
					//input_ctxt->signal_lost = 1;
					return -1;
				}

				if (QCARCAM_RET_OK != ret) {
					LOGERR("Get frame context %p ret %d",
							input_ctxt->qcarcam_context, ret);
					return -1;
				}

#if 0
				BVAVM_DBGMSG("%d received frame: %d %p", frame_info.idx,
						gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_info.idx].planes[0].size,
						gCtxt.inputs[input_ctxt->idx].p_buffers_output.buffers[frame_info.idx].planes[0].p_buf);
#endif
#if 1
				if ((input_ctxt->idx >= 0 && input_ctxt->idx < BVAVM_QCAM_NUM)
						&& (frame_info.idx >= 0 && frame_info.idx < BVAVM_QCAM_BUF_NUM) 
						&& (rvc_exit != 1) && (rvc_st == RVC_SHOW_ST)) {
					//pthread_mutex_lock(&gCtxt.mutex_bvavm);
					//bv_get_time(&start_time);
					//printf("******%d, %d, %x, %x\n", input_ctxt->idx, frame_info.idx,
					//		 yuvData[input_ctxt->idx], g_ptr[input_ctxt->idx][frame_info.idx]);
					//LOGINF("input_ctxt->idx========%d\n", input_ctxt->idx);
					if (0 == input_ctxt->idx) {
						//pthread_mutex_lock(&gCtxt.mutex_bvavm);
						memcpy(yuvData[0],
								g_ptr[input_ctxt->idx][frame_info.idx],
								BVAVM_QCAM_SIZE);
						//pthread_mutex_unlock(&gCtxt.mutex_bvavm);
						if (!front_num) {
							LOGINF("gd_rvc get first frame front");
							front_num++;
						}
						#if 0
						if (!front_num) {
							nret = bv_save_image_to_file(input_ctxt, frame_info.idx);
							if (!nret)
								front_num++;
						}
						frameCount_f++;
						if ((frameCount_f % 10 == 0) && (frameCount_f < 100)){
						LOGINF("*****************%d", frameCount_f);
						FILE *fp = NULL;
						sprintf(fileName, "/avm_config/frame_original_%d_f.nv12", frameCount_f);
						fp = fopen(fileName, "wb+");
						fwrite(yuvData[0], 1, sizeof(char)* 1280 * 800 * 3 / 2, fp);
						fclose(fp);
						}
						#endif
					} 
					else if (1 == input_ctxt->idx) {
						//pthread_mutex_lock(&gCtxt.mutex_bvavm);
						memcpy(yuvData[1],
								g_ptr[input_ctxt->idx][frame_info.idx],
								BVAVM_QCAM_SIZE);
						//pthread_mutex_unlock(&gCtxt.mutex_bvavm);
						if (!rear_num) {
							LOGINF("gd_rvc get first frame rear");
							rear_num++;
						}
						#if 0
						if (!rear_num) {
							nret = bv_save_image_to_file(input_ctxt,
									frame_info.idx);
							if (!nret)
								rear_num++;
						}
						frameCount_b++;
						if ((frameCount_b % 10 == 0) && (frameCount_b < 100)){
						LOGINF("*****************%d", frameCount_b);
						FILE *fp = NULL;
						sprintf(fileName, "/avm_config/frame_original_%d_b.nv12", frameCount_b);
						fp = fopen(fileName, "wb+");
						fwrite(yuvData[1], 1, sizeof(char)* 1280 * 800 * 3 / 2, fp);
						fclose(fp);
						}
						#endif
					} 
					else if (2 == input_ctxt->idx) {
						//pthread_mutex_lock(&gCtxt.mutex_bvavm);
						memcpy(yuvData[2],
								g_ptr[input_ctxt->idx][frame_info.idx],
								BVAVM_QCAM_SIZE);
						//pthread_mutex_unlock(&gCtxt.mutex_bvavm);
						if (!left_num) {
							LOGINF("gd_rvc get first frame left");
							left_num++;
						}
						#if 0
						if (!left_num) {
							nret = bv_save_image_to_file(input_ctxt,
									frame_info.idx);
							if (!nret)
								left_num++;
						}
						frameCount_l++;
						if ((frameCount_l % 10 == 0) && (frameCount_l < 100)){
						LOGINF("*****************%d", frameCount_l);
						FILE *fp = NULL;
						sprintf(fileName, "/avm_config/frame_original_%d_l.nv12", frameCount_l);
						fp = fopen(fileName, "wb+");
						fwrite(yuvData[2], 1, sizeof(char)* 1280 * 800 * 3 / 2, fp);
						fclose(fp);
						}
						#endif
					} 
					else if (3 == input_ctxt->idx) {
						//pthread_mutex_lock(&gCtxt.mutex_bvavm);
						memcpy(yuvData[3],
								g_ptr[input_ctxt->idx][frame_info.idx],
								BVAVM_QCAM_SIZE);
						//pthread_mutex_unlock(&gCtxt.mutex_bvavm);
						if (!right_num) {
							LOGINF("gd_rvc get first frame right");
							right_num++;
						}
						#if 0
						if (!right_num) {
							nret = bv_save_image_to_file(input_ctxt,
									frame_info.idx);
							if (!nret)
								right_num++;
						}
						frameCount_r++;
						if ((frameCount_r % 10 == 0) && (frameCount_r < 100)){
						LOGINF("*****************%d", frameCount_r);
						FILE *fp = NULL;
						sprintf(fileName, "/avm_config/frame_original_%d_r.nv12", frameCount_r);
						fp = fopen(fileName, "wb+");
						fwrite(yuvData[3], 1, sizeof(char)* 1280 * 800 * 3 / 2, fp);
						fclose(fp);
						}
						#endif
					}
					//bv_get_time(&end_time);
					//printf("count time: %lld(ms)\n", (end_time - start_time));
					//pthread_mutex_unlock(&gCtxt.mutex_bvavm);
				}
#endif
				qcarcam_release_frame(input_ctxt->qcarcam_context, frame_info.idx);
			}
			break;
		}
		case QCARCAM_EVENT_INPUT_SIGNAL: {
#if 1
			if (gCtxt.enableBridgeErrorDetect)
			{
				qcarcam_handle_input_signal(input_ctxt, (qcarcam_input_signal_t)event_msg.payload.uint_payload);
			}
#endif
			break;
		}
		case QCARCAM_EVENT_ERROR: {
			switch (event_msg.payload.uint_payload) {
			case QCARCAM_CONN_ERROR:
				LOGINF("Connetion to server lost. id:%d qid:%d",
						input_ctxt->idx, input_ctxt->qcarcam_input_id)
				;

				///abort_test();
				break;
			case QCARCAM_FATAL_ERROR:
				LOGINF("fatal error %d on id:%d qid:%d",
						event_msg.payload.uint_payload, input_ctxt->idx,
						input_ctxt->qcarcam_input_id);

				qcarcam_handle_fatal_error(input_ctxt, gCtxt.enableFatalErrorRecover);
				break;
			case QCARCAM_FRAMESYNC_ERROR:
				LOGINF("fatal error %d on id:%d qid:%d",
						event_msg.payload.uint_payload, input_ctxt->idx,
						input_ctxt->qcarcam_input_id)
				;

				qcarcam_handle_fatal_error(input_ctxt, TRUE);

				break;
			case QCARCAM_IFE_OVERFLOW_ERROR:
				LOGINF("output overflow on id:%d qid:%d",
						input_ctxt->idx, input_ctxt->qcarcam_input_id)
				;

				//qcarcam_handle_fatal_error(input_ctxt, gCtxt.enableIFEOverflowhandle);
				break;
			default:
				break;
			}

			break;
		}
		case QCARCAM_EVENT_VENDOR: {
			LOGINF("receive QCARCAM_EVENT_VENDOR data[0]=%u data[1]=%u",
					event_msg.payload.array[0], event_msg.payload.array[1]);
			break;
		}
		case QCARCAM_EVENT_PROPERTY_NOTIFY:
			///qcarcam_test_handle_set_event_notification(input_ctxt, (qcarcam_param_t)event_msg.payload.uint_payload);
			break;
		case QCARCAM_EVENT_FRAME_FREEZE: {
			ais_log_kpi(AIS_EVENT_KPI_CLIENT_FRAME_FREEZE);
			LOGINF(
					"Detected QCARCAM_EVENT_FRAME_FREEZE, Impacted Frame id = %d",
					event_msg.payload.frame_freeze.seq_no);
			break;
		}
		default:
			LOGINF("%d received unsupported event %d", input_ctxt->idx,
					event_msg.event_id)
			;
			break;
		}
	#endif
	}

	LOGINF("leave %s", __func__);
	return 0;
}

int qcarcam_setup_input_flag = 0;
static int bv_qcarcam_setup_input_ctxt_thread(void *arg) {
	qcarcam_ret_t ret = QCARCAM_RET_OK;
	bv_qcarcam_input_t *input_ctxt = (bv_qcarcam_input_t *) arg;
	unsigned long long t_before = 0;
	unsigned long long t_after = 0;

	LOGINF("enter %s", __func__);

	if (!input_ctxt)
		return -1;

	LOGINF("setup_input_ctxt_thread idx = %d, input_desc=%d",
			input_ctxt->idx, input_ctxt->qcarcam_input_id);
#if 1
	bv_get_time(&t_before);

	input_ctxt->qcarcam_context = qcarcam_open(input_ctxt->qcarcam_input_id);
	pthread_mutex_lock(&gCtxt.mutex_open_cnt);
	gCtxt.opened_stream_cnt++;
	pthread_mutex_unlock(&gCtxt.mutex_open_cnt);
	if (input_ctxt->qcarcam_context == 0) {
		input_ctxt->state = BV_QCARCAM_STATE_ERROR;
		LOGERR("qcarcam_open() failed");
		goto qcarcam_thread_fail;
	}

	input_ctxt->state = BV_QCARCAM_STATE_OPEN;

	bv_get_time(&t_after);
	LOGINF("qcarcam_open (idx %u) : %lu ms", input_ctxt->idx,
			(t_after - t_before));
	t_before = t_after;

	LOGINF(
			"render_thread idx = %d, input_desc=%d context=%p p_buffers_output=%p 0x%x %d %d",
			input_ctxt->idx, input_ctxt->qcarcam_input_id,
			input_ctxt->qcarcam_context, input_ctxt->p_buffers_output.buffers,
			input_ctxt->p_buffers_output.color_fmt,
			input_ctxt->p_buffers_output.flags,
			input_ctxt->p_buffers_output.n_buffers);
#endif
	ret = qcarcam_s_buffers(input_ctxt->qcarcam_context,
			&input_ctxt->p_buffers_output);
	if (ret != QCARCAM_RET_OK) {
		LOGERR("qcarcam_s_buffers() failed");
		goto qcarcam_thread_fail;
	}

#ifdef ENABLE_INJECTION_SUPPORT
	if (input_ctxt->is_injection)
	{
		LOGINF("read from input_file - set input buffers");

		ret = qcarcam_s_input_buffers(input_ctxt->qcarcam_context, &input_ctxt->p_buffers_input);
		if (ret != QCARCAM_RET_OK)
		{
			LOGERR("qcarcam_s_input_buffers() failed");
			goto qcarcam_thread_fail;
		}
	}
#endif

	bv_get_time(&t_after);
	LOGINF("qcarcam_s_buffers (idx %u) : %lu ms", input_ctxt->idx,
			(t_after - t_before));
	t_before = t_after;

	LOGINF("qcarcam_s_buffers done, qcarcam_start ...");

	///if (input_ctxt->use_event_callback)
	{
		qcarcam_param_value_t param;
		static int waittime = 0;
		param.ptr_value = (void *) bv_qcarcam_event_cb;
		ret = qcarcam_s_param(input_ctxt->qcarcam_context,
				QCARCAM_PARAM_EVENT_CB, &param);
		if (ret != QCARCAM_RET_OK) {
			LOGERR("qcarcam_s_param(QCARCAM_PARAM_EVENT_CB) failed");
			goto qcarcam_thread_fail;
		}

		param.uint_value = QCARCAM_EVENT_FRAME_READY
				| QCARCAM_EVENT_INPUT_SIGNAL | QCARCAM_EVENT_ERROR
				| QCARCAM_EVENT_VENDOR | QCARCAM_EVENT_FRAME_FREEZE;
		ret = qcarcam_s_param(input_ctxt->qcarcam_context,
				QCARCAM_PARAM_EVENT_MASK, &param);
		if (ret != QCARCAM_RET_OK) {
			LOGERR("qcarcam_s_param(QCARCAM_PARAM_EVENT_MASK) failed");
			goto qcarcam_thread_fail;
		}

		bv_get_time(&t_after);
		LOGINF("qcarcam_s_param (idx %u) : %lu ms", input_ctxt->idx,
				(t_after - t_before));
		t_before = t_after;
#if 1
		ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_STATUS, &param);
		//LOGINF("***********%d, %s\n", ret, param.video_lock_status.sensor_video_lock_status ? "true":"flase");
		if (param.video_lock_status.sensor_video_lock_status){
			LOGINF("qcarcam_g_param success for stream id %d %s!", input_ctxt->qcarcam_input_id, 
					param.video_lock_status.sensor_video_lock_status ? "true":"flase");
			input_ctxt->is_first_start = TRUE;
		}
		else{
			input_ctxt->is_first_start = FALSE;
			//LOGERR("qcarcam_g_param failed for stream id %d! ret = %d", input_ctxt->qcarcam_input_id, ret);
	        while ((!param.video_lock_status.sensor_video_lock_status) && (waittime < 25)) {
				usleep(200000);
				waittime++;
				ret = qcarcam_g_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_STATUS, &param);
				LOGERR("qcarcam_g_param failed for stream id %d %s! waittime %d", input_ctxt->qcarcam_input_id, 
				param.video_lock_status.sensor_video_lock_status ? "true":"flase", waittime);
				if(param.video_lock_status.sensor_video_lock_status){
					input_ctxt->is_first_start = TRUE;
					LOGINF("input_ctxt->is_first_start==%d, lock_status==%s", 
					input_ctxt->is_first_start, param.video_lock_status.sensor_video_lock_status ? "true":"flase");
				}
			}
		}
#endif
	}
#if 0
	if (input_ctxt->frame_rate_config.frame_drop_mode != QCARCAM_KEEP_ALL_FRAMES)
	{
		qcarcam_param_value_t param;

		param.frame_rate_config = input_ctxt->frame_rate_config;

		ret = qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_FRAME_RATE, &param);
		if (ret != QCARCAM_RET_OK)
		{
			LOGINF("qcarcam_s_param(QCARCAM_PARAM_FRAME_RATE) failed");
			goto qcarcam_thread_fail;
		}

		bv_get_time(&t_after);
		LOGINF("qcarcam_s_param fps (idx %u): %lu ms", input_ctxt->idx, (t_after - t_before));
		t_before = t_after;

		LOGINF("Provided QCARCAM_PARAM_FRAME_RATE: mode = %d, period = %d, pattern = 0x%x",
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
			LOGINF("qcarcam_s_param(QCARCAM_PARAM_OPMODE) failed");
			goto qcarcam_thread_fail;
		}

		bv_get_time(&t_after);
		LOGINF("qcarcam_s_param opmode (idx %u): %lu ms", input_ctxt->idx, (t_after - t_before));
		t_before = t_after;
	}
#endif

	/*single threaded handles frames outside this function*/
	//if (gCtxt.multithreaded)
	{
		//pthread_mutex_lock(&input_ctxt->mutex);
		
		//input_ctxt->is_first_start = TRUE;
		if(input_ctxt->is_first_start == TRUE){
			ret = qcarcam_input_start(input_ctxt);
		}
		else{
			LOGINF("qcarcam_g_param failed, cant qcarcam_input_start %s", 
			input_ctxt->is_first_start ? "true":"flase");
		}

		//pthread_mutex_unlock(&input_ctxt->mutex);

#ifdef ENABLE_INJECTION_SUPPORT
		if (input_ctxt->is_injection)
		{
			qcarcam_param_value_t param;
			param.uint_value = 0; //TODO: fix this to be appropriate buffer
			qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_INJECTION_START, &param);
		}
#endif

		bv_get_time(&t_after);
		LOGINF("qcarcam_start (idx %u) : %lu ms", input_ctxt->idx,
				(t_after - t_before));
		t_before = t_after;
#if 0
		if (input_ctxt->manual_exposure != -1)
		{
			/* Set exposure configuration */
			qcarcam_param_value_t param;
			param.exposure_config.exposure_time = input_ctxt->exp_time;
			param.exposure_config.gain = input_ctxt->gain;
			param.exposure_config.exposure_mode_type = input_ctxt->manual_exposure ? QCARCAM_EXPOSURE_MANUAL : QCARCAM_EXPOSURE_AUTO;

			if (input_ctxt->manual_exposure)
			{
				LOGINF("Provided QCARCAM_PARAM_MANUAL_EXPOSURE : time =%f ms, gain = %f", input_ctxt->exp_time, input_ctxt->gain);
			}
			else
			{
				LOGINF("AUTO EXPOSURE MODE");
			}

			qcarcam_s_param(input_ctxt->qcarcam_context, QCARCAM_PARAM_EXPOSURE, &param);
		}
		else
		{
			LOGINF("Exposure is not configured, use default setting");
		}
#endif

		bv_get_time(&t_after);
		LOGINF("qcarcam setexposure (idx %u) : %lu ms", input_ctxt->idx,
				(t_after - t_before));
		t_before = t_after;

#if 0
		if (!input_ctxt->use_event_callback)
		{
			while (!g_aborted)
			{
				if (qcarcam_test_handle_new_frame(input_ctxt))
				break;
			}
		}
		else
#endif
		{
			pthread_mutex_lock(&gCtxt.mutex_abort);
			if (!g_aborted) {
				pthread_cond_wait(&gCtxt.cond_abort, &gCtxt.mutex_abort);
			}
			pthread_mutex_unlock(&gCtxt.mutex_abort);
		}
	}

#if 1
	if(input_ctxt->idx == 3){
		qcarcam_setup_input_flag = 1;
		LOGINF("!qcarcam_setup_input_flag okokokokokok");
	}
#endif
	LOGINF("exit setup_input_ctxt_thread idx = %d", input_ctxt->idx);
	LOGINF("leave %s", __func__);
	return 0;

	qcarcam_thread_fail: if (input_ctxt->qcarcam_context) {
		ret = qcarcam_close(input_ctxt->qcarcam_context);
		if (ret != QCARCAM_RET_OK) {
			LOGINF("qcarcam_close failed: %d", ret);
		}
		input_ctxt->qcarcam_context = NULL;

		input_ctxt->state = BV_QCARCAM_STATE_ERROR;
	}

	LOGINF("<-1>leave %s", __func__);
	return -1;
}

int bv_qcarcam_init(void) {
	int nret = 0;
	qcarcam_ret_t ret = QCARCAM_RET_OK;
	qcarcam_init_t qcarcam_init = { 0 };
	int i;
	int input_idx;
	char thread_name[64];
	static int waittime = 0;

	LOGINF("enter %s", __func__);

	qcarcam_init.version = QCARCAM_VERSION;
	qcarcam_init.debug_tag = (char *) "qcarcam_bvavm";
	ret = qcarcam_initialize(&qcarcam_init);
	if (ret != QCARCAM_RET_OK) {
		LOGINF("qcarcam_initialize failed %d", ret);
		nret = -1;
		return -1;
	}

	/*query qcarcam*/
	int enableRetry = 0;
	qcarcam_input_t *pInputs = NULL;
	unsigned int queryNumInputs = 0, queryFilled = 0;
	/*retry used for early camera handoff for RVC*/
	do {
		ret = qcarcam_query_inputs(NULL, 0, &queryNumInputs);
		while ((QCARCAM_RET_OK != ret) && (waittime < 20)) {
			usleep(100000);
			waittime++;
			ret = qcarcam_query_inputs(NULL, 0, &queryNumInputs);
			LOGERR("Failed qcarcam_query_inputs number of inputs with ret %d, enableRetry %d", ret, enableRetry);
		}
		if (QCARCAM_RET_OK != ret || queryNumInputs == 0) {
			LOGERR("Failed qcarcam_query_inputs number of inputs with ret %d, enableRetry %d", ret, enableRetry);
			if (enableRetry == 0) {
				//exit(-1);
				nret = -1;
				goto fail;
			}
		} else {
			break;
		}
		if (enableRetry == 1) {
			usleep(500000);
		}
	} while (enableRetry == 1);
	LOGINF("queryNumInputs=%d", queryNumInputs);

	pInputs = (qcarcam_input_t *) calloc(queryNumInputs, sizeof(*pInputs));
	if (!pInputs) {
		LOGERR("Failed to allocate pInputs");
		//exit(-1);
		nret = -1;
		goto fail;
	}
	g_pInputs = pInputs;

	ret = qcarcam_query_inputs(pInputs, queryNumInputs, &queryFilled);
	if (QCARCAM_RET_OK != ret || queryFilled != queryNumInputs) {
		LOGERR("Failed qcarcam_query_inputs with ret %d %d %d", ret,
				queryFilled, queryNumInputs);
		//exit(-1);
		nret = -1;
		goto fail;
	}

	LOGINF("--- QCarCam Queried Inputs ----");
	for (i = 0; i < queryFilled; i++) {
		LOGINF(
				"%d: input_id=%d, res=%dx%d fmt=0x%08x 0x%08x fps=%.2f flags=0x%x",
				i, pInputs[i].desc, pInputs[i].res[0].width,
				pInputs[i].res[0].height, pInputs[i].color_fmt[0],
				QCARCAM_FMT_UYVY_8, pInputs[i].res[0].fps, pInputs[i].flags);
	}


	for (input_idx = 0; input_idx < 4; input_idx++) {
		int rc = 0;
		bv_qcarcam_input_t *input_ctxt = &gCtxt.inputs[input_idx];

		input_ctxt->idx = input_idx;
		snprintf(thread_name, sizeof(thread_name), "inpt_ctxt_%d", input_idx);

		rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0,
				&bv_qcarcam_setup_input_ctxt_thread, input_ctxt, 0, thread_name,
				&input_ctxt->thread_handle);
		if (rc) {
			LOGERR("CameraCreateThread failed : %s", thread_name);
			goto fail;
		}

		rc = CameraCreateSignal(&gCtxt.inputs[input_idx].m_eventHandlerSignal);
		if (rc) {
			LOGERR("CameraCreateSignal failed, rc=%d", rc);
			goto fail;
		}
#if 1
		snprintf(thread_name, sizeof(thread_name), "process_cb_event_%d",
				input_idx);
		rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &process_cb_event_thread,
				&gCtxt.inputs[input_idx], 0, thread_name,
				&input_ctxt->process_cb_event_handle);
		if (rc) {
			LOGERR("Create cb event process thread failed, rc=%d", rc);
			goto fail;
		}
#endif
#if 0
		if (input_ctxt->delay_time)
		{
			snprintf(thread_name, sizeof(thread_name), "inpt_ctxt_usr_process_%d", input_idx);
			rc = CameraCreateThread(QCARCAM_THRD_PRIO, 0, &inpt_ctxt_usr_process_thread, &gCtxt.inputs[input_idx], 0, thread_name, &input_ctxt->usr_process_thread_handle);
			if (rc)
			{
				LOGINF("CameraCreateThread failed : %s", thread_name);
				goto fail;
			}
		}
#endif
	}

	fail: 
	if (pInputs) {
		free(pInputs); // ������?��???��??����?
	}
	LOGINF("leave %s", __func__);
	return nret;
}

void bv_qcarcam_exit(void) {
	LOGINF("enter %s", __func__);

	if (g_pInputs) {
		free(g_pInputs);
		g_pInputs = NULL;
	}
	
	qcarcam_uninitialize();
	LOGINF("leave %s", __func__);
}

static void bv_read_pps_data(void) {
	int fd = -1, viewmode;
	size_t numBytes;
	char cmd[128], buffer[PPS_BUFFER_SIZE_MAX];

	bool show3D = true;
	LOGINF("enter %s", __func__);
	sprintf(cmd, PPS_SERVICE_PATH);
	fd = open(cmd, O_RDONLY);
	if (fd < 0) {
		LOGERR("Couldn't open pps first time.");
		while (fd < 0) {
			usleep(500000);
			fd = open(cmd, O_RDONLY);
		}
	}
	pps_decoder_t decoder;
	pps_decoder_error_t ret;

	//int num = 0;
	while (!rvc_exit) {
		//num++;
		///if(num > 5) break;

		uint32_t recoveryState = 0;

		numBytes = read(fd, buffer, sizeof(buffer));
		if (numBytes > 0 && numBytes < PPS_BUFFER_SIZE_MAX) {
			ret = pps_decoder_initialize(&decoder, NULL);
			if (ret == PPS_DECODER_OK) {
				buffer[numBytes] = 0;
				pps_decoder_parse_pps_str(&decoder, buffer);
				pps_decoder_push(&decoder, NULL);

				int t;
				const char *name;

				t = pps_decoder_type(&decoder, NULL);
				//LOGINF("t = %d", t);

				if (PPS_TYPE_OBJECT == t) {
					name = pps_decoder_name(&decoder);
					if (!strcmp(name, "data")) {
						if (PPS_DECODER_OK
								== pps_decoder_push(&decoder, "data")) {
							if (PPS_DECODER_OK
									== pps_decoder_push_array(&decoder,
											"data")) {
								int cnt, k;
								uint8_t iData[1024];

								cnt = pps_decoder_length(&decoder);
								//LOGINF("cnt = %d,sizeof(iData) = %d.\n",cnt,sizeof(iData));
								memset(iData, 0, sizeof(iData));
								if (cnt > sizeof(iData)) {
									cnt = sizeof(iData);
								}

								if (cnt > 34)
									cnt = 34;
								for (k = 0; k < cnt + 1; k++) {
									int v, r;
									float steering_angle = 0;

									if (PPS_DECODER_OK == (r = pps_decoder_get_int( &decoder, NULL, &v))) {
										iData[k] = (uint8_t) v;

										if (3 == k) {//((2 == k) || (3 == k)) {
											//LOGINF("steering_angle=%02x, %02x\n", (char)iData[2], (char)iData[3]);
											//steering_angle = ((int8)iData[2]) << 8;
											//steering_angle += (int8)iData[3];
											steering_angle = ((char) iData[3] << 8) | (char) iData[2];
											//LOGINF("steering_angle=%0.1f\n", (float)((steering_angle / 10.0f) - 780) * 36 / 780);
											//bwSetWheelAngle((float) ((780 - (steering_angle / 10.0f)) * 36 / 780 * 540 / 36));
											//bwSetWheelAngle((float) ((780 - (steering_angle / 10.0f)) / 780 * 540 * 1.5));
											if(((float) (780 - (steering_angle / 10.0f)) > -540) &&
													((float) (780 - (steering_angle / 10.0f)) < 540)){
												bwSetWheelAngle((float) (780 - (steering_angle / 10.0f)));
											}
											//printf("WheelAngle=======%02f\n", (float) (780 - (steering_angle / 10.0f)));
										} else if (8 == k) {
											//LOGINF("gear=%d\n", iData[8]);
											switch (iData[8]) {
											case RVC_D_GEAR:
												if (RVC_D_GEAR != rvc_gear) {
													bwSetCarIsDgear(1);
													bwSetCarIsBack(0);
													rvc_gear = RVC_D_GEAR;
													//LOGINF("recv D gear %d", rvc_st);
												}
												break;
											case RVC_N_GEAR:
												if (RVC_N_GEAR != rvc_gear) {
													bwSetCarIsDgear(1);
													bwSetCarIsBack(0);
													rvc_gear = RVC_N_GEAR;
													//LOGINF("recv N gear");
												}
												break;
											case RVC_R_GEAR:
												if (RVC_R_GEAR != rvc_gear) {
													bwSetCarIsDgear(0);
													bwSetCarIsBack(1);
													rvc_gear = RVC_R_GEAR;
													//LOGINF("##################recv R gear\n");
												}
												if (rvc_st != RVC_SHOW_ST)
													rvc_st = RVC_SHOW_ST;
												break;
											case RVC_P_GEAR:
												if (RVC_P_GEAR != rvc_gear) {
													rvc_gear = RVC_P_GEAR;
													//LOGINF("recv P gear");
												}
												if (rvc_st != RVC_HIDE_ST)
													rvc_st = RVC_HIDE_ST;
												break;
											default:
												break;
											}
										}
										//else if((15 >= k) && (17 <= k))
										else if ((15 == k) || (16 == k) || (17 == k)) {
											//LOGINF("radar M=%d, R=%d, L=%d\n", iData[15], iData[16], iData[17]);
											bwSetBackRadarStatus(iData[17], iData[15], iData[16]);
										}
									} 
									else {
										LOGERR("pps_decoder_get_int error: r = %d", r);
									}
								}
								recoveryState = iData[0];

								pps_decoder_pop(&decoder);
							}
							pps_decoder_pop(&decoder);
						}
					} 
					else {
						LOGINF("[%s][Line:%d]decode failed!!!", __FUNCTION__, __LINE__);
					}
				}

				pps_decoder_pop(&decoder);
				pps_decoder_cleanup(&decoder);

				//LOGINF("[%s][Line:%d]recoveryState = 0x%x!!!", __FUNCTION__, __LINE__, recoveryState);

			}
		}

		//sleep(1);
		usleep(50*1000);
	}
	LOGINF("leave %s", __func__);
}

static void bv_read_notifyrvc_pps_data(void) {
	int fd = -1, viewmode;
	size_t numBytes;
	char cmd[128], buffer[PPS_BUFFER_SIZE_MAX];

	LOGINF("enter %s", __func__);
	sprintf(cmd, PPS_NOTIFYRVC_PATH);
	fd = open(cmd, O_RDONLY);
	if (fd < 0) {
		LOGERR("Couldn't open notifyrvc_pps first time.");
		printf("Couldn't open notifyrvc_pps first time.\n");
		while (fd < 0) {
			usleep(500000);
			fd = open(cmd, O_RDONLY);
		}
	}
	pps_decoder_t decoder;
	pps_decoder_error_t ret;
	
	while (!rvc_exit) {

		uint32_t appState = 0;

		numBytes = read(fd, buffer, sizeof(buffer));
		if (numBytes > 0 && numBytes < PPS_BUFFER_SIZE_MAX) {
			ret = pps_decoder_initialize(&decoder, NULL);
			if (ret == PPS_DECODER_OK) {
				buffer[numBytes] = 0;
				pps_decoder_parse_pps_str(&decoder, buffer);
				pps_decoder_push(&decoder, NULL);

				int t;
				const char *name;

				t = pps_decoder_type(&decoder, NULL);
				//LOGINF("t = %d", t);

				if (PPS_TYPE_OBJECT == t) {
					name = pps_decoder_name(&decoder);
					if (!strcmp(name, "data")) {
						if (PPS_DECODER_OK == pps_decoder_push(&decoder, "data")) {
							int v = 0;
							pps_decoder_get_int( &decoder, NULL, &v);
							appState = (uint8_t)v;
							if((rvc_st == RVC_SHOW_ST) && (appState == 2)){
								LOGINF("rvc_exit!!!%d %d", rvc_st, appState);
								//printf("rvc_exit!!!%d %d\n", rvc_st, appState);
								rvc_exit = 1;
								g_aborted = 1;
							}else if((rvc_st == RVC_HIDE_ST) && (appState == 0)){
								LOGINF("rvc_exit!!!%d %d", rvc_st, appState);
								//printf("rvc_exit!!!%d %d\n", rvc_st, appState);
								rvc_exit = 1;
								g_aborted = 1;
							}
							pps_decoder_pop(&decoder);
						}
					} 
					else {
						LOGINF("[%s][Line:%d]decode failed!!!", __FUNCTION__, __LINE__);
					}
				}

				pps_decoder_pop(&decoder);
				pps_decoder_cleanup(&decoder);

				//LOGINF("[%s][Line:%d]appState = 0x%x!!!", __FUNCTION__, __LINE__, appState);

			}
		}

		//sleep(1);
		usleep(50*1000);
	}
	LOGINF("leave %s", __func__);
}

static bool bv_write_pps_data(void) {
#if 1
	int m_avmStatus_fd = -1;
	pps_encoder_t encoder;
	const char *str_avmStatus = "/tmp/pps/ivi_service/tx"; //"/var/pps/hmi/avmStatus?nopersist";
	LOGINF("enter %s", __func__);
	m_avmStatus_fd= open(str_avmStatus, O_RDWR | O_CREAT);

	if ( m_avmStatus_fd < 0 )
	{
		LOGERR("[%s] first time m_avmStatus_fd < 0 !!!", __FUNCTION__);
		while (m_avmStatus_fd < 0) {
			usleep(500*1000);
			m_avmStatus_fd = open(str_avmStatus, O_RDWR | O_CREAT);
		}
		//return false;
	}

	uint8_t arr_size = 1;//sizeof(AVMSTATUS);
	uint8_t arr_avmStatus[arr_size] = {0};
	memset(arr_avmStatus, 0, arr_size);
	//memcpy(arr_avmStatus, rvc_st, arr_size);
	arr_avmStatus[0] = 2;//rvc_st;

	pps_encoder_initialize(&encoder, false);
	pps_encoder_start_object(&encoder, "data");
	pps_encoder_add_int(&encoder, "len", 1 );
	pps_encoder_start_array(&encoder, "avmstatus");

	for ( int i = 0; i < arr_size; i++ )
	{
		pps_encoder_add_int(&encoder, 0, arr_avmStatus[i]);
	}

	pps_encoder_end_array(&encoder);
	pps_encoder_end_object(&encoder);

	if ( pps_encoder_buffer(&encoder) != NULL ) {
		write( m_avmStatus_fd, pps_encoder_buffer(&encoder), pps_encoder_length(&encoder) );
	}
	pps_encoder_cleanup(&encoder);
#endif
	LOGINF("leave %s", __func__);
	return true;
}

void *bv_pps_thread(void *arg) {
	LOGINF("enter %s", __func__);

	bv_read_pps_data();

	pthread_exit(0);

	LOGINF("leave %s", __func__);
	return NULL;
}

void *bv_notifyrvc_pps_thread(void *arg) {
	LOGINF("enter %s", __func__);
	bv_read_notifyrvc_pps_data();

	pthread_exit(0);

	LOGINF("leave %s", __func__);
	return NULL;
}

int bv_pps_init(void) {
	int ret = 0;
	pthread_t tid = 0;
	pthread_t tid_app = 0;

	LOGINF("enter %s", __func__);

	if ((ret = pthread_create(&tid, NULL, bv_pps_thread, NULL)) != 0) {
		LOGERR("create pps thread failed");
		exit(1);
	}
	ret = pthread_detach(tid);

	if ((ret = pthread_create(&tid_app, NULL, bv_notifyrvc_pps_thread, NULL)) != 0) {
		LOGERR("create notifyrvc thread failed");
		exit(1);
	}
	ret = pthread_detach(tid_app);

	LOGINF("leave %s: %d", __func__, ret);
	return ret;
}

void bv_pps_exit(void) {
}

#if 1
struct demo_globle_data
{
	struct pm_ops_s demo_ops;
	pm_client_t pm_handle;
}demo_data;

//demo_globle_data demo_data;

static int bv_demo_pm_prepare_cb(void *ctxt)
{
	//struct demo_globle_data *data = (struct demo_globle_data *) ctxt;
	LOGINF("prepare\n");
	
	//you should release any resource
	
	// EOK
	return 0;
}

static int bv_demo_pm_suspend_cb(void *ctxt)
{
	//struct demo_globle_data *data = (struct demo_globle_data *) ctxt;
	LOGINF("suspend\n");
	
	//you should release any resource
	/*if (rvc_st != RVC_HIDE_ST){
		rvc_st = RVC_HIDE_ST;
	}*/
	g_aborted = 1;
	usleep(50);
	#if 1
	bv_deinit_context();
	usleep(100);
	bv_qcarcam_exit();
	//bv_exit_avm_buf();
	#endif
	// EOK
	return 0;
}

static int bv_demo_pm_resume_cb(void *ctxt)
{
	//int ret = 0;
	unsigned long long s_time;
	unsigned long long e_time;
	LOGINF("resume\n");
	#if 1
	g_aborted = 0;
	//bv_init_avm_buf();
	bv_get_time(&s_time);
	bv_init_context();
	bv_get_time(&e_time);
	LOGINF("bv_init_context time: %lld(ms)", (e_time - s_time));
	bv_get_time(&s_time);
	bv_qcarcam_init();
	bv_get_time(&e_time);
	LOGINF("bv_qcarcam_init time: %lld(ms)", (e_time - s_time));
	#endif
	/*usleep(50);
	switch (rvc_st) {
		case RVC_SHOW_ST: 
			if (SCREEN_SHOW_ST != screen_st) {
				ret = bwSetAVMVisible(1);
				LOGINF("bwSetAVMVisible(1) return %d", ret);
				screen_st = SCREEN_SHOW_ST;
			}
			break;
		case RVC_HIDE_ST:
			if (SCREEN_HIDE_ST != screen_st) {
				ret = bwSetAVMVisible(0);
				LOGINF("bwSetAVMVisible(0) return %d", ret);
				screen_st = SCREEN_HIDE_ST;
			}
			break;
		default :
			if (SCREEN_HIDE_ST != screen_st) {
				ret = bwSetAVMVisible(0);
				LOGINF("bwSetAVMVisible(0) return %d", ret);
				screen_st = SCREEN_HIDE_ST;
			}
			break;
		}*/
	return 0;
}

static int bv_demo_pm_complete_cb(void *ctxt)
{
	LOGINF("complete\n");
	return 0;
}

static int bv_demo_pm_pulse_register(struct demo_globle_data *data)
{
	int ret;
	// memset first !!!
	memset((char *)&data->demo_ops, 0, sizeof(data->demo_ops));
	data->demo_ops.suspend  = bv_demo_pm_suspend_cb;
	data->demo_ops.resume  = bv_demo_pm_resume_cb;
	data->demo_ops.prepare  = bv_demo_pm_prepare_cb;
	data->demo_ops.complete   = bv_demo_pm_complete_cb;

	// level 0:
	ret = pm_register("bv_demo_pm", PM_PRIO_LEVEL_0, &data->demo_ops, 0, (void *)data, &data->pm_handle);
	if(0 != ret)
	{
		LOGERR("bv_demo_pm register failed: %d\n", ret);
	}

	LOGINF("bv_demo_pm register success\n");

	return ret;
}

static int bv_demo_pm_pulse_deregister(struct demo_globle_data *data)
{
	int ret = -1;
	
	if (data->pm_handle == NULL)
		return ret;
	
	ret = pm_deregister(data->pm_handle);

	LOGINF("bv_demo_pm deregister %s: %d\n", ret?"fail":"success", ret);
	
	return ret;
}
#endif

#ifdef QNX_GLES_SCREEN_TRIANGLE
void setup_scissor_test() {
	glEnable(GL_SCISSOR_TEST);
	glScissor(100, 100, 200, 200); // $)Ah.>g=.h#???:e?o<??????????0:e/8f?g$:d?
}

void disable_scissor_test() {
	glDisable(GL_SCISSOR_TEST);
}

int bv_triangle_test(void)
{
	int rc;
	int visible = 0;

	rc = initScreen();
	if (rc != EXIT_SUCCESS) {
		return EXIT_FAILURE;
	}

	rc = initEGL();
	if (rc != EXIT_SUCCESS) {
		return EXIT_FAILURE;
	}

	rc = initOpenGL();
	if (rc != EXIT_SUCCESS) {
		return EXIT_FAILURE;
	}

	while (1) {
		LOGINF("render 1");
		if (screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_SIZE, (const int[]) {800, 600}) < 0) {
			LOGERR("screen_set_window_property_iv");
		}
		render();
		rc = eglSwapBuffers(egl_display, egl_surface);
		if (rc != EGL_TRUE) {
			LOGERR("eglSwapBuffers failed\n");
			break;
		}
		sleep(5);

		LOGINF("render 2");
		visible = 0;
		screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_VISIBLE, &visible);
		render();
		rc = eglSwapBuffers(egl_display, egl_surface);
		if (rc != EGL_TRUE) {
			LOGERR("eglSwapBuffers failed\n");
			break;
		}
		sleep(5);
		visible = 1;
		screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_VISIBLE, &visible);

		LOGINF("render 3");
		if (screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_SIZE, (const int[]) {1920, 1080}) < 0) {
			LOGINF("screen_set_window_property_iv");
		}
		render();
		rc = eglSwapBuffers(egl_display, egl_surface);
		if (rc != EGL_TRUE) {
			LOGERR("eglSwapBuffers failed\n");
			break;
		}
		sleep(5);

		LOGINF("render 4");
		visible = 0;
		screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_VISIBLE, &visible);
		rc = eglSwapBuffers(egl_display, egl_surface);
		if (rc != EGL_TRUE) {
			LOGERR("eglSwapBuffers failed\n");
			break;
		}
		sleep(5);
		visible = 1;
		screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_VISIBLE, &visible);

		LOGINF("render 5");
		if (screen_set_window_property_iv(screen_win, SCREEN_PROPERTY_SIZE, (const int[]) {800, 600}) < 0) {
			LOGINF("screen_set_window_property_iv");
		}
		render();
		rc = eglSwapBuffers(egl_display, egl_surface);
		if (rc != EGL_TRUE) {
			LOGERR("eglSwapBuffers failed\n");
			break;
		}
		sleep(5);

		break;
	}

	// Clean up EGL related objects
	eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
	eglDestroySurface(egl_display, egl_surface);
	eglDestroyContext(egl_display, egl_ctx);
	eglTerminate(egl_display);
	eglReleaseThread();

	// Clean up screen-related objects
	screen_destroy_window(screen_win);
	screen_destroy_context(screen_ctx);
}
#endif

int main(int argc, char *argv[]) {
	int ret = 0;
	int count = 0;
	unsigned long long start_time, s_time;
	unsigned long long end_time, e_time;

#ifdef QNX_GLES_SCREEN_TRIANGLE
	printf("QNX rvc test!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	bv_triangle_test();
	return 0;
#endif
#if 0
	bv_init_context();
	bv_qcarcam_init();
	count = 0;
	while (!rvc_exit) {
#if 1
		count++;
		if (count > 2000)
			break;
#endif
		usleep(20000);
	}
	
	rvc_exit = 1;
	g_aborted = 1;
#endif
#if 1
	rvc_exit = 0;
	rvc_st = RVC_DEINIT_ST;
	rvc_gear = RVC_UKN_GEAR;
	s_time = 0;
	e_time = 0;
	static int qcarcam_waittime = 0;

	LOGINF("rvc start: %s %s", __DATE__, __TIME__);
	usleep(5000*1000);
	printf("rvc start: %s %s\n", __DATE__, __TIME__);
	/* Should be placed before any forked thread */
	sigfillset(&g_sigset);
	for (int i = 0; exceptsigs[i] != -1; i++) {
		sigdelset(&g_sigset, exceptsigs[i]);
	}
	pthread_sigmask(SIG_BLOCK, &g_sigset, NULL);

	bv_get_time(&s_time);
	bv_pps_init();
	bv_get_time(&e_time);
	LOGINF("bv_pps_init time: %lld(ms)", (e_time - s_time));
	bv_init_avm_buf();
	bv_get_time(&s_time);
	bv_init_context();
	bv_get_time(&e_time);
	LOGINF("bv_init_context time: %lld(ms)", (e_time - s_time));
	bv_get_time(&s_time);
	bv_qcarcam_init();
#if 1
	while ((qcarcam_setup_input_flag != 1) && (qcarcam_waittime < 20)) {
		usleep(200*1000);
		qcarcam_waittime++;
		LOGINF("qcarcam_setup_input_flag error %d, %d", qcarcam_setup_input_flag, qcarcam_waittime);
	}
#endif
#if 1
	bv_get_time(&e_time);
	LOGINF("bv_qcarcam_init time: %lld(ms)", (e_time - s_time));
	LOGINF("enter avmInit");
	avmInit();
	LOGINF("leave avmInit");
	//bv_write_pps_data();
	bv_demo_pm_pulse_register(&demo_data);
#endif
#if 1
	count = 0;
	while (!rvc_exit) {
		start_time = 0;
		end_time = 0;
		//bv_get_time(&s_time);
		//bv_get_time(&start_time);
		//pthread_mutex_lock(&gCtxt.mutex_bvavm);

		//LOGINF("rvc_st=%d rvc_gear=%d screen_st=%d", rvc_st, rvc_gear, screen_st);

		//bv_get_time(&end_time);
		//printf("render time: %lld(ms)\n", (end_time - start_time));
		//avmRender(yuvData, BW_2D_REAR_UNDISTORT);
#if 1
		//ret = avmRender(yuvData, BW_2D_REAR_UNDISTORT);
		if (RVC_R_GEAR == rvc_gear) {
			//bv_get_time(&start_time);
			ret = avmRender(yuvData, BW_2D_REAR_UNDISTORT);
			//bv_get_time(&end_time);
			//LOGINF("render time: %lld(ms)", (end_time - start_time));
			//printf("render time: %lld(ms)\n", (end_time - start_time));
		} else /*if (RVC_D_GEAR == rvc_gear || RVC_N_GEAR == rvc_gear) */{
			//bv_get_time(&start_time);
			ret = avmRender(yuvData, BW_2D_FRONT_UNDISTORT);
			//bv_get_time(&end_time);
			//LOGINF("render time: %lld(ms)", (end_time - start_time));
		}
		switch (rvc_st) {
		case RVC_SHOW_ST: {
			//pthread_mutex_lock(&gCtxt.mutex_bvavm);
			
			//pthread_mutex_unlock(&gCtxt.mutex_bvavm);

			if (SCREEN_SHOW_ST != screen_st) {
				ret = bwSetAVMVisible(1);
				LOGINF("bwSetAVMVisible(1) return %d", ret);
				screen_st = SCREEN_SHOW_ST;
			}
		}
			break;
		case RVC_HIDE_ST:
			if (SCREEN_HIDE_ST != screen_st) {
				ret = bwSetAVMVisible(0);
				LOGINF("bwSetAVMVisible(0) return %d", ret);
				screen_st = SCREEN_HIDE_ST;
			}
			break;
		default :
			if (SCREEN_HIDE_ST != screen_st) {
				ret = bwSetAVMVisible(0);
				LOGINF("bwSetAVMVisible(0) return %d", ret);
				screen_st = SCREEN_HIDE_ST;
			}
			break;
		}
#endif
		//pthread_mutex_unlock(&gCtxt.mutex_bvavm);

#if 1
		count++;
		if (count > 1100)//500)
			break;
#endif

		usleep(20000);
		//bv_get_time(&end_time);
		//printf("count time: %lld(ms)\n", (end_time - start_time));
	}
#endif
#if 1
	rvc_exit = 1;
	g_aborted = 1;
	usleep(100);

	avmDeInit();
	//bv_pps_exit();
	bv_exit_avm_buf();
	bv_deinit_context();
	bv_qcarcam_exit();
	bv_demo_pm_pulse_deregister(&demo_data);
	LOGINF("rvc finish");
#endif
#endif
	return EXIT_SUCCESS;
}
