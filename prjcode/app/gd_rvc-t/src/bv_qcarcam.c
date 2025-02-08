#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../inc/qcarcam.h"
#include "../inc/bv_qcarcam.h"
#include "../inc/bv_log.h"

qcarcam_input_t *g_pInputs = NULL;

int bv_qcarcam_init(void)
{
    int nret = -1;
    qcarcam_ret_t ret = QCARCAM_RET_OK;
    qcarcam_init_t qcarcam_init = {0};
    int i = 0;
    qcarcam_input_t *pInputs = NULL;
    unsigned int queryNumInputs = 0, queryFilled = 0;
    const unsigned int MAX_RETRY_COUNT = 10;
    unsigned int retryCount = 0;

    /* Initialize qcarcam */
    qcarcam_init.version = QCARCAM_VERSION;
    qcarcam_init.debug_tag = (char *)"qcarcam_bvavm";
    ret = qcarcam_initialize(&qcarcam_init);
    if (ret != QCARCAM_RET_OK)
    {
        BVAVM_ERRORMSG("qcarcam_initialize failed %d", ret);
        goto exit_flag_1;
    }

    /* Query number of inputs with retry mechanism */
    do {
        ret = qcarcam_query_inputs(NULL, 0, &queryNumInputs);
        if (QCARCAM_RET_OK == ret && queryNumInputs > 0)
        {
            break;
        }
        BVAVM_ERRORMSG("Failed qcarcam_query_inputs number of inputs with ret %d", ret);
        usleep(500000);
        retryCount++;
    } while (retryCount < MAX_RETRY_COUNT);

    if (retryCount >= MAX_RETRY_COUNT)
    {
        BVAVM_ERRORMSG("Max retry count reached for query inputs");
        goto exit_flag_2;
    }

    if (queryNumInputs > 32)  /* Reasonable upper limit for number of cameras */
    {
        BVAVM_ERRORMSG("Unreasonable number of inputs: %d", queryNumInputs);
        goto exit_flag_2;
    }

    BVAVM_INFOMSG("queryNumInputs=%d", queryNumInputs);

    /* Allocate memory for inputs */
    pInputs = (qcarcam_input_t *)calloc(queryNumInputs, sizeof(*pInputs));
    if (!pInputs)
    {
        BVAVM_ERRORMSG("Failed to allocate pInputs");
        goto exit_flag_2;
    }

    /* Query input details */
    ret = qcarcam_query_inputs(pInputs, queryNumInputs, &queryFilled);
    if (QCARCAM_RET_OK != ret || queryFilled != queryNumInputs)
    {
        BVAVM_ERRORMSG("Failed qcarcam_query_inputs with ret %d %d %d", ret, queryFilled, queryNumInputs);
        goto exit_flag_3;
    }

    /* Log input details */
    BVAVM_INFOMSG("--- QCarCam Queried Inputs ----");
    for (i = 0; i < queryFilled; i++)
    {
        BVAVM_INFOMSG("%d: input_id=%d, res=%dx%d fmt=0x%08x 0x%08x fps=%.2f flags=0x%x",
            i, pInputs[i].desc,
            pInputs[i].res[0].width, pInputs[i].res[0].height,
            pInputs[i].color_fmt[0], QCARCAM_FMT_UYVY_8,
            pInputs[i].res[0].fps, pInputs[i].flags);
    }

    g_pInputs = pInputs;
    nret = 0;  /* Success */
    return nret;

exit_flag_3:
    free(pInputs);
exit_flag_2:
    qcarcam_uninitialize();
exit_flag_1:
    return nret;
}

int bv_qcarcam_get_frame(void)
{
    qcarcam_ret_t ret = QCARCAM_RET_OK;
    
    if (!g_pInputs) {
        BVAVM_ERRORMSG("Inputs not initialized");
        return -1;
    }

    /* Basic frame capture implementation */
    qcarcam_frame_t frame = {0};
    ret = qcarcam_get_frame(g_pInputs[0].desc, &frame, 100); /* 100ms timeout */
    if (ret != QCARCAM_RET_OK) {
        BVAVM_ERRORMSG("Failed to get frame: %d", ret);
        return -1;
    }

    /* Process frame here */
    /* Note: Actual frame processing would depend on application requirements */
    
    ret = qcarcam_release_frame(g_pInputs[0].desc, &frame);
    if (ret != QCARCAM_RET_OK) {
        BVAVM_ERRORMSG("Failed to release frame: %d", ret);
        return -1;
    }

    return 0;
}

void bv_qcarcam_exit(void)
{
    qcarcam_ret_t ret;

    if (g_pInputs)
    {
        free(g_pInputs);
        g_pInputs = NULL;
    }

    ret = qcarcam_uninitialize();
    if (ret != QCARCAM_RET_OK)
    {
        BVAVM_ERRORMSG("Failed to uninitialize qcarcam: %d", ret);
    }
}
