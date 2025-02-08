#ifndef MAIN_RVC_H
#define MAIN_RVC_H

#include <queue>
#include <mutex>
#include <condition_variable>
#include <cstdint>

// 模拟qcarcam相关定义
typedef enum {
    QCARCAM_RET_OK = 0,
    QCARCAM_RET_FAILED = -1
} qcarcam_ret_t;

typedef struct {
    int dummy;
} qcarcam_init_t;

// 模拟CameraOSServices相关定义
typedef struct {
    int dummy; 
} CameraOSServices;

// 模拟pmem相关定义
typedef struct {
    int dummy;
} pmem;

// 定义和声明main_rvc.cpp中的公共接口
int bv_qcarcam_init(void);
void bv_qcarcam_exit(void);
int avmRender(unsigned char** yuvData, int mode);
int bv_demo_pm_suspend_cb(void *ctxt);

// 定义测试需要的常量
#define BVAVM_QCAM_SIZE (1280*800*2)
#define BW_2D_REAR_UNDISTORT 0
#define BW_2D_FRONT_UNDISTORT 1

#endif // MAIN_RVC_H
