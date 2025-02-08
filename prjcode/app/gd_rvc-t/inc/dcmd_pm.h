/*=======================================================================
Copyright (c) 2019-2020 QUALCOMM Technologies, Inc. (QTI). All Rights Reserved.
========================================================================*/
#ifndef _QC_PM_DCMD_
#define _QC_PM_DCMD_

#include <devctl.h>
#include <inttypes.h>

#define PM_MAX_NAME_LEN (64)

//Helper macro to properly initialize the pm_register_s structure
#define INIT_PM_REGISTER_STRUCT(__init) \
        (__init)->pulse_codes[PM_STATE_PREPARE] = -1;\
        (__init)->pulse_codes[PM_STATE_SUSPEND] = -1;\
        (__init)->pulse_codes[PM_STATE_RESUME] = -1;\
        (__init)->pulse_codes[PM_STATE_COMPLETE] = -1;\
        (__init)->pulse_codes[PM_STATE_VOLT_NOK] = -1;\
        (__init)->pulse_codes[PM_STATE_VOLT_OK] = -1;\
        (__init)->priority = (enum pm_prio_level)-1;\
        (__init)->chid = -1;\
        (__init)->flags = 0;\
        (__init)->name[0] = '\0';

enum pm_prio_level {
    PM_PRIO_LEVEL_0,
    PM_PRIO_LEVEL_1,
    PM_PRIO_LEVEL_2,
    PM_PRIO_LEVEL_3,
    PM_PRIO_LEVEL_4,
    PM_PRIO_LEVEL_5,
    PM_PRIO_LEVEL_6,
    PM_PRIO_LEVEL_7,
    PM_PRIO_LEVEL_8,
    PM_PRIO_LEVEL_9,
    PM_PRIO_LEVEL_MAX,
};

enum pm_state {
    PM_STATE_PREPARE,
    PM_STATE_SUSPEND,
    PM_STATE_RESUME,
    PM_STATE_COMPLETE,
    PM_STATE_VOLT_NOK,
    PM_STATE_VOLT_OK,
    PM_STATE_MAX,
};

struct pm_register_s {
    char name [PM_MAX_NAME_LEN] ; 
    int8_t pulse_codes [ PM_STATE_MAX ] ;
    enum pm_prio_level priority;
    int chid ; 
#define PM_FLAG_NO_SUSPEND       (0x00000001)
    int flags ; 
};

struct pm_ack_s {
    int rc ; 
    enum pm_state state;
};

struct pm_str_s {
    uint64_t entry_timeout ; 
    uint64_t wakeup_timeout ;
};

enum pm_task_cmd {
    PM_TASK_STOP,
    PM_TASK_RUN,
};

struct pm_task_ctl {
    pid_t pid ; 
    enum pm_task_cmd cmd ;
};

#define DCMD_PM_REGISTER        __DIOT (_DCMD_MISC, 0, struct pm_register_s )
#define DCMD_PM_ACK             __DIOT (_DCMD_MISC, 1, struct pm_ack_s )
#define DCMD_PM_ENTER_STR       __DIOT (_DCMD_MISC, 2, struct pm_str_s )
#define DCMD_PM_RESUME          __DION (_DCMD_MISC, 3)
#define DCMD_PM_TASK_CTL        __DIOT (_DCMD_MISC, 4, struct pm_task_ctl )

#endif
