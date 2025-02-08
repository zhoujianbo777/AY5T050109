/**===========================================================================

  @file gpio_devctl.h
  @brief Macros for gpio_client/gpio_drv.
  
  @details Macros for gpio_client/gpio_drv.

  @copyright
         Copyright (c) 2012-2017 QUALCOMM Technologies Incorporated.
         All rights reserved. Licensed Material - Restricted Use.
         Qualcomm Confidential and Proprietary.

===========================================================================*/

/*===========================================================================

                      EDIT HISTORY FOR FILE

This section contains comments describing changes made to this file.
Notice that changes are listed in reverse chronological order.

$Header$
$DateTime$
$Author$
$Change$ 
 
when       who          what, where, why
--------   ---       ------------------------------------------------------
02/22/17   aneeketp    Adding dalconfig support for pin types
04/17/13   aneeketp  Update mask configs for gpio_set_config for pmic 
                     gpio for the CR464575 for all Families.
03/18/13   aneeketp  Update documentation as part of the CR464575 Fix
===========================================================================*/

#ifndef GPIO_DEVCTL_H_
#define GPIO_DEVCTL_H_
/*===========================================================================
                     INCLUDE FILES
===========================================================================*/
#include <stdint.h>
#include <devctl.h>
#include "gpio_client.h"

#define _DCMD_GPIO   _DCMD_MISC

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
struct gpio_header{
	uint32_t gpio;
	uint32_t cfg;
	uint32_t val;
	uint32_t trigger;
	uint32_t mask; 		//cfg and get/set(multiple pins) mask
	uint32_t pid;
	uint32_t chid;
}_gpio_header;

typedef struct _gpio_type_t {
    char *gpio_type_name;
    uint32_t moduleid;
    uint32_t start_gpio;
    uint32_t total_gpio;
} gpio_type_t;

/* devctl commands*/
#define DCMD_GPIO_SET_CONFIG                __DIOT (_DCMD_GPIO, 	1, 	_gpio_header)
#define DCMD_GPIO_GET_CONFIG                __DIOT (_DCMD_GPIO, 	2, 	_gpio_header)
#define DCMD_GPIO_SET_PIN                   __DIOT (_DCMD_GPIO, 	3, 	_gpio_header)
#define DCMD_GPIO_GET_PIN                   __DIOT (_DCMD_GPIO, 	4, 	_gpio_header)
#define DCMD_GPIO_SET_INTERRUPT_CFG         __DIOT (_DCMD_GPIO, 	5, 	_gpio_header)
#define DCMD_GPIO_GET_INTERRUPT_CFG         __DIOT (_DCMD_GPIO, 	6, 	_gpio_header)
#define DCMD_GPIO_GET_MULTIPLE_PIN          __DIOT (_DCMD_GPIO, 	7, 	_gpio_header)
#define DCMD_GPIO_SET_MULTIPLE_PIN          __DIOT (_DCMD_GPIO, 	8, 	_gpio_header)
#define DCMD_GPIO_TRIGGER_INTERRUPT         __DIOTF(_DCMD_GPIO, 	9, 	_gpio_header)
#define DCMD_GPIO_INTERRUPT_CHANNEL_CREATE  __DIOT (_DCMD_GPIO, 	10, _gpio_header)
#define DCMD_GPIO_INTERRUPT_WAIT  			__DIOT (_DCMD_GPIO, 	11, _gpio_header)
#define DCMD_GPIO_INTERRUPT_CONTROL  		__DIOT (_DCMD_GPIO, 	12, _gpio_header)
#define DCMD_GPIO_INTERRUPT_GET_PEND_STS  	__DIOT (_DCMD_GPIO, 	13, _gpio_header)

/******Init functions*********/
int 	gpio_rm_init(const char *devname );
void*	gpiolib_open(char * dev_name);
int 	gpiolib_set_config (void * handle, uint32_t gpio,uint32_t cfg_mask, uint32_t cfg );
int32_t gpiolib_set_interrupt_cfg (void *handle, uint32_t gpio, uint32_t trigger,void *isr);
int32_t gpiolib_get_interrupt_cfg (void *handle, uint32_t gpio, uint32_t *status); 
int32_t gpiolib_get_interrupt_trigger (void *handle, uint32_t gpio, uint32_t *status);
#endif /* GPIO_DEVCTL_H_ */
