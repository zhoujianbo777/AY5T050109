/**===========================================================================

  @file gpio_client_msmgpio.h
  @brief MSM GPIO configuration related macros and masks Header File
  
  @details  MSM GPIO macros and masks.

  @copyright
         Copyright (c) 2013-2014 QUALCOMM Technologies Incorporated.
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
 
when       	who          	what, where, why
--------   	---       		------------------------------------------------------
05/26/15    	KNR			MSM GPIO configuration related macros and masks.
===========================================================================*/

#ifndef GPIO_CLIENT_MSMGPIO_H_
#define GPIO_CLIENT_MSMGPIO_H_
/* How to use the gpio client to configure msm/tlmm gpio ( cfg_mask and cfg)
 * ----------------------------------------------------
 *  See MACRO Definitions to see what 'parameters' are needed to be configured
 *  For INPUT Configuration :
 *                              cfg_mask = 0x0 ( This is ignored for TLMM gpios ) 
 *                              cfg = GPIO_PIN_CFG(parameters)
 *
 *  For OUTPUT Configuration :
 *                              cfg_mask = 0x0 ( This is ignored for TLMM gpios ) 
 *                              cfg = GPIO_PIN_CFG(parameters)
 *
*/

//++++++++++++ MACROS for MSM (TLMM) GPIOs ++++++++++++++++++++++++++++++++++

#define GPIO_PIN_CFG(dir,pull,drive,func) 	( dir | pull | drive | GPIO_FUNC(func) )

//Parameters for GPIO_PIN_CFG Macro
//dir (0-3th bit)
#define GPIO_INPUT              		0x00000000
#define GPIO_OUTPUT             		0x00000001

// pull (4-7 bits)
#define GPIO_NO_PULL            		0x00000000
#define GPIO_PULL_DOWN          		0x00000010
#define GPIO_KEEPER             		0x00000020
#define GPIO_PULL_UP            		0x00000030

//drive (8-11 bits)
#define	GPIO_STRENGTH_2MA       		0x00000000
#define	GPIO_STRENGTH_4MA       		0x00000100
#define	GPIO_STRENGTH_6MA       		0x00000200
#define	GPIO_STRENGTH_8MA       		0x00000300
#define	GPIO_STRENGTH_10MA      		0x00000400
#define	GPIO_STRENGTH_12MA      		0x00000500
#define	GPIO_STRENGTH_14MA      		0x00000600
#define	GPIO_STRENGTH_16MA      		0x00000700

//Func : If not using any special function : Then func = 0 . 
//Please refer to gpio maps in ipcat to see list alternate functions can be configured for each gpio.
#define GPIO_FUNC_MASK          			0x000F0000
#define GPIO_FUNC(func)        				(GPIO_FUNC_MASK & ((func) << 0x10))

//Parameters for gpio_set_pin api
//low/high
#define GPIO_LOW_VALUE          		0x00000000
#define GPIO_HIGH_VALUE         		0x00000001

//gpio interrupt trigger type
#define GPIO_INTERRUPT_TRIGGER_DISABLE     0x0
#define GPIO_INTERRUPT_TRIGGER_HIGH        0x1
#define	GPIO_INTERRUPT_TRIGGER_LOW         0x2
#define GPIO_INTERRUPT_TRIGGER_RISING      0x3
#define GPIO_INTERRUPT_TRIGGER_FALLING     0x4
#define GPIO_INTERRUPT_TRIGGER_BOTH_EDGES  0x5

//
// - 0 indicates this gpio does not have interrupt configured or error
// - 0x80000000 indicates that ISR has already been configured within QNPCore and
//   client will use channel to wait for interrupts
// - non-zero is the IRQ number which client can directly attach ISRs
//
#define GPIO_INTERRUPT_STATUS_ENABLED   0x80000000
#define GPIO_INTERRUPT_STATUS_DISABLED  0

#endif /* GPIO_CLIENT_MSMGPIO_H_ */
