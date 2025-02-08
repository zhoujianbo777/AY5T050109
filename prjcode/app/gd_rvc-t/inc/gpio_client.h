/**===========================================================================

  @file gpio_client.h
  
  @brief GPIO Resource Manager Utility Header File

  @copyright
            Copyright (c) 2021 QUALCOMM Technologies Incorporated.
            All rights reserved. Licensed Material - Restricted Use.
            Qualcomm Confidential and Proprietary.

===========================================================================*/

/*===========================================================================

                           EDIT HISTORY FOR FILE

$Header: //deploy/qcom/qct/platform/qnp/qnx/components/rel/plt_gpio.qxa/1.0/gpio_client/public/amss/gpio_client.h#2 $
$DateTime: $
$Author: aneeketp $
$Change: 5921114 $

when       who      what, where, why
--------   ---      ----------------------------------------------------------
06/13/16   aneeketp Cleanup documentation 
05/19/14   aneeketp Add gpio_close api to close gpio dev handle 
===========================================================================*/

#ifndef GPIO_CLIENT_H_
#define GPIO_CLIENT_H_

#include "gpio_client_msmgpio.h"
#include "gpio_client_pmicgpio.h"
#include <inttypes.h>

/*===========================================================================
 MACROS
============================================================================*/
#define GPIO_NUM_MASK        0xFFFF
#define GPIO_MODULE_ID_MASK  0xFFFF0000
#define MODULE_ID_SHIFT		 16
#define GPIO_FAIL       	 -1
#define GPIO_SUCCESS     	 0


/*
 * The gpio_client.h supports different types of GPIOs with common apis.
// To uniquely identify the gpios between each module , we reference them with 32bit identifier
// The gpio numbers referenced by the APIs in the header 
// are 32bit identifier which are to be configured as follows
// gpio = GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT
*/

/*******************************************************
 *               GPIO_MODULE_TYPE_16BIT                *
 *******************************************************/
#define TLMM_MODULE             (0x0000<<MODULE_ID_SHIFT)
#define PMIC_MODULE             (0x0001<<MODULE_ID_SHIFT)
#define PMIC_1_GPIO_MODULE      (0x0001<<MODULE_ID_SHIFT)
#define PMIC_2_GPIO_MODULE      (0x0002<<MODULE_ID_SHIFT)
#define PMIC_3_GPIO_MODULE      (0x0003<<MODULE_ID_SHIFT)
#define PMIC_4_GPIO_MODULE      (0x0004<<MODULE_ID_SHIFT)
#define SSC_TLMM_MODULE         (0x0005<<MODULE_ID_SHIFT)


/*******************************************************
 *        Example to configure32 bit GPIO identifier   *
 *******************************************************
   The high 16 bits identify the module as follows:
        0x0000   TLMM/MSM
        0x0001   PMIC_1_GPIO_MODULE
        0x0002   PMIC_2_GPIO_MODULE
        ...
        0x0005    SSC_TLMM_MODULE

   The lower 16 bits identify the gpio number.

   Examples:
        TLMM GPIO 140             	= TLMM_MODULE|6 		    =    0x00000006
        PMIC 1 GPIO 8               = PMIC_1_GPIO_MODULE|8	=    0x00100008
*/


// Extract GPIO_MODULE_TYPE from 32 bit gpio identifier 
#define GPIO_MODULE_TYPE(gpio)	   		 (gpio&GPIO_MODULE_ID_MASK)
// Extract GPIO_NUMBER from 32 bit gpio identifier
#define GPIO_NUM(gpio)					 (gpio&GPIO_NUM_MASK)
//For Legacy Purposes
#define GET_IONUM(module_id,pin_num)     (module_id|pin_num) 
/*===========================================================================
 FUNCTION PROTOTYPES
============================================================================*/

/*
 * Gain access to internal GPIO driver node
 * Returns a handle that is passed to all other functions.
 *
 * Parameters:
 * (in)  dev_name  Resource Manager connection ("/dev/gpio")
 *                   This devname is currently ignored as it internally decides to open the correct device.
 *
 * Returns:
 *   Valid fd on success 
 *   GPIO_FAIL on failure
 */
int gpio_open(char *dev_name);


/*
 * Close handle opened by gpio_open()
 *
 * Parameters:
 * (in)  dev_name  Resource Manager connection ("/dev/gpio")
 *
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_close(char *dev_name);


/*
 * Set the configuration of the GPIO pin with the input cfg value
 * The cfg_mask and cfg values can be configured with helper macros pertaining to the GPIO Type
 * a) For TLMM GPIO Please see gpio_client_msmgpio.h. The cfg_mask is to be IGNORED for TLMM GPIOs
 * b) For PMIC GPIO Please see gpio_client_pmicgpio.h. Both cfg_mask and cfg need to be configured for PMIC GPIOs
 *
 * Parameters:
 * (in)  fd        File descriptor from gpio_open()
 * (in)  gpio      32 bit GPIO identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *                   See Documentation in the beginning of the header.
 * (in)  cfg_mask  Mask to be applied on PMIC GPIOs. This allows to set the selective configurations 
 *                   This Mask is ignored for TLMM GPIO. Refer to appropriate GPIO specific headers for macros
 * (in)  cfg       Configuration to be applied on GPIO pins. Refer to appropriate GPIO specific header for macros 
 * 
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_set_config(int fd, uint32_t gpio, uint32_t cfg_mask, uint32_t cfg);


/*
 * Get the configuration of the GPIO pin
 * The cfg values can be parsed using the helper macros pertaining to the GPIO Type
 * a) For TLMM GPIO See gpio_client_msmgpio.h 
 * b) For PMIC GPIO See gpio_client_pmicgpio.h
 *
 * Parameters:
 * (in)  fd    File descriptor from gpio_open()
 * (in)  gpio  32 bit GPIO identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *               See documentation in the beginning of the header.
 * (out) *cfg  Configuration of the given GPIO pin 
 * 
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_get_config(int fd, uint32_t gpio, uint32_t *cfg);


/*
 * Set the value of a GPIO pin. Assign the GPIO a value of HIGH (1) or LOW (0)
 *
 * Parameters:
 * (in)  fd    File descriptor from gpio_open()
 * (in)  gpio  32 bit GPIO identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *               See documentation in the beginning of the header.
 * (in)  val   Value to assign to GPIO pin (1 or 0)
 *
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_set_pin(int fd, uint32_t gpio, uint32_t val);


/*
 * Get the value of a GPIO pin
 *
 * Parameters:
 * (in)  fd    File descriptor from gpio_open()
 * (in)  gpio  32 bit GPIO identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *               See documentation in the beginning of the header.
 * (out) *val  Value on the actual GPIO pin
 *
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_get_pin(int fd, uint32_t gpio, uint32_t *val);


/*
 * Set interrupt trigger on the specified pin number. 
 * This API supports only TLMM interrupts.
 *
 * Parameters:
 * (in)  fd       File descriptor from gpio_open()
 * (in)  gpio     32 bit gpio identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *                  See documentation in the beginning of the header.
 * (in)  trigger  Interrupt trigger type. Can be one of the following:
 *                  (List taken from gpio_client_msmgpio.h)
 *                  GPIO_INTERRUPT_TRIGGER_DISABLE     0x0
 *                  GPIO_INTERRUPT_TRIGGER_HIGH        0x1
 *                  GPIO_INTERRUPT_TRIGGER_LOW         0x2
 *                  GPIO_INTERRUPT_TRIGGER_RISING      0x3
 *                  GPIO_INTERRUPT_TRIGGER_FALLING     0x4
 *                  GPIO_INTERRUPT_TRIGGER_BOTH_EDGES  0x5
 * 
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_set_interrupt_cfg(int fd, uint32_t gpio, uint32_t trigger, void *event);


/*
 * Get interrupt number corresponding to the specified GPIO pin. 
 * This API supports only TLMM interrupts.
 *
 * Parameters:
 * (in)  fd           File descriptor from gpio_open()
 * (in)  gpio         32 bit gpio identifier (GPIO_MODULE_TYPE_16BIT | GPIO_NUMBER_16_BIT)
 *                      See documentation in the beginning of the header.
 * (out) *irq_number  irq number corresponding to the GPIO pin 
 * 
 * Returns:
 *  GPIO_SUCCESS on success 
 *  GPIO_FAIL on failure
 */
int gpio_get_interrupt_cfg (int fd,  uint32_t gpio, uint32_t *irq_number);


/*===========================================================================
DESCRIPTION		:	This function creates the channel on which the interrupt handler thread can wait. 
PARAMETERS		:	fd		- input	- Descriptor from gpio_open (/dev/gpioexp)
					gpio		- input	- GPIO pin number
					chid  	- output  - channel ID.					
RETURN VALUE	:	0 		- on success
					nonzero	- on failure
===========================================================================*/
int gpio_interrupt_attach(int fd,  uint32_t gpio, uint32_t *chid);

/*===========================================================================
DESCRIPTION		:	This function waits for the interrupt to be triggered.
PARAMETERS		:	fd		- input	- Descriptor from gpio_open (/dev/gpioexp)
					gpio		- input	- GPIO pin number
					chid 		- input	- channel ID returned by the gpio_interrupt_attach() function

RETURN VALUE	:	0 		- on success
					nonzero	- on failure
===========================================================================*/
int gpio_interrupt_wait(int fd,  uint32_t *gpio, uint32_t chid );

/*===========================================================================
DESCRIPTION		:	This function will enable/disble the interrupt
PARAMETERS		:	fd		- input	- Descriptor from gpio_open (/dev/gpioexp)
					gpio		- input	- GPIO pin number
 					intr_ctrl	- input	- interrupt control  (clear/diable/enable etc)
RETURN VALUE	:	0 		- on success
					nonzero	- on failure
===========================================================================*/
int gpio_interrupt_control (int fd,  uint32_t gpio, uint32_t intr_ctrl);

/*===========================================================================
DESCRIPTION		:	This reads interrupt pending status of the given gpio
PARAMETERS		:	fd		- input	- Descriptor from gpio_open (/dev/gpioexp)
					gpio		- input	- GPIO pin number
 					intr_pend_status	- output	- 1 if interrupt is pending else 0
RETURN VALUE	:	0 		- on success
					nonzero	- on failure
===========================================================================*/
int gpio_interrupt_get_pending_status (int fd,  uint32_t gpio, uint32_t *intr_pend_status);

#endif /* GPIO_CLIENT_H_ */
