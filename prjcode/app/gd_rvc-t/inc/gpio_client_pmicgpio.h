/*===========================================================================

  @file gpio_client_pmicgpio.h
  @brief PMIC GPIO configuration related macros and masks utility header File
  
  @details  PMIC GPIO macros and masks.

  @copyright
         Copyright (c) 2021 QUALCOMM Technologies Incorporated.
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
06/13/16    	aneeketp	Simplify the pmic gpio configurations
05/26/15    	KNR			pmic gpio configuration related macros and masks.
===========================================================================*/


#ifndef GPIO_CLIENT_PMICGPIO_H_
#define GPIO_CLIENT_PMICGPIO_H_
#include "pm_gpio.h"

/* How to use the gpio client to configure pmic gpio ( cfg_mask and cfg)
 * ----------------------------------------------------
 *  See MACRO Definitions to see what 'parameters' are needed to be configured
 *  For INPUT Configuration :
 *                              cfg_mask = PMIC_GPIO_SET_INPUT_CONF_MASK
 *                              cfg = PMIC_GPIO_SET_INPUT_CONF(parameters)
 *
 *  For OUTPUT Configuration :
 *                              cfg_mask = PMIC_GPIO_SET_OUTPUT_CONF_MASK
 *                              cfg = PMIC_GPIO_SET_OUTPUT_CONF(parameters)
 *
 *  For INPUT/OUTPUT(bidirectional) Configuration :
 *                              cfg_mask = PMIC_GPIO_SET_INPUT_OUTPUT_CONF_MASK
 *                              cfg = PMIC_GPIO_SET_INPUT_OUTPUT_CONF(parameters)
 *
 *  user defined Configurations:This needs thorough understanding of the pmic gpio
 *                              datasheet.Should be used only if the above macros 
 *                              cannot be used for the required configuration 
 *                              cfg_mask = OR of the *_TYPE macros in PMIC GPIO Masks
 *                              cfg      = OR of the corresponding values.
 *
 *
 *  For Debugging ONLY parse and print the configuration returned from gpio_get_config
 *  PRINT_PMIC_GPIO_CONFIG
*/
// Values for cfg_mask and cfg for the gpio_client.h apis for pmic gpios
/****************************************************************
 * MASK & CONFIG MACRO PAIR TO CONFIGURE PMIC GPIO AS INPUT PIN *
 ***************************************************************/
#define PMIC_GPIO_SET_INPUT_CONF_MASK  (PMIC_GPIO_MODE_TYPE | \
                                        PMIC_GPIO_VOLTAGE_SRC_TYPE | \
                                        PMIC_GPIO_EXT_PIN_CFG_TYPE)
                                       
                                       
#define PMIC_GPIO_SET_INPUT_CONF(PMIC_GPIO_VOLTAGE_SRC_TYPE_val) \
                                                    (PMIC_GPIO_INPUT_ON | \
                                                     PMIC_GPIO_VOLTAGE_SRC_TYPE_val | \
                                                     PMIC_GPIO_EXT_PIN_ENABLE)
                                                                                                        

/*****************************************************************
 * MASK & CONFIG MACRO PAIR TO CONFIGURE PMIC GPIO AS OUTPUT PIN *
 ****************************************************************/
#define PMIC_GPIO_SET_OUTPUT_CONF_MASK  (PMIC_GPIO_MODE_TYPE | \
                                         PMIC_GPIO_VOLTAGE_SRC_TYPE |	\
                                         PMIC_GPIO_I_SOURCE_PULL_TYPE |  \
                                         PMIC_GPIO_SOURCE_CFG_TYPE |	\
                                         PMIC_GPIO_INVERSION_CONFIG_TYPE | \
                                         PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE | \
                                         PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE | \
                                         PMIC_GPIO_EXT_PIN_ENABLE )

#define PMIC_GPIO_SET_OUTPUT_CONF(PMIC_GPIO_VOLTAGE_SRC_TYPE_val, \
                                PMIC_GPIO_I_SOURCE_PULL_TYPE_val, \
                                PMIC_GPIO_INVERSION_CONFIG_TYPE_val, \
                                PMIC_GPIO_SOURCE_CFG_TYPE_val, \
                                PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE_val, \
                                PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE_val) \
                                                 (PMIC_GPIO_OUTPUT_ON | \
                                                 PMIC_GPIO_VOLTAGE_SRC_TYPE_val |	\
                                                 PMIC_GPIO_I_SOURCE_PULL_TYPE_val |  \
                                                 PMIC_GPIO_INVERSION_CONFIG_TYPE_val | \
                                                 PMIC_GPIO_SOURCE_CFG_TYPE_val |	\
                                                 PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE_val | \
                                                 PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE_val | \
                                                 PMIC_GPIO_EXT_PIN_ENABLE )
	  	  	  	  	  	  	  	  	  	  	  	

/***********************************************************************
 * MASK & CONFIG MACRO PAIR TO CONFIGURE PMIC GPIO AS INPUT/OUTPUT PIN *
 **********************************************************************/
#define PMIC_GPIO_SET_INPUT_OUTPUT_CONF_MASK  (PMIC_GPIO_MODE_TYPE|PMIC_GPIO_VOLTAGE_SRC_TYPE|PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE| \
                                              PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE|PMIC_GPIO_I_SOURCE_PULL_TYPE|PMIC_GPIO_SOURCE_CFG_TYPE| \
                                              PMIC_GPIO_EXT_PIN_CFG_TYPE|PMIC_GPIO_INTERRUPT_POLARITY_TYPE|PMIC_GPIO_INVERSION_CONFIG_TYPE)

#define PMIC_GPIO_SET_INPUT_OUTPUT_CONF(PMIC_GPIO_VOLTAGE_SRC_TYPE_val, \
                                      PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE_val, \
                                      PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE_val, \
                                      PMIC_GPIO_I_SOURCE_PULL_TYPE_val, \
                                      PMIC_GPIO_SOURCE_CFG_TYPE_val, \
                                      PMIC_GPIO_INTERRUPT_POLARITY_TYPE_val) \
                                                    (PMIC_GPIO_INPUT_OUTPUT_ON | \
                                                    PMIC_GPIO_VOLTAGE_SRC_TYPE_val | \
                                                    PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE_val  | \
                                                    PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE_val | \
                                                    PMIC_GPIO_I_SOURCE_PULL_TYPE_val |	\
                                                    PMIC_GPIO_SOURCE_CFG_TYPE_val | \
                                                    PMIC_GPIO_EXT_PIN_ENABLE |	\
                                                    PMIC_GPIO_INTERRUPT_POLARITY_TYPE_val | \
                                                    PMIC_GPIO_INVERSION_CONFIG_DISABLE)							


/***********************************************************************
 * MACRO TO PRINT THE cfg from  gpio_get_config() for pmic gpio*
 **********************************************************************/

#define DEBUG_PRINT(fmt,...) logger_log(QCLOG_AMSS_QNP_QAL_GPIO_CLIENT,__LINE__, QCLOG_DEBUG2, fmt , ##__VA_ARGS__)
#define PRINT_PMIC_GPIO_CONFIG(cfg)  { \
                                      DEBUG_PRINT("PMIC_GPIO_MODE=0x%" PRIx32 " \n" ,GET_PMIC_GPIO_MODE(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_VOLTAGE_SRC=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_VOLTAGE_SRC(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_INTERRUPT_POLARITY=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_INTERRUPT_POLARITY(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_OUT_BUFFER_CONFIG=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_OUT_BUFFER_CONFIG(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_OUT_BUFFER_DRV_STR=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_OUT_BUFFER_DRV_STR(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_I_SOURCE_PULL=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_I_SOURCE_PULL(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_SOURCE_CFG=0x%" PRIx32 "\n",GET_PMIC_GPIO_SOURCE_CFG(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_DTEST_BUF_ONOFF=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_DTEST_BUF_ONOFF(cfg)); \
                                      DEBUG_PRINT("PMIC_GPIO_INVERSION_CONFIG=0x%" PRIx32 "\n" ,GET_PMIC_GPIO_INVERSION_CONFIG(cfg)); \
                                     }

//++++++++++++ MACROS for PMIC GPIOs ++++++++++++++++++++++++++++++++++

/*****************************************************
* pm_gpio_mode_select_type -MASK:PMIC_GPIO_MODE_TYPE *
******************************************************/
//masks
//-----
#define PMIC_GPIO_MODE_TYPE             	             0x0000000F	// mask
#define PMIC_GPIO_MODE_SHIFT             		     0  	// shift

//values
//------
#define PMIC_GPIO_INPUT_ON                           PM_GPIO_DIG_IN        <<PMIC_GPIO_MODE_SHIFT        //0x00000000
#define PMIC_GPIO_OUTPUT_ON                          PM_GPIO_DIG_OUT       <<PMIC_GPIO_MODE_SHIFT        //0x00000001
#define PMIC_GPIO_INPUT_OUTPUT_ON                    PM_GPIO_DIG_IN_DIG_OUT<<PMIC_GPIO_MODE_SHIFT        //0x00000002
#define PMIC_GPIO_ANA_PASS_THRU                      PM_GPIO_ANA_PASS_THRU <<PMIC_GPIO_MODE_SHIFT        //0x00000003

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_MODE(cfg)                      ((cfg<<PMIC_GPIO_MODE_SHIFT)&(PMIC_GPIO_MODE_TYPE))
#define	GET_PMIC_GPIO_MODE(cfg)                      ((cfg&(PMIC_GPIO_MODE_TYPE))>>PMIC_GPIO_MODE_SHIFT)


/*****************************************************
* pm_gpio_voltage_source_type MASK:PMIC_GPIO_VOLTAGE_SRC_TYPE
******************************************************/

//masks
//-----
#define PMIC_GPIO_VOLTAGE_SRC_TYPE      			 0x000000F0 	// mask
#define PMIC_GPIO_VOLTAGE_SRC_SHIFT      			 4 	        // shift


//value
//------
#define	PMIC_GPIO_V0                                 PM_GPIO_VIN0<<PMIC_GPIO_VOLTAGE_SRC_SHIFT     //0x00000000
#define	PMIC_GPIO_V1                                 PM_GPIO_VIN1<<PMIC_GPIO_VOLTAGE_SRC_SHIFT     //0x00000010

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_VOLTAGE_SRC(cfg)               ((cfg<<PMIC_GPIO_VOLTAGE_SRC_SHIFT)&(PMIC_GPIO_VOLTAGE_SRC_TYPE))
#define	GET_PMIC_GPIO_VOLTAGE_SRC(cfg)               ((cfg&(PMIC_GPIO_VOLTAGE_SRC_TYPE))>>PMIC_GPIO_VOLTAGE_SRC_SHIFT)


/*****************************************************
* interrupt_polarity MASK: PMIC_GPIO_INTERRUPT_POLARITY_TYPE
******************************************************/

//masks
//-----
#define PMIC_GPIO_INTERRUPT_POLARITY_TYPE 			 0x00000F00 	// mask
#define PMIC_GPIO_INTERRUPT_POLARITY_SHIFT 			 8 	// shift

//value
//------
#define	PMIC_GPIO_INTERRUPT_POLARITY_NO_INVERT       FALSE<<PMIC_GPIO_INTERRUPT_POLARITY_SHIFT 
#define	PMIC_GPIO_INTERRUPT_POLARITY_INVERT          TRUE<<PMIC_GPIO_INTERRUPT_POLARITY_SHIFT

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_INTERRUPT_POLARITY(cfg)        ((cfg<<PMIC_GPIO_INTERRUPT_POLARITY_SHIFT)&(PMIC_GPIO_INTERRUPT_POLARITY_TYPE))
#define	GET_PMIC_GPIO_INTERRUPT_POLARITY(cfg)        ((cfg&(PMIC_GPIO_INTERRUPT_POLARITY_TYPE))>>PMIC_GPIO_INTERRUPT_POLARITY_SHIFT)

/*****************************************************
* pm_gpio_out_buffer_config_type MASK:PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE
******************************************************/
//masks
//-----
#define PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE      	     0x0000F000 	// mask
#define PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT            12 	// shift


//value
//------
#define	PMIC_GPIO_OUT_BUFFER_CONFIG_CMOS             PM_GPIO_OUT_BUFFER_CONFIG_CMOS<<PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT //0x00000000
#define	PMIC_GPIO_OUT_BUFFER_CONFIG_OPEN_DRAIN_NM0S  PM_GPIO_OUT_BUFFER_CONFIG_OPEN_DRAIN_NMOS<<PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT //0x00001000
#define	PMIC_GPIO_OUT_BUFFER_CONFIG_OPEN_DRAIN_PMOS  PM_GPIO_OUT_BUFFER_CONFIG_OPEN_DRAIN_PMOS<<PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT //0x00002000


//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_OUT_BUFFER_CONFIG(cfg)         ((cfg<<PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT)&(PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE))     
#define	GET_PMIC_GPIO_OUT_BUFFER_CONFIG(cfg)         ((cfg&(PMIC_GPIO_OUT_BUFFER_CONFIG_TYPE))>>PMIC_GPIO_OUT_BUFFER_CONFIG_SHIFT)

/*****************************************************
//pm_gpio_out_buffer_drive_strength_type MASK:PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE
******************************************************/
//masks
//-----
#define PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE  		 0x000F0000 	// mask
#define PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT  		 16 	       // shift

//value
//-----
#define	PMIC_GPIO_OUT_BUFFER_OFF                     PM_GPIO_OUT_BUFFER_RESERVED<<PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT //0x00000000
#define	PMIC_GPIO_OUT_BUFFER_LOW                     PM_GPIO_OUT_BUFFER_LOW<<PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT      //0x00010000
#define	PMIC_GPIO_OUT_BUFFER_MEDIUM                  PM_GPIO_OUT_BUFFER_MEDIUM<<PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT   //0x00020000
#define	PMIC_GPIO_OUT_BUFFER_HIGH                    PM_GPIO_OUT_BUFFER_HIGH<<PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT   //0x00020000

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_OUT_BUFFER_DRV_STR(cfg)        ((cfg<<PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT)&(PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE))
#define	GET_PMIC_GPIO_OUT_BUFFER_DRV_STR(cfg)        ((cfg&(PMIC_GPIO_OUT_BUFFER_DRV_STR_TYPE))>>PMIC_GPIO_OUT_BUFFER_DRV_STR_SHIFT)

/********************************************************************
//pm_gpio_current_source_pulls_type MASK:PMIC_GPIO_I_SOURCE_PULL_TYPE
*******************************************************************/
//masks
//-----
#define PMIC_GPIO_I_SOURCE_PULL_TYPE    			 0x00F00000 	// mask
#define PMIC_GPIO_I_SOURCE_PULL_SHIFT    			 20 	// shift

//value
//-----
#define	PMIC_GPIO_I_SOURCE_PULL_UP_30uA              PM_GPIO_I_SRC_PULL_UP_30uA<<PMIC_GPIO_I_SOURCE_PULL_SHIFT     //0x00000000
#define	PMIC_GPIO_I_SOURCE_PULL_UP_1_5uA             PM_GPIO_I_SRC_PULL_UP_1_5uA<<PMIC_GPIO_I_SOURCE_PULL_SHIFT  //0x00100000
#define	PMIC_GPIO_I_SOURCE_PULL_UP_31_5uA            PM_GPIO_I_SRC_PULL_UP_31_5uA<<PMIC_GPIO_I_SOURCE_PULL_SHIFT //0x00200000
#define	PMIC_GPIO_I_SOURCE_PULL_UP_1_5uA_PLUS_30uA_BOOST PM_GPIO_I_SRC_PULL_UP_1_5uA_PLUS_30uA_BOOST<<PMIC_GPIO_I_SOURCE_PULL_SHIFT //0x00300000
#define	PMIC_GPIO_I_SOURCE_PULL_DOWN_10uA            PM_GPIO_I_SRC_PULL_DOWN_10uA<<PMIC_GPIO_I_SOURCE_PULL_SHIFT //0x00400000
#define	PMIC_GPIO_I_SOURCE_PULL_NO_PULL              PM_GPIO_I_SRC_PULL_NO_PULL<<PMIC_GPIO_I_SOURCE_PULL_SHIFT //0x00500000

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_I_SOURCE_PULL(cfg)             ((cfg<<PMIC_GPIO_I_SOURCE_PULL_SHIFT)&(PMIC_GPIO_I_SOURCE_PULL_TYPE))
#define	GET_PMIC_GPIO_I_SOURCE_PULL(cfg)             ((cfg&(PMIC_GPIO_I_SOURCE_PULL_TYPE))>>PMIC_GPIO_I_SOURCE_PULL_SHIFT)

/********************************************************************
//pm_gpio_source_config_type MASK:PMIC_GPIO_SOURCE_CFG_TYPE
*******************************************************************/
//masks
//-----
#define PMIC_GPIO_SOURCE_CFG_TYPE                    0x0F000000 	// mask
#define PMIC_GPIO_SOURCE_CFG_SHIFT                   24 	// shift

//value
//-----
#define	PMIC_GPIO_SOURCE_GND                         PM_GPIO_SOURCE_GND<<PMIC_GPIO_SOURCE_CFG_SHIFT                //0x00000000
#define	PMIC_GPIO_SOURCE_PAIRED_GPIO                 PM_GPIO_SOURCE_PAIRED_GPIO<<PMIC_GPIO_SOURCE_CFG_SHIFT,       //0x01000000
#define	PMIC_GPIO_SOURCE_SPECIAL_FUNCTION1           PM_GPIO_SOURCE_SPECIAL_FUNCTION1<<PMIC_GPIO_SOURCE_CFG_SHIFT  //0x02000000
#define	PMIC_GPIO_SOURCE_SPECIAL_FUNCTION2           PM_GPIO_SOURCE_SPECIAL_FUNCTION2<<PMIC_GPIO_SOURCE_CFG_SHIFT  //0x03000000
#define	PMIC_GPIO_SOURCE_DTEST1                      PM_GPIO_SOURCE_DTEST1<<PMIC_GPIO_SOURCE_CFG_SHIFT             //0x04000000
#define	PMIC_GPIO_SOURCE_DTEST2                      PM_GPIO_SOURCE_DTEST2<<PMIC_GPIO_SOURCE_CFG_SHIFT             //0x05000000
#define	PMIC_GPIO_SOURCE_DTEST3                      PM_GPIO_SOURCE_DTEST3<<PMIC_GPIO_SOURCE_CFG_SHIFT             //0x06000000
#define	PMIC_GPIO_SOURCE_DTEST4                      PM_GPIO_SOURCE_DTEST4<<PMIC_GPIO_SOURCE_CFG_SHIFT             //0x07000000

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_SOURCE_CFG(cfg)                ((cfg<<PMIC_GPIO_SOURCE_CFG_SHIFT)&(PMIC_GPIO_SOURCE_CFG_TYPE))
#define	GET_PMIC_GPIO_SOURCE_CFG(cfg)                ((cfg&(PMIC_GPIO_SOURCE_CFG_TYPE))>>PMIC_GPIO_SOURCE_CFG_SHIFT)

/***************************************
//pm_gpio_dtest_buffer_OnOff_type
***************************************/
//mask
//----
#define	PMIC_GPIO_DTEST_BUF_ONOFF_TYPE               0x10000000	// mask
#define	PMIC_GPIO_DTEST_BUF_ONOFF_SHIFT              28	// shift

//value
//-----
#define	PMIC_GPIO_DTEST_DISABLE                      PM_GPIO_DTEST_DISABLE<<PMIC_GPIO_DTEST_BUF_ONOFF_SHIFT  //0x00000000
#define	PMIC_GPIO_DTEST_ENABLE                       PM_GPIO_DTEST_ENABLE<<PMIC_GPIO_DTEST_BUF_ONOFF_SHIFT   //0x10000000

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_DTEST_BUF_ONOFF(cfg)           ((cfg<<PMIC_GPIO_DTEST_BUF_ONOFF_SHIFT)&(PMIC_GPIO_DTEST_BUF_ONOFF_TYPE))
#define	GET_PMIC_GPIO_DTEST_BUF_ONOFF(cfg)           ((cfg&(PMIC_GPIO_DTEST_BUF_ONOFF_TYPE))>>PMIC_GPIO_DTEST_BUF_ONOFF_SHIFT)

/************************************************
//pm_gpio_ext_pin_config_type
*************************************************/
//mask
//----
#define PMIC_GPIO_EXT_PIN_CFG_TYPE      			 0x40000000 	// mask
#define PMIC_GPIO_EXT_PIN_CFG_SHIFT      			 30 	// shift

//value
#define	PMIC_GPIO_EXT_PIN_ENABLE                     TRUE<<PMIC_GPIO_EXT_PIN_CFG_SHIFT  // 0x40000000
#define	PMIC_GPIO_EXT_PIN_DISABLE                    FALSE<<PMIC_GPIO_EXT_PIN_CFG_SHIFT   // 0x00000000

//Get/Set Macros
//--------------
#define	SET_PMIC_GPIO_EXT_PIN_CFG(cfg)               ((cfg<<PMIC_GPIO_EXT_PIN_CFG_SHIFT)&(PMIC_GPIO_EXT_PIN_CFG_TYPE))
#define	GET_PMIC_GPIO_EXT_PIN_CFG(cfg)               ((cfg&(PMIC_GPIO_EXT_PIN_CFG_TYPE))>>PMIC_GPIO_EXT_PIN_CFG_SHIFT)

/***********************************************
//gpio inversion config boolean 
************************************************/
//mask
//----
#define PMIC_GPIO_INVERSION_CONFIG_TYPE 			 0x80000000 	// mask
#define PMIC_GPIO_INVERSION_CONFIG_SHIFT 			 31 	// shift

//value
#define	PMIC_GPIO_INVERSION_CONFIG_DISABLE           PM_GPIO_EXT_PIN_DISABLE<<PMIC_GPIO_INVERSION_CONFIG_SHIFT  // 0x00000000
#define	PMIC_GPIO_INVERSION_CONFIG_ENABLE            PM_GPIO_EXT_PIN_ENABLE<<PMIC_GPIO_INVERSION_CONFIG_SHIFT   // 0x80000000

//Get/Set Macros
#define	SET_PMIC_GPIO_INVERSION_CONFIG(cfg)          ((cfg<<PMIC_GPIO_INVERSION_CONFIG_SHIFT)&(PMIC_GPIO_INVERSION_CONFIG_TYPE))
#define	GET_PMIC_GPIO_INVERSION_CONFIG(cfg)          ((cfg&(PMIC_GPIO_INVERSION_CONFIG_TYPE))>>PMIC_GPIO_INVERSION_CONFIG_SHIFT)


#endif /* GPIO_CLIENT_PMICGPIO_H_ */
