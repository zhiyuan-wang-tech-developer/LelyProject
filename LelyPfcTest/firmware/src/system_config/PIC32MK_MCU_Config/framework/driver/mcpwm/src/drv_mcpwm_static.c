/*******************************************************************************
  MCPWM Driver Functions for Static Single Instance Driver

  Company:
    Microchip Technology Inc.

  File Name:
    drv_mcpwm_static.c

  Summary:
   MCPWM driver implementation for the static single instance driver.

  Description:
    The MCPWM device driver provides a simple interface to manage the MCPWM
    modules on Microchip microcontrollers.
    
  Remarks:
    Static interfaces incorporate the driver instance number within the names
    of the routines, eliminating the need for an object ID or object handle.
    Static single-open interfaces also eliminate the need for the open handle.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2017 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
//DOM-IGNORE-END

// *****************************************************************************
// *****************************************************************************
// Header Includes
// *****************************************************************************
// *****************************************************************************
#include "framework/driver/mcpwm/drv_mcpwm_static.h"


// *****************************************************************************
// *****************************************************************************
// Section: MCPWM static driver functions
// *****************************************************************************
// *****************************************************************************

void DRV_MCPWM_Enable (void)
{
/*Enable MCPWM*/

PLIB_MCPWM_Enable (MCPWM_ID_0);

}

void DRV_MCPWM_Disable (void)
{
/*Disable MCPWM*/

PLIB_MCPWM_Disable (MCPWM_ID_0);

}

void DRV_MCPWM_Initialize(void)
{

PLIB_MCPWM_Disable (MCPWM_ID_0);


/* Configure Primary MCPWM Master Timer */
PLIB_MCPWM_PrimaryTimerSetup (MCPWM_ID_0 , MCPWM_CLOCK_DIVIDE_BY_1 , 800);

/* Configure Secondary MCPWM Master Timer */
PLIB_MCPWM_SecondaryTimerSetup (MCPWM_ID_0 , MCPWM_CLOCK_DIVIDE_BY_1 , 800);

/* Configure Primary Special Event Trigger */
PLIB_MCPWM_PrimarySpecialEventTriggerSetup (MCPWM_ID_0 , 0 , MCPWM_TRIGGER_DIVIDE_BY_1);


/* Configure Secondary Special Event Trigger */
PLIB_MCPWM_SecondarySpecialEventTriggerSetup (MCPWM_ID_0 , 0 , MCPWM_TRIGGER_DIVIDE_BY_1);


/* Configure Chop Clock */
PLIB_MCPWM_ChopClockSetup (MCPWM_ID_0 , 2 ,MCPWM_CHOP_CLOCK_DISABLE);
/*Configure PWM Channel */

// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL9 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL9, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL9 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL9 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL9 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL9 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL9, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL9, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL9, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL9);


	
	


	
	

										

// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL2 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL2, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL2 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL2 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL2 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL2 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL2, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL2, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL2, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL2);


	
	


	
	


// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL3 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL3, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL3 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL3 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL3 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL3 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL3, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL3, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL3, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL3);


	
	


	
	


// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL8 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL8, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL8 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL8 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL8 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL8 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL8, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL8, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL8, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL8);


	
	


	
	



// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL7 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL7, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL7 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL7 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL7 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL7 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL7, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL7, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL7, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL7);


	
	


	
	


// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL10 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL10, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL10 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL10 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL10 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL10 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL10, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL10, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL10, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL10);


	
	


	
	



// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL1 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL1, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL1 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL1 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL1 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL1 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL1, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL1, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL1, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL1);


	
	


	
	



// *****************************************************************************
/* MCPWM Channel instance MCPWM_CHANNEL4 */
	PLIB_MCPWM_ChannelSetup (MCPWM_ID_0 , MCPWM_CHANNEL4, MCPWM_TIME_BASE_SOURCE_PRIMARY , MCPWM_TIME_BASE_SYNCHRONIZED , MCPWM_EDGE_ALIGNED,
	MCPWM_OUTPUT_REDUNDANT_MODE, MCPWM_PWMxH_ACTIVEHIGH ,MCPWM_PWMxL_ACTIVEHIGH , MCPWM_DEADTIME_DISABLE, MCPWM_DEADTIME_COMPENSATION_POLARITY_ACTIVE_HIGH );
	
	PLIB_MCPWM_ChannelPrimaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	PLIB_MCPWM_ChannelSecondaryDutyCycleSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	PLIB_MCPWM_ChannelPWMxHDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	PLIB_MCPWM_ChannelPWMxLDeadtimeSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	PLIB_MCPWM_ChannelDeadtimeCompSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	PLIB_MCPWM_ChannelPhaseSet (MCPWM_ID_0 , MCPWM_CHANNEL4 , 0);
	
	
	PLIB_MCPWM_ChannelTriggerSetup (MCPWM_ID_0 , MCPWM_CHANNEL4 , MCPWM_TRIGGER_DIVIDE_BY_1, 
	MCPWM_PRIMARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_SECONDARY_TRIGGER_DURING_TIMER_INCREMENTING_DECREMENTING , MCPWM_ADC_TRIGGER_SOURCE_PRIMARY,
	MCPWM_TRIGGER_INTERRUPT_SOURCE_PRIMARY, 0 , 0);
	

	PLIB_MCPWM_ChannelLEBSetup (MCPWM_ID_0 , MCPWM_CHANNEL4 , MCPWM_FAULT_INPUT_LEB_DISABLE, MCPWM_CURRENTLIMIT_INPUT_LEB_DISABLE, 
	0);	
	



	
	PLIB_MCPWM_ChannelChopSetup (MCPWM_ID_0 , MCPWM_CHANNEL4 , MCPWM_CHOP_CLOCK_SOURCE_IS_CHOP_CLOCK_GENERATOR, MCPWM_PWMxH_CHOP_DISABLED, MCPWM_PWMxL_CHOP_DISABLED);
	
	
	PLIB_MCPWM_ChannelFaultSetup (MCPWM_ID_0 , MCPWM_CHANNEL4, MCPWM_FAULT_SOURCE_IS_FLT15 , MCPWM_FAULT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_FAULT_OVERRIDE_PWMxH_0, MCPWM_FAULT_OVERRIDE_PWMxL_0, MCPWM_FAULT_MODE_DISABLED);
	
	
	PLIB_MCPWM_ChannelCurrentLimitSetup (MCPWM_ID_0 , MCPWM_CHANNEL4, MCPWM_CURRENTLIMIT_SOURCE_IS_FLT15 , MCPWM_CURRENTLIMIT_INPUT_POLARITY_ACTIVE_HIGH,
	MCPWM_CURRENTLIMIT_OVERRIDE_PWMxH_0,MCPWM_CURRENTLIMIT_OVERRIDE_PWMxL_0, MCPWM_CURRENTLIMIT_DISABLE );
	
	
	PLIB_MCPWM_ChannelOverrideOutputSet (MCPWM_ID_0 , MCPWM_CHANNEL4, MCPWM_OVERRIDE_PWMxH_0, MCPWM_OVERRIDE_PWMxL_0);
	
	
	
	PLIB_MCPWM_ChannelPWMxHEnable (MCPWM_ID_0 ,MCPWM_CHANNEL4);


	
	


	
	








}

/*******************************************************************************
 End of File
*/
