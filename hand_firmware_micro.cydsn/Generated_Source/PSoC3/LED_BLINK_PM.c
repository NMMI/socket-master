/*******************************************************************************
* File Name: LED_BLINK_PM.c
* Version 3.30
*
* Description:
*  This file provides the power management source code to API for the
*  PWM.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "LED_BLINK.h"

static LED_BLINK_backupStruct LED_BLINK_backup;


/*******************************************************************************
* Function Name: LED_BLINK_SaveConfig
********************************************************************************
*
* Summary:
*  Saves the current user configuration of the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  LED_BLINK_backup:  Variables of this global structure are modified to
*  store the values of non retention configuration registers when Sleep() API is
*  called.
*
*******************************************************************************/
void LED_BLINK_SaveConfig(void) 
{

    #if(!LED_BLINK_UsingFixedFunction)
        #if(!LED_BLINK_PWMModeIsCenterAligned)
            LED_BLINK_backup.PWMPeriod = LED_BLINK_ReadPeriod();
        #endif /* (!LED_BLINK_PWMModeIsCenterAligned) */
        LED_BLINK_backup.PWMUdb = LED_BLINK_ReadCounter();
        #if (LED_BLINK_UseStatus)
            LED_BLINK_backup.InterruptMaskValue = LED_BLINK_STATUS_MASK;
        #endif /* (LED_BLINK_UseStatus) */

        #if(LED_BLINK_DeadBandMode == LED_BLINK__B_PWM__DBM_256_CLOCKS || \
            LED_BLINK_DeadBandMode == LED_BLINK__B_PWM__DBM_2_4_CLOCKS)
            LED_BLINK_backup.PWMdeadBandValue = LED_BLINK_ReadDeadTime();
        #endif /*  deadband count is either 2-4 clocks or 256 clocks */

        #if(LED_BLINK_KillModeMinTime)
             LED_BLINK_backup.PWMKillCounterPeriod = LED_BLINK_ReadKillTime();
        #endif /* (LED_BLINK_KillModeMinTime) */

        #if(LED_BLINK_UseControl)
            LED_BLINK_backup.PWMControlRegister = LED_BLINK_ReadControlRegister();
        #endif /* (LED_BLINK_UseControl) */
    #endif  /* (!LED_BLINK_UsingFixedFunction) */
}


/*******************************************************************************
* Function Name: LED_BLINK_RestoreConfig
********************************************************************************
*
* Summary:
*  Restores the current user configuration of the component.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  LED_BLINK_backup:  Variables of this global structure are used to
*  restore the values of non retention registers on wakeup from sleep mode.
*
*******************************************************************************/
void LED_BLINK_RestoreConfig(void) 
{
        #if(!LED_BLINK_UsingFixedFunction)
            #if(!LED_BLINK_PWMModeIsCenterAligned)
                LED_BLINK_WritePeriod(LED_BLINK_backup.PWMPeriod);
            #endif /* (!LED_BLINK_PWMModeIsCenterAligned) */

            LED_BLINK_WriteCounter(LED_BLINK_backup.PWMUdb);

            #if (LED_BLINK_UseStatus)
                LED_BLINK_STATUS_MASK = LED_BLINK_backup.InterruptMaskValue;
            #endif /* (LED_BLINK_UseStatus) */

            #if(LED_BLINK_DeadBandMode == LED_BLINK__B_PWM__DBM_256_CLOCKS || \
                LED_BLINK_DeadBandMode == LED_BLINK__B_PWM__DBM_2_4_CLOCKS)
                LED_BLINK_WriteDeadTime(LED_BLINK_backup.PWMdeadBandValue);
            #endif /* deadband count is either 2-4 clocks or 256 clocks */

            #if(LED_BLINK_KillModeMinTime)
                LED_BLINK_WriteKillTime(LED_BLINK_backup.PWMKillCounterPeriod);
            #endif /* (LED_BLINK_KillModeMinTime) */

            #if(LED_BLINK_UseControl)
                LED_BLINK_WriteControlRegister(LED_BLINK_backup.PWMControlRegister);
            #endif /* (LED_BLINK_UseControl) */
        #endif  /* (!LED_BLINK_UsingFixedFunction) */
    }


/*******************************************************************************
* Function Name: LED_BLINK_Sleep
********************************************************************************
*
* Summary:
*  Disables block's operation and saves the user configuration. Should be called
*  just prior to entering sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  LED_BLINK_backup.PWMEnableState:  Is modified depending on the enable
*  state of the block before entering sleep mode.
*
*******************************************************************************/
void LED_BLINK_Sleep(void) 
{
    #if(LED_BLINK_UseControl)
        if(LED_BLINK_CTRL_ENABLE == (LED_BLINK_CONTROL & LED_BLINK_CTRL_ENABLE))
        {
            /*Component is enabled */
            LED_BLINK_backup.PWMEnableState = 1u;
        }
        else
        {
            /* Component is disabled */
            LED_BLINK_backup.PWMEnableState = 0u;
        }
    #endif /* (LED_BLINK_UseControl) */

    /* Stop component */
    LED_BLINK_Stop();

    /* Save registers configuration */
    LED_BLINK_SaveConfig();
}


/*******************************************************************************
* Function Name: LED_BLINK_Wakeup
********************************************************************************
*
* Summary:
*  Restores and enables the user configuration. Should be called just after
*  awaking from sleep.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Global variables:
*  LED_BLINK_backup.pwmEnable:  Is used to restore the enable state of
*  block on wakeup from sleep mode.
*
*******************************************************************************/
void LED_BLINK_Wakeup(void) 
{
     /* Restore registers values */
    LED_BLINK_RestoreConfig();

    if(LED_BLINK_backup.PWMEnableState != 0u)
    {
        /* Enable component's operation */
        LED_BLINK_Enable();
    } /* Do nothing if component's block was disabled before */

}


/* [] END OF FILE */
