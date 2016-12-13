/*******************************************************************************
* File Name: BOARD_LED.c  
* Version 2.10
*
* Description:
*  This file contains API to enable firmware control of a Pins component.
*
* Note:
*
********************************************************************************
* Copyright 2008-2014, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#include "cytypes.h"
#include "BOARD_LED.h"


/*******************************************************************************
* Function Name: BOARD_LED_Write
********************************************************************************
*
* Summary:
*  Assign a new value to the digital port's data output register.  
*
* Parameters:  
*  prtValue:  The value to be assigned to the Digital Port. 
*
* Return: 
*  None 
*  
*******************************************************************************/
void BOARD_LED_Write(uint8 value) 
{
    uint8 staticBits = (BOARD_LED_DR & (uint8)(~BOARD_LED_MASK));
    BOARD_LED_DR = staticBits | ((uint8)(value << BOARD_LED_SHIFT) & BOARD_LED_MASK);
}


/*******************************************************************************
* Function Name: BOARD_LED_SetDriveMode
********************************************************************************
*
* Summary:
*  Change the drive mode on the pins of the port.
* 
* Parameters:  
*  mode:  Change the pins to one of the following drive modes.
*
*  BOARD_LED_DM_STRONG     Strong Drive 
*  BOARD_LED_DM_OD_HI      Open Drain, Drives High 
*  BOARD_LED_DM_OD_LO      Open Drain, Drives Low 
*  BOARD_LED_DM_RES_UP     Resistive Pull Up 
*  BOARD_LED_DM_RES_DWN    Resistive Pull Down 
*  BOARD_LED_DM_RES_UPDWN  Resistive Pull Up/Down 
*  BOARD_LED_DM_DIG_HIZ    High Impedance Digital 
*  BOARD_LED_DM_ALG_HIZ    High Impedance Analog 
*
* Return: 
*  None
*
*******************************************************************************/
void BOARD_LED_SetDriveMode(uint8 mode) 
{
	CyPins_SetPinDriveMode(BOARD_LED_0, mode);
}


/*******************************************************************************
* Function Name: BOARD_LED_Read
********************************************************************************
*
* Summary:
*  Read the current value on the pins of the Digital Port in right justified 
*  form.
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value of the Digital Port as a right justified number
*  
* Note:
*  Macro BOARD_LED_ReadPS calls this function. 
*  
*******************************************************************************/
uint8 BOARD_LED_Read(void) 
{
    return (BOARD_LED_PS & BOARD_LED_MASK) >> BOARD_LED_SHIFT;
}


/*******************************************************************************
* Function Name: BOARD_LED_ReadDataReg
********************************************************************************
*
* Summary:
*  Read the current value assigned to a Digital Port's data output register
*
* Parameters:  
*  None 
*
* Return: 
*  Returns the current value assigned to the Digital Port's data output register
*  
*******************************************************************************/
uint8 BOARD_LED_ReadDataReg(void) 
{
    return (BOARD_LED_DR & BOARD_LED_MASK) >> BOARD_LED_SHIFT;
}


/* If Interrupts Are Enabled for this Pins component */ 
#if defined(BOARD_LED_INTSTAT) 

    /*******************************************************************************
    * Function Name: BOARD_LED_ClearInterrupt
    ********************************************************************************
    *
    * Summary:
    *  Clears any active interrupts attached to port and returns the value of the 
    *  interrupt status register.
    *
    * Parameters:  
    *  None 
    *
    * Return: 
    *  Returns the value of the interrupt status register
    *  
    *******************************************************************************/
    uint8 BOARD_LED_ClearInterrupt(void) 
    {
        return (BOARD_LED_INTSTAT & BOARD_LED_MASK) >> BOARD_LED_SHIFT;
    }

#endif /* If Interrupts Are Enabled for this Pins component */ 


/* [] END OF FILE */
