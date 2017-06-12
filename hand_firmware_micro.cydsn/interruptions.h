// ----------------------------------------------------------------------------
// BSD 3-Clause License

// Copyright (c) 2016, qbrobotics
// Copyright (c) 2017, Centro "E.Piaggio"
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// POSSIBILITY OF SUCH DAMAGE.
// ----------------------------------------------------------------------------

/**
* \file         interruptions.h
*
* \brief        Interruptions header file.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED

//==================================================================     include
#include <device.h>

//=====================================================        Interrupt Handler
 
/** \name Interruptions */
/** \{ */
//====================================================     RS485 interruption
/** This interruption sets a flag to let the firmware know that a communication 
 *	interruption is pending and needs to be handled. The interruption will be
 *	handled in predefined moments during the firmware execution. 
 * 	When this interruption is handled, it unpacks the package received on the 
 *  RS485 communication bus.  
**/
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);

//====================================================     interrupt declaration
/** This interruption sets a flag to let the firmware know that a watchdog
 *	interruption is pending and needs to be handled. The interrpution will be 
 * 	handled in predefined moments during the firmware execution.
 *  When this interruption is handled, it deactivates the board because the 
 *  watchdog timer has expired. 
**/
CY_ISR_PROTO(ISR_WATCHDOG_Handler);

/** \} */


//=====================================================     functions declarations

/** \name General function scheduler */
/** \{ */
//=====================================================     function_scheduler
/** This function schedules the other functions in an order that optimizes the 
 *	controller usage.	
**/
void function_scheduler(void);
/** \} */


/** \name Encoder reading function */
/** \{ */
//=====================================================     encoder_reading
/** This functions reads the value from the encoder pointed by index.
 *
 *	\param index 	The number of the encoder that must be read.
**/
void encoder_reading(const uint8 index);
/** \} */


/** \name Motor control function */
/** \{ */
//=====================================================     motor_control
/** This function controls the motor direction and velocity, depending on 
 * 	the input and control modality set. 
**/
void motor_control();
/** \} */

/** \name Analog readings */
/** \{ */
//=====================================================     analog_read_end
/** This function executes and terminates the analog readings.
**/
void analog_read_end();


/** \name Interrupt manager */
/** \{ */ 
//=====================================================     interrupt_manager
/** This function is called in predefinited moments during firmware execution
 *  in order to unpack the received package. 
**/
void interrupt_manager();
/** \} */

/** \name Utility functions */
/** \{ */
//=====================================================     pwm_limit_search
/** This function scales the pwm value of the motor, depending on the power 
 * 	supply voltage, in order to not make the motor wind too fast.
**/
void pwm_limit_search();

//=====================================================     overcurrent_control
/** This function increases or decreases the pwm value, depending on the current
 * 	absorbed by the motor.
**/
void overcurrent_control();

//=====================================================     command_slave
/** This function is used to create a package and send it to another device, only
 *  if the actual board is in master mode.
**/
void command_slave();

//=====================================================     deactivate_slave
/** This function is used to create a package to deactivate motors on another device, only
 *  if the actual board is in master mode.
**/
void deactivate_slave();

/** \} */

// ----------------------------------------------------------------------------

#endif

/* [] END OF FILE */