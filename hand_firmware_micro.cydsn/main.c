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
* \file         main.c
*
* \brief        Firmware main file.
* \date         October 01, 2017
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
* \mainpage     Firmware
* \brief        This is the firmware of the SoftHand Pro board.
* \version      6.1.1
*
* \details      This is the firmware of the SoftHand Pro board in Master configuration. 
*				It reads EMG connected to socket and control a motor of an attached SoftHand. 
*				Also can read and convert analog measurements connected to the PSoC microcontroller.                
*
*/


// ----------------------------------------------------------------------------
// This version changes:
//      - not reported


//=================================================================     includes

#include <device.h>
#include <globals.h> // ALL GLOBAL DEFINITIONS, STRUCTURES AND MACROS HERE
#include <interruptions.h>
#include <command_processing.h>
#include <utils.h>

//==============================================================================
//                                                                 MAIN FUNCTION
//==============================================================================

int main()
{
    // Iterator
    uint8 i;         
    
    // Variable declarations for DMA 
    
    uint8 DMA_Chan;
    uint8 DMA_TD[1];

    //================================     initializations - psoc and components

    // EEPROM

    EEPROM_Start();
    memRecall();                                        // Recall configuration.

    // FTDI chip enable

    CyDelay(100);
    FTDI_ENABLE_REG_Write(0x01);
    
    // BOARD LED Enable
    LED_CTRL_Write(1);
    //PWM Blink Enable
    LED_BLINK_EN_Write(0);
    LED_BLINK_Start();
    LED_BLINK_WriteCompare(128);
    
    // RS485

    UART_RS485_Stop();                                  // Stop UART.
    UART_RS485_Start();                                 // Start UART.
    UART_RS485_Init();

    UART_RS485_ClearRxBuffer();
    UART_RS485_ClearTxBuffer();

    ISR_RS485_RX_StartEx(ISR_RS485_RX_ExInterrupt);     // RS485 isr function.
    
    // WATCHDOG
    
    WATCHDOG_COUNTER_Start();
    
    ISR_WATCHDOG_StartEx(ISR_WATCHDOG_Handler);         // WATCHDOG isr function.   

    // PWM

    PWM_MOTORS_Start();
    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);
    MOTOR_DIR_Write(0);
    MOTOR_ON_OFF_Write(0x00);


    // SSI encoder initializations

    COUNTER_ENC_Start();

    SHIFTREG_ENC_1_Start();
    SHIFTREG_ENC_2_Start();
    SHIFTREG_ENC_3_Start();

    // ADC

    ADC_Start();                                        // Start ADC.
    ADC_SOC_Write(0x01);                                // Force first read cycle.
   
    // DMA
    DMA_Chan = DMA_DmaInitialize(DMA_BYTES_PER_BURST, DMA_REQUEST_PER_BURST, HI16(DMA_SRC_BASE), HI16(DMA_DST_BASE));
    DMA_TD[0] = CyDmaTdAllocate();                                                                          // Allocate TD
    CyDmaTdSetConfiguration(DMA_TD[0], 2 * 4, DMA_TD[0], TD_SWAP_EN | DMA__TD_TERMOUT_EN | TD_INC_DST_ADR); // DMA Configurations
    CyDmaTdSetAddress(DMA_TD[0], LO16((uint32)ADC_DEC_SAMP_PTR), LO16((uint32)ADC_buf));                    // Set Register Address
    CyDmaChSetInitialTd(DMA_Chan, DMA_TD[0]);                                                               // Initialize Channel
    
    CyDmaChEnable(DMA_Chan, 1);                                                                             // Enable DMA

    RS485_CTS_Write(0);                                 // Clear To Send on RS485.

    // TIMER

    MY_TIMER_Start();           
    PACER_TIMER_Start();

    CYGlobalIntEnable;                                  // Enable interrupts.

//========================================     initializations - clean variables

    RESET_COUNTERS_Write(0x00);                         // Activate encoder counters.

    CyDelay(10);                                        // Wait for encoders to have a valid value.

    //---------------------------------------------------  Initialize reference structure
    for (i = NUM_OF_MOTORS; i--;) 
        g_ref.pos[i] = 0;


    if (c_mem.emg_calibration_flag) {
        if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
            (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
            (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
            (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV))
            g_ref.onoff = 0x00;
        else
            g_ref.onoff = c_mem.activ;
    } 
    else
        g_ref.onoff = c_mem.activ;
    
    //---------------------------------------------------  Initialize measurement structure
    for (i = NUM_OF_SENSORS; i--;) { 
        g_meas.pos[i] = 0;
        g_meas.rot[i] = 0;
    }

    g_refNew = g_ref;                                   // Initialize k+1 measurements structure.

    g_ref.onoff = c_mem.activ;                          // Initalize Activation.
    
    //---------------------------------------------------  Initialize emg structure
    g_meas.emg[0] = 0;
    g_meas.emg[1] = 0;

    MOTOR_ON_OFF_Write(g_ref.onoff);                    // Activating motors.

    dev_pwm_limit = 0;                                  // Init PWM limit.
    pow_tension = 12000;       //12000 mV (12 V)
    tension_valid = FALSE;                              // Init tension_valid BIT.
    first_tension_valid = TRUE;

    reset_last_value_flag = 0;

    //------------------------------------------------- Initialize package on receive from RS485
    g_rx.length = 0;
    g_rx.ready  = 0;
        
    // enable master_mode by default
    
    master_mode = 1;
    count_tension_valid = 0;
    
    rest_enabled = 1;
    g_mem.rest_position_flag = TRUE;

    //============================================================     main loop

    for(;;)
    {
        // Put the FF reset pin to LOW
        RESET_FF_Write(0x00);

        // Call function scheduler
        function_scheduler();

        //  Wait until the FF is set to 1
        while(FF_STATUS_Read() == 0){
            // On interrupt from RS485
            if (interrupt_flag){
                // Reset WDT
                WATCHDOG_REFRESH_Write(0x01);
                // Reset flags
                interrupt_flag = FALSE;
                watchdog_flag = FALSE;
                // Manage Interrupt on rs485
                interrupt_manager();
            }
            // On interrupt from WDT
            else { 
                if (watchdog_flag){
                    // Reset WDT
                    WATCHDOG_REFRESH_Write(0x01);
                    // Deactivate motors
                    g_refNew.onoff = 0x00;
                }
            }
        };

        // Command a FF reset
        RESET_FF_Write(0x01);

        // Wait for FF to be reset
        while(FF_STATUS_Read() == 1);

        if(UART_RS485_ReadRxStatus() & UART_RS485_RX_STS_SOFT_BUFF_OVER)
            UART_RS485_ClearRxBuffer();
    }
    return 0;
}



/* [] END OF FILE */