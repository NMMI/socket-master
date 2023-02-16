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
* \file         interruptions.c
*
* \brief        Interruption handling and firmware core functions
* \date         Feb 14th, 2023
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2023 Centro "E.Piaggio". All rights reserved.
*/


//=================================================================     includes
#include <interruptions.h>
#include <command_processing.h>
#include <globals.h>
#include <utils.h>


//==============================================================================
//                                                            RS485 RX INTERRUPT
//==============================================================================
// Processing RS-485 data frame:
//
// - 0:     Waits for beginning characters
// - 1:     Waits for ID;
// - 2:     Data length;
// - 3:     Receive all bytes;
// - 4:     Wait for another device end of transmission;
//
//==============================================================================

CY_ISR(ISR_RS485_RX_ExInterrupt) {

    // Set RS485 flag
    
    interrupt_flag = TRUE;
     
}

//==============================================================================
//                                                             INTERRUPT MANAGER
//==============================================================================
// Manage interrupt from RS485 
//==============================================================================
// Processing RS-485 data frame:
//
// - WAIT_START:    Waits for beginning characters;
// - WAIT_ID:       Waits for ID;
// - WAIT_LENGTH:   Data length;
// - RECEIVE:       Receive all bytes;
// - UNLOAD:        Wait for another device end of transmission;
//
//==============================================================================

void interrupt_manager(){

    
    //===========================================     local variables definition

    static uint8 CYDATA state = WAIT_START;                      // state
    
    //------------------------------------------------- local data packet
    static uint8 CYDATA data_packet_index;
    static uint8 CYDATA data_packet_length;
    static uint8 data_packet_buffer[128];                     
    static uint8 CYDATA rx_queue[3];                    // last 2 bytes received
    //-------------------------------------------------

    uint8 CYDATA    rx_data;                            // RS485 UART rx data
    CYBIT           rx_data_type;                       // my id?
    uint8 CYDATA    package_count = 0;                     

    //======================================================     receive routine
    
    // Get data until buffer is not empty 
    
    while(UART_RS485_GetRxBufferSize() && (package_count < 6)){  
        // 6 - estimated maximum number of packets we can read without blocking firmware execution
        
        // Get next char
        rx_data = UART_RS485_GetChar();
        
        switch (state) {
            //-----     wait for frame start     -------------------------------
            case WAIT_START:
            
                rx_queue[0] = rx_queue[1];
                rx_queue[1] = rx_queue[2];
                rx_queue[2] = rx_data;
                
                // Check for header configuration package
                if ((rx_queue[1] == 58) && (rx_queue[2] == 58)) {
                    rx_queue[0] = 0;
                    rx_queue[1] = 0;
                    rx_queue[2] = 0;
                    state       = WAIT_ID;                    
                }else
                    if ((rx_queue[0] == 63) &&      //ASCII - ?
                        (rx_queue[1] == 13) &&      //ASCII - CR
                        (rx_queue[2] == 10))        //ASCII - LF)
                        infoGet(INFO_ALL);
                break;

            //-----     wait for id     ----------------------------------------
            case  WAIT_ID:

                // packet is for my ID or is broadcast
                if (rx_data == c_mem.id || rx_data == 0 || (c_mem.is_myo2_master && master_mode && rx_data == c_mem.SH_ID))
                    rx_data_type = FALSE;
                else                //packet is for others
                    rx_data_type = TRUE;
                
                data_packet_length = 0;
                state = WAIT_LENGTH;
                break;

            //-----     wait for length     ------------------------------------
            case  WAIT_LENGTH:

 
                data_packet_length = rx_data;
                // check validity of pack length
                if (data_packet_length <= 1) {
                    data_packet_length = 0;
                    state = WAIT_START;
                } else if (data_packet_length > 128) {
                    data_packet_length = 0;
                    state = WAIT_START;
                } else {
                    data_packet_index = 0;
                    
                    if(rx_data_type == FALSE)
                        state = RECEIVE;          // packet for me or broadcast
                    else
                        state = UNLOAD;           // packet for others
                }
                break;

            //-----     receiving     -------------------------------------------
            case RECEIVE:

                data_packet_buffer[data_packet_index] = rx_data;
                data_packet_index++;
                
                // check end of transmission
                if (data_packet_index >= data_packet_length) {
                    // verify if frame ID corresponded to the device ID
                    if (rx_data_type == FALSE) {
                        // copying data from buffer to global packet
                        memcpy(g_rx.buffer, data_packet_buffer, data_packet_length);
                        g_rx.length = data_packet_length;
                        g_rx.ready  = 1;
                
                        commProcess();
                    }
                    
                    data_packet_index  = 0;
                    data_packet_length = 0;
                    state = WAIT_START;
                    package_count++;
                
                }
                break;

            //-----     other device is receving     ---------------------------
            case UNLOAD:
                if (!(--data_packet_length)) {
                    data_packet_index  = 0;
                    data_packet_length = 0;
                    RS485_CTS_Write(1);
                    RS485_CTS_Write(0);
                    state              = WAIT_START;
                    package_count++;
                }
                break;
        }
    }
}
//==============================================================================
//                                                            FUNCTION SCHEDULER
//==============================================================================
// Call all the function with the right frequency
//==============================================================================
// Base frequency 1000 Hz
//==============================================================================

void function_scheduler(void) {
 
    static uint16 counter_calibration = DIV_INIT_VALUE;

    timer_value0 = (uint32)MY_TIMER_ReadCounter();

    // Start ADC Conversion, SOC = 1

    ADC_SOC_Write(0x01); 
    
    // Check Interrupt 

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //---------------------------------- Control Motor
   
    motor_control();

    // Check Interrupt 

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //---------------------------------- Read conversion buffer - LOCK function

    analog_read_end();

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
    
    //---------------------------------- Master Mode
      
    if(master_mode){
        
        if (c_mem.is_myo2_master){
            
            // Rest position check (only if Master mode is enabled)
            if (c_mem.rest_position_flag == TRUE){
                if (counter_calibration == CALIBRATION_DIV) {
                    
                        // Read Measure (valid since this routine is enabled only in Master mode)
                        if (c_mem.is_force_fb_present || c_mem.is_proprio_fb_present){
                            curr_pos_res = SH_current_position;
                        } 
                        else {
                            curr_pos_res = (int32)commReadMeasFromSH();
                        }
                            

                        check_rest_position();
                        counter_calibration = 0;
                }
                counter_calibration++;

                // Check Interrupt     
                if (interrupt_flag){
                    interrupt_flag = FALSE;
                    interrupt_manager();
                }
            }
            
            drive_SH();  
            
            // Check Interrupt     
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
        }
                
        if (c_mem.is_force_fb_present){
            
            if (c_mem.is_proprio_fb_present && c_mem.ForceF_ID == c_mem.ProprioF_ID){
                drive_force_proprio_fb();
            }
            else {
                drive_force_fb(); 
            }
            
            // Check Interrupt     
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            
        }
        
        if (c_mem.is_proprio_fb_present){
            
            if (!(c_mem.is_force_fb_present && c_mem.ForceF_ID == c_mem.ProprioF_ID)) {
                drive_proprio_fb();
            }
            
            // Check Interrupt     
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
        }
        
        if (c_mem.is_vibrotactile_fb_present){
            
            drive_vibrotactile_fb();
            
            
            // Check Interrupt     
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
        }
    }
            
    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    //---------------------------------- Calibration 

    // Divider 10, freq = 500 Hz
    if (calib.enabled == TRUE) {
        if (counter_calibration == CALIBRATION_DIV) {
            calibration();
            counter_calibration = 0;
        }
        counter_calibration++;
    }

    // Check Interrupt 
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
    
    
    //---------------------------------- Update States
    
    // Load k-1 state
    memcpy( &g_measOld, &g_meas, sizeof(g_meas) );
    memcpy( &g_refOld, &g_ref, sizeof(g_ref) );

    // Load k+1 state
    memcpy( &g_ref, &g_refNew, sizeof(g_ref) );

    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }

    timer_value = (uint32)MY_TIMER_ReadCounter();
    MY_TIMER_WriteCounter(5000000);

}

//==============================================================================
//                                                                MOTORS CONTROL
//==============================================================================

void motor_control() {

    int32 CYDATA handle_value;

    int32 CYDATA err_emg_1, err_emg_2;
    float CYDATA joy_perc;

    // Static Variables
    static uint8 current_emg = 0;   // 0 NONE
                                    // 1 EMG 1
                                    // 2 EMG 2
                                    // wait for both to get down

    err_emg_1 = g_meas.emg[0] - c_mem.emg_threshold[0];
    err_emg_2 = g_meas.emg[1] - c_mem.emg_threshold[1];
    
    // =========================== POSITION INPUT ==============================
    switch(c_mem.input_mode) {

        case INPUT_MODE_ENCODER3:

            if (master_mode) {
                // The only attached encoder is the handle lever
                handle_value = g_meas.pos[0] * c_mem.motor_handle_ratio;
            }
            else {
                // Like other firmwares working
                // Calculate handle value based on position of handle in the
                // sensor chain and multiplication factor between handle and motor position
                handle_value = 0;
            }

            // Read handle and use it as reference for the motor
            if (((handle_value - g_ref.pos[0]) > c_mem.max_step_pos) && (c_mem.max_step_pos != 0))
                g_ref.pos[0] += c_mem.max_step_pos;
            else {
                if (((handle_value - g_ref.pos[0]) < c_mem.max_step_neg) && (c_mem.max_step_neg != 0))
                    g_ref.pos[0] += c_mem.max_step_neg;
                else
                    g_ref.pos[0] = handle_value;
            }
            break;

        case INPUT_MODE_EMG_PROPORTIONAL:
            if (err_emg_1 > 0)
                g_ref.pos[0] = (err_emg_1 * g_mem.pos_lim_sup[0]) / (1024 - c_mem.emg_threshold[0]);
            else
                g_ref.pos[0] = 0;
            break;

        case INPUT_MODE_EMG_INTEGRAL:
            g_ref.pos[0] = g_refOld.pos[0];
            if (err_emg_1 > 0) {
                g_ref.pos[0] = g_refOld.pos[0] + (err_emg_1 * (int)g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[0]);
            }
            if (err_emg_2 > 0) {
                g_ref.pos[0] = g_refOld.pos[0] - (err_emg_2 * (int)g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[1]);
            }
            break;

        case INPUT_MODE_EMG_FCFS:
            g_ref.pos[0] = g_refOld.pos[0];
            switch (current_emg) {
                case 0:
                    // Look for the first EMG passing the threshold
                    if (err_emg_1 > 0) {
                        current_emg = 1;
                        break;
                    }
                    if (err_emg_2 > 0) {
                        current_emg = 2;
                        break;
                    }
                    break;

                case 1:
                    // EMG 1 is first
                    if (err_emg_1 < 0) {
                        current_emg = 0;
                        break;
                    }
                    g_ref.pos[0] = g_refOld.pos[0] + (err_emg_1 * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[0]);
                    break;

                case 2:
                    // EMG 2 is first
                    if (err_emg_2 < 0) {
                        current_emg = 0;
                        break;
                    }
                    g_ref.pos[0] = g_refOld.pos[0] - (err_emg_2 * g_mem.emg_speed * 2) / (1024 - c_mem.emg_threshold[1]);
                    break;

                default:
                    break;
            }

            break;

        case INPUT_MODE_EMG_FCFS_ADV:
            g_ref.pos[0] = g_refOld.pos[0];
            switch (current_emg) {
                // Look for the first EMG passing the threshold
                case 0:
                    if (err_emg_1 > 0) {
                        current_emg = 1;
                        break;
                    }
                    if (err_emg_2 > 0) {
                        current_emg = 2;
                        break;
                    }
                    break;

                // EMG 1 is first
                case 1:
                    // If both signals are under threshold go back to status 0
                    if ((err_emg_1 < 0) && (err_emg_2 < 0)) {
                        current_emg = 0;
                        break;
                    }
                    // but if the current signal come back over threshold, continue using it
                    if (err_emg_1 > 0) 
                        g_ref.pos[0] = g_refOld.pos[0] + (err_emg_1 * g_mem.emg_speed << 2) / (1024 - c_mem.emg_threshold[0]);
                    
                    break;

                // EMG 2 is first
                case 2:
                    // If both signals are under threshold go back to status 0
                    if ((err_emg_1 < 0) && (err_emg_2 < 0)) {
                        current_emg = 0;
                        break;
                    }
                    // but if the current signal come back over threshold, continue using it
                    if (err_emg_2 > 0) {
                        g_ref.pos[0] = g_refOld.pos[0] - (err_emg_2 * c_mem.emg_speed << 2) / (1024 - c_mem.emg_threshold[1]);
                    }
                    break;

                default:
                    break;
            }
  
            break;
            
        case INPUT_MODE_JOYSTICK:
            
            joy_perc = (float)(g_meas.joystick[0])/((float)c_mem.joystick_gain/2);            
            g_ref.pos[0] = g_refOld.pos[0] + c_mem.joystick_closure_speed * joy_perc;         
        
            break;
        default:
            break;
    }

    // Position limit saturation
    if (c_mem.pos_lim_flag) {
        if (g_ref.pos[0] < c_mem.pos_lim_inf[0]) 
            g_ref.pos[0] = c_mem.pos_lim_inf[0];
        if (g_ref.pos[0] > c_mem.pos_lim_sup[0]) 
            g_ref.pos[0] = c_mem.pos_lim_sup[0];
    }

    if (c_mem.rest_position_flag == TRUE) {
        if (rest_enabled == 1){
            // Change position reference to drive motor to rest position smoothly
            g_ref.pos[0] = rest_pos_curr_ref;
        }
        
        if (forced_open == 1) {
            // Open the SoftHand as EMG PROPORTIONAL input mode 
            if (err_emg_2 > 0)
                g_ref.pos[0] = g_mem.rest_pos - (err_emg_2 * g_mem.rest_pos) / (1024 - c_mem.emg_threshold[1]);
            else {
                g_ref.pos[0] = g_mem.rest_pos;
                forced_open = 0;
            }
        }
    }
}

//==============================================================================
//                                                           ANALOG MEASUREMENTS
//==============================================================================

void analog_read_end() {

    /* =========================================================================
    /   Ideal formulation to calculate tension and current
    /
    /   tension = ((read_value_mV - offset) * 101) / gain -> [mV]
    /   current = ((read_value_mV - offset) * 375) / (gain * resistor) -> [mA]
    /
    /   Definition:
    /   read_value_mV = counts_read / 0.819 -> conversion from counts to mV
    /   offset = 2000 -> hardware amplifier bias in mV
    /   gain = 8.086 -> amplifier gain
    /   resistor = 18 -> resistor of voltage divider in KOhm 
    /
    /   Real formulation: tradeoff in good performance and accurancy, ADC_buf[] 
    /   and offset unit of measurment is counts, instead dev_tension and
    /   g_meas.curr[] are converted in properly unit.
    /  =========================================================================
    */

    int32 CYDATA i_aux;

    static emg_status CYDATA emg_1_status = RESET; 
    static emg_status CYDATA emg_2_status = RESET;                                             
    
    static joystick_status CYDATA UD_status = RESET;
    static joystick_status CYDATA LR_status = RESET;                                          

    static uint16 CYDATA UD_counter = 0;
    static uint16 CYDATA LR_counter = 0;
    
    static int32 CYDATA UD_mean_value;
    static int32 CYDATA LR_mean_value;
	
    static uint16 emg_counter_1 = 0;
    static uint16 emg_counter_2 = 0;
    static uint8 count = 0;
    static uint32 count2 = 0;
    
  //  static uint8 first_tension_valid = TRUE;

    // Wait for conversion end
    
    while(!ADC_STATUS_Read()){
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }
    }
    
    // Convert tension read
    dev_tension = ((int32)(ADC_buf[0] - 1638) * 1952) >> 7;
    
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
        
    if (first_tension_valid && tension_valid) {
        count = count + 1;
        
        if (count == 1000){
            if (dev_tension < 9000) {   // 8 V case
                pow_tension = 8000;
            }
            else {      // 12 V - 24 V cases
                if (dev_tension < 13000) {
                    pow_tension = 12000;
                }
                else
                    pow_tension = 24000;
            }

            first_tension_valid = FALSE;
        }
    }

    // Until there is no valid input tension repeat this measurement
    
    if (dev_tension > 7000) {       //at least > 7.36 V (92% of 8 V) that is minimum charge of smallest battery
        // Set tension valid bit to TRUE

        if (count2 == 1000){
            tension_valid = TRUE;   
        }
        else {                      // wait for battery voltage stabilization
            count2 = count2 + 1;
            dev_tension_f = filter_voltage(dev_tension);
        }

        // Check Interrupt 
    
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }

        // if calibration is not needed go to "normal execution"
        if (!g_mem.emg_calibration_flag){
            emg_1_status = NORMAL; // normal execution
            emg_2_status = NORMAL; // normal execution
        }

        // State machines for EMG or Joystick reading
        if (c_mem.input_mode == INPUT_MODE_JOYSTICK) {
            // Read ADC_buf[2] and ADC_buf[3] as JOYSTICK analog inputs
            
            // Joystick UD    
            switch (UD_status) {
                case NORMAL:
                    i_aux = ADC_buf[2];
                    // Remap the analog reading from -1024 to 1024.  
                    i_aux = (int32) (((float) (i_aux - UD_mean_value) / UD_mean_value) * g_mem.joystick_gain); 
                    
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }

                    //Saturation
                    if (i_aux < -1024) 
                        i_aux = -1024;
                    if (i_aux > 1024)
                        i_aux = 1024;

                    g_meas.joystick[1] = (int16) i_aux;

                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }

                break;

                case RESET: // reset variables
                    UD_counter = 0;
                    UD_mean_value = 0;
                    UD_status = WAIT; // go to waiting status

                break;

                case DISCARD: // discard first EMG_SAMPLE_TO_DISCARD samples
                    UD_counter++;
                    if (UD_counter == JOYSTICK_SAMPLE_TO_DISCARD) {
                        UD_counter = 0;                     // reset counter

                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }

                        UD_status = SUM_AND_MEAN;           // sum and mean status
                    }

                break;

                case SUM_AND_MEAN: // sum first SAMPLES_FOR_EMG_MEAN samples
                    // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                    UD_counter++;
                    UD_mean_value += ADC_buf[2];

                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }

                    if (UD_counter == SAMPLES_FOR_JOYSTICK_MEAN) {
                        UD_mean_value = UD_mean_value / SAMPLES_FOR_JOYSTICK_MEAN; // calc mean

                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }

                        UD_counter = 0;          // reset counter
                        UD_status = NORMAL;           // goto normal execution
                    }
                break;

                case WAIT: // wait for both EMG calibrations to be done
                    if (emg_1_status == NORMAL && emg_2_status == NORMAL)
                        UD_status = DISCARD;           // goto discard sample
                break;
            }
            
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            // Joystick LR
            switch (LR_status) {
                case NORMAL:
                    i_aux = ADC_buf[3];

                    i_aux = (int32) (((float) (i_aux - LR_mean_value) / LR_mean_value) * g_mem.joystick_gain); 
                    
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }

                    //Saturation
                    if (i_aux < -1024)
                        i_aux = -1024;
                    if (i_aux > 1024)
                        i_aux = 1024;

                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }

                    g_meas.joystick[0] = (int16) i_aux;

                break;

                case RESET: // reset variables
                    LR_counter = 0;
                    LR_mean_value = 0;
                    LR_status = WAIT; // goes waiting for all conversions to be done
                break;

                case DISCARD: // discard first EMG_SAMPLE_TO_DISCARD samples8
                    LR_counter++;
                    if (LR_counter == JOYSTICK_SAMPLE_TO_DISCARD) {
                        LR_counter = 0;                     // reset counter

                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }

                        LR_status = SUM_AND_MEAN;           // sum and mean status
                    }
                break;

                case SUM_AND_MEAN: // sum first SAMPLES_FOR_EMG_MEAN samples
                    // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                    LR_counter++;
                    LR_mean_value += ADC_buf[3];
                    if (LR_counter == SAMPLES_FOR_JOYSTICK_MEAN) {
                        LR_mean_value = LR_mean_value / SAMPLES_FOR_JOYSTICK_MEAN; // calc mean
                        
                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }

                        LR_counter = 0;               // reset counter
                        LR_status = NORMAL;           // goto normal execution
                    }
                break;

                case WAIT:
                    if(emg_1_status == NORMAL && emg_2_status == NORMAL && UD_status == NORMAL)
                        LR_status = DISCARD;
                break;
            }

            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            
        } 
        else {
            // Read ADC_buf[2] and ADC_buf[3] as EMGs
            
            // EMG 1 calibration state machine
        
            // Calibration state machine
            switch(emg_1_status) {
                case NORMAL: // normal execution
                    i_aux = (int32)ADC_buf[2];
                    if (i_aux < 0) 
                        i_aux = 0;
                    i_aux = filter_ch1(i_aux);
                    i_aux = (i_aux << 10) / g_mem.emg_max_value[0];
        
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    //Saturation
                    if (i_aux < 0)
                        i_aux = 0;
                    else
                        if (i_aux > 1024) 
                            i_aux = 1024;
                    
                    g_meas.emg[0] = i_aux;
        
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    
                    break;

                case RESET: // reset variables
                    emg_counter_1 = 0;
                    g_mem.emg_max_value[0] = 0;
                    emg_1_status = DISCARD; // goto next status
                    
                    break;

                case DISCARD: // discard first EMG_SAMPLE_TO_DISCARD samples
                    emg_counter_1++;
                    if (emg_counter_1 == EMG_SAMPLE_TO_DISCARD) {
                        emg_counter_1 = 0;          // reset counter
                        
                        LED_CTRL_Write(1);
                        LED_BLINK_EN_Write(0);
                            
                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }
                        
                        emg_1_status = SUM_AND_MEAN;           // sum and mean status
                    }
                    break;

                case SUM_AND_MEAN: // sum first SAMPLES_FOR_EMG_MEAN samples
                    // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                    emg_counter_1++;
                    if (ADC_buf[2] < 0) 
                        ADC_buf[2] = 0;
                    g_mem.emg_max_value[0] += filter_ch1((int32)ADC_buf[2]);
                    
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    
                    if (emg_counter_1 == SAMPLES_FOR_EMG_MEAN) {
                        g_mem.emg_max_value[0] = g_mem.emg_max_value[0] / SAMPLES_FOR_EMG_MEAN; // calc mean
        
                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }                    
                        
                        LED_CTRL_Write(0);
                        LED_BLINK_EN_Write(0);
                        
                        emg_counter_1 = 0;          // reset counter

                        emg_1_status = NORMAL;           // goto normal execution
                    }
                    break;

                default:
                    break;
            }
        
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
            // EMG 2 calibration state machine
            switch(emg_2_status) {
                case NORMAL: // normal execution
                    i_aux = (int32)ADC_buf[3];
                    if (i_aux < 0)
                        i_aux = 0;
                    i_aux = filter_ch2(i_aux);
                    i_aux = (i_aux << 10) / g_mem.emg_max_value[1];
        
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    
                    if (i_aux < 0) 
                        i_aux = 0;
                    else
                        if (i_aux > 1024)
                            i_aux = 1024;
                    
                    g_meas.emg[1] = i_aux;

                    break;

                case RESET: // reset variables
                    emg_counter_2 = 0;
                    g_mem.emg_max_value[1] = 0;
        
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    
                    emg_2_status = WAIT; // wait for EMG 1 calibration
                    break;

                case DISCARD: // discard first EMG_SAMPLE_TO_DISCARD samples
                    emg_counter_2++;
                    if (emg_counter_2 == EMG_SAMPLE_TO_DISCARD) {
                        emg_counter_2 = 0;          // reset counter
                        
                        LED_CTRL_Write(1);
                        LED_BLINK_EN_Write(0);
        
                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }
                        
                        emg_2_status = SUM_AND_MEAN;           // sum and mean status
                    }
                    break;

                case SUM_AND_MEAN: // sum first SAMPLES_FOR_EMG_MEAN samples
                    // NOTE max(value)*SAMPLES_FOR_EMG_MEAN must fit in 32bit
                    emg_counter_2++;
                    if (ADC_buf[3] < 0)
                        ADC_buf[3] = 0;
                    g_mem.emg_max_value[1] += filter_ch2((int32)ADC_buf[3]);
        
                    if (interrupt_flag){
                        interrupt_flag = FALSE;
                        interrupt_manager();
                    }
                    
                    if (emg_counter_2 == SAMPLES_FOR_EMG_MEAN) {
                        g_mem.emg_max_value[1] = g_mem.emg_max_value[1] / SAMPLES_FOR_EMG_MEAN; // calc mean
                        
                        LED_CTRL_Write(0);
                        LED_BLINK_EN_Write(0);
                        
                        emg_counter_2 = 0;          // reset counter
                    
                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }
                        
                        emg_2_status = WAIT_EoC;           // goto end of calibration wait
                    }
                    break;

                case WAIT: // wait for EMG calibration to be done
                    if (emg_1_status == NORMAL)
                        emg_2_status = DISCARD;           // goto discard sample
                    break;

                case WAIT_EoC:  // wait for end of calibration procedure (only for LED visibility reasons)
                    emg_counter_2++;
                    if (emg_counter_2 == EMG_SAMPLE_TO_DISCARD) {
                        emg_counter_2 = 0;          // reset counter

                        if (interrupt_flag){
                            interrupt_flag = FALSE;
                            interrupt_manager();
                        }
                        
                        // if EMG control mode active, activate motors accordingly with startup value
                        if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
                            (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
                            g_ref.pos[0] = g_meas.pos[0];
                            g_ref.pos[1] = g_meas.pos[1];
                        }
                            
                        emg_2_status = NORMAL;           // goto normal execution
                    }
                    break;
                    
                default:
                    break;
            }
                
            if (interrupt_flag){
                interrupt_flag = FALSE;
                interrupt_manager();
            }
        }
    }
    else {

        emg_1_status = RESET; 
        emg_2_status = RESET;

        UD_status = RESET;
        LR_status = RESET;
		
        tension_valid = FALSE;
        
        //fixed
        LED_CTRL_Write(1);
        //PWM Blink Enable
        LED_BLINK_EN_Write(0);
            
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }
        
        if (c_mem.emg_calibration_flag) {
            if ((c_mem.input_mode == INPUT_MODE_EMG_PROPORTIONAL) ||
                (c_mem.input_mode == INPUT_MODE_EMG_INTEGRAL) ||
                (c_mem.input_mode == INPUT_MODE_EMG_FCFS) ||
                (c_mem.input_mode == INPUT_MODE_EMG_FCFS_ADV)) {
            }
        }

        // Reset emg
        g_meas.emg[0] = 0;
        g_meas.emg[1] = 0;
        
        // Reset Joystick
        g_meas.joystick[0] = 0;
        g_meas.joystick[1] = 0;

    }
    
    // The board LED blinks if attached battery is not fully charged
    if (!first_tension_valid && tension_valid == TRUE && emg_1_status == NORMAL && emg_2_status == NORMAL){
        dev_tension_f = filter_voltage(dev_tension);
        if (dev_tension_f > 0.9 * pow_tension){     //0.9*12000 mV = 10800 mV
            //fixed
            LED_CTRL_Write(1);
            
            //PWM Blink Enable
            LED_BLINK_EN_Write(0);
        }
        else {
            // blink
            LED_CTRL_Write(0);
            
            //PWM Blink Enable
            LED_BLINK_EN_Write(1);
            
            // Disable slave or motors because of not fully charged battery
            if (master_mode) {
                deactivate_slaves();
                
                // Check Interrupt 
                if (interrupt_flag){
                    interrupt_flag = FALSE;
                    interrupt_manager();
                }
                
                master_mode = 0;        // exit from master mode
            }
        }
    }
        
    if (interrupt_flag){
        interrupt_flag = FALSE;
        interrupt_manager();
    }
}

/* [] END OF FILE */