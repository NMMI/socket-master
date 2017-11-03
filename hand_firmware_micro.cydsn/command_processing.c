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
* \file         command_processing.c
*

* \brief        Command processing functions.
* \date         October 01, 2017
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

//=================================================================     includes
#include <command_processing.h>
#include <interruptions.h>
#include <stdio.h>
#include <utils.h>

#include "commands.h"

//================================================================     variables

reg8 * EEPROM_ADDR = (reg8 *) CYDEV_EE_BASE;

//==============================================================================
//                                                            RX DATA PROCESSING
//==============================================================================
//  This function checks for the availability of a data packet and process it:
//      - Verify checksum;
//      - Process commands;
//==============================================================================

void commProcess(void){

    uint8 CYDATA rx_cmd;

    rx_cmd = g_rx.buffer[0];

//==========================================================     verify checksum

    if (!(LCRChecksum(g_rx.buffer, g_rx.length - 1) == g_rx.buffer[g_rx.length - 1])){
        // Wrong checksum
        g_rx.ready = 0;
        return;
    }

    switch(rx_cmd) {

//=============================================================     CMD_ACTIVATE
        case CMD_ACTIVATE:
            cmd_activate();
            break;

//===========================================================     CMD_SET_INPUTS

        case CMD_SET_INPUTS:
            cmd_set_inputs();
            break;

//=====================================================     CMD_GET_MEASUREMENTS

        case CMD_GET_MEASUREMENTS:
                cmd_get_measurements();
            break;

//=========================================================     CMD_GET_CURRENTS

        case CMD_GET_CURRENTS:
            cmd_get_currents();
            break;

//=========================================================     CMD_GET_EMG

        case CMD_GET_EMG:
            cmd_get_emg();
            break;

//=============================================================     CMD_WATCHDOG
            
        case CMD_SET_WATCHDOG:
            cmd_set_watchdog();
            break;
            
//=========================================================     CMD_GET_ACTIVATE
            
        case CMD_GET_ACTIVATE:
            cmd_get_activate();
            break;
            
//=========================================================     CMD_SET_BAUDRATE
            
        case CMD_SET_BAUDRATE:
            cmd_set_baudrate();
            break;  
            
//============================================================     CMD_GET_INPUT

        case CMD_GET_INPUTS:
            cmd_get_inputs();
            break;

//=============================================================     CMD_GET_INFO

        case CMD_GET_INFO:
            infoGet( *((uint16 *) &g_rx.buffer[1]));
            break;

//============================================================     CMD_SET_PARAM

        case CMD_SET_ZEROS:
            setZeros();
            break;

//============================================================     CMD_GET_PARAM

        case CMD_GET_PARAM_LIST:
            get_param_list( *((uint16 *) &g_rx.buffer[1]) );
            break;

//=================================================================     CMD_PING
            
        case CMD_PING:
            cmd_ping();
            break;

//=========================================================     CMD_STORE_PARAMS
            
        case CMD_STORE_PARAMS:
            cmd_store_params();
            break;

//=================================================     CMD_STORE_DEFAULT_PARAMS

        case CMD_STORE_DEFAULT_PARAMS:
            if(memStore(DEFAULT_EEPROM_DISPLACEMENT))
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=======================================================     CMD_RESTORE_PARAMS

        case CMD_RESTORE_PARAMS:
            if(memRestore())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//=============================================================     CMD_INIT_MEM

        case CMD_INIT_MEM:
            if(memInit())
                sendAcknowledgment(ACK_OK);
            else
                sendAcknowledgment(ACK_ERROR);
            break;

//===========================================================     CMD_BOOTLOADER

        case CMD_BOOTLOADER:
            //Not sure if ACK_OK is correct, should check
            sendAcknowledgment(ACK_OK);
            CyDelay(1000);
            FTDI_ENABLE_REG_Write(0x00);
            CyDelay(1000);
            Bootloadable_Load();
            break;

//============================================================     CMD_CALIBRATE

        case CMD_HAND_CALIBRATE:
            calib.speed = *((int16 *) &g_rx.buffer[1]);
            calib.repetitions = *((int16 *) &g_rx.buffer[3]);
            
            // Speed & repetitions saturations
            if (calib.speed < 0) {
                calib.speed = 0;
            } else if (calib.speed > 200) {
                calib.speed = 200;
            }
            if (calib.repetitions < 0) {
                calib.repetitions = 0;
            } else if (calib.repetitions > 32767) {
                calib.repetitions = 32767;
            }
            
            g_refNew.pos[0] = 0;
            calib.enabled = TRUE;
            break;

//============================================================     CMD_GET_HAND_MEASUREMENTS
            
        case CMD_GET_HAND_MEASUREMENTS:
            cmd_get_measurements_from_SH();
            break;

//=========================================================== ALL OTHER COMMANDS
        default:
            break;

    }
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================

void infoSend(void){
    unsigned char packet_string[1100];
    infoPrepare(packet_string);
    UART_RS485_PutString(packet_string);
}


//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type) {
    unsigned char packet_string[1500] = "";

    //==================================     choose info type and prepare string

    switch (info_type) {
        case INFO_ALL:
            infoPrepare(packet_string);
            UART_RS485_PutString(packet_string);
            break;

        default:
            break;
    }
}

//==============================================================================
//                                                                GET PARAM LIST
//==============================================================================

void get_param_list(uint16 index) {
    //Package to be sent variables
    uint8 packet_data[1751] = "";
    uint16 packet_lenght = 1751;

    //Auxiliary variables
    uint8 CYDATA i;
    uint8 CYDATA string_lenght;
    int32 aux_int;
    uint8 aux_uchar;

    //Parameters menu string definitions
    char id_str[15] = "1 - Device ID:";
    char pos_pid_str[28] = "2 - Position PID [P, I, D]:";
    char curr_pid_str[27] = "3 - Current PID [P, I, D]:";
    char startup_str[28] = "4 - Startup Activation:";
    char input_str[34] = "5 - Input mode:";
    char contr_str[39] = "6 - Control mode:";
    char res_str[17] = "7 - Resolutions:";
    char m_off_str[25] = "8 - Measurement Offsets:";
    char mult_str[17] = "9 - Multipliers:";
    char pos_lim_flag_str[28] = "10 - Pos. limit active:";
    char pos_lim_str[29] = "11 - Pos. limits [inf, sup]:";
    char max_step_str[27] = "12 - Max steps [neg, pos]:";
    char curr_limit_str[20] = "13 - Current limit:";
    char emg_flag_str[37] = "14 - EMG calibration on startup:";
    char emg_thr_str[21] = "15 - EMG thresholds:";
    char emg_max_val_str[21] = "16 - EMG max values:";
    char emg_max_speed_str[20] = "17 - EMG max speed:";
    char abs_enc_str[36] = "18 - Absolute encoder position:";
    char handle_ratio_str[25] = "19 - Motor handle ratio:"; 
    char motor_type_str[24] = "20 - PWM rescaling:";
    char rest_pos_str[20] = "21 - Rest position:";
    char rest_pos_delay_str[36] = "22 - Rest position time delay (ms):";
    char rest_vel_str[35] = "23 - Rest vel closure (ticks/sec):";
    char rest_ratio_str[17] = "24 - Rest ratio:";
    char curr_lookup_str[21] = "25 - Current lookup:";


    //Parameters menus
    char input_mode_menu[99] = "0 -> Usb\n1 -> Handle\n2 -> EMG proportional\n3 -> EMG Integral\n4 -> EMG FCFS\n5 -> EMG FCFS Advanced\n";
    char control_mode_menu[92] = "0 -> Position\n1 -> PWM\n2 -> Current\n3 -> Position and Current\n4 -> Position w. rest check\n";
    char yes_no_menu[42] = "0 -> Deactivate [NO]\n1 -> Activate [YES]\n";

    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA pos_pid_str_len = strlen(pos_pid_str);
    uint8 CYDATA curr_pid_str_len = strlen(curr_pid_str);

    uint8 CYDATA res_str_len = strlen(res_str);
    uint8 CYDATA m_off_str_len = strlen(m_off_str);
    uint8 CYDATA mult_str_len = strlen(mult_str);

    uint8 CYDATA pos_lim_str_len = strlen(pos_lim_str);
    uint8 CYDATA max_step_str_len = strlen(max_step_str);
    uint8 CYDATA curr_limit_str_len = strlen(curr_limit_str);

    uint8 CYDATA emg_thr_str_len = strlen(emg_thr_str);
    uint8 CYDATA emg_max_val_str_len = strlen(emg_max_val_str);
    uint8 CYDATA emg_max_speed_str_len = strlen(emg_max_speed_str);

    uint8 CYDATA handle_ratio_str_len = strlen(handle_ratio_str);
    uint8 CYDATA curr_lookup_str_len = strlen(curr_lookup_str);
    uint8 CYDATA input_mode_menu_len = strlen(input_mode_menu);
    uint8 CYDATA control_mode_menu_len = strlen(control_mode_menu);
    uint8 CYDATA yes_no_menu_len = strlen(yes_no_menu);
    uint8 CYDATA rest_pos_str_len = strlen(rest_pos_str);
    uint8 CYDATA rest_pos_delay_str_len = strlen(rest_pos_delay_str);
    uint8 CYDATA rest_vel_str_len = strlen(rest_vel_str);
    uint8 CYDATA rest_ratio_str_len = strlen(rest_ratio_str);

    packet_data[0] = CMD_GET_PARAM_LIST;
    packet_data[1] = NUM_OF_PARAMS;

    switch(index) {
        case 0:         //List of all parameters with relative types
            /*-----------------ID-----------------*/

            packet_data[2] = TYPE_UINT8;
            packet_data[3] = 1;
            packet_data[4] = c_mem.id;
            for(i = id_str_len; i != 0; i--)
                packet_data[5 + id_str_len - i] = id_str[id_str_len - i];

            /*-------------POSITION PID-----------*/

            packet_data[52] = TYPE_FLOAT;
            packet_data[53] = 3;
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                *((float *) (packet_data + 54)) = (float) c_mem.k_p / 65536;
                *((float *) (packet_data + 58)) = (float) c_mem.k_i / 65536;
                *((float *) (packet_data + 62)) = (float) c_mem.k_d / 65536;
            }
            else {
                *((float *) (packet_data + 54)) = (float) c_mem.k_p_dl / 65536;
                *((float *) (packet_data + 58)) = (float) c_mem.k_i_dl / 65536;
                *((float *) (packet_data + 62)) = (float) c_mem.k_d_dl / 65536;
            }

            for(i = pos_pid_str_len; i != 0; i--)
                packet_data[66 + pos_pid_str_len - i] = pos_pid_str[pos_pid_str_len - i];

            /*--------------CURRENT PID-----------*/

            packet_data[102] = TYPE_FLOAT;
            packet_data[103] = 3;
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                *((float *) (packet_data + 104)) = (float) c_mem.k_p_c / 65536;
                *((float *) (packet_data + 108)) = (float) c_mem.k_i_c / 65536;
                *((float *) (packet_data + 112)) = (float) c_mem.k_d_c / 65536;
            }
            else {
                *((float *) (packet_data + 104)) = (float) c_mem.k_p_c_dl / 65536;
                *((float *) (packet_data + 108)) = (float) c_mem.k_i_c_dl / 65536;
                *((float *) (packet_data + 112)) = (float) c_mem.k_d_c_dl / 65536;
            }
            
            for(i = curr_pid_str_len; i != 0; i--)
                packet_data[116 + curr_pid_str_len - i] = curr_pid_str[curr_pid_str_len - i];

            /*----------STARTUP ACTIVATION--------*/

            packet_data[152] = TYPE_FLAG;
            packet_data[153] = 1;
            packet_data[154] = c_mem.activ;
            if(c_mem.activ) {
                strcat(startup_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(startup_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[155 + string_lenght - i] = startup_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[155 + string_lenght] = 3;

            /*--------------INPUT MODE------------*/
            
            packet_data[202] = TYPE_FLAG;
            packet_data[203] = 1;
            packet_data[204] = c_mem.input_mode;
            switch(c_mem.input_mode) {
                case INPUT_MODE_EXTERNAL:
                    strcat(input_str, " Usb\0");
                    string_lenght = 20;
                break;
                case INPUT_MODE_ENCODER3:
                    strcat(input_str, " Handle\0");
                    string_lenght = 23;
                break;
                case INPUT_MODE_EMG_PROPORTIONAL:
                    strcat(input_str, " EMG proportional\0");
                    string_lenght = 33;
                break;
                case INPUT_MODE_EMG_INTEGRAL:
                    strcat(input_str, " EMG integral\0");
                    string_lenght = 29;
                break;
                case INPUT_MODE_EMG_FCFS:
                    strcat(input_str, " EMG FCFS\0");
                    string_lenght = 25;
                break;
                case INPUT_MODE_EMG_FCFS_ADV:
                    strcat(input_str, " EMG FCFS Advanced\0");
                    string_lenght = 34;
                break;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[205 + string_lenght - i] = input_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[205 + string_lenght] = 1;
            
            /*-------------CONTROL MODE-----------*/
            
            packet_data[252] = TYPE_FLAG;
            packet_data[253] = 1;
            packet_data[254] = c_mem.control_mode;
            switch(c_mem.control_mode){
                case CONTROL_ANGLE:
                    strcat(contr_str, " Position\0");
                    string_lenght = 27;
                break;
                case CONTROL_PWM:
                    strcat(contr_str, " PWM\0");
                    string_lenght = 22;
                break;
                case CONTROL_CURRENT:
                    strcat(contr_str, " Current\0");
                    string_lenght = 26;
                break;
                case CURR_AND_POS_CONTROL:
                    strcat(contr_str, " Position and Current\0");
                    string_lenght = 39;
                break;
                case CONTROL_ANGLE_AND_REST_POS:
                    strcat(contr_str, " Position and Rest\0");
                    string_lenght = 36;
                break;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[255 + string_lenght - i] = contr_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[255 + string_lenght] = 2;
            
            /*-------------RESOLUTIONS------------*/
            
            packet_data[302] = TYPE_UINT8;
            packet_data[303] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                packet_data[i + 304] = c_mem.res[i];
            for(i = res_str_len; i != 0; i--)
                packet_data[307 + res_str_len - i] = res_str[res_str_len - i];
            
            /*----------MEASUREMENT OFFSET--------*/
            
            packet_data[352] = TYPE_INT16;
            packet_data[353] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++) 
                *((int16 *) ( packet_data + 354 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
            for(i = m_off_str_len; i != 0; i--)
                packet_data[360 + m_off_str_len - i] = m_off_str[m_off_str_len - i];
            
            /*------------MULTIPLIERS-------------*/
            
            packet_data[402] = TYPE_FLOAT;
            packet_data[403] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                *((float *) ( packet_data + 404 + (i * 4) )) = c_mem.m_mult[i];
            for(i = mult_str_len; i != 0; i--)
                packet_data[416 + mult_str_len - i] = mult_str[mult_str_len - i];

            /*-----------POS LIMIT FLAG-----------*/
            
            packet_data[452] = TYPE_FLAG;
            packet_data[453] = 1;
            packet_data[454] = c_mem.pos_lim_flag;
            if(c_mem.pos_lim_flag) {
                strcat(pos_lim_flag_str, " YES\0");
                string_lenght = 28;
            }
            else {
                strcat(pos_lim_flag_str, " NO\0");
                string_lenght = 27;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[455 + string_lenght - i] = pos_lim_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[455 + string_lenght] = 3;
            
            /*-----------POSITION LIMITS----------*/
            
            packet_data[502] = TYPE_INT32;
            packet_data[503] = 2;
            for (i = 0; i < NUM_OF_MOTORS - 1; i++) {
                *((int32 *)( packet_data + 504 + (i * 2 * 4) )) = (c_mem.pos_lim_inf[i] >> c_mem.res[i]);
                *((int32 *)( packet_data + 504 + (i * 2 * 4) + 4)) = (c_mem.pos_lim_sup[i] >> c_mem.res[i]);
            }
            for(i = pos_lim_str_len; i != 0; i--)
                packet_data[512 + pos_lim_str_len - i] = pos_lim_str[pos_lim_str_len - i];

            /*--------------MAX STEPS-------------*/
            
            packet_data[552] = TYPE_INT32;
            packet_data[553] = 2;
            *((int32 *)(packet_data + 554)) = c_mem.max_step_neg;
            *((int32 *)(packet_data + 558)) = c_mem.max_step_pos;
            for(i = max_step_str_len; i != 0; i--)
                packet_data[562 + max_step_str_len - i] = max_step_str[max_step_str_len - i];

            /*------------CURRENT LIMIT-----------*/

            packet_data[602] = TYPE_INT16;
            packet_data[603] = 1;
            *((int16 *)(packet_data + 604)) = c_mem.current_limit;
            for(i = curr_limit_str_len; i != 0; i--)
                packet_data[606 + curr_limit_str_len - i] = curr_limit_str[curr_limit_str_len - i];

            /*-----------EMG CALIB FLAG-----------*/            
            
            packet_data[652] = TYPE_FLAG;
            packet_data[653] = 1;
            packet_data[654] = c_mem.emg_calibration_flag;
            if(c_mem.emg_calibration_flag) {
                strcat(emg_flag_str, " YES\0");
                string_lenght = 37;
            }
            else {
                strcat(emg_flag_str, " NO\0");
                string_lenght = 36;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[655 + string_lenght - i] = emg_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[655 + string_lenght] = 3;
            
            /*-----------EMG THRESHOLDS-----------*/
        
            packet_data[702] = TYPE_UINT16;
            packet_data[703] = 2;
            *((uint16 *)(packet_data + 704)) = c_mem.emg_threshold[0];
            *((uint16 *)(packet_data + 706)) = c_mem.emg_threshold[1];
            for(i = emg_thr_str_len; i != 0; i--)
                packet_data[708 + emg_thr_str_len - i] = emg_thr_str[emg_thr_str_len - i];
            
            /*------------EMG MAX VALUE-----------*/
            
            packet_data[752] = TYPE_UINT32;
            packet_data[753] = 2;
            *((uint32 *)(packet_data + 754)) = c_mem.emg_max_value[0];
            *((uint32 *)(packet_data + 758)) = c_mem.emg_max_value[1];
            for(i = emg_max_val_str_len; i != 0; i--)
                packet_data[762 + emg_max_val_str_len - i] = emg_max_val_str[emg_max_val_str_len - i];

            /*--------------EMG SPEED-------------*/
            
            packet_data[802] = TYPE_UINT8;
            packet_data[803] = 1;
            packet_data[804] = c_mem.emg_speed;
            for(i = emg_max_speed_str_len; i != 0; i--)
                packet_data[805 + emg_max_speed_str_len - i] = emg_max_speed_str[emg_max_speed_str_len - i];
            
            /*------------DOUBLE ENCODER----------*/
            
            packet_data[852] = TYPE_FLAG;
            packet_data[853] = 1;
            packet_data[854] = c_mem.double_encoder_on_off;
            if(c_mem.double_encoder_on_off) {
                strcat(abs_enc_str, " YES\0");
                string_lenght = 36;
            }
            else {
                strcat(abs_enc_str, " NO\0");
                string_lenght = 35;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[855 + string_lenght - i] = abs_enc_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[855 + string_lenght] = 3;
            
            /*-------------HANDLE RATIO-----------*/
            
            packet_data[902] = TYPE_INT8;
            packet_data[903] = 1;
            *((int8 *)(packet_data + 904)) = c_mem.motor_handle_ratio;
            for(i = handle_ratio_str_len; i != 0; i--)
                packet_data[905 + handle_ratio_str_len - i] = handle_ratio_str[handle_ratio_str_len - i];
            
            /*-------------MOTOR SUPPLY-----------*/
            
            packet_data[952] = TYPE_FLAG;
            packet_data[953] = 1;
            packet_data[954] = c_mem.activate_pwm_rescaling;
            if(c_mem.activate_pwm_rescaling) {
                strcat(motor_type_str, " YES\0");
                string_lenght = 24;
            }
            else {
                strcat(motor_type_str, " NO\0");
                string_lenght = 23;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[955 + string_lenght - i] = motor_type_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[955 + string_lenght] = 3;

            /*-----------REST POSITION----------*/
            
            packet_data[1002] = TYPE_INT32;
            packet_data[1003] = 1;
            *((int32 *)( packet_data + 1004 )) = (c_mem.rest_pos >> c_mem.res[0]);
            for(i = rest_pos_str_len; i != 0; i--)
                packet_data[1008 + rest_pos_str_len - i] = rest_pos_str[rest_pos_str_len - i];
                
            /*-----------REST POSITION TIME DELAY----------*/
            
            packet_data[1052] = TYPE_FLOAT;
            packet_data[1053] = 1;
            *((float *)( packet_data + 1054 )) = c_mem.rest_delay;
            for(i = rest_pos_delay_str_len; i != 0; i--)
                packet_data[1058 + rest_pos_delay_str_len - i] = rest_pos_delay_str[rest_pos_delay_str_len - i];
                
            /*-----------REST POSITION VELOCITY----------*/
            
            packet_data[1102] = TYPE_FLOAT;
            packet_data[1103] = 1;
            *((float *)( packet_data + 1104 )) = (float)(c_mem.rest_vel*1000.0);
            for(i = rest_vel_str_len; i != 0; i--)
                packet_data[1108 + rest_vel_str_len - i] = rest_vel_str[rest_vel_str_len - i];   
                
            /*-----------REST RATIO----------*/
            
            packet_data[1152] = TYPE_FLOAT;
            packet_data[1153] = 1;
            *((float *)( packet_data + 1154 )) = (float)(c_mem.rest_ratio);
            for(i = rest_ratio_str_len; i != 0; i--)
                packet_data[1158 + rest_ratio_str_len - i] = rest_ratio_str[rest_ratio_str_len - i];   
                
            /*---------CURRENT LOOKUP TABLE---------*/

            packet_data[1202] = TYPE_FLOAT;
            packet_data[1203] = 6;
            for(i = 0; i < LOOKUP_DIM; i++)
                *((float *) ( packet_data + 1204 + (i * 4) )) = c_mem.curr_lookup[i];
            for(i = curr_lookup_str_len; i != 0; i--)
                packet_data[1228 + curr_lookup_str_len - i] = curr_lookup_str[curr_lookup_str_len - i];
                
            /*------------PARAMETERS MENU-----------*/

            for(i = input_mode_menu_len; i != 0; i--)
                packet_data[1252 + input_mode_menu_len - i] = input_mode_menu[input_mode_menu_len - i];

            for(i = control_mode_menu_len; i != 0; i--)
                packet_data[1402 + control_mode_menu_len - i] = control_mode_menu[control_mode_menu_len - i];

            for(i = yes_no_menu_len; i!= 0; i--)
                packet_data[1552 + yes_no_menu_len - i] = yes_no_menu[yes_no_menu_len - i];

            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;
        
//=======================================================     set_pid_parameters
        case 2:         //Position PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                g_mem.k_p = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
        break;

//==================================================     set_curr_pid_parameters
        case 3:         //Current PID - float[3]
            if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
                g_mem.k_p_c = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            else {
                g_mem.k_p_c_dl = *((float *) &g_rx.buffer[3]) * 65536;
                g_mem.k_i_c_dl = *((float *) &g_rx.buffer[3 + 4]) * 65536;
                g_mem.k_d_c_dl = *((float *) &g_rx.buffer[3 + 8]) * 65536;
            }
            
        break;

//===================================================     set_startup_activation        
        case 4:         //Startup flag - uint8
            if(g_rx.buffer[3])
                g_mem.activ = 0x03;
            else
                g_mem.activ = 0x00;
        break;

//===========================================================     set_input_mode        
        case 5:         //Input mode - uint8
            g_mem.input_mode = *((uint8*) &g_rx.buffer[3]);
        break;
        
//=========================================================     set_control_mode
        case 6:         //Control mode - uint8
            g_mem.control_mode = *((uint8*) &g_rx.buffer[3]);
        break;
        
//===========================================================     set_resolution
        case 7:         //Resolution - uint8[3]
            for (i =0; i < NUM_OF_SENSORS; i++) {
                g_mem.res[i] = g_rx.buffer[i+3];
            }
        break;
        
//===============================================================     set_offset
        case 8:         //Measurement Offset - int32[3] 
            for(i = 0; i < NUM_OF_SENSORS; ++i) {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
        break;
        
//===========================================================     set_multiplier
        case 9:         //Multipliers - float[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i)
                g_mem.m_mult[i] = *((float *) &g_rx.buffer[3 + i * 4]);
        break;
        
//=====================================================     set_pos_limit_enable
        case 10:        //Position limit flag - uint8
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
        break;

//============================================================     set_pos_limit
        case 11:        //Position limits - int32[4]
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4)]);
                g_mem.pos_lim_sup[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4) + 4]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];
            }
        break;

//==================================================     set_max_steps_per_cycle
        case 12:        //Max steps - int32[2]
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0)
                g_mem.max_step_neg = aux_int;

            aux_int = *((int32 *) &g_rx.buffer[3 + 4]);
            if (aux_int >= 0) 
                g_mem.max_step_pos = aux_int;
            
        break;
        
//========================================================     set_current_limit
        case 13:        //Current limit - int16
            g_mem.current_limit = *((int16*) &g_rx.buffer[3]);
        break;
        
//=======================================================     set_emg_calib_flag
        case 14:        //Emg calibration flag - int8
            g_mem.emg_calibration_flag = *((uint8*) &g_rx.buffer[3]);
        break;
        
//========================================================     set_emg_threshold
        case 15:        //Emg threshold - uint16[2]
            g_mem.emg_threshold[0] = *((uint16*) &g_rx.buffer[3]);
            g_mem.emg_threshold[1] = *((uint16*) &g_rx.buffer[5]);
        break;
        
//========================================================     set_emg_max_value
        case 16:        //Emg max value - uint32[2]
            g_mem.emg_max_value[0] = *((uint32*) &g_rx.buffer[3]);
            g_mem.emg_max_value[1] = *((uint32*) &g_rx.buffer[7]);
        break;
        
//============================================================     set_emg_speed
        case 17:        //Emg max speed - uint8
            g_mem.emg_speed = *((uint8*) &g_rx.buffer[3]);
        break;
        
//================================================     set_double_encoder_on_off
        case 18:        //Absolute encoder flag - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.double_encoder_on_off = 1;
            } else {
                g_mem.double_encoder_on_off = 0;
            }
        break;
        
//===================================================     set_motor_handle_ratio
        case 19:        //Motor handle ratio - int8
            g_mem.motor_handle_ratio = *((int8*) &g_rx.buffer[3]);
        break;
        
//===================================================     set_motor_supply_type
        case 20:        //Motor type - uint8
            g_mem.activate_pwm_rescaling = g_rx.buffer[3];
        break;
//============================================================     set_rest_pos
        case 21:        //Rest Position - int32
            g_mem.rest_pos = *((int32 *) &g_rx.buffer[3]);
            g_mem.rest_pos = g_mem.rest_pos << g_mem.res[0];
        break;   
//============================================================     set_rest_delay_pos
        case 22:        //Rest Position Time Delay - float
            g_mem.rest_delay = *((float *) &g_rx.buffer[3]);
        break;   
//============================================================     set_rest_vel
        case 23:        //Rest Position Velocity - float
            g_mem.rest_vel = *((float *) &g_rx.buffer[3]);
            g_mem.rest_vel = g_mem.rest_vel/1000.0;       //conversion [s -> ms]
        break;     
//============================================================     set_rest_ratio
        case 24:        //Rest Ratio - float
            g_mem.rest_ratio = *((float *) &g_rx.buffer[3]);
        break;                 
//===================================================     set_curr_lookup_table
        case 25:        //Current lookup table - float
            for(i = 0; i < LOOKUP_DIM; i++)
                g_mem.curr_lookup[i] = *((float *) &g_rx.buffer[3 + i*4]);
        break;
    }
}

//==============================================================================
//                                                            COMMAND SET ZEROS
//==============================================================================

void setZeros()
{
    uint8 CYDATA i;        // iterator

    for(i = 0; i < NUM_OF_SENSORS; ++i) {
        g_mem.m_off[i] = *((int16 *) &g_rx.buffer[1 + i * 2]);
        g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

        g_meas.rot[i] = 0;
    }
    reset_last_value_flag = 1;

    sendAcknowledgment(ACK_OK);
}

//==============================================================================
//                                                           PREPARE DEVICE INFO
//==============================================================================

void infoPrepare(unsigned char *info_string)
{
    int i;
    if(c_mem.id != 250){                //To avoid dummy board ping
        unsigned char str[100];
        strcpy(info_string, "");
        strcat(info_string, "\r\n");
        strcat(info_string, "Firmware version: ");
        strcat(info_string, VERSION);
        strcat(info_string, ".\r\n\r\n");

        strcat(info_string, "DEVICE INFO\r\n");
        sprintf(str, "ID: %d\r\n", (int) c_mem.id);
        strcat(info_string, str);
        strcat(info_string, "PWM rescaling activation: ");
        if(c_mem.activate_pwm_rescaling == MAXON_12V)
            strcat(info_string, "YES\n");
        else
            strcat(info_string, "NO\n");
        
        sprintf(str, "PWM Limit: %d\r\n", (int) dev_pwm_limit);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        strcat(info_string, "MOTOR INFO\r\n");
        strcat(info_string, "Motor reference");
        
        if(g_mem.control_mode == CONTROL_CURRENT)
            strcat(info_string," - Currents: ");
        else {
            if (g_mem.control_mode == CONTROL_PWM)
                strcat(info_string," - Pwm: ");
            else
                strcat(info_string," - Position: ");
        }
    
        for (i = 0; i < NUM_OF_MOTORS; i++) {
            if(g_mem.control_mode == CONTROL_CURRENT) {
                sprintf(str, "%d ", (int)(g_ref.curr[i]));
                strcat(info_string,str);
            }
        else {
            if(g_mem.control_mode == CONTROL_PWM) {
                sprintf(str, "%d ", (int)(g_ref.pwm[i]));
                strcat(info_string,str);
            }
            else {
                sprintf(str, "%d ", (int)(g_ref.pos[i] >> c_mem.res[i]));
                strcat(info_string,str);
            }
        }
    }
    strcat(info_string,"\r\n");
        strcat(info_string, "\r\n");

        sprintf(str, "Motor enabled: ");
        if (g_ref.onoff & 0x03) {
            strcat(str, "YES\r\n");
        } else {
            strcat(str, "NO\r\n");
        }
        strcat(info_string, str);

        strcat(info_string, "\r\nMEASUREMENTS INFO\r\n");
        strcat(info_string, "Sensor value:\r\n");
        for (i = 0; i < NUM_OF_SENSORS; i++) {
            sprintf(str, "%d -> %d", i+1,
            (int)(g_meas.pos[i] >> c_mem.res[i]));
            strcat(info_string, str);
            strcat(info_string, "\r\n");
        }

        sprintf(str, "Battery Voltage (mV): %ld", (int32) dev_tension );
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Full charge power tension (mV): %ld", (int32) pow_tension );
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        sprintf(str, "Current (mA): %ld", (int32) g_meas.curr[0] );
        strcat(info_string, str);
        strcat(info_string, "\r\n");


        strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

        strcat(info_string, "PID Controller:\r\n");
        if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
            sprintf(str, "P -> %f  ", ((double) c_mem.k_p / 65536));
            strcat(info_string, str);
            sprintf(str, "I -> %f  ", ((double) c_mem.k_i / 65536));
            strcat(info_string, str);
            sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d / 65536));
            strcat(info_string, str);
        }
        else { 
            sprintf(str, "P -> %f  ", ((double) c_mem.k_p_dl / 65536));
            strcat(info_string, str);
            sprintf(str, "I -> %f  ", ((double) c_mem.k_i_dl / 65536));
            strcat(info_string, str);
            sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_dl / 65536));
            strcat(info_string, str);
        }

        strcat(info_string, "Current PID Controller:\r\n");
        if(c_mem.control_mode != CURR_AND_POS_CONTROL) {
            sprintf(str, "P -> %f  ", ((double) c_mem.k_p_c / 65536));
            strcat(info_string, str);
            sprintf(str, "I -> %f  ", ((double) c_mem.k_i_c / 65536));
            strcat(info_string, str);
            sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_c / 65536));
            strcat(info_string, str);
        }
        else {
            sprintf(str, "P -> %f  ", ((double) c_mem.k_p_c_dl / 65536));
            strcat(info_string, str);
            sprintf(str, "I -> %f  ", ((double) c_mem.k_i_c_dl / 65536));
            strcat(info_string, str);
            sprintf(str, "D -> %f\r\n", ((double) c_mem.k_d_c_dl / 65536));
            strcat(info_string, str);
        }

        strcat(info_string, "\r\n");


        if (c_mem.activ == 0x03) {
            strcat(info_string, "Startup activation: YES\r\n");
        } else {
            strcat(info_string, "Startup activation: NO\r\n");
        }

        switch(c_mem.input_mode) {
            case INPUT_MODE_EXTERNAL:
                strcat(info_string, "Input mode: USB\r\n");
                break;
            case INPUT_MODE_ENCODER3:
                strcat(info_string, "Input mode: Handle\r\n");
                break;
            case INPUT_MODE_EMG_PROPORTIONAL:
                strcat(info_string, "Input mode: EMG proportional\r\n");
                break;
            case INPUT_MODE_EMG_INTEGRAL:
                strcat(info_string, "Input mode: EMG integral\r\n");
                break;
            case INPUT_MODE_EMG_FCFS:
                strcat(info_string, "Input mode: EMG FCFS\r\n");
                break;
            case INPUT_MODE_EMG_FCFS_ADV:
                strcat(info_string, "Input mode: EMG FCFS ADV\r\n");
                break;
        }

        switch(c_mem.control_mode) {
            case CONTROL_ANGLE:
                strcat(info_string, "Control mode: Position\r\n");
                break;
            case CONTROL_PWM:
                strcat(info_string, "Control mode: PWM\r\n");
                break;
            case CONTROL_CURRENT:
                strcat(info_string, "Control mode: Current\r\n");
                break;
            case CURR_AND_POS_CONTROL:
                strcat(info_string, "Control mode: Position and Current\r\n");
                break;
            case CONTROL_ANGLE_AND_REST_POS:
                strcat(info_string, "Control mode: Position and Rest check\r\n");
                break;
            default:
                break;
        }

        if (c_mem.double_encoder_on_off) {
            strcat(info_string, "Absolute encoder position: YES\r\n");
        } else {
            strcat(info_string, "Absolute encoder position: NO\r\n");
        }

        sprintf(str, "Motor-Handle Ratio: %d\r\n", (int)c_mem.motor_handle_ratio);
        strcat(info_string, str);



        strcat(info_string, "Sensor resolution:\r\n");
        for (i = 0; i < NUM_OF_SENSORS; ++i) {
            sprintf(str, "%d -> %d", (int) (i + 1), (int) c_mem.res[i]);
            strcat(info_string, str);
            strcat(info_string, "\r\n");
        }


        strcat(info_string, "Measurement Offset:\r\n");
        for (i = 0; i < NUM_OF_SENSORS; ++i) {
            sprintf(str, "%d -> %ld", (int) (i + 1), (int32) c_mem.m_off[i] >> c_mem.res[i]);
            strcat(info_string, str);
            strcat(info_string, "\r\n");
        }

        strcat(info_string, "Measurement Multiplier:\r\n");
        for (i = 0; i < NUM_OF_SENSORS; ++i) {
            sprintf(str,"%d -> %f", (int)(i + 1), (double) c_mem.m_mult[i]);
            strcat(info_string, str);
            strcat(info_string,"\r\n");
        }
        
        strcat(info_string, "Current lookup table:\r\n");
        sprintf(str, "p[0] - p[2]: %f, %f, %f\n", c_mem.curr_lookup[0], c_mem.curr_lookup[1], c_mem.curr_lookup[2]);
        strcat(info_string, str);
        sprintf(str, "p[3] - p[5]: %f, %f, %f\n", c_mem.curr_lookup[3], c_mem.curr_lookup[4], c_mem.curr_lookup[5]);
        strcat(info_string, str);
        strcat(info_string,"\r\n");
        

        sprintf(str, "Position limit active: %d", (int)g_mem.pos_lim_flag);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        for (i = 0; i < NUM_OF_MOTORS - 1; i++) {
            sprintf(str, "Position limit motor %d: inf -> %ld  ", (int)(i + 1),
            (int32)g_mem.pos_lim_inf[i] >> g_mem.res[i]);
            strcat(info_string, str);

            sprintf(str, "sup -> %ld\r\n",
            (int32)g_mem.pos_lim_sup[i] >> g_mem.res[i]);
            strcat(info_string, str);
        }

        sprintf(str, "Max step pos and neg: %d %d", (int)g_mem.max_step_pos, (int)g_mem.max_step_neg);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        sprintf(str, "Current limit: %d", (int)g_mem.current_limit);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        sprintf(str, "EMG thresholds [0 - 1024]: %u, %u", g_mem.emg_threshold[0], g_mem.emg_threshold[1]);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        sprintf(str, "EMG max values [0 - 4096]: %lu, %lu", g_mem.emg_max_value[0], g_mem.emg_max_value[1]);
        strcat(info_string, str);
        strcat(info_string, "\r\n");

        if (g_mem.emg_calibration_flag) {
            strcat(info_string, "Calibration enabled: YES\r\n");
        } else {
            strcat(info_string, "Calibration enabled: NO\r\n");
        }

        sprintf(str, "EMG max speed: %d", (int)g_mem.emg_speed);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Rest time delay (ms): %f", (float)g_mem.rest_delay);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Rest velocity closure (ticks/sec): %f", (float)(g_mem.rest_vel*1000));
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Rest position: %d", (int)(g_mem.rest_pos >> c_mem.res[0]));
        strcat(info_string, str);
        strcat(info_string, "\r\n");   
        
        sprintf(str, "Rest ratio: %f", (float)(g_mem.rest_ratio));
        strcat(info_string, str);
        strcat(info_string, "\r\n"); 
      /*  
        sprintf(str, "Hand meas: %d", (int)(g_measOld.hand_meas));
        strcat(info_string, str);
        strcat(info_string, "\r\n"); 
        
        sprintf(str, "Check: %d", (int)(check2));
        strcat(info_string, str);
        strcat(info_string, "\r\n"); 
        
        sprintf(str, "Check 5: %d", (int)(check5));
        strcat(info_string, str);
        strcat(info_string, "\r\n"); 
        
        sprintf(str, "Check 3 and 4: %d, %d", (int)(check3), (int)check4);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        */
        sprintf(str, "debug: %ld", (uint32)timer_value0 - (uint32)timer_value); //5000001
        strcat(info_string, str);
        strcat(info_string, "\r\n");
    }
}

//==============================================================================
//                                                     WRITE FUNCTIONS FOR RS485
//==============================================================================

void commWrite_old_id(uint8 *packet_data, uint16 packet_lenght, uint8 old_id)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    //if(old_id)
        UART_RS485_PutChar(old_id);
    //else
        //UART_RS485_PutChar(g_mem.id);
        
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

void commWrite(uint8 *packet_data, uint16 packet_lenght)
{
    uint16 CYDATA index;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    UART_RS485_PutChar(g_mem.id);
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(index = 0; index < packet_lenght; ++index) {
        UART_RS485_PutChar(packet_data[index]);
    }

    index = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && index++ <= 1000){}

    RS485_CTS_Write(1);
    RS485_CTS_Write(0);
}

//==============================================================================
//                                READ MEASUREMENTS FUNCTION FROM ANOTHER DEVICE
//==============================================================================

int32 commReadMeasFromAnother()
{
    uint8 packet_data[10];
    uint8 packet_lenght;
    int32 aux_val;
//    uint16 b_l, b_h;
    uint32 t_start, t_end;
    
//    uint8 CYDATA index;

    packet_lenght = 2;
    packet_data[0] = CMD_GET_MEASUREMENTS;
    packet_data[1] = CMD_GET_MEASUREMENTS;
    commWriteAnother(packet_data, packet_lenght);
     
    receive_meas_from_hand = 1;
    t_start = (uint32) MY_TIMER_ReadCounter();
    while(g_rx.buffer[0] != CMD_GET_HAND_MEASUREMENTS) {
        if (interrupt_flag){
            interrupt_flag = FALSE;
            //receive_meas_from_hand = 1;
            interrupt_manager();
        }

        t_end = (uint32) MY_TIMER_ReadCounter();
        if((t_start - t_end) > 4500000){            // 4.5 s timeout
            master_mode = 0;                        // exit from master mode
            break;
        }
    }

    //b_l = (uint16)g_rx.buffer[1];
    //b_h = (uint16)g_rx.buffer[2];
    //curr_pos = (int16)((b_h << 8) | b_l);
//    ((int8 *) &curr_pos)[0] = (int8)g_rx.buffer[2];
//    ((int8 *) &curr_pos)[1] = (int8)g_rx.buffer[1];
 //   curr_pos = *((int16 *) &g_rx.buffer[1]);
  //  ((int16 *) &curr_pos)[0] = (int16)g_rx.buffer[2];
  //  ((int16 *) &curr_pos)[1] = (int16)g_rx.buffer[1];
    
  //  aux_val_2 = *((int16 *) &g_rx.buffer[1]);
    

//    for (index = 1; index--;) 
//        curr_pos = *((int16 *) &g_rx.buffer[(index << 1) + 1]);
    
    aux_val = *((int16 *) &g_rx.buffer[1]);
    
    //g_meas.hand_meas = aux_val;
    
    check2 = aux_val;
  //  aux_val = (int32)curr_pos;
    //aux_val = aux_val << g_mem.res[0];

    check3 = (uint8)g_rx.buffer[1];
    check4 = (uint8)g_rx.buffer[2];
    
    check5 = (int16)(aux_val << g_mem.res[0]);

    
    receive_meas_from_hand = 0;
  //  return aux_val;
    
    return (int32)(g_refOld.pos[0]);
}

//==============================================================================
//                                             WRITE FUNCTION FOR ANOTHER DEVICE
//==============================================================================

void commWriteAnother(uint8 *packet_data, uint16 packet_lenght)
{
    static uint16 CYDATA i;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    UART_RS485_PutChar(g_mem.id - 1);
    // frame - length
    UART_RS485_PutChar((uint8)packet_lenght);
    // frame - packet data
    for(i = 0; i < packet_lenght; ++i) {
        UART_RS485_PutChar(packet_data[i]);
    }

    i = 0;

    while(!(UART_RS485_ReadTxStatus() & UART_RS485_TX_STS_COMPLETE) && i++ <= 1000){}
}

//==============================================================================
//                                                             CHECKSUM FUNCTION
//==============================================================================

// Performs a XOR byte by byte on the entire vector

uint8 LCRChecksum(uint8 *data_array, uint8 data_length) {
    
    uint8 CYDATA i;
    uint8 CYDATA checksum = 0x00;
    
    for(i = 0; i < data_length; ++i)
       checksum ^= data_array[i];
  
    return checksum;
}


//==============================================================================
//                                                       ACKNOWLEDGMENT FUNCTION
//==============================================================================

void sendAcknowledgment(uint8 value) {
    int packet_lenght = 2;
    uint8 packet_data[2];

    packet_data[0] = value;
    packet_data[1] = value;

    commWrite(packet_data, packet_lenght);
}

//==============================================================================
//                                                                  STORE MEMORY
//==============================================================================


uint8 memStore(int displacement)
{
    int i;  // iterator
    uint8 writeStatus;
    int pages;
    uint8 ret_val = 1;

    // Disable Interrupt
    ISR_RS485_RX_Disable();

    // Stop motors
    PWM_MOTORS_WriteCompare1(0);
    PWM_MOTORS_WriteCompare2(0);

    // Retrieve temperature for better writing performance
    CySetTemp();

    memcpy( &c_mem, &g_mem, sizeof(g_mem) );

    pages = sizeof(g_mem) / 16 + (sizeof(g_mem) % 16 > 0);

    for(i = 0; i < pages; ++i) {
        writeStatus = EEPROM_Write(&g_mem.flag + 16 * i, i + displacement);
        if(writeStatus != CYRET_SUCCESS) {
            ret_val = 0;
            break;
        }
    }

    memcpy( &g_mem, &c_mem, sizeof(g_mem) );

    // Re-Enable Interrupt
    ISR_RS485_RX_Enable();

    return ret_val;
}


//==============================================================================
//                                                                 RECALL MEMORY
//==============================================================================


void memRecall(void)
{
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        memRestore();
    } else {
        memcpy( &c_mem, &g_mem, sizeof(g_mem) );
    }
}


//==============================================================================
//                                                                RESTORE MEMORY
//==============================================================================


uint8 memRestore(void) {
    uint16 i;

    for (i = 0; i < sizeof(g_mem); i++) {
        ((reg8 *) &g_mem.flag)[i] = EEPROM_ADDR[i + (DEFAULT_EEPROM_DISPLACEMENT * 16)];
    }

    //check for initialization
    if (g_mem.flag == FALSE) {
        return memInit();
    } else {
        return memStore(0);
    }
}

//==============================================================================
//                                                                   MEMORY INIT
//==============================================================================

uint8 memInit(void)
{
    uint8 i;

    //initialize memory settings
    g_mem.id            = 1;

    g_mem.k_p           = 0.015 * 65536;
    g_mem.k_i           =     0 * 65536;
    g_mem.k_d           = 0.007 * 65536;  //Changed in order to avoid metallic clatter previous value 0.2
    g_mem.k_p_c         =     1 * 65536;
    g_mem.k_i_c         = 0.001 * 65536;
    g_mem.k_d_c         =     0 * 65536;

    g_mem.k_p_dl        =   0.1 * 65536;
    g_mem.k_i_dl        =     0 * 65536;
    g_mem.k_d_dl        =     0 * 65536;
    g_mem.k_p_c_dl      =   0.3 * 65536;
    g_mem.k_i_c_dl      =0.0002 * 65536;
    g_mem.k_d_c_dl      =     0 * 65536;

    g_mem.activ         = 0;
    g_mem.input_mode    = INPUT_MODE_EXTERNAL;
    g_mem.control_mode  = CONTROL_ANGLE;

    g_mem.pos_lim_flag = 1;

    g_mem.activate_pwm_rescaling = MAXON_24V;           //rescaling active for 12V motors

    g_mem.res[0] = 3;
    g_mem.res[1] = 3;
    g_mem.res[2] = 3;

    for (i = 0; i < NUM_OF_MOTORS; i++) {
        g_mem.pos_lim_inf[i] = 0;
        g_mem.pos_lim_sup[i] = (int32)19000 << g_mem.res[0];
    }

    for(i = 0; i < NUM_OF_SENSORS; ++i)
    {
        g_mem.m_mult[i] = 1;
        g_mem.m_off[i] = (int32)0 << g_mem.res[i];
    }

    g_mem.max_step_pos = 0;
    g_mem.max_step_neg = 0;

    g_mem.current_limit = DEFAULT_CURRENT_LIMIT;

    // EMG calibration enabled by default
    g_mem.emg_calibration_flag = 0;

    g_mem.emg_max_value[0] = 0;
    g_mem.emg_max_value[1] = 0;

    g_mem.emg_threshold[0] = 100;
    g_mem.emg_threshold[1] = 100;

    g_mem.emg_speed = 100;

    g_mem.double_encoder_on_off = 1;
    g_mem.motor_handle_ratio = 22;

    // set the initialized flag to show EEPROM has been populated
    g_mem.flag = TRUE;
    
    //write that configuration to EEPROM
    return ( memStore(0) && memStore(DEFAULT_EEPROM_DISPLACEMENT) );
}

//==============================================================================
//                                                    ROUTINE INTERRUPT FUNCTION
//==============================================================================
/**
* Bunch of functions used on request from UART communication
**/

void cmd_get_measurements(){
 
    uint8 CYDATA index;
   
    // Packet: header + measure(int16) + crc
    
    uint8 packet_data[8]; 
    
    //Header package
    packet_data[0] = CMD_GET_MEASUREMENTS;   
    
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 1) + 1]) = (int16)(g_measOld.pos[index] >> g_mem.res[index]);
            
    // Calculate Checksum and send message to UART 

    packet_data[7] = LCRChecksum (packet_data, 7);

    commWrite(packet_data, 8);
   
}

void cmd_get_measurements_from_SH(){
 
//    ((int8 *) &curr_pos)[0] = (int8)g_rx.buffer[2];
//    ((int8 *) &curr_pos)[1] = (int8)g_rx.buffer[1];
    g_meas.hand_meas = *((int16 *) &g_rx.buffer[1]);
   
}

void cmd_set_inputs(){
    
    // Store position setted in right variables

    if(g_mem.control_mode == CONTROL_CURRENT) {
        g_refNew.curr[0] = *((int16 *) &g_rx.buffer[1]);
        g_refNew.curr[1] = *((int16 *) &g_rx.buffer[3]);
    }
    else {
        if(g_mem.control_mode == CONTROL_PWM) {
            g_refNew.pwm[0] = *((int16 *) &g_rx.buffer[1]);
            g_refNew.pwm[1] = *((int16 *) &g_rx.buffer[3]);
        }
        else {
            g_refNew.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1
            g_refNew.pos[0] = g_refNew.pos[0] << g_mem.res[0];

            g_refNew.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
            g_refNew.pos[1] = g_refNew.pos[1] << g_mem.res[1];
        }
    }

    // Check Position Limit cmd

    if (c_mem.pos_lim_flag && 
        (g_mem.control_mode == CURR_AND_POS_CONTROL
        || g_mem.control_mode == CONTROL_ANGLE)) {                      // pos limiting
        
        if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_inf[0];
        if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_inf[1];

        if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) 
            g_refNew.pos[0] = c_mem.pos_lim_sup[0];
        if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) 
            g_refNew.pos[1] = c_mem.pos_lim_sup[1];
    }
}

void cmd_activate(){
    
    // Store new value reads
    g_refNew.onoff = g_rx.buffer[1];
    
    // Check type of control mode enabled
    if ((g_mem.control_mode == CONTROL_ANGLE) || (g_mem.control_mode == CURR_AND_POS_CONTROL)) {
        g_refNew.pos[0] = g_meas.pos[0];
        g_refNew.pos[1] = g_meas.pos[1];
    }

    // Activate/Disactivate motors
    MOTOR_ON_OFF_Write(g_refNew.onoff);
}

void cmd_get_activate(){
    
    uint8 packet_data[3];

    // Header        
    packet_data[0] = CMD_GET_ACTIVATE;
    
    // Fill payload
    packet_data[1] = g_ref.onoff;
    
    // Calculate checksum
    packet_data[2] = LCRChecksum(packet_data, 2);
    
    // Send package to UART
    commWrite(packet_data, 3);

}

void cmd_get_curr_and_meas(){
    
    uint8 CYDATA index;
   
    //Packet: header + curr_meas(int16) + pos_meas(int16) + CRC
    
    uint8 packet_data[12]; 

    //Header package
    packet_data[0] = CMD_GET_CURR_AND_MEAS;
    
    // Currents
    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) filter_curr_diff(((int32) g_measOld.curr[0] - curr_estim(g_measOld.pos[0],g_measOld.vel[0], g_measOld.acc[0])));

    // Positions
    for (index = NUM_OF_SENSORS; index--;) 
        *((int16 *) &packet_data[(index << 2) + 5]) = (int16) (g_measOld.pos[index] >> g_mem.res[index]);
        
    // Calculate Checksum and send message to UART 
        
    packet_data[11] = LCRChecksum (packet_data, 11);
    commWrite(packet_data, 12);
   
}

void cmd_get_currents(){
    
    // Packet: header + motor_measure(int16) + crc
    
    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_CURRENTS;

    *((int16 *) &packet_data[1]) = (int16) g_measOld.curr[0];
    *((int16 *) &packet_data[3]) = (int16) filter_curr_diff(((int32) g_measOld.curr[0] - curr_estim(g_measOld.pos[0],g_measOld.vel[0], g_measOld.acc[0])));

    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6);
}

void cmd_set_baudrate(){
    
    // Set BaudRate
    c_mem.baud_rate = g_rx.buffer[1];
    
    switch(g_rx.buffer[1]){
        case 13:
            CLOCK_UART_SetDividerValue(13);
            break;
        default:
            CLOCK_UART_SetDividerValue(3);
    }
}

void cmd_ping(){

    uint8 packet_data[2];

    // Header
    packet_data[0] = CMD_PING;
    
    // Load Payload
    packet_data[1] = CMD_PING;

    // Send Package to uart
    commWrite(packet_data, 2);
}

void cmd_set_watchdog(){
      
    if (g_rx.buffer[1] <= 0){
        // Deactivate Watchdog
        WATCHDOG_ENABLER_Write(1); 
        g_mem.watchdog_period = 0;   
    }
    else{
        // Activate Watchdog        
        if (g_rx.buffer[1] > MAX_WATCHDOG_TIMER)
            g_rx.buffer[1] = MAX_WATCHDOG_TIMER;
            
        // Period * Time_CLK = WDT
        // Period = WTD / Time_CLK =     (WTD    )  / ( ( 1 / Freq_CLK ) )
        // Set request watchdog period - (WTD * 2)  * (250 / 1024        )
        g_mem.watchdog_period = (uint8) (((uint32) g_rx.buffer[1] * 2 * 250 ) >> 10);   
        WATCHDOG_COUNTER_WritePeriod(g_mem.watchdog_period); 
        WATCHDOG_ENABLER_Write(0); 
    }
}

void cmd_get_inputs(){

    // Packet: header + motor_measure(int16) + crc

    uint8 packet_data[6]; 
    
    //Header package

    packet_data[0] = CMD_GET_INPUTS;
    
    *((int16 *) &packet_data[1]) = (int16) (g_refOld.pos[0]  >> g_mem.res[0]);
    *((int16 *) &packet_data[3]) = (int16) (g_refOld.pos[1]  >> g_mem.res[1]);
    
    // Calculate Checksum and send message to UART 

    packet_data[5] = LCRChecksum(packet_data, 5);

    commWrite(packet_data, 6);
}

void cmd_store_params(){
    
    // Check input mode enabled
    uint32 off_1, off_2;
    float mult_1, mult_2;
    uint8 CYDATA packet_lenght = 2;
    uint8 CYDATA packet_data[2];
    uint8 CYDATA old_id;
    
    if( c_mem.input_mode == INPUT_MODE_EXTERNAL ) {
        off_1 = c_mem.m_off[0];
        off_2 = c_mem.m_off[1];
        mult_1 = c_mem.m_mult[0];
        mult_2 = c_mem.m_mult[1];

        g_refNew.pos[0] = (int32)((float)g_refNew.pos[0] / mult_1);
        g_refNew.pos[1] = (int32)((float)g_refNew.pos[1] / mult_2);

        g_refNew.pos[0] = (int32)((float)g_refNew.pos[0] * g_mem.m_mult[0]);
        g_refNew.pos[1] = (int32)((float)g_refNew.pos[1] * g_mem.m_mult[1]);

        g_refNew.pos[0] += (g_mem.m_off[0] - off_1);
        g_refNew.pos[1] += (g_mem.m_off[1] - off_2);
        
        // Check position Limits

        if (c_mem.pos_lim_flag) {                   // position limiting
            if (g_refNew.pos[0] < c_mem.pos_lim_inf[0]) 
                g_refNew.pos[0] = c_mem.pos_lim_inf[0];
            if (g_refNew.pos[1] < c_mem.pos_lim_inf[1]) 
                g_refNew.pos[1] = c_mem.pos_lim_inf[1];

            if (g_refNew.pos[0] > c_mem.pos_lim_sup[0]) 
                g_refNew.pos[0] = c_mem.pos_lim_sup[0];
            if (g_refNew.pos[1] > c_mem.pos_lim_sup[1]) 
                g_refNew.pos[1] = c_mem.pos_lim_sup[1];
        }
    }
    // Store params 

    if (c_mem.id != g_mem.id) {     //If a new id is going to be set we will lose communication 
        old_id = c_mem.id;          //after the memstore(0) and the ACK won't be recognised
        if(memStore(0)) {
            packet_data[0] = ACK_OK;
            packet_data[1] = ACK_OK;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }    
        else{
            packet_data[0] = ACK_ERROR;
            packet_data[1] = ACK_ERROR;
            commWrite_old_id(packet_data, packet_lenght, old_id);
        }
    }    
    else {
        if(memStore(0))
            sendAcknowledgment(ACK_OK);
        else
            sendAcknowledgment(ACK_ERROR);
    }
}

void cmd_get_emg(){
    
    uint8 packet_data[6];

    // Header        
    packet_data[0] = CMD_GET_EMG;
    
    *((int16 *) &packet_data[1]) = (int16) g_measOld.emg[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.emg[1];

    packet_data[5] = LCRChecksum (packet_data, 5);

    commWrite(packet_data, 6);

}

/* [] END OF FILE */