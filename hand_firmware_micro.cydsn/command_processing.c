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
            
//=========================================================     CMD_GET_JOYSTICK

        case CMD_GET_JOYSTICK:
            cmd_get_joystick();
            break;

//=========================================================     CMD_GET_EMG

        case CMD_GET_EMG:
            cmd_get_emg();
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

//=========================================================== ALL OTHER COMMANDS
        default:
            break;

    }
}


//==============================================================================
//                                                                     INFO SEND
//==============================================================================

void infoSend(void){
    unsigned char packet_string[1500];
    infoPrepare(packet_string);
    UART_RS485_PutString(packet_string);
}


//==============================================================================
//                                                              COMMAND GET INFO
//==============================================================================

void infoGet(uint16 info_type) {
    unsigned char packet_string[1700] = "";

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
    uint8 packet_data[2051] = "";
    uint16 packet_lenght = 2051;

    //Auxiliary variables
    uint8 CYDATA i;
    uint8 CYDATA string_lenght;
    int32 aux_int;
    uint8 aux_uchar;

    //Parameters menu string definitions
    char id_str[15] = "1 - Device ID:";
    char input_str[34] = "2 - Input mode:";
    char res_str[17] = "3 - Resolutions:";
    char m_off_str[25] = "4 - Measurement Offsets:";
    char mult_str[17] = "5 - Multipliers:";
    char pos_lim_flag_str[27] = "6 - Pos. limit active:";
    char pos_lim_str[28] = "7 - Pos. limits [inf, sup]:";
    char max_step_str[26] = "8 - Max steps [neg, pos]:";
    char emg_flag_str[36] = "9 - EMG calibration on startup:";
    char emg_thr_str[21] = "10 - EMG thresholds:";
    char emg_max_val_str[21] = "11 - EMG max values:";
    char emg_max_speed_str[20] = "12 - EMG max speed:";
    char handle_ratio_str[25] = "13 - Motor handle ratio:"; 
	char joystick_closure_speed_str[29] = "14 - Joystick closure speed:";
    char joystick_gains_str[20] = "15 - Joystick gain:";
	char myo2_master_str[36] = "16 - Myoelectric case 2 Master:";
    char rest_pos_flag_str[29] = "17 - Rest position flag:";
    char rest_pos_str[20] = "18 - Rest position:";
    char rest_pos_delay_str[40] = "19 - Rest position time delay (>10 ms):";
    char rest_vel_str[35] = "20 - Rest vel closure (ticks/sec):";
    char curr_prop_gain_str[40] = "21 - Current proportional gain (force):";
    char curr_sat_str[44]       = "22 - Current difference saturation (force):";
    char curr_dead_zone_str[32] = "23 - Current dead zone (force):";
    char max_slide_str[35]      = "24 - Max slide movement (proprio):";
    char max_SH_pos_str[37]     = "25 - Max SoftHand closure (proprio):";
    char master_mode_force_str[35] = "26 - Master with Force device:";
    char master_mode_proprio_str[39] = "27 - Master with Proprioc. device:";
    char SH_ID_str[18]          = "28 - SoftHand ID:";
    char FF_ID_str[22]          = "29 - Force device ID:";
    char PF_ID_str[31]          = "30 - Proprioceptive device ID:";
    char F_right_left_par_str[21] = "31 - Arm side:";

    //Parameters menus
    char input_mode_menu[100] = "0 -> Usb\n1 -> Handle\n2 -> EMG prop.\n3 -> EMG Integ.\n4 -> EMG FCFS\n5 -> EMG FCFS Adv.\n6 -> Joystick\n";
    char F_right_left_menu[22] = "0 -> Right\n1 -> Left\n";
    char yes_no_menu[42] = "0 -> Deactivate [NO]\n1 -> Activate [YES]\n";


    //Strings lenghts
    uint8 CYDATA id_str_len = strlen(id_str);
    uint8 CYDATA res_str_len = strlen(res_str);
    uint8 CYDATA m_off_str_len = strlen(m_off_str);
    uint8 CYDATA mult_str_len = strlen(mult_str);
    uint8 CYDATA pos_lim_str_len = strlen(pos_lim_str);
    uint8 CYDATA max_step_str_len = strlen(max_step_str);
    uint8 CYDATA emg_thr_str_len = strlen(emg_thr_str);
    uint8 CYDATA emg_max_val_str_len = strlen(emg_max_val_str);
    uint8 CYDATA emg_max_speed_str_len = strlen(emg_max_speed_str);
    uint8 CYDATA handle_ratio_str_len = strlen(handle_ratio_str);
    uint8 CYDATA joystick_closure_speed_str_len = strlen(joystick_closure_speed_str);
    uint8 CYDATA joystick_gains_str_len = strlen(joystick_gains_str); 
    uint8 CYDATA rest_pos_str_len = strlen(rest_pos_str);
    uint8 CYDATA rest_pos_delay_str_len = strlen(rest_pos_delay_str);
    uint8 CYDATA rest_vel_str_len = strlen(rest_vel_str);   
    uint8 CYDATA curr_prop_gain_str_len = strlen(curr_prop_gain_str);
    uint8 CYDATA curr_sat_str_len = strlen(curr_sat_str);
    uint8 CYDATA curr_dead_zone_str_len = strlen(curr_dead_zone_str);
    uint8 CYDATA max_slide_str_len = strlen(max_slide_str);
    uint8 CYDATA max_SH_pos_str_len = strlen(max_SH_pos_str);
    uint8 CYDATA SH_ID_str_len = strlen(SH_ID_str);
    uint8 CYDATA FF_ID_str_len = strlen(FF_ID_str);
    uint8 CYDATA PF_ID_str_len = strlen(PF_ID_str);
    
    uint8 CYDATA input_mode_menu_len = strlen(input_mode_menu);
    uint8 CYDATA F_right_left_menu_len = strlen(F_right_left_menu);
    uint8 CYDATA yes_no_menu_len = strlen(yes_no_menu);
    
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

            /*--------------INPUT MODE------------*/
            
            packet_data[52] = TYPE_FLAG;
            packet_data[53] = 1;
            packet_data[54] = c_mem.input_mode;
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
                case INPUT_MODE_JOYSTICK:
                    strcat(input_str, " Joystick\0");
                    string_lenght = 25;
                break;                    
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[55 + string_lenght - i] = input_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[55 + string_lenght] = 1;
            
            /*-------------RESOLUTIONS------------*/
            
            packet_data[102] = TYPE_UINT8;
            packet_data[103] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                packet_data[i + 104] = c_mem.res[i];
            for(i = res_str_len; i != 0; i--)
                packet_data[107 + res_str_len - i] = res_str[res_str_len - i];
            
            /*----------MEASUREMENT OFFSET--------*/
            
            packet_data[152] = TYPE_INT16;
            packet_data[153] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++) 
                *((int16 *) ( packet_data + 154 + (i * 2) )) = (int16) (c_mem.m_off[i] >> c_mem.res[i]);
            for(i = m_off_str_len; i != 0; i--)
                packet_data[160 + m_off_str_len - i] = m_off_str[m_off_str_len - i];
            
            /*------------MULTIPLIERS-------------*/
            
            packet_data[202] = TYPE_FLOAT;
            packet_data[203] = 3;
            for(i = 0; i < NUM_OF_SENSORS; i++)
                *((float *) ( packet_data + 204 + (i * 4) )) = c_mem.m_mult[i];
            for(i = mult_str_len; i != 0; i--)
                packet_data[216 + mult_str_len - i] = mult_str[mult_str_len - i];

            /*-----------POS LIMIT FLAG-----------*/
            
            packet_data[252] = TYPE_FLAG;
            packet_data[253] = 1;
            packet_data[254] = c_mem.pos_lim_flag;
            if(c_mem.pos_lim_flag) {
                strcat(pos_lim_flag_str, " YES\0");
                string_lenght = 27;
            }
            else {
                strcat(pos_lim_flag_str, " NO\0");
                string_lenght = 26;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[255 + string_lenght - i] = pos_lim_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[255 + string_lenght] = 3;
            
            /*-----------POSITION LIMITS----------*/
            
            packet_data[302] = TYPE_INT32;
            packet_data[303] = 2;
            for (i = 0; i < NUM_OF_MOTORS - 1; i++) {
                *((int32 *)( packet_data + 304 + (i * 2 * 4) )) = (c_mem.pos_lim_inf[i] >> c_mem.res[i]);
                *((int32 *)( packet_data + 304 + (i * 2 * 4) + 4)) = (c_mem.pos_lim_sup[i] >> c_mem.res[i]);
            }
            for(i = pos_lim_str_len; i != 0; i--)
                packet_data[312 + pos_lim_str_len - i] = pos_lim_str[pos_lim_str_len - i];

            /*--------------MAX STEPS-------------*/
            
            packet_data[352] = TYPE_INT32;
            packet_data[353] = 2;
            *((int32 *)(packet_data + 354)) = c_mem.max_step_neg;
            *((int32 *)(packet_data + 358)) = c_mem.max_step_pos;
            for(i = max_step_str_len; i != 0; i--)
                packet_data[362 + max_step_str_len - i] = max_step_str[max_step_str_len - i];

            /*-----------EMG CALIB FLAG-----------*/            
            
            packet_data[402] = TYPE_FLAG;
            packet_data[403] = 1;
            packet_data[404] = c_mem.emg_calibration_flag;
            if(c_mem.emg_calibration_flag) {
                strcat(emg_flag_str, " YES\0");
                string_lenght = 36;
            }
            else {
                strcat(emg_flag_str, " NO\0");
                string_lenght = 35;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[405 + string_lenght - i] = emg_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[405 + string_lenght] = 3;
            
            /*-----------EMG THRESHOLDS-----------*/
        
            packet_data[452] = TYPE_UINT16;
            packet_data[453] = 2;
            *((uint16 *)(packet_data + 454)) = c_mem.emg_threshold[0];
            *((uint16 *)(packet_data + 456)) = c_mem.emg_threshold[1];
            for(i = emg_thr_str_len; i != 0; i--)
                packet_data[458 + emg_thr_str_len - i] = emg_thr_str[emg_thr_str_len - i];
            
            /*------------EMG MAX VALUE-----------*/
            
            packet_data[502] = TYPE_UINT32;
            packet_data[503] = 2;
            *((uint32 *)(packet_data + 504)) = c_mem.emg_max_value[0];
            *((uint32 *)(packet_data + 508)) = c_mem.emg_max_value[1];
            for(i = emg_max_val_str_len; i != 0; i--)
                packet_data[512 + emg_max_val_str_len - i] = emg_max_val_str[emg_max_val_str_len - i];

            /*--------------EMG SPEED-------------*/
            
            packet_data[552] = TYPE_UINT8;
            packet_data[553] = 1;
            packet_data[554] = c_mem.emg_speed;
            for(i = emg_max_speed_str_len; i != 0; i--)
                packet_data[555 + emg_max_speed_str_len - i] = emg_max_speed_str[emg_max_speed_str_len - i];
            
            /*-------------HANDLE RATIO-----------*/
            
            packet_data[602] = TYPE_INT8;
            packet_data[603] = 1;
            *((int8 *)(packet_data + 604)) = c_mem.motor_handle_ratio;
            for(i = handle_ratio_str_len; i != 0; i--)
                packet_data[605 + handle_ratio_str_len - i] = handle_ratio_str[handle_ratio_str_len - i];
                           
            /*-------------JOYSTICK CLOSURE SPEED------------*/

            packet_data[652] = TYPE_UINT16;
            packet_data[653] = 1;
            *((uint16 *) (packet_data + 654)) = c_mem.joystick_closure_speed;
            for(i = joystick_closure_speed_str_len; i != 0; i--)
                packet_data[656 + joystick_closure_speed_str_len - i] = joystick_closure_speed_str[joystick_closure_speed_str_len - i]; 

            /*------------JOYSTICK GAIN------------*/
                
            packet_data[702] = TYPE_UINT16;
            packet_data[703] = 1;
            *((uint16 *) (packet_data + 704)) = c_mem.joystick_gain;
            for(i = joystick_gains_str_len; i != 0; i--)
                packet_data[706 + joystick_gains_str_len - i] = joystick_gains_str[joystick_gains_str_len - i];
                                       
            /*------------MASTER MODE MYO2----------*/
            
            packet_data[752] = TYPE_FLAG;
            packet_data[753] = 1;
            packet_data[754] = c_mem.is_myo2_master;
            if(c_mem.is_myo2_master) {
                strcat(myo2_master_str, " YES\0");
                string_lenght = 36;
            }
            else {
                strcat(myo2_master_str, " NO\0");
                string_lenght = 35;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[755 + string_lenght - i] = myo2_master_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[755 + string_lenght] = 3;
            
            /*-----------REST POSITION FLAG-----------*/            
            
            packet_data[802] = TYPE_FLAG;
            packet_data[803] = 1;
            packet_data[804] = c_mem.rest_position_flag;
            if(c_mem.rest_position_flag) {
                strcat(rest_pos_flag_str, " YES\0");
                string_lenght = 29;
            }
            else {
                strcat(rest_pos_flag_str, " NO\0");
                string_lenght = 28;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[805 + string_lenght - i] = rest_pos_flag_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[805 + string_lenght] = 3;
            
            /*-----------REST POSITION----------*/
            
            packet_data[852] = TYPE_INT32;
            packet_data[853] = 1;
            *((int32 *)( packet_data + 854 )) = (c_mem.rest_pos >> c_mem.res[0]);
            for(i = rest_pos_str_len; i != 0; i--)
                packet_data[858 + rest_pos_str_len - i] = rest_pos_str[rest_pos_str_len - i];
                
            /*-----------REST POSITION TIME DELAY----------*/
            
            packet_data[902] = TYPE_INT32;
            packet_data[903] = 1;
            *((int32 *)( packet_data + 904 )) = c_mem.rest_delay;
            for(i = rest_pos_delay_str_len; i != 0; i--)
                packet_data[908 + rest_pos_delay_str_len - i] = rest_pos_delay_str[rest_pos_delay_str_len - i];
                
            /*-----------REST POSITION VELOCITY----------*/
            
            packet_data[952] = TYPE_INT32;
            packet_data[953] = 1;
            *((int32 *)( packet_data + 954 )) = c_mem.rest_vel;
            for(i = rest_vel_str_len; i != 0; i--)
                packet_data[958 + rest_vel_str_len - i] = rest_vel_str[rest_vel_str_len - i]; 
                
            /*-----CURRENT PROPORTIONAL GAIN-----*/

            packet_data[1002] = TYPE_FLOAT;
            packet_data[1003] = 1;
            *((float *)(packet_data + 1004)) = c_mem.curr_prop_gain;
            for(i = curr_prop_gain_str_len; i!= 0; i--)
                packet_data[1008 + curr_prop_gain_str_len - i] = curr_prop_gain_str[curr_prop_gain_str_len - i];

            /*---------CURRENT SATURATION--------*/

            packet_data[1052] = TYPE_INT16;
            packet_data[1053] = 1;
            *((int16 *)(packet_data + 1054)) = c_mem.curr_sat;
            for(i = curr_sat_str_len; i!= 0; i--)
                packet_data[1056 + curr_sat_str_len - i] = curr_sat_str[curr_sat_str_len - i];

            /*---------CURRENT DEAD ZONE---------*/

            packet_data[1102] = TYPE_INT16;
            packet_data[1103] = 1;
            *((int16 *)(packet_data + 1104)) = c_mem.curr_dead_zone;
            for(i = curr_dead_zone_str_len; i!= 0; i--)
                packet_data[1106 + curr_dead_zone_str_len - i] = curr_dead_zone_str[curr_dead_zone_str_len - i];
                
            /*---------MAX SLIDE---------*/

            packet_data[1152] = TYPE_INT32;
            packet_data[1153] = 1;
            *((int32 *)(packet_data + 1154)) = c_mem.max_slide;
            for(i = max_slide_str_len; i!= 0; i--)
                packet_data[1158 + max_slide_str_len - i] = max_slide_str[max_slide_str_len - i];
                
            /*---------MAX SH POS---------*/

            packet_data[1202] = TYPE_INT32;
            packet_data[1203] = 1;
            *((int32 *)(packet_data + 1204)) = c_mem.max_SH_pos;
            for(i = max_SH_pos_str_len; i!= 0; i--)
                packet_data[1208 + max_SH_pos_str_len - i] = max_SH_pos_str[max_SH_pos_str_len - i];
            
            /*------------MASTER MODE FORCE----------*/
            
            packet_data[1252] = TYPE_FLAG;
            packet_data[1253] = 1;
            packet_data[1254] = c_mem.is_force_fb_present;
            if(c_mem.is_force_fb_present) {
                strcat(master_mode_force_str, " YES\0");
                string_lenght = 35;
            }
            else {
                strcat(master_mode_force_str, " NO\0");
                string_lenght = 34;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[1255 + string_lenght - i] = master_mode_force_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[1255 + string_lenght] = 3;
            
            /*------------MASTER MODE PROPRIO----------*/
            
            packet_data[1302] = TYPE_FLAG;
            packet_data[1303] = 1;
            packet_data[1304] = c_mem.is_proprio_fb_present;
            if(c_mem.is_proprio_fb_present) {
                strcat(master_mode_proprio_str, " YES\0");
                string_lenght = 39;
            }
            else {
                strcat(master_mode_proprio_str, " NO\0");
                string_lenght = 38;
            }
            for(i = string_lenght; i != 0; i--)
                packet_data[1305 + string_lenght - i] = master_mode_proprio_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to send
            packet_data[1305 + string_lenght] = 3;
           
            /*---------HAND ID---------*/

            packet_data[1352] = TYPE_UINT8;
            packet_data[1353] = 1;
            *((uint8 *)(packet_data + 1354)) = c_mem.SH_ID;
            for(i = SH_ID_str_len; i!= 0; i--)
                packet_data[1355 + SH_ID_str_len - i] = SH_ID_str[SH_ID_str_len - i];
                
            /*---------FORCE FEEDBACK DEVICE ID---------*/

            packet_data[1402] = TYPE_UINT8;
            packet_data[1403] = 1;
            *((uint8 *)(packet_data + 1404)) = c_mem.ForceF_ID;
            for(i = FF_ID_str_len; i!= 0; i--)
                packet_data[1405 + FF_ID_str_len - i] = FF_ID_str[FF_ID_str_len - i];
                
            /*---------PROPRIOCEPTIVE FEEDBACK DEVICE ID---------*/

            packet_data[1452] = TYPE_UINT8;
            packet_data[1453] = 1;
            *((uint8 *)(packet_data + 1454)) = c_mem.ProprioF_ID;
            for(i = PF_ID_str_len; i!= 0; i--)
                packet_data[1455 + PF_ID_str_len - i] = PF_ID_str[PF_ID_str_len - i];                

            /*--------RIGHT LEFT-------*/

            packet_data[1502] = TYPE_FLAG;
            packet_data[1503] = 1;
            packet_data[1504] = c_mem.F_right_left;
            if(c_mem.F_right_left) {
                strcat(F_right_left_par_str, " Left\0");
                string_lenght = 20;
            }
            else {
                strcat(F_right_left_par_str, " Right\0");
                string_lenght = 21;
            }
            for(i = string_lenght; i!=0; i--)
                packet_data[1505 + string_lenght - i] = F_right_left_par_str[string_lenght - i];
            //The following byte indicates the number of menus at the end of the packet to sen
            packet_data[1505 + string_lenght] = 2;

            /*------------PARAMETERS MENU-----------*/

            for(i = input_mode_menu_len; i != 0; i--)
                packet_data[1552 + input_mode_menu_len - i] = input_mode_menu[input_mode_menu_len - i];

            for(i = F_right_left_menu_len; i!= 0; i--)
                packet_data[1702 + F_right_left_menu_len - i] = F_right_left_menu[F_right_left_menu_len - i];

            for(i = yes_no_menu_len; i!= 0; i--)
                packet_data[1852 + yes_no_menu_len - i] = yes_no_menu[yes_no_menu_len - i];
            
            packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
            commWrite(packet_data, packet_lenght);
        break;

//===================================================================     set_id
        case 1:         //ID - uint8
            g_mem.id = g_rx.buffer[3];
        break;

//===========================================================     set_input_mode        
        case 2:         //Input mode - uint8
            g_mem.input_mode = *((uint8*) &g_rx.buffer[3]);
        break;
        
//===========================================================     set_resolution
        case 3:         //Resolution - uint8[3]
            for (i =0; i < NUM_OF_SENSORS; i++) {
                g_mem.res[i] = g_rx.buffer[i+3];
            }
        break;
        
//===============================================================     set_offset
        case 4:         //Measurement Offset - int32[3] 
            for(i = 0; i < NUM_OF_SENSORS; ++i) {
                g_mem.m_off[i] = *((int16 *) &g_rx.buffer[3 + i * 2]);
                g_mem.m_off[i] = g_mem.m_off[i] << g_mem.res[i];

                g_meas.rot[i] = 0;
            }
            reset_last_value_flag = 1;
        break;
        
//===========================================================     set_multiplier
        case 5:         //Multipliers - float[3]
            for(i = 0; i < NUM_OF_SENSORS; ++i)
                g_mem.m_mult[i] = *((float *) &g_rx.buffer[3 + i * 4]);
        break;
        
//=====================================================     set_pos_limit_enable
        case 6:        //Position limit flag - uint8
            g_mem.pos_lim_flag = *((uint8 *) &g_rx.buffer[3]);
        break;

//============================================================     set_pos_limit
        case 7:        //Position limits - int32[4]
            for (i = 0; i < NUM_OF_MOTORS; i++) {
                g_mem.pos_lim_inf[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4)]);
                g_mem.pos_lim_sup[i] = *((int32 *) &g_rx.buffer[3 + (i * 2 * 4) + 4]);

                g_mem.pos_lim_inf[i] = g_mem.pos_lim_inf[i] << g_mem.res[i];
                g_mem.pos_lim_sup[i] = g_mem.pos_lim_sup[i] << g_mem.res[i];
            }
        break;

//==================================================     set_max_steps_per_cycle
        case 8:        //Max steps - int32[2]
            aux_int = *((int32 *) &g_rx.buffer[3]);
            if (aux_int <= 0)
                g_mem.max_step_neg = aux_int;

            aux_int = *((int32 *) &g_rx.buffer[3 + 4]);
            if (aux_int >= 0) 
                g_mem.max_step_pos = aux_int;
            
        break;
        
//=======================================================     set_emg_calib_flag
        case 9:        //Emg calibration flag - int8
            g_mem.emg_calibration_flag = *((uint8*) &g_rx.buffer[3]);
        break;
        
//========================================================     set_emg_threshold
        case 10:        //Emg threshold - uint16[2]
            g_mem.emg_threshold[0] = *((uint16*) &g_rx.buffer[3]);
            g_mem.emg_threshold[1] = *((uint16*) &g_rx.buffer[5]);
        break;
        
//========================================================     set_emg_max_value
        case 11:        //Emg max value - uint32[2]
            g_mem.emg_max_value[0] = *((uint32*) &g_rx.buffer[3]);
            g_mem.emg_max_value[1] = *((uint32*) &g_rx.buffer[7]);
        break;
        
//============================================================     set_emg_speed
        case 12:        //Emg max speed - uint8
            g_mem.emg_speed = *((uint8*) &g_rx.buffer[3]);
        break;
        
//===================================================     set_motor_handle_ratio
        case 13:        //Motor handle ratio - int8
            g_mem.motor_handle_ratio = *((int8*) &g_rx.buffer[3]);
        break;
//===================================================     set_joystick_closure_speed
        case 14:
            g_mem.joystick_closure_speed = *((uint16 *) &g_rx.buffer[3]);
        break;
//===================================================     set_joystick_gain
        case 15:
            g_mem.joystick_gain = *((uint16 *) &g_rx.buffer[3]);
        break;         
//================================================     set_myo2_master
        case 16:        //Is Myo2 master present - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.is_myo2_master = 1;
            } else {
                g_mem.is_myo2_master = 0;
            }
        break;  
//=======================================================     set_rest_position_flag
        case 17:        //Rest position flag - int8
            g_mem.rest_position_flag = *((uint8*) &g_rx.buffer[3]);
        break; 
//============================================================     set_rest_pos
        case 18:        //Rest Position - int32
            g_mem.rest_pos = *((int32 *) &g_rx.buffer[3]);
            g_mem.rest_pos = g_mem.rest_pos << g_mem.res[0];
        break;   
//============================================================     set_rest_delay_pos
        case 19:        //Rest Position Time Delay - int32
            g_mem.rest_delay = *((int32 *) &g_rx.buffer[3]);
            if (g_mem.rest_delay < 10) g_mem.rest_delay = 10;
        break;   
//============================================================     set_rest_vel
        case 20:        //Rest Position Velocity - int32
            g_mem.rest_vel = *((int32 *) &g_rx.buffer[3]);
        break; 
//=======================================================     set_curr_prop_gain
        case 21:
            g_mem.curr_prop_gain = *((float*) &g_rx.buffer[3]);
        break;
//=============================================================     set_curr_sat
        case 22: 
            g_mem.curr_sat = *((int16*) &g_rx.buffer[3]);
        break;
//=======================================================     set_curr_dead_zone
        case 23:
            g_mem.curr_dead_zone = *((int16*) &g_rx.buffer[3]);
        break;            
//=============================================================     set_max_slide
        case 24: 
            g_mem.max_slide = *((int32*) &g_rx.buffer[3]);
        break;            
//=============================================================     set_max_SH_pos
        case 25: 
            g_mem.max_SH_pos = *((int32*) &g_rx.buffer[3]);
        break;              
//================================================     set_master_mode_force
        case 26:        //Is force feedback present - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.is_force_fb_present = 1;
            } else {
                g_mem.is_force_fb_present = 0;
            }
        break;
//================================================     set_master_mode_proprio
        case 27:        //Is proprio feedback present - uint8
            aux_uchar = *((uint8*) &g_rx.buffer[3]);
            if (aux_uchar) {
                g_mem.is_proprio_fb_present = 1;
            } else {
                g_mem.is_proprio_fb_present = 0;
            }
        break;    
//=============================================================     set_SH_ID
        case 28: 
            g_mem.SH_ID = *((uint8*) &g_rx.buffer[3]);
        break;   
//=============================================================     set_FF_ID
        case 29: 
            g_mem.ForceF_ID = *((uint8*) &g_rx.buffer[3]);
        break; 
//=============================================================     set_PF_ID
        case 30: 
            g_mem.ProprioF_ID = *((uint8*) &g_rx.buffer[3]);
        break;
//=============================================================     set_F_right_left
        case 31:
            g_mem.F_right_left = *((uint8*) & g_rx.buffer[3]);
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

        strcat(info_string, "\r\nDEVICE PARAMETERS\r\n");

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
            case INPUT_MODE_JOYSTICK:
                strcat(info_string, "Input mode: Joystick\r\n");
                break;
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

        sprintf(str, "Joystick closure speed: %d", c_mem.joystick_closure_speed);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        sprintf(str, "Joystick gain: %hu", c_mem.joystick_gain);
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
        
        sprintf(str, "Rest time delay (ms): %d", (int)g_mem.rest_delay);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Rest velocity closure (ticks/sec): %d", (int)g_mem.rest_vel);
        strcat(info_string, str);
        strcat(info_string, "\r\n");
        
        sprintf(str, "Rest position: %d", (int)(g_mem.rest_pos >> c_mem.res[0]));
        strcat(info_string, str);
        strcat(info_string, "\r\n");   
        
        if (g_mem.is_myo2_master) {
            strcat(info_string, "Myoelectric case 2 Master: YES\r\n");
        } else {
            strcat(info_string, "Myoelectric case 2 Master: NO\r\n");
        }
        
        if (g_mem.is_force_fb_present) {
            strcat(info_string, "Master with Force device: YES\r\n");
        } else {
            strcat(info_string, "Master with Force device: NO\r\n");
        }
        
        if (g_mem.is_proprio_fb_present) {
            strcat(info_string, "Master with Proprioceptive device: YES\r\n");
        } else {
            strcat(info_string, "Master with Proprioceptive device: NO\r\n");
        }

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
//                                      READ MEASUREMENTS FUNCTION FROM SOFTHAND
//==============================================================================

int32 commReadMeasFromSH()
{
    uint8 packet_data[10];
    uint8 packet_lenght;
    int32 aux_val = 0;
    uint32 t_start, t_end;

    packet_lenght = 2;
    packet_data[0] = CMD_GET_MEASUREMENTS;
    packet_data[1] = CMD_GET_MEASUREMENTS;
    commWriteID(packet_data, packet_lenght, c_mem.SH_ID);

    t_start = (uint32) MY_TIMER_ReadCounter();
    while(g_rx.buffer[0] != CMD_GET_MEASUREMENTS) {
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }

        t_end = (uint32) MY_TIMER_ReadCounter();
        if((t_start - t_end) > 4500000){            // 4.5 s timeout
            master_mode = 0;                        // exit from master mode
            break;
        }
    }

    if (master_mode) {
        aux_val = *((int16 *) &g_rx.buffer[1]);

        aux_val = (aux_val << g_mem.res[0]);       //g_mem.res[0] (SoftHand sensor resolution)
    }
    
    return aux_val;
}


//==============================================================================
//                                  READ RESIDUAL CURRENT FUNCTION FROM SOFTHAND
//==============================================================================

int16 commReadResCurrFromSH()
{
    uint8 packet_data[16];
    uint8 packet_lenght;
    int16 curr_diff = 0;
    uint32 t_start, t_end;
    int32 aux_val = 0;

    packet_lenght = 2;
    packet_data[0] = CMD_GET_CURR_AND_MEAS;
    packet_data[1] = CMD_GET_CURR_AND_MEAS;
    commWriteID(packet_data, packet_lenght, c_mem.SH_ID);

    t_start = (uint32) MY_TIMER_ReadCounter();
    while(g_rx.buffer[0] != CMD_GET_CURR_AND_MEAS) {
        if (interrupt_flag){
            interrupt_flag = FALSE;
            interrupt_manager();
        }

        t_end = (uint32) MY_TIMER_ReadCounter();
        if((t_start - t_end) > 10000000){            // 4.5 s timeout
            master_mode = 0;
            break;
        }
    }

    if (master_mode) {
        curr_diff = *((int16 *) &g_rx.buffer[3]);
        
        aux_val = *((int16 *) &g_rx.buffer[5]);
        SH_current_position = (aux_val << g_mem.res[0]);
    }
    
    return curr_diff;
}

//==============================================================================
//                                             WRITE FUNCTION FOR ANOTHER DEVICE
//==============================================================================

void commWriteID(uint8 *packet_data, uint16 packet_lenght, uint8 id)
{
    static uint16 CYDATA i;    // iterator

    // frame - start
    UART_RS485_PutChar(':');
    UART_RS485_PutChar(':');
    // frame - ID
    UART_RS485_PutChar(id);
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
//                                                                DRIVE SOFTHAND
//==============================================================================
 
void drive_SH() {
    uint8 packet_data[6];
    uint8 packet_lenght;
    
    // If not the use of handle or an emg input mode is set, exit from master_mode
    if( c_mem.input_mode != INPUT_MODE_ENCODER3          &&
		c_mem.input_mode != INPUT_MODE_JOYSTICK          &&
        c_mem.input_mode != INPUT_MODE_EMG_PROPORTIONAL  &&
        c_mem.input_mode != INPUT_MODE_EMG_INTEGRAL      &&
        c_mem.input_mode != INPUT_MODE_EMG_FCFS          &&
        c_mem.input_mode != INPUT_MODE_EMG_FCFS_ADV     ){
        master_mode = 0;
        return;
    }
        
    if (dev_tension < 7000){
        master_mode = 0;
        return;
    }
    
       
    //Sends a Set inputs command to a second board
    packet_data[0] = CMD_SET_INPUTS;
    *((int16 *) &packet_data[1]) = (int16) (g_ref.pos[0] >> g_mem.res[0]);
    *((int16 *) &packet_data[3]) = 0;
    packet_lenght = 6;
    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    commWriteID(packet_data, packet_lenght, c_mem.SH_ID);

}

//==============================================================================
//                                                   DRIVE FORCE FEEDBACK DEVICE
//==============================================================================
/* Function called when is_force_fb_present is set. It asks current difference to 
the SoftHand and sets force feedback device inputs proportionally to this difference.*/

void drive_force_fb() {
    CYDATA uint8 packet_data[6];    // output packet
    int16 curr_diff;
    int32 aux_val;
    int32 aux_p1, aux_p2;

    curr_diff = (int16)commReadResCurrFromSH();
    
    // Current difference saturation
    if(curr_diff > g_mem.curr_sat)
        curr_diff = g_mem.curr_sat;
    //Current difference dead zone
    if(curr_diff < g_mem.curr_dead_zone)
        curr_diff = 0;
    else
        curr_diff -= g_mem.curr_dead_zone;

    aux_val = ((curr_diff * g_mem.curr_prop_gain) * 65535 / 1440);

    aux_p1 = -(aux_val << g_mem.res[0]);
    aux_p2 =  (aux_val << g_mem.res[1]);
        
    // Send SET_CUFF_INPUTS to ForceF ID
    packet_data[0] = CMD_SET_INPUTS;
    *((int16 *) &packet_data[1]) = (int16) (aux_p1 >> g_mem.res[0]);
    *((int16 *) &packet_data[3]) = (int16) (aux_p2 >> g_mem.res[1]);
    packet_data[5] = LCRChecksum(packet_data, 5); 
    commWriteID(packet_data, 6, c_mem.ForceF_ID);         // To force feedback device ID
    
}

//==============================================================================
//                                          DRIVE PROPRIOCEPTIVE FEEDBACK DEVICE
//==============================================================================
/* Function called when is_proprio_fb_present is set. It asks position to 
the SoftHand and sets proprioceptive feedback device inputs proportionally to this value.*/
void drive_proprio_fb(){ 
    CYDATA uint8 packet_data[6];
    int32 aux_val;
    
    // Get SoftHand position
    SH_current_position = (int32)commReadMeasFromSH();
    
    aux_val = SH_current_position >> g_mem.res[0];
    aux_val = (int32)((aux_val * g_mem.max_slide) / g_mem.max_SH_pos);
    
    if (c_mem.F_right_left) {
        // LEFT
        aux_val = -aux_val;
    }
    
    // Send SoftHand position to ProprioF device
    packet_data[0] = CMD_SET_INPUTS;
    *((int16 *) &packet_data[1]) = (int16) (aux_val);
    *((int16 *) &packet_data[3]) = (int16) (aux_val);
    packet_data[5] = LCRChecksum(packet_data, 5); 
    commWriteID(packet_data, 6, c_mem.ProprioF_ID);         // To proprioceptive feedback device ID

}

//==============================================================================
//                               DRIVE FORCE  AND PROPRIOCEPTIVE FEEDBACK DEVICE
//==============================================================================
/* Function called when is_force_fb_present, is_proprio_fb_present is set and force feedback 
device ID si the same as proprioceptive device ID. It asks current difference and position to 
the SoftHand and sets force feedback device inputs proportionally to this difference.*/

void drive_force_proprio_fb() {
    CYDATA uint8 packet_data[6];    // output packet
    int16 curr_diff;
    int32 aux_val;
    int32 aux_p1, aux_p2;
    int32 ref_slide, ref_force;

    curr_diff = (int16)commReadResCurrFromSH();
    
    // Current difference saturation
    if(curr_diff > g_mem.curr_sat)
        curr_diff = g_mem.curr_sat;
    //Current difference dead zone
    if(curr_diff < g_mem.curr_dead_zone)
        curr_diff = 0;
    else
        curr_diff -= g_mem.curr_dead_zone;

    // Sliding     
    aux_val = SH_current_position >> g_mem.res[0];
    ref_slide = (int32)((aux_val * g_mem.max_slide) / g_mem.max_SH_pos);
        
    // Force
    ref_force = ((curr_diff * g_mem.curr_prop_gain) * 65535 / 1440);
    
    if (c_mem.F_right_left) {
        // LEFT
        aux_p1 = -ref_slide - ref_force;
        aux_p2 = -ref_slide + ref_force;        
    }
    else{
        // RIGHT
        aux_p1 = ref_slide - ref_force;
        aux_p2 = ref_slide + ref_force;
    }
    
    // Send INPUTS to ForceF ID
    packet_data[0] = CMD_SET_INPUTS;
    *((int16 *) &packet_data[1]) = (int16) (aux_p1);
    *((int16 *) &packet_data[3]) = (int16) (aux_p2);
    packet_data[5] = LCRChecksum(packet_data, 5); 
    commWriteID(packet_data, 6, c_mem.ForceF_ID);         // To force and proprio feedback device ID
    
}

//==============================================================================
//                                                             DEACTIVATE SLAVES
//==============================================================================
 
void deactivate_slaves() {
    uint8 packet_data[10];
    uint8 packet_lenght;
    
    // If not a emg input mode is set, exit from master_mode
    if(c_mem.input_mode != INPUT_MODE_EMG_PROPORTIONAL  &&
        c_mem.input_mode != INPUT_MODE_EMG_INTEGRAL      &&
        c_mem.input_mode != INPUT_MODE_EMG_FCFS          &&
        c_mem.input_mode != INPUT_MODE_EMG_FCFS_ADV     ){
        master_mode = 0;
        return;
    }
    
    //Sends a Set inputs command to a second board
    packet_data[0] = CMD_ACTIVATE;

    *((int16 *) &packet_data[1]) = 0;   //3 to activate, 0 to deactivate
    packet_lenght = 3;
    packet_data[packet_lenght - 1] = LCRChecksum(packet_data,packet_lenght - 1);
    
    commWrite(packet_data, packet_lenght);
    
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

    g_mem.input_mode    = INPUT_MODE_EXTERNAL;

    g_mem.pos_lim_flag = 1;

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

    g_mem.curr_prop_gain = 0;
    g_mem.curr_sat = 0;
    g_mem.curr_dead_zone = 0;
    
    g_mem.max_step_pos = 0;
    g_mem.max_step_neg = 0;

    // EMG calibration enabled by default
    g_mem.emg_calibration_flag = 0;

    g_mem.emg_max_value[0] = 1024;
    g_mem.emg_max_value[1] = 1024;

    g_mem.emg_threshold[0] = 100;
    g_mem.emg_threshold[1] = 100;

    g_mem.emg_speed = 100;

    g_mem.motor_handle_ratio = 22;
    g_mem.joystick_closure_speed = 150;
    g_mem.joystick_gain = 1024;
    
    //Initialize rest position parameters        
    g_mem.rest_position_flag = 1;
    g_mem.rest_pos = (int32)7000 << g_mem.res[0]; // 56000
    g_mem.rest_delay = 10;
    g_mem.rest_vel = 10000;
    
    g_mem.is_force_fb_present = 0;
    g_mem.is_proprio_fb_present = 0;
    g_mem.is_myo2_master = 1;
    
    g_mem.curr_prop_gain = 0.3;
    g_mem.curr_sat = 1000;
    g_mem.curr_dead_zone = 75;
    g_mem.max_slide = 10000;
    g_mem.max_SH_pos = (g_mem.pos_lim_sup[0] >> g_mem.res[0]);
    
    g_mem.SH_ID = 2;
    g_mem.ForceF_ID = 3;
    g_mem.ProprioF_ID = 4;
    g_mem.F_right_left = 0;       // RIGHT

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

void cmd_set_inputs(){
    
    // Store position setted in right variables

    g_refNew.pos[0] = *((int16 *) &g_rx.buffer[1]);   // motor 1
    g_refNew.pos[0] = g_refNew.pos[0] << g_mem.res[0];

    g_refNew.pos[1] = *((int16 *) &g_rx.buffer[3]);   // motor 2
    g_refNew.pos[1] = g_refNew.pos[1] << g_mem.res[1];

    // Check Position Limit cmd

    if (c_mem.pos_lim_flag) {                      // pos limiting
        
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
    
    // Check type of control mode enabled
    g_refNew.pos[0] = g_meas.pos[0];
    g_refNew.pos[1] = g_meas.pos[1];
}

void cmd_get_joystick() {

    uint8 packet_data[6];
    
    packet_data[0] = CMD_GET_JOYSTICK;

    *((int16 *) &packet_data[1]) = (int16) g_measOld.joystick[0];
    *((int16 *) &packet_data[3]) = (int16) g_measOld.joystick[1];

    packet_data[5] = LCRChecksum(packet_data, 5);

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