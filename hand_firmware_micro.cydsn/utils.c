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
* \file         utils.c
*
* \brief        Definition of utility functions.
* \date         October 01, 2017
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017 Centro "E.Piaggio". All rights reserved.
*/

#include <utils.h>
#include <math.h>

//==============================================================================
//                                                   Voltage and current filters
//==============================================================================

int32 filter_v(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + (new_value << 6) * (ALPHA)) >> 10;

    old_value = aux;

    return (aux >> 6);
}

int32 filter_i1(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - ALPHA) + (new_value << 6) * (ALPHA)) >> 10;

    old_value = aux;

    return (aux >> 6);
}

//==============================================================================
//                                                              First Emg Filter
//==============================================================================

int32 filter_ch1(int32 new_value) {

   static int32 old_value, aux;    
     
    if (new_value < 0)
        new_value = 0;

    aux = (old_value * (1024 - BETA) + (new_value << 6) * (BETA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                             Second Emg Filter
//==============================================================================

int32 filter_ch2(int32 new_value) {

    static int32 old_value, aux;
    
    if (new_value < 0)
        new_value = 0;

    aux = (old_value * (1024 - BETA) + (new_value << 6) * (BETA)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                              Velocity filters
//==============================================================================

int32 filter_vel_1(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - GAMMA) + new_value * (GAMMA)) / 1024;

    old_out = aux;

    return aux;
}

int32 filter_vel_2(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - GAMMA) + new_value * (GAMMA)) / 1024;

    old_out = aux;

    return aux;
}

int32 filter_vel_3(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - GAMMA) + new_value * (GAMMA)) / 1024;

    old_out = aux;

    return aux;
}

//==============================================================================
//                                                          Acceleration filters
//==============================================================================

int32 filter_acc_1(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - DELTA) + new_value * (DELTA)) / 1024;

    old_out = aux;

    return aux;
}

int32 filter_acc_2(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - DELTA) + new_value * (DELTA)) / 1024;

    old_out = aux;

    return aux;
}

int32 filter_acc_3(int32 new_value) {

    static int32 old_out, aux;

    aux = (old_out * (1024 - DELTA) + new_value * (DELTA)) / 1024;

    old_out = aux;

    return aux;
}

//==============================================================================
//                                                        Voltage reading filter
//==============================================================================

int32 filter_voltage(int32 new_value) {

    static int32 old_value = 12000;
    static int32 aux;

    aux = (old_value * (1024 - EPSILON) + (new_value << 6) * (EPSILON)) /1024;

    old_value = aux;

    return (aux /64);
}

//==============================================================================
//                                                                CHECK ENC DATA
//==============================================================================

// Returns 1 if the encoder data is correct, 0 otherwise

CYBIT check_enc_data(const uint32 *value) {

    const uint8* CYIDATA p = (const uint8*)value;
    uint8 CYDATA a = *p;

    a = a ^ *(++p);
    a = a ^ *(++p);
    a = a ^ *(++p);
    a = (a & 0x0F) ^ (a>>4);

    return (0x9669 >> a) & 0x01;
    //0x9669 is a bit vector representing the !(bitwise XOR) of 4bits
}

//==============================================================================
//                                                                ROUND_FUNCTION
//==============================================================================

int my_round(const double x) {

    if (x < 0.0)
        return (int)(x - 0.5);
    else
        return (int)(x + 0.5);
}

//==============================================================================
//                                                                        MODULE
//==============================================================================

uint32 my_mod(int32 val, int32 divisor) {

    if (val >= 0) {
        return (int32)(val % divisor);
    } else {
        return (int32)(divisor - (-val % divisor));
    }
}


//==============================================================================
//                                                                     CALIBRATE
//==============================================================================

void calibration(void) {
    static uint8 direction;                 //0 closing, 1 opening.
    static uint16 closure_counter;          //Range [0 - 2^16].


    // closing
    if (direction == 0) {
        g_ref.pos[0] += (calib.speed << g_mem.res[0]);
        if ((g_ref.pos[0]) > g_mem.pos_lim_sup[0]) {
            direction = 1;
        }
    } else { //opening
        g_ref.pos[0] -= (calib.speed << g_mem.res[0]);
        if (SIGN(g_ref.pos[0]) != 1) {
            direction = 0;
            closure_counter++;
            if (closure_counter == calib.repetitions) {
                closure_counter = 0;
                calib.enabled = FALSE;
            }
        }
    }
}


//==============================================================================
//                                                      DOUBLE ENCODER CALC TURN
//==============================================================================

// Use this matlab function to calculate I1 and I2
//
// function [inv_a, inv_b] = mod_mul_inv(a, b)
//     a = int32(a);
//     b = int32(b);
//     if b == 0
//         inv_a = 1;
//         inv_b = 0;
//         return
//     else
//         q = idivide(a,b,'floor');
//         r = mod(a,b);
//         [s, t] = mod_mul_inv(b, r);
//     end
//     inv_a = t;
//     inv_b = s - q * t;
// return

// Number of teeth of the two wheels
#define N1 15           ///< Teeth of the first encoder wheel.
#define N2 14           ///< Teeth of the second encoder wheel.

#define I1 1            ///< First wheel invariant value.
#define I2 (-1)         ///< Second wheel invariant value.

// Number of ticks per turn
#define M 65536          ///< Number of encoder ticks per turn.


int calc_turns_fcn(const int32 pos1, const int32 pos2) {
    
    int32 x = (my_mod( - N2*pos2 - N1*pos1, M*N2) + M/2) / M;

    int32 aux = my_mod(x*I1, N2);
    
    return (my_mod(aux + N2/2, N2) - N2/2);
}


//==============================================================================
//                                                                 REST POSITION
//==============================================================================

void check_rest_position(void) {     // 100 Hz frequency.
    
    static uint32 count = 0;        // Range [0 - 2^31].
    static uint8 flag_count = 1;
    static uint8 first_time = 1;
    static float m = 0;
    static int32 abs_err_thr = 0;
    static int32 delta_inc = 0;
    int32 curr_pos = 0;
    static int32 rest_error;
    float rest_vel_ticks_ms = ((float)g_mem.rest_vel)/1000.0;    //[ticks/s] -> [ticks/ms]
    
    if (first_time){
        count = 0;
        first_time = 0;
    }
    
    curr_pos = (int32)(curr_pos_res >> g_mem.res[0]);
    
    if ( ( (c_mem.input_mode >= 2 && c_mem.input_mode <= 5 && g_meas.emg[0] < 200 && g_meas.emg[1] < 200) || 
           (c_mem.input_mode == INPUT_MODE_JOYSTICK && g_meas.joystick[0] < 50 && g_meas.joystick[0] > -50)) && curr_pos < 10000){
        if (flag_count == 1){
            count = count + 1;
        }
    }
    else {
        count = 0;
        rest_enabled = 0;
        flag_count = 1;
    }
    
    /********** Velocity closure procedure *************
    * m = error/time
    * m = (g_mem.rest_pos - init_pos)/time
    * time = g_mem.rest_pos/rest_vel_ticks_ms (rest_vel_ticks_ms is in [ticks/ms])
    ***************************************************/
    
    if (count == (uint32)(g_mem.rest_delay/CALIBRATION_DIV)){ 
        // This routine is executed every 10 firmware cycles -> g_mem.rest_delay must be major than 10 ms
        rest_enabled = 1;
        rest_pos_curr_ref = curr_pos_res;
        
        // Ramp angular coefficient
        m = ((float)(g_mem.rest_pos - curr_pos_res)/((float)g_mem.rest_pos))*(rest_vel_ticks_ms);
        
        // Stop condition threshold is related to m coefficient by REST_RATIO value
        abs_err_thr = (int32)( (int32)(((float)REST_POS_ERR_THR_GAIN)*m*((float)CALIBRATION_DIV)) << g_mem.res[0]);
        abs_err_thr = (abs_err_thr>0)?abs_err_thr:(0-abs_err_thr);
        
        rest_error = g_mem.rest_pos - curr_pos_res;    
        
        delta_inc = (int32)( ((int32)(m*(float)CALIBRATION_DIV)) << g_mem.res[0] );
        delta_inc = (delta_inc>0)?delta_inc:(0-delta_inc);
        
        count = 0;
        flag_count = 0;
    }
    
    if (rest_enabled){

        if (rest_error < abs_err_thr && rest_error > -abs_err_thr){
            // Stop condition
            rest_pos_curr_ref = g_mem.rest_pos;
            
            if (c_mem.input_mode >= 2)   // EMG input mode
                forced_open = 1; 
            
            count = 0;
        }
        else {
            rest_error = g_mem.rest_pos - curr_pos_res;        

            if (rest_error > 0){
                rest_pos_curr_ref = rest_pos_curr_ref + delta_inc;
            }
            if (rest_error < 0){
                rest_pos_curr_ref = rest_pos_curr_ref - delta_inc;
            }
        } 
    }
    
    // Position limit saturation
    if (c_mem.pos_lim_flag) {
        if (rest_pos_curr_ref < c_mem.pos_lim_inf[0]) 
            rest_pos_curr_ref = c_mem.pos_lim_inf[0];
        if (rest_pos_curr_ref > c_mem.pos_lim_sup[0]) 
            rest_pos_curr_ref = c_mem.pos_lim_sup[0];
    }

}


/* [] END OF FILE */