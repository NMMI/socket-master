// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/**
* \file         utils.c
*
* \brief        Definition of utility functions.
* \date         June 06, 2016
* \author       _qbrobotics_
* \copyright    (C)  qbrobotics. All rights reserved.
*/

#include <utils.h>
#include <math.h>

//==============================================================================
//                                                            Current Estimation
//==============================================================================

int32 curr_estim (int32 pos, int32 vel, int32 acc) {
    return (abs(g_mem.curr_lookup[0]*pos +
                g_mem.curr_lookup[1]*pow(pos,2) + 
                g_mem.curr_lookup[2]*pow(pos,3) + 
                g_mem.curr_lookup[3]*vel + 
                g_mem.curr_lookup[4]*pow(vel,2) + 
                g_mem.curr_lookup[5]*acc));
}

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

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

    return aux;
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
//                                                     Current difference filter
//==============================================================================

int32 filter_curr_diff(int32 curr_diff) {

    static int32 old_out, aux, old_input;

    aux = (old_out * (1024 - 2) + (curr_diff + old_input) * (0.8)) / 1024;

    old_out = aux;
    old_input = curr_diff;

    return aux;
}

//==============================================================================
//                                                             Second Emg Filter
//==============================================================================

int32 filter_ch2(int32 new_value) {

    static int32 old_value, aux;

    aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

    old_value = aux;

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
    static uint8 direction;                 //0 closing, 1 opening
    static uint16 closure_counter;          //Range [0 - 2^16]


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
#define N1 28           ///< Teeth of the first encoder wheel
#define N2 27           ///< Teeth of the second encoder wheel

#define I1 1            ///< First wheel invariant value
#define I2 (-1)         ///< Second wheel invariant value

// Number of ticks per turn
#define M 65536          ///< Number of encoder ticks per turn


int calc_turns_fcn(const int32 pos1, const int32 pos2) {
    
    int32 x = (my_mod( - N2*pos2 - N1*pos1, M*N2) + M/2) / M;

    int32 aux = my_mod(x*I1, N2);
    
    return (my_mod(aux + N2/2, N2) - N2/2);
}

/* [] END OF FILE */