// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/**
* \file         globals.c
*
* \brief        Global variables.
* \date         June 06, 2016
* \author       _qbrobotics_
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref, g_refNew, g_refOld;  // motor variables
struct st_meas  g_meas, g_measOld;          // measurements
struct st_data  g_rx;                       // income data
struct st_mem   g_mem, c_mem;               // memory
struct st_calib calib;

float tau_feedback;

// Timer value for debug field

uint32 timer_value;
uint32 timer_value0;

// Device Data

int32   dev_tension;                // Power supply tension
uint8   dev_pwm_limit;

// Bit Flag

CYBIT reset_last_value_flag;
CYBIT tension_valid;
CYBIT interrupt_flag;
CYBIT watchdog_flag;

// DMA Buffer

int16 ADC_buf[4]; 

// PWM value
int8 pwm_sign;

// Mater mode
uint8 master_mode;

/* END OF FILE */