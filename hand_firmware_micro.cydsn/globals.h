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
* \file         globals.h
*
* \brief        Global definitions and macros are set in this file.
* \date         Feb 14th, 2023
* \author       _Centro "E.Piaggio"_
* \copyright    (C) 2012-2016 qbrobotics. All rights reserved.
* \copyright    (C) 2017-2023 Centro "E.Piaggio". All rights reserved.
*/

#ifndef GLOBALS_H_INCLUDED
#define GLOBALS_H_INCLUDED
// -----------------------------------------------------------------------------

//=================================================================     includes
#include <device.h>
#include "STDLIB.H"
#include "MATH.H"
#include "commands.h"

//==============================================================================
//                                                                        DEVICE
//==============================================================================

#define VERSION                 "SoftHand v6.3 - Master w/Vibrotactile fb Socket [TOAST]"

#define NUM_OF_MOTORS           2       /*!< Number of motors.*/
#define NUM_OF_SENSORS          3       /*!< Number of encoders.*/
#define NUM_OF_EMGS             2       /*!< Number of emg channels.*/
#define NUM_OF_ANALOG_INPUTS    4       /*!< Total number of analogic inputs.*/
#define NUM_OF_PARAMS           32      /*!< Number of parameters saved in the EEPROM.*/

//==============================================================================
//                                                               SYNCHRONIZATION
//==============================================================================

//Main frequency 1000 Hz
#define CALIBRATION_DIV         10      /*!< Frequency divisor for hand calibration (100Hz).*/

#define DIV_INIT_VALUE          1       /*!< Initial value for hand counter calibration.*/

//==============================================================================
//                                                                           DMA
//==============================================================================
    
#define DMA_BYTES_PER_BURST 2
#define DMA_REQUEST_PER_BURST 1
#define DMA_SRC_BASE (CYDEV_PERIPH_BASE)
#define DMA_DST_BASE (CYDEV_SRAM_BASE)
    
//==============================================================================
//                                                                     INTERRUPT
//==============================================================================

#define    WAIT_START   0               /*!< Package start waiting status.*/
#define    WAIT_ID      1               /*!< Package ID waiting status.*/
#define    WAIT_LENGTH  2               /*!< Package lenght waiting status.*/
#define    RECEIVE      3               /*!< Package data receiving status.*/
#define    UNLOAD       4               /*!< Package data flush status.*/

//==============================================================================
//                                                                         OTHER
//==============================================================================

#define FALSE                   0
#define TRUE                    1

#define DEFAULT_EEPROM_DISPLACEMENT 50   /*!< Number of pages occupied by the EEPROM.*/

#define MAX_WATCHDOG_TIMER      250     /*!< num * 2 [cs] */

#define PWM_MAX_VALUE           100     /*!< Maximum value of the PWM signal.*/

#define ANTI_WINDUP             1000    /*!< Anti windup saturation.*/ 
#define DEFAULT_CURRENT_LIMIT   1000    /*!< Default Current limit, 0 stands for unlimited.*/

#define CURRENT_HYSTERESIS      10      /*!< milliAmperes of hysteresis for current control.*/

#define EMG_SAMPLE_TO_DISCARD   500     /*!< Number of sample to discard before calibration.*/
#define SAMPLES_FOR_MEAN        100     /*!< Number of samples used to mean current values.*/

#define SAMPLES_FOR_EMG_MEAN    1000    /*!< Number of samples used to mean emg values.*/
#define REST_POS_ERR_THR_GAIN   10      /*!< Gain related to stop condition threshold in rest position routine.*/
#define SAMPLES_FOR_JOYSTICK_MEAN   200
#define JOYSTICK_SAMPLE_TO_DISCARD  100
    
#define CALIB_DECIMATION        1
#define NUM_OF_CLOSURES         5

#define POS_INTEGRAL_SAT_LIMIT  50000000    /*!< Anti windup on position control.*/
#define CURR_INTEGRAL_SAT_LIMIT 100000      /*!< Anti windup on current control.*/

#define MIN_CURR_SAT_LIMIT      30

#define N_IMU_SH                2
//==============================================================================
//                                                        structures definitions
//==============================================================================

//=========================================================     motor references
/** \brief Motor Reference structure
 * 
**/
struct st_ref {
    int32 pos[NUM_OF_MOTORS];       /*!< Motor position reference.*/
};

//=============================================================     measurements

struct st_meas {
    int32 pos[NUM_OF_SENSORS];      /*!< Encoder sensor position.*/
    int8 rot[NUM_OF_SENSORS];       /*!< Encoder sensor rotations.*/

    int32 emg[NUM_OF_EMGS];         /*!< EMG sensors values.*/
    int32 vel[NUM_OF_SENSORS];      /*!< Encoder rotational velocity.*/
    int32 acc[NUM_OF_SENSORS];      /*!< Encoder rotational acceleration.*/
    int16 joystick[NUM_OF_MOTORS];  /*!< Joystick measurements.*/
};

//==============================================================     data packet
/** \brief Data sent/received structure
 *
**/
struct st_data {
    uint8   buffer[128];            /*!< Data buffer [CMD | DATA | CHECKSUM].*/
    int16   length;                 /*!< Data buffer length.*/
    int16   ind;                    /*!< Data buffer index.*/
    uint8   ready;                  /*!< Data buffer flag to see if the data is ready.*/
};

//============================================     settings stored on the memory

struct st_mem {
    uint8   flag;                       /*!< If checked the device has been configured.*/                   //1
    uint8   id;                         /*!< Device id.*/                                                   //1

    uint8   input_mode;                 /*!< Motor Input mode.*/                                            //1

    uint8   res[NUM_OF_SENSORS];        /*!< Angle resolution.*/                                            //3
    int32   m_off[NUM_OF_SENSORS];      /*!< Measurement offset.*/                                          //12
    float   m_mult[NUM_OF_SENSORS];     /*!< Measurement multiplier.*/                                      //12 30

    uint8   pos_lim_flag;               /*!< Position limit active/inactive.*/                              //1
    int32   pos_lim_inf[NUM_OF_MOTORS]; /*!< Inferior position limit for motors.*/                          //8
    int32   pos_lim_sup[NUM_OF_MOTORS]; /*!< Superior position limit for motors.*/                          //8

    int32   max_step_pos;               /*!< Maximum number of steps per cycle for positive positions.*/    //4
    int32   max_step_neg;               /*!< Maximum number of steps per cycle for negative positions.*/    //4 25

    uint16  emg_threshold[NUM_OF_EMGS]; /*!< Minimum value for activation of EMG control.*/                 //4

    uint8   emg_calibration_flag;       /*!< Enable emg calibration on startup.*/                           //1
    uint32  emg_max_value[NUM_OF_EMGS]; /*!< Maximum value for EMG.*/                                       //8

    uint8   emg_speed;                  /*!< Maximum closure speed when using EMG.*/                        //1
    
    int8    motor_handle_ratio;         /*!< Discrete multiplier for handle device.*/                       //1 
    
    uint8   baud_rate;                  /*!< Baud Rate set.*/                                               //1
    
    uint8   rest_position_flag;         /*!< Enable rest position feature.*/                                //1    
    int32   rest_pos;                   /*!< Hand rest position while in EMG mode.*/                        //4
    int32   rest_delay;                 /*!< Hand rest position delay while in EMG mode.*/                  //4
    int32   rest_vel;                   /*!< Hand velocity closure for rest position reaching.*/            //4
    
    uint8   is_force_fb_present;        /*!< Flag to know if a force feedback device is present */          // 1
    uint8   is_proprio_fb_present;      /*!< Flag to know if a proprioceptive feedback device is present */ // 1
    uint8   is_vibrotactile_fb_present; /*!< Flag to know if a vibrotactile feedback device is present */   // 1
    uint8   is_myo2_master;             /*!< Flag to know if Myoelectric case 2 is present */               // 1
    
    // Force feedback device parameters
    float   curr_prop_gain;             // 4
    int16   curr_sat;                   // 2
    int16   curr_dead_zone;             // 2  
    
    // Proprioceptive device parameters
    int32   max_slide;                  // 4
    int32   max_SH_pos;                 // 4
    
    // Devices IDs
    uint8   SH_ID;                      // 1
    uint8   ForceF_ID;                  // 1
    uint8   ProprioF_ID;                // 1
    uint8   F_right_left;               // 1

    uint16  joystick_closure_speed;     // Joystick - Hand closure speed            2 
    uint16  joystick_gain;              // Joystick measurements gain               4
                                                                //TOT           179 bytes
};

//==============================================     hand calibration parameters
/** \brief Hand calibration structure
 *
**/ 
struct st_calib {
    uint8   enabled;                /*!< Calibration enabling flag.*/
    uint8   direction;              /*!< Direction of motor winding.*/
    int16   speed;                  /*!< Speed of hand opening/closing.*/
    int16   repetitions;            /*!< Number of cycles of hand closing/opening.*/
};

//=================================================     emg status
typedef enum {

    NORMAL        = 0,              /*!< Normal execution.*/
    RESET         = 1,              /*!< Reset analog measurements.*/
    DISCARD       = 2,              /*!< Discard first samples to obtain a correct value.*/
    SUM_AND_MEAN  = 3,              /*!< Sum and mean a definite value of samples.*/
    WAIT          = 4,               /*!< The second emg waits until the first emg has a valid value.*/
	WAIT_EoC      = 5               /*!< The second emg waits for end of calibration.*/

} emg_status, joystick_status;                       /*!< EMG and Joystick status enumeration.*/

//====================================      external global variables definition

extern struct st_ref    g_ref, g_refNew, g_refOld;  /*!< Reference variables.*/
extern struct st_meas   g_meas, g_measOld;          /*!< Measurements.*/
extern struct st_data   g_rx;                       /*!< Incoming/Outcoming data.*/
extern struct st_mem    g_mem, c_mem;               /*!< Memory parameters.*/
extern struct st_calib  calib;

extern uint32 timer_value;                          /*!< End time of the firmware main loop.*/
extern uint32 timer_value0;                         /*!< Start time of the firmware main loop*/

// Device Data

extern int32   dev_tension;                         /*!< Power supply tension.*/
extern int32   dev_tension_f;                       /*!< Filtered power supply tension.*/
extern int32   pow_tension;

// Bit Flag

extern CYBIT reset_last_value_flag;                 /*!< This flag is set when the encoders last values must be resetted.*/
extern CYBIT tension_valid;                         /*!< Tension validation bit.*/
extern CYBIT interrupt_flag;                        /*!< Interrupt flag enabler.*/

// DMA Buffer

extern int16 ADC_buf[4];                            /*! ADC measurements buffer.*/

extern uint8 master_mode;               /*!< Flag used to set/unset master mode to send messages to other boards.*/
extern uint32 count_tension_valid;
extern uint8 first_tension_valid;

extern uint8 rest_enabled;				/*!< Rest position flag.*/
extern uint8 forced_open;               /*!< Forced open flag (used in position with rest position control).*/
extern int32 rest_pos_curr_ref;			/*!< Rest position current reference.*/
extern int32 SH_current_position;       /*!< SoftHand current position (used with proprioception devices).*/
extern int32 curr_pos_res;

extern float imu_values[3*N_IMU_SH];

// -----------------------------------------------------------------------------


#endif

//[] END OF FILE