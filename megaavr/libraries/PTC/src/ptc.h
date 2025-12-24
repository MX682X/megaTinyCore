/*
  library to use the PTC module in AVR devices
  Copyright (c) 2023, MX682X

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  This license applies to all files that are part of this library
  (ptc.c, ptc.h, ptc_io.h)
*/



#pragma once
#ifndef PTC_H
#define PTC_H

#include <avr/io.h>

#if (defined(__AVR_ATtiny814__) || defined(__AVR_ATtiny1614__) || defined(__AVR_ATtiny3214__))
typedef uint8_t  ptc_id_t;
typedef uint8_t  ptc_ch_bm_t;
typedef uint8_t  ptc_ch_arr_t[1];
#define __PTC_Tiny__
#define __PTC_Pincount__ 6
#elif (defined(__AVR_ATtiny816__) || defined(__AVR_ATtiny1616__) || defined(__AVR_ATtiny3216__))
typedef uint8_t  ptc_id_t;
typedef uint16_t ptc_ch_bm_t;
typedef uint16_t ptc_ch_arr_t[1];
#define __PTC_Tiny__
#define __PTC_Pincount__ 12
#elif (defined(__AVR_ATtiny817__) || defined(__AVR_ATtiny1617__) || defined(__AVR_ATtiny3217__))
typedef uint8_t  ptc_id_t;
typedef uint16_t ptc_ch_bm_t;
typedef uint16_t ptc_ch_arr_t[1];
#define __PTC_Tiny__
#define __PTC_Pincount__ 14
#elif (defined(__AVR_AVR32DA28__) || defined(__AVR_AVR64DA28__) || defined(__AVR_AVR128DA28__))
typedef uint16_t ptc_id_t;
typedef uint64_t ptc_ch_bm_t;
typedef uint8_t  ptc_ch_arr_t[5];
#define __PTC_DA__
#define __PTC_Pincount__ 18
#elif (defined(__AVR_AVR32DA32__) || defined(__AVR_AVR64DA32__) || defined(__AVR_AVR128DA32__))
typedef uint16_t ptc_id_t;
typedef uint64_t ptc_ch_bm_t;
typedef uint8_t  ptc_ch_arr_t[5];
#define __PTC_DA__
#define __PTC_Pincount__ 22
#elif (defined(__AVR_AVR32DA48__) || defined(__AVR_AVR64DA48__) || defined(__AVR_AVR128DA48__))
typedef uint16_t ptc_id_t;
typedef uint64_t ptc_ch_bm_t;
typedef uint8_t  ptc_ch_arr_t[5];
#define __PTC_DA__
#define __PTC_Pincount__ 32
#elif (defined(__AVR_AVR32DA64__) || defined(__AVR_AVR64DA64__) || defined(__AVR_AVR128DA64__))
typedef uint16_t ptc_id_t;
typedef uint64_t ptc_ch_bm_t;
typedef uint8_t  ptc_ch_arr_t[6];
#define __PTC_DA__
#define __PTC_Pincount__ 46
#else
#error "PTC not supported by this chip"
#endif


typedef enum ptc_freq_enum {
  FREQ_SEL_0,
  FREQ_SEL_1,
  FREQ_SEL_2,
  FREQ_SEL_3,
  FREQ_SEL_4,
  FREQ_SEL_5,
  FREQ_SEL_6,
  FREQ_SEL_7,
  FREQ_SEL_8,
  FREQ_SEL_9,
  FREQ_SEL_10,
  FREQ_SEL_11,
  FREQ_SEL_12,
  FREQ_SEL_13,
  FREQ_SEL_14,
  FREQ_SEL_15,
  FREQ_SEL_SPREAD
} ptc_freq_t;


#if defined (__PTC_Tiny__)
typedef enum PTC_PRESC_enum {
  PTC_PRESC_DIV2_gc   = (0x00 << 0), /* CLK_PER divided by 2 */
  PTC_PRESC_DIV4_gc   = (0x01 << 0), /* CLK_PER divided by 4 */
  PTC_PRESC_DIV8_gc   = (0x02 << 0), /* CLK_PER divided by 8 */
  PTC_PRESC_DIV16_gc  = (0x03 << 0), /* CLK_PER divided by 16 */
  PTC_PRESC_DIV32_gc  = (0x04 << 0), /* CLK_PER divided by 32 */
  PTC_PRESC_DIV64_gc  = (0x05 << 0), /* CLK_PER divided by 64 */
  PTC_PRESC_DIV128_gc = (0x06 << 0), /* CLK_PER divided by 128 */
  PTC_PRESC_DIV256_gc = (0x07 << 0), /* CLK_PER divided by 256 */
} ptc_presc_t;

typedef enum PTC_RSEL_enum {
  RSEL_VAL_0,
  RSEL_VAL_20,
  RSEL_VAL_50,
  RSEL_VAL_70,
  RSEL_VAL_100,
  RSEL_VAL_200
} ptc_rsel_t;

// typedef enum ptc_gain_enum {
  // PTC_GAIN_1            = 0x00,
  // PTC_GAIN_2            = 0x23,
  // PTC_GAIN_4            = 0x34,
  // PTC_GAIN_8            = 0x3A,
  // PTC_GAIN_16           = 0x3C,
  // PTC_GAIN_32           = 0x3E,
  // PTC_GAIN_MAX          = 0x3F,
// } ptc_gain_t;
typedef enum ptc_gain_enum {
  PTC_GAIN_1,
  PTC_GAIN_2,
  PTC_GAIN_4,
  PTC_GAIN_8,
  PTC_GAIN_16,
  PTC_GAIN_32,
  PTC_GAIN_MAX,
} ptc_gain_t;

 const uint8_t ptc_again_lut[] = {
    0x3F, 0x1C, 0x0B, 0x05, 0x03, 0x01,
  };

#elif defined (__PTC_DA__)
typedef enum PTC_PRESC_enum {
  PTC_PRESC_DIV2_gc   = (0x00 << 0), /* CLK_PER divided by 2 */
  PTC_PRESC_DIV4_gc   = (0x01 << 0), /* CLK_PER divided by 4 */
  PTC_PRESC_DIV6_gc   = (0x02 << 0), /* CLK_PER divided by 2 */
  PTC_PRESC_DIV8_gc   = (0x03 << 0), /* CLK_PER divided by 8 */
  PTC_PRESC_DIV10_gc  = (0x04 << 0), /* CLK_PER divided by 8 */
  PTC_PRESC_DIV12_gc  = (0x05 << 0), /* CLK_PER divided by 8 */
  PTC_PRESC_DIV14_gc  = (0x06 << 0), /* CLK_PER divided by 8 */
  PTC_PRESC_DIV16_gc  = (0x07 << 0), /* CLK_PER divided by 16 */
} ptc_presc_t;

typedef enum tag_rsel_val_t {
  RSEL_VAL_0,
  RSEL_VAL_20,
  RSEL_VAL_50,
  RSEL_VAL_70,
  RSEL_VAL_80,
  RSEL_VAL_100,
  RSEL_VAL_120,
  RSEL_VAL_200
} ptc_rsel_t;

// typedef enum ptc_gain_enum {
  // PTC_GAIN_1            = 0x00,
  // PTC_GAIN_2            = 0x10,
  // PTC_GAIN_4            = 0x18,
  // PTC_GAIN_8            = 0x1C,
  // PTC_GAIN_16           = 0x1E,
  // PTC_GAIN_MAX          = 0x1F,
// } ptc_gain_t;

typedef enum ptc_gain_enum {
  PTC_GAIN_1,
  PTC_GAIN_2,
  PTC_GAIN_4,
  PTC_GAIN_8,
  PTC_GAIN_16,
  PTC_GAIN_MAX,
} ptc_gain_t;

const uint8_t ptc_again_lut []  = {
	 0x1F, 0x0F, 0x07, 0x03, 0x01
};
#endif

typedef struct ptc_node_state_type {
  uint8_t error: 1;
  uint8_t win_comp: 1;
  uint8_t low_power: 1;
  uint8_t data_ready: 1;
  uint8_t disabled: 1;
} ptc_node_state_t;


typedef enum ptc_sm_enum {
  PTC_SM_NOINIT_CAL     = 0x00,
  PTC_SM_RECAL_FLT      = 0x01,
  PTC_SM_NT_LOW_FLT     = 0x02,
  PTC_SM_NO_TOUCH       = 0x04,
  PTC_SM_NT_HIGH_FLT    = 0x08,
  PTC_SM_TOUCH_IN_FLT   = 0x10,
  PTC_SM_TOUCH_OUT_FLT  = 0x20,
  PTC_SM_TOUCH_DETECT   = 0x40,
  PTC_SM_LOW_POWER      = 0x80
} ptc_sm_t;

typedef enum ptc_ret_enum {
  // Normal return types
  PTC_LIB_SUCCESS         = 0x00,
  PTC_LIB_WAS_WAKEN,
  PTC_LIB_ASLEEP,
  PTC_LIB_CALIB_DONE,
  PTC_LIB_CALIB_TOO_LOW,
  PTC_LIB_CALIB_TOO_HIGH,
  PTC_LIB_CALIB_TOO_LONG,
  // Error Return types
  PTC_LIB_ERROR           = 0x10,
  PTC_LIB_BAD_POINTER,
  PTC_LIB_BAD_ARGUMENT,
  PTC_LIB_WRONG_STATE,
  // Node Return types
  PTC_NODE_WRONG_STATE    = 0x30,
  PTC_NODE_TOUCHED,
  PTC_NODE_NOT_TOUCHED
} ptc_ret_t;


typedef enum ptc_node_type_enum {
  NODE_TYPE_NOCONV_bm     = (0x00),
  NODE_MUTUAL_bm          = (0x01),
  NODE_RESERVED_bm        = (0x02),
  NODE_SELFCAP_bm         = (0x04),
  NODE_SHIELD_bm          = (0x08),
  NODE_SELFCAP_SHIELD_bm  = (NODE_SHIELD_bm | NODE_SELFCAP_bm),  // 0x0C
  NODE_TYPE_bm            = (NODE_MUTUAL_bm | NODE_SELFCAP_bm | NODE_SHIELD_bm),
} ptc_node_type_t;


typedef enum ptc_cb_event_enum {
  PTC_CB_EVENT_TOUCH              = 0x10,
  PTC_CB_EVENT_TOUCH_DETECT       = (PTC_CB_EVENT_TOUCH | 0x01),        // 0x11
  PTC_CB_EVENT_TOUCH_RELEASE      = (PTC_CB_EVENT_TOUCH | 0x02),        // 0x12
  PTC_CB_EVENT_TOUCH_WAKE         = (PTC_CB_EVENT_TOUCH | 0x04),        // 0x14
  PTC_CB_EVENT_WAKE_TOUCH         = (PTC_CB_EVENT_TOUCH_WAKE | 0x01),   // 0x15
  PTC_CB_EVENT_WAKE_NO_TOUCH      = (PTC_CB_EVENT_TOUCH_WAKE | 0x02),   // 0x16
  PTC_CB_EVENT_CONV_CMPL          = 0x20,
  PTC_CB_EVENT_CONV_MUTUAL_CMPL   = (PTC_CB_EVENT_CONV_CMPL | NODE_MUTUAL_bm),          // 0x21
  PTC_CB_EVENT_CONV_SELF_CMPL     = (PTC_CB_EVENT_CONV_CMPL | NODE_SELFCAP_bm),         // 0x24
  PTC_CB_EVENT_CONV_SHIELD_CMPL   = (PTC_CB_EVENT_CONV_CMPL | NODE_SELFCAP_SHIELD_bm),  // 0x28
  PTC_CB_EVENT_CONV_TYPE_CMPL_MSK = (NODE_TYPE_bm),                     // 0x0D
  PTC_CB_EVENT_CONV_CALIB         = 0x40,
  PTC_CB_EVENT_ERR_CALIB_LOW      = (PTC_CB_EVENT_CONV_CALIB | 0x01),   // 0x41
  PTC_CB_EVENT_ERR_CALIB_HIGH     = (PTC_CB_EVENT_CONV_CALIB | 0x02),   // 0x42
  PTC_CB_EVENT_ERR_CALIB_TO       = (PTC_CB_EVENT_CONV_CALIB | 0x04),   // 0x44
  PTC_CB_EVENT_ERR_CALIB_MSK      = (0x04 | 0x02 | 0x01),               // 0x07
  PTC_CB_EVENT_BEG_CALIB          = 0x80,
  PTC_CB_EVENT_BEG_CALIB_LOW      = (PTC_CB_EVENT_BEG_CALIB  | 0x01),   // 0x81	when we spend too long in touch state
  PTC_CB_EVENT_BEG_CALIB_FORCE    = (PTC_CB_EVENT_BEG_CALIB  | 0x02),   // 0x82	when the reference moved too far, try to move it back to the middle
  PTC_CB_EVENT_BEG_CALIB_HIGH	  = (PTC_CB_EVENT_BEG_CALIB  | 0x04),   // 0x84 when the out of touch measured value was too low
} ptc_cb_event_t;





typedef enum ptc_lib_enum {
  PTC_LIB_IDLE          = 0x00,
  PTC_LIB_CONV_PROG     = 0x01,
  PTC_LIB_CONV_COMPL    = 0x02,
  PTC_LIB_EVENT         = 0x04,
  PTC_LIB_CONV_WCMP_HT  = 0x08,
  PTC_LIB_CONV_WCMP_LT  = 0x10,
  PTC_LIB_SUSPENDED     = 0x80
} ptc_lib_t;

typedef struct cap_sensor_type {
  struct cap_sensor_type *nextNode;
  ptc_node_type_t   type;
  ptc_id_t  id;             // number for easier identification in the callback

  ptc_ch_arr_t hw_xCh_bm;   // the code relies on this bitmaps to be together,
  ptc_ch_arr_t hw_yCh_bm;   // do not separate them or change order.
  uint16_t hw_compCaps;      // [13:12] rough; [11:8] course; [7:4] fine; [3:0] accurate (on Tinies only)
  uint8_t  hw_rsel_presc;    // [7:4] RSEL, [3:0] PRESC
  uint8_t  hw_gain_ovs;      // [7:4] Analog Gain, [3:0] Oversampling  /* PTC_AGAIN / CTRLB.SAMPNUM */
  uint8_t  hw_csd;           // [4:0] Charge Share Delay /* SAMPLEN in SAMPCTRL */

  ptc_node_state_t state;
  uint16_t sensorData;      // ADC data, Oversampling-corrected
  uint16_t reference;       // Compare Value for detection
  int16_t  touch_in_th;     // this value is compared to the sensorData - threshold delta
  int16_t  touch_out_th;    // this value is compared to the sensorData - threshold delta

  uint8_t  stateMachine;
  uint8_t  lastStateChange;  // stateChangeCounter
} cap_sensor_t;


// Abbreviation: NoM: Number of Measurements
typedef struct ptc_lib_sm_settings_type {
  uint16_t force_recal_delta;     // if the threshold value exceeds this compared to optimal (512), force recalibration.  Default: 150
  uint8_t  touched_detect_nom;    // NoM above node threshold for the node to become touched.         Default: 3
  uint8_t  untouched_detect_nom;  // NoM below node threshold for the node to become untouched.       Default: 3
  uint8_t  touched_max_nom;       // NoM a touch was detected until a recalibration is forced.        Value +1. Disabled with 0xFF. Default: 200
  uint8_t  drift_up_nom;          // NoM when no touch is detected, until the threshold is increased. Value +1. Disabled with 0xFF. Default: 20
  uint8_t  drift_down_nom;        // NoM when no touch is detected, until the threshold is decreased. Value +1. Disabled with 0xFF. Default: 20
} ptc_lib_sm_set_t;


#if defined(MEGATINYCORE) || defined(DXCORE)
  #include <Arduino.h>
#else
  #include <avr/interrupt.h>
  #ifndef _fastPtr_d
    #define _fastPtr_d(_x_, _y_) _x_ = _y_;
  #endif
#endif

#include "ptc_io.h"


#ifndef NULL
  #define NULL  (void*)0
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define NODE_GAIN(_a_, _d_)      (uint8_t)(((_a_) << 4) | ((_d_) & 0x0F))
#define NODE_RSEL_PRSC(_r_, _p_) (uint8_t)(((_r_) << 4) | ((_p_) & 0x0F))
#define NUM_TO_BM(__n__)    (0x01 << __n__)






#define PTC_CHECK_FOR_BAD_POINTER(__p__)  \
  if (NULL == __p__) {                    \
    if (__builtin_constant_p(__p__)) {    \
      badArg("Null pointer detected");    \
    }                                     \
    return PTC_LIB_BAD_POINTER;           \
  }


void badArg(const char *)  __attribute__((weak, error("")));
void badCall(const char *) __attribute__((weak, error("")));




//extern void ptc_conversion_complete(uint8_t type);
//extern void ptc_error_callback(uint8_t source, cap_sensor_t *node);
extern void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t *node);

extern void ptc_event_cb_touch(const ptc_cb_event_t eventType, cap_sensor_t *node);
extern void ptc_event_cb_wake(const ptc_cb_event_t eventType, cap_sensor_t *node);
extern void ptc_event_cb_conversion(const ptc_cb_event_t eventType, cap_sensor_t *node);
extern void ptc_event_cb_calibration(const ptc_cb_event_t eventType, cap_sensor_t *node);
extern void ptc_event_cb_error(const ptc_cb_event_t eventType, cap_sensor_t *node);


// Enables the node. Can be called while an acquisition is in progress.
uint8_t ptc_enable_node(cap_sensor_t *node);

// Disables a node. If the conversion is started, it will be finished
uint8_t ptc_disable_node(cap_sensor_t *node);

// Can be used outside an acquisition process to select ADC/SELFCAP/MUTUAL/SHIELD
void ptc_set_next_conversion_type(ptc_node_type_t type);

// Main task handle for PTC. Handles State-Machine, drift, and calibration
void ptc_process(uint16_t currTime);


// Set the threshold for touch detection and away from touch for a node.
// a "0" will be interpreted as don't change
uint8_t ptc_node_set_thresholds(cap_sensor_t *node, int16_t th_in, int16_t th_out);


// Change Resistor Setting. Note: Only has an effect on mutual sensors
uint8_t ptc_node_set_resistor(cap_sensor_t *node, ptc_rsel_t res);

// Change prescaler.
uint8_t ptc_node_set_prescaler(cap_sensor_t *node, ptc_presc_t presc);

// Sets the gain through adjusting the charge integrator (increases the sensitivity (and noise))
uint8_t ptc_node_set_gain(cap_sensor_t *node, ptc_gain_t gain);

// Sets the number of oversamples. (the value is right-shifted automatically (reduces noise))
uint8_t ptc_node_set_oversamples(cap_sensor_t *node, uint8_t ovs);

// Sets the number of additional PTC Clocks for sampling a node. See also: ADC.SAMPCTRL
uint8_t ptc_node_set_charge_share_delay(cap_sensor_t *node, uint8_t csd);

// Resets all values to default values. Useful to recover from error states
uint8_t ptc_reset_node(cap_sensor_t *node);

// this is an internal function, there is no sense in calling it directly
uint8_t ptc_add_node(cap_sensor_t *node, uint8_t *pCh, const uint8_t type);


inline uint8_t ptc_add_selfcap_node(cap_sensor_t *node, const ptc_ch_bm_t xCh, const ptc_ch_bm_t yCh) {
  if (__builtin_constant_p(yCh)) {
    if (yCh == 0) {
      badArg("yCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    }
    if (yCh & xCh) {
      badArg("pin bitmap overlap detected");
    }
  }

  // this places only the significant number of bits on stack. Significantly reduces register pressure on DAs
  const uint8_t tsize = sizeof(ptc_ch_arr_t);
  uint8_t pCh[tsize * 2];

  pCh[0]         = (uint8_t)(xCh >>  0);
  pCh[0 + tsize] = (uint8_t)(yCh >>  0);

  #if __PTC_Pincount__ >= 8
  pCh[1]         = (uint8_t)(xCh >>  8);
  pCh[1 + tsize] = (uint8_t)(yCh >>  8);

  #if __PTC_Pincount__ >= 16
  pCh[2]         = (uint8_t)(xCh >> 16);
  pCh[2 + tsize] = (uint8_t)(yCh >> 16);
  pCh[3]         = (uint8_t)(xCh >> 24);
  pCh[3 + tsize] = (uint8_t)(yCh >> 24);
  pCh[4]         = (uint8_t)(xCh >> 32);
  pCh[4 + tsize] = (uint8_t)(yCh >> 32);

  #if __PTC_Pincount__ >=   40
  pCh[5]         = (uint8_t)(xCh >> 40);
  pCh[5 + tsize] = (uint8_t)(yCh >> 40);

  #endif
  #endif
  #endif

  return ptc_add_node(node, pCh, NODE_SELFCAP_bm);
};


inline uint8_t ptc_add_mutualcap_node(cap_sensor_t *node, const ptc_ch_bm_t xCh, const ptc_ch_bm_t yCh) {
  if (__builtin_constant_p(yCh) && __builtin_constant_p(xCh)) {
    if (yCh == 0) {
      badArg("yCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    }
    if (xCh == 0) {
      badArg("xCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    }
    if (yCh & xCh) {
      badArg("pin overlap detected");
    }
  }

  // this places only the significant number of bits on stack. Significantly reduces register pressure on DAs
  const uint8_t tsize = sizeof(ptc_ch_arr_t);
  uint8_t pCh[tsize * 2];

  pCh[0]         = (uint8_t)(xCh >>  0);
  pCh[0 + tsize] = (uint8_t)(yCh >>  0);

  #if __PTC_Pincount__ >= 8
  pCh[1]          = (uint8_t)(xCh >>  8);
  pCh[1 + tsize] = (uint8_t)(yCh >>  8);

  #if __PTC_Pincount__ >= 16
  pCh[2]          = (uint8_t)(xCh >> 16);
  pCh[2 + tsize] = (uint8_t)(yCh >> 16);
  pCh[3]          = (uint8_t)(xCh >> 24);
  pCh[3 + tsize] = (uint8_t)(yCh >> 24);
  pCh[4]          = (uint8_t)(xCh >> 32);
  pCh[4 + tsize] = (uint8_t)(yCh >> 32);

  #if __PTC_Pincount__ >=   40
  pCh[5]          = (uint8_t)(xCh >> 40);
  pCh[5 + tsize] = (uint8_t)(yCh >> 40);

  #endif
  #endif
  #endif

  return ptc_add_node(node, pCh, NODE_MUTUAL_bm);
};



uint8_t ptc_suspend(void);
void ptc_resume(void);

// If you want to know the compensation capacitance in femto Farad
uint16_t ptc_get_node_cc_femto(cap_sensor_t *node);

ptc_lib_sm_set_t *ptc_get_sm_settings();

// X and Y channel bitmasks
inline ptc_ch_bm_t ptc_get_node_xCh_bm(cap_sensor_t *node) {
  if (node == NULL) {
    return 0x00;
  }
  ptc_ch_bm_t retval = 0;
  memcpy(&retval, node->hw_xCh_bm, sizeof(node->hw_xCh_bm));
  return retval;
}

inline ptc_ch_bm_t ptc_get_node_yCh_bm(cap_sensor_t *node) {
  if (node == NULL) {
    return 0x00;
  }
  ptc_ch_bm_t retval = 0;
  memcpy(&retval, node->hw_yCh_bm, sizeof(node->hw_yCh_bm));
  return retval;
}

// the measured PTC value. 512 is 0. 0x00 means BAD_POINTER
inline uint16_t ptc_get_node_sensor_value(cap_sensor_t *node) {
  if (node == NULL) {
    return 0x00;
  }
  return node->sensorData;
}

// returns true, if node is a valid pointer and node is touched, otherwise false.
// No other return value so there can be easy checks - not null or null
inline uint8_t ptc_get_node_touched(cap_sensor_t *node) {
  if (node == NULL) {
    return 0x00;
  }

  if (node->stateMachine & (PTC_SM_TOUCH_DETECT | PTC_SM_TOUCH_OUT_FLT)) {
    return 0x01;
  }
  return 0x00;
}


inline uint8_t ptc_get_node_sm(cap_sensor_t *node) {
  PTC_CHECK_FOR_BAD_POINTER(node);
  return node->stateMachine;
}


inline int16_t ptc_get_node_delta(cap_sensor_t *node) {
  if (node == NULL) {
    return 0x8000;  //-32k - impossible value for normal operation
  }
  return (node->sensorData - node->reference);
}

inline uint8_t ptc_get_node_state(cap_sensor_t *node) {
  if (node == NULL) {
    return 0xFF;
  }
  return (node->stateMachine);
}

inline ptc_id_t ptc_get_node_id(cap_sensor_t *node) {
  if (node == NULL) {
    return 0xFF;
  }
  return (node->id);
}


inline uint8_t ptc_node_request_recal(cap_sensor_t *node) {
  PTC_CHECK_FOR_BAD_POINTER(node);

  node->stateMachine = PTC_SM_NOINIT_CAL;
  node->lastStateChange = 0;
  return PTC_LIB_SUCCESS;
}


void ptc_init_ADC0(void);
uint8_t ptc_lp_init(cap_sensor_t *node, uint8_t event_ch);
uint8_t ptc_lp_disable(void);
uint8_t ptc_lp_was_waken(void);

// Called by the interrupt routine. Saves result and selects next node
extern void ptc_eoc(void);


#ifdef __cplusplus
}
#endif

#endif
