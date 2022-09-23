/*
 * ADXL355Z_Drv.h
 *
 *  Created on: 07 april 2020
 *      Author: Alex BUdnik
 *
 * Standart driver fo ADXL355 accelerometer
 */


/* Define to prevent recursive inclusion ---------------------------------------- */

#ifndef __ADXL355_DRV_H
#define __ADXL355_DRV_H

#ifdef __cplusplus
extern "C" {
#endif


/* Includes --------------------------------------------------------------------- */

#include "stdint.h"
#include "stdbool.h"


/* Type definitions ------------------------------------------------------------- */

typedef float float32_t;


/* Measuremet ranges (sensitivity).
 */
typedef enum  adxl355__meas_range
{
    ADXL355__RANGE_2g     = 0x01u, /* sensitivity +- 2g.  */
    ADXL355__RANGE_4g     = 0x02u, /* sensitivity +- 4g.  */
    ADXL355__RANGE_8g     = 0x03u  /* sensitivity +- 8g.  */
} adxl355__meas_range_t;


/* high-pass Frequency, −3 dB
 * Point Relative to ODR Settings.
*/
typedef enum  adxl355__hp_filter
{
    ADXL355__HP_OFF          = 0x00u,  /* high-pass filter disabled.  */
    ADXL355__HP_247_000_X_10 = 0x01u,  /* [24.7 × 10−4]   × ODR.      */
    ADXL355__HP_062_084_X_10 = 0x02u,  /* [6.2084 × 10−4] × ODR.      */
    ADXL355__HP_015_545_X_10 = 0x03u,  /* [1.5545 × 10−4] × ODR.      */
    ADXL355__HP_003_862_X_10 = 0x04u,  /* [0.3862 × 10−4] × ODR.      */
    ADXL355__HP_000_954_X_10 = 0x05u,  /* [0.0954 × 10−4] × ODR.      */
    ADXL355__HP_000_238_X_10 = 0x06u,  /* [0.0238 × 10−4] × ODR.      */

} adxl355__hp_filter_t;


/* ODR and low-pass filter corners.
 */
typedef enum  adxl355_odr_and_lp_filter
{
    ADXL355__ODR_4000_HZ_AND_1000_HZ     = 0x00u,    /* 4000 Hz and 1000 Hz.    */
    ADXL355__ODR_2000_HZ_AND_500_HZ      = 0x01u,    /* 2000 Hz and 500 Hz.     */
    ADXL355__ODR_1000_HZ_AND_250_HZ      = 0x02u,    /* 1000 Hz and 250 Hz.     */
    ADXL355__ODR_500_HZ_AND_125_HZ       = 0x03u,    /* 500 Hz and 125 Hz.      */
    ADXL355__ODR_250_HZ_AND_62_5_HZ      = 0x04u,    /* 250 Hz and 62.5 Hz.     */
    ADXL355__ODR_125_HZ_AND_31_25_HZ     = 0x05u,    /* 125 Hz and 31.25 Hz.    */
    ADXL355__ODR_62_5_HZ_AND_15_625_HZ   = 0x06u,    /* 62.5 Hz and 15.625 Hz.  */
    ADXL355__ODR_31_25_HZ_AND_7_813_HZ   = 0x07u,    /* 31.25 Hz and 7.813 Hz.  */
    ADXL355__ODR_15_625_HZ_AND_3_906_HZ  = 0x08u,    /* 15.625 Hz and 3.906 Hz. */
    ADXL355__ODR_7_813_HZ_AND_1_953_HZ   = 0x09u,    /* 7.813 Hz and 1.953 Hz.  */
    ADXL355__ODR_3_906_HZ_AND_0_977_HZ   = 0x0Au     /* 3.906 Hz and 0.977 Hz.  */

} adxl355_odr_and_lp_filter_t;


/* i2c speed modes (adxl setting).
 */
typedef enum  adxl355__i2c_speed
{
    ADXL355__I2C_HIGH_SPEED  = 0x00u, /* High speed mode. */
    ADXL355__I2C_FARST_SPEED = 0x01u, /* Farts speed mode. */
} adxl355__i2c_speed_t;


/* Interupts active polarity.
 */
typedef enum  adxl355__int_polarity
{
    ADXL355__INT_POLARITY_ACTIVE_LOW  = 0x00u, /* INT1 and INT2 are active low polarity.  */
    ADXL355__INT_POLARITY_ACTIVE_HIGH = 0x01u  /* INT1 and INT2 are active high polarity. */
} adxl355__int_polarity_t;


/* This struct includes bits that describe
 * the various conditions of the ADXL355.
 */
struct adxl355__status
{
    bool is_nvm_nusy;  /* NVM controller is busy with either refresh, programming, or built-in, self test.     */
    bool is_activiry;  /* Activity, as defined in the THRESH_ACT and COUNT_ACT registers, is detected.         */
    bool is_fifo_ovr;  /* FIFO has overrun, and the oldest data is lost.                                       */
    bool is_fifo_full; /* FIFO watermark is reached.                                                           */
    bool is_data_rdy;  /* A complete x-axis, y-axis, and z-axis measurement was made and results can be read.  */

} typedef adxl355__status_t;


/* This is structure includes configures the
 * interrupt pins.
 * Fields which function(s) generate
 * an interrupt on the INT1 and INT2 pins.
 * Multiple events can be configured.
 * If the corresponding bit is set to 1, the function
 * generates an interrupt on the interrupt pins.
 */
struct adxl355__interrupt_map
{
    bool act_en2;  /* Activity interrupt enable on INT2.  */
    bool ovr_en2;  /* FIFO_OVR interrupt enable on INT2.  */
    bool full_en2; /* FIFO_FULL interrupt enable on INT2. */
    bool rdy_en2;  /* DATA_RDY interrupt enable on INT2.  */
    bool act_en1;  /* Activity interrupt enable on INT1.  */
    bool ovr_en1;  /* FIFO_OVR interrupt enable on INT1.  */
    bool full_en1; /* FIFO_FULL interrupt enable on INT1. */
    bool rdy_en1;  /* DATA_RDY interrupt enable on INT1.  */

} typedef adxl355__interrupt_map_t;


/* This is structure includes
 * general information about of adxl355
 */
struct adxl355__general_info
{
    uint8_t dev_ad_id;   /* Analog Devices ID.      */
    uint8_t dev_mst_id;  /* Analog Devices MEMS ID. */
    uint8_t dev_part_id; /* Device ID.              */
    uint8_t dev_rev_id;  /* Product revision ID.    */

} typedef adxl355__general_info_t;


/* This is structure includes:
 *  - ADXL355 communication device settings.
 */
struct adxl355
{
    /* Pointer to function of read reg.
    *
    * [in]  reg   - register address.
    * [out] value - value of register.
    * [in] size  - size in bytes value.
    */
    void (*const read) (const uint8_t reg, uint8_t *const value, const uint16_t size);


    /* Pointer to function of write reg.
    * [in] reg   - register address.
    * [in] value - value of register.
    * [in] size  - size in bytes value.
    */
    void (*const write)(const uint8_t reg, uint8_t *const value, const uint16_t size);


} typedef adxl355_t;



/* Function prototypes -----------------------------------------------------------*/

/* Get all general information about ADXL355.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] gen_info - General information about ADXL355
 */
void adxl355__get_general_info(const adxl355_t *const hndl,
                               adxl355__general_info_t *const gen_info);


/* Get status.
 * [in]  hndl   - ADXL355 handle.
 * [out] status - Status of ADXL355
 */
void adxl355__get_status(const  adxl355_t *const hndl,
                                adxl355__status_t *const status);


/* Get int temperature
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] temp     - Temperature.
*/
void adxl355__get_temperature(const adxl355_t *const hndl,
                              int32_t *const temp);


/* Get float temperature in c
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] temp     - Temperature.
*/
void adxl355__get_temperature_c(const adxl355_t *const hndl,
                                float *const temp);


/* Get x sample.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] sample_x - X sample(format - mg).
*/
void adxl355__get_int_x_sample(const adxl355_t *const hndl,
                               int32_t *const sample_x);


/* Get y sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_y - Y sample(format - mg).
*/
void adxl355__get_int_y_sample(const adxl355_t *const hndl,
                               int32_t *const sample_y);


/* Get z sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] samples_z - Z sample.
*/
void adxl355__get_int_z_sample(const adxl355_t *const hndl,
                               int32_t *const samples_z);


/* Get x sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_x - X sample(format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_x_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_x,
                                 adxl355__meas_range_t range);


/* Get y sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_y - Y sample(format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_y_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_y,
                                 adxl355__meas_range_t range);


/* Get z sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_z - Z sample (format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_z_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_z,
                                 adxl355__meas_range_t range);


/* Get xyz samp0le.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] samples_x - X sample.
 * [out] samples_y - y sample.
 * [out] samples_z - Z sample.
 * [in]  range     - Measurement range.
*/
void adxl355__get_int_xyz_sample(const adxl355_t *const hndl,
                                 int32_t *const sample_x,
                                 int32_t *const sample_y,
                                 int32_t *const sample_z);


/* Get xyz samp0le.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] samples_x - X sample (format - mg).
 * [out] samples_y - y sample (format - mg).
 * [out] samples_z - Z sample (format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_xyz_sample(const adxl355_t *const hndl,
                                   float32_t *const sample_x,
                                   float32_t *const sample_y,
                                   float32_t *const sample_z,
                                   adxl355__meas_range_t range);


/* Get aviable samoles count in fifo bufer.
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samoles_count - pointer to out value.
*/
void adxl355__get_fifo_samples_count(const adxl355_t *const hndl,
                                     uint8_t *const samoles_count);


/* Get value when fifo full condition is triggered.
 *
 * [in]  hndl       - ADXL355 handle.
 * [out] samp_count - Pointer to out value, number of samples
 *      stored in the FIFO that triggers a FIFO_FULL condition.
 *      Values range from 1 to 96.
*/
void adxl355__get_fifo_full_condition(const adxl355_t *const hndl,
                                      uint8_t *const samp_count);


/* Set value when fifo full condition is triggered.
 *
 * [in] hndl       - ADXL355 handle.
 * [in] samp_count - Value of number of samples
 *      stored in the FIFO that triggers a FIFO_FULL condition.
 *      Values range from 1 to 96.
*/
void adxl355__set_fifo_full_condition(const adxl355_t *const hndl,
                                      uint8_t samp_count);


/* Get aviable samples from fifo buffer.
 * (Read fifo сount register).
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samples_x     - Buffer of x samples.
 * [out] samples_y     - Buffer of y samples.
 * [out] samples_z     - Buffer of z samples.
 * [in]  range         - Measurement range.
 * [out] count_samples - Count of samples readed.
*/
void adxl355__get_int_fifo_samples_of_aviable(const adxl355_t *const hndl,
                                              int32_t *const samples_x,
                                              int32_t *const samples_y,
                                              int32_t *const samples_z,
                                              uint8_t *const count_samples);


/* Get aviable samples from fifo buffer.
 * (Read fifo сount register).
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samples_x     - Buffer of x samples (format - mg).
 * [out] samples_y     - Buffer of y samples (format - mg).
 * [out] samples_z     - Buffer of z samples (format - mg).
 * [in]  range         - Measurement range.
 * [out] count_samples - Count of samples readed.
*/
void adxl355__get_float_fifo_samples_of_aviable(const adxl355_t *const hndl,
                                                float *const samples_x,
                                                float *const samples_y,
                                                float *const samples_z,
                                                adxl355__meas_range_t range,
                                                uint8_t  *const count_samples);


/* Get aviable samples from fifo buffer.
 * (Read to empty watermark).
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samples_x     - Buffer of x samples.
 * [out] samples_y     - Buffer of y samples.
 * [out] samples_z     - Buffer of z samples.
 * [in]  range         - Measurement range.
 * [out] count_samples - Count of samples readed.
 *
 * WARNING: The number of samples may differ from the set fifo count,
 * because while reading data FIFO will replenish and the mark will shift.
*/
void adxl355__get_int_fifo_samples_to_watermark(const adxl355_t *const hndl,
                                                int32_t *const samples_x,
                                                int32_t *const samples_y,
                                                int32_t *const samples_z,
                                                uint8_t  *const count_samples);


/* Get aviable samples from fifo buffer.
 * (Read to empty watermark).
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samples_x     - Buffer of x samples (format - mg).
 * [out] samples_y     - Buffer of y samples (format - mg).
 * [out] samples_z     - Buffer of z samples (format - mg).
 * [in]  range         - Measurement range.
 * [out] count_samples - Count of samples readed.
 *
 * WARNING: The number of samples may differ from the set fifo count,
 * because while reading data FIFO will replenish and the mark will shift.
*/
void adxl355__get_float_fifo_samples_to_watermark(const adxl355_t *const hndl,
                                                  float *const samples_x,
                                                  float *const samples_y,
                                                  float *const samples_z,
                                                  adxl355__meas_range_t range,
                                                  uint8_t  *const count_samples);

/* Get axes offsets.
 * Offset added to axes data after all other signal processing.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] x_offset - X offset.
 * [out] y_offset - Y offset.
 * [out] z_offset - Z offset.
*/
void adxl355__get_int_offsets(const adxl355_t *const hndl,
                              int32_t *const x_offset,
                              int32_t *const y_offset,
                              int32_t *const z_offset);


/* Set axes offsets.
 * Offset added to axes data after all other signal processing.
 *
 * [in] hndl     - ADXL355 handle.
 * [in] x_offset - X offset.
 * [in] y_offset - Y offset.
 * [in] z_offset - Z offset.
*/
void adxl355__set_int_offsets(const adxl355_t *const hndl,
                              const int32_t x_offset,
                              const int32_t y_offset,
                              const int32_t z_offset);


/* Get axes offsets.
 * Offset added to axes data after all other signal processing.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] x_offset - X offset (format - mg).
 * [out] y_offset - Y offset (format - mg).
 * [out] z_offset - Z offset (format - mg).
*/
void adxl355__get_float_offsets(const adxl355_t *const hndl,
                                float32_t *const x_offset,
                                float32_t *const y_offset,
                                float32_t *const z_offset,
                                const adxl355__meas_range_t range);


/* Set axes offsets.
 * Offset added to axes data after all other signal processing.
 *
 * [in] hndl     - ADXL355 handle.
 * [in] x_offset - X offset (format - mg).
 * [in] y_offset - Y offset (format - mg).
 * [in] z_offset - Z offset (format - mg).
*/
void adxl355__set_float_offsets(const adxl355_t *const hndl,
                                const float x_offset,
                                const float y_offset,
                                const float z_offset,
                                adxl355__meas_range_t range);


/* Get activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in] hndl        - ADXL355 handle.
 * [out] act_detect - detect value.
*/
void adxl355__get_int_act_detect_val(const adxl355_t *const hndl,
                                     uint32_t *const act_detect);


/* Set activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in] hndl        - ADXL355 handle.
 * [in] act_detect  - detect value.
*/
void adxl355__set_int_act_detect_val(const adxl355_t *const hndl,
                                      const uint32_t act_detect);


/* Get activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in]  hndl        - ADXL355 handle.
 * [out] act_detect  - detect value (format in mg).
 * [in]  range       - Measurement range.
*/
void adxl355__get_float_act_detect_val(const adxl355_t *const hndl,
                                       float32_t *const act_detect,
                                       adxl355__meas_range_t range);


/* Set activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in]  hndl        - ADXL355 handle.
 * [in] act_detect   - detect value (format in mg).
 * [in]  range       - Measurement range.
*/
void adxl355__set_float_act_detect_val(const adxl355_t *const hndl,
                                       const float32_t act_detect,
                                       adxl355__meas_range_t range);


/* Get activity detection count.
 * Number of consecutive events above threshold required to detect activity.
 *
 * [in]  hndl        - ADXL355 handle.
 * [out] act_detect  - detection count
*/
void adxl355__get_act_detect_count(const adxl355_t *const hndl,
                                   uint8_t *const act_detect_cnt);


/* Set activity detection count.
 * Number of consecutive events above threshold required to detect activity.
 *
 * [in]  hndl        - ADXL355 handle.
 * [in] act_detect   - detection count
*/
void adxl355__set_act_detect_count(const adxl355_t *const hndl,
                                   uint8_t act_detect_cnt);


/* Set axes a component of the activity detection algorithm
 *
 * [in] hndl     - ADXL355 handle.
 * [in] x_axis   - x axis (true if algorithm is enabled of this axis)
 * [in] y_axis   - y axis (true if algorithm is enabled of this axis)
 * [in] z_axis   - z axis (true if algorithm is enabled of this axis)
*/
void adxl355__set_act_detection_axes(const adxl355_t *const hndl,
                                     const bool x_axis,
                                     const bool y_axis,
                                     const bool z_axis);


/* Set axes a component of the activity detection algorithm
 *
 * [in] hndl     - ADXL355 handle.
 * [in] x_axis   - x axis (true if algorithm is enabled of this axis)
 * [in] y_axis   - y axis (true if algorithm is enabled of this axis)
 * [in] z_axis   - z axis (true if algorithm is enabled of this axis)
*/
void adxl355__get_act_detection_axes(const adxl355_t *const hndl,
                                     bool *const x_axis,
                                     bool *const y_axis,
                                     bool *const z_axis);


/* Get axes high-pass filter, low-pass filter,
 * and output data rate.
 *
 * [in]  hndl        - ADXL355 handle.
 * [out] hp_filter   − 3 dB filter corner for the first-order,
 *                     high-pass filter relative to the output data rate.
 * [out] odr_and_lpf - output data rate and low-pass filter corner.
*/
void adxl355__get_filter(const adxl355_t *const hndl,
                         adxl355__hp_filter_t *const hp_filter,
                         adxl355_odr_and_lp_filter_t *const odr_and_lpf);


/* Set axes high-pass filter, low-pass filter,
 * and output data rate.
 *
 * [in] hndl        - ADXL355 handle.
 * [in] hp_filter   − 3 dB filter corner for the first-order,
 *                     high-pass filter relative to the output data rate.
 * [in] odr_and_lpf - output data rate and low-pass filter corner.
*/
void adxl355__set_filter(const adxl355_t *const hndl,
                         const adxl355__hp_filter_t hp_filter,
                         const adxl355_odr_and_lp_filter_t odr_and_lpf);


/* Get data ready pin mode,
 *
 * [in]  hndl            - ADXL355 handle.
 * [out] drdy_is_enable  − True if DRDY output is enable
 *                         modes where it is normally signal data ready.
 *
*/
void adxl355__get_drdy_pin_mode(const adxl355_t *const hndl,
                                bool *const drdy_is_enable);


/* Set data ready pin mode,
 *
 * [in] hndl             - ADXL355 handle.
 * [in] drdy_is_enable   − Set true to DRDY output is 0 in
 *                         modes where it is normally signal data ready.
*/
void adxl355__set_drdy_pin_mode(const adxl355_t *const hndl,
                                const bool drdy_enable);


/* Get temperature measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] drdy_is_enable   − True if temperature measurement mode is enable.
*/
void adxl355__get_temp_mode(const adxl355_t *const hndl,
                            bool *const temp_meas_is_enable);


/* Get temperature measurement mode
 *
 * [in] hndl             - ADXL355 handle.
 * [in] drdy_is_enable   − Set true to enable temperature measurement mode.
*/
void adxl355__set_temp_mode(const adxl355_t *const hndl,
                            const bool temp_meas_enable);


/* Get measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] meas_is_enable   − True if measurement mode is emabled.
*/
void adxl355__get_meas_mode(const adxl355_t *const hndl,
                            bool *const meas_is_enable);


/* Set measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] meas_is_enable   − Set true to emabled measurement mode.
*/
void adxl355__set_meas_mode(const adxl355_t *const hndl,
                            const bool meas_enable);


/* Get mrasurement range.
 *
 * [in]  hndl  - ADXL355 handle.
 * [out] range - Measurement range.
 *
 * return operation code.
*/
void adxl355__get_range(const adxl355_t  *const hndl,
                        adxl355__meas_range_t *const range);


/* Set mrasurement range.
 *
 * [in] hndl  - ADXL355 handle.
 * [in] range - Measurement range.
 *
 * return operation code.
*/
void adxl355__set_range(const adxl355_t *const hndl,
                        const adxl355__meas_range_t range);


/* Get interrupt palarity
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] int_pol   − Interupt polarity.
 *
 * return operation code.
*/
void adxl355__get_int_polarity(const adxl355_t *const hndl,
                               adxl355__int_polarity_t *const int_pol);


/* Set interrupt palarity
 *
 * [in] hndl      - ADXL355 handle.
 * [in] int_pol   − Interupt polarity.
*/
void adxl355__set_int_polarity(const adxl355_t *const hndl,
                               const adxl355__int_polarity_t int_pol);


/* Get i2c speed
 *
 * [in] hndl    - ADXL355 handle.
 * [out] speed  − I2c speed.
*/
void adxl355__get_i2c_speed(const adxl355_t *const hndl,
                            adxl355__i2c_speed_t *const speed);


/* Set i2c speed
 *
 * [in] hndl   - ADXL355 handle.
 * [in] speed  − I2c speed.
*/
void adxl355__set_i2c_speed(const adxl355_t *const hndl,
                            const adxl355__i2c_speed_t speed);


/* Get interrupt map
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] int_map   − Interrupt map.
*/
void adxl355__get_int_map(const adxl355_t *const hndl,
                          adxl355__interrupt_map_t *const int_map);



/* Set interrupt map
 *
 * [in] hndl      - ADXL355 handle.
 * [in] int_map   − Interrupt map.
 *
 * return operation code.
*/
void adxl355__set_int_map(const adxl355_t *const hndl,
                          const adxl355__interrupt_map_t int_map);


/* Get self test force mode
 *
 * [in]  hndl                - ADXL355 handle.
 * [out] self_test_is_force  − True if self test is force enabled.
*/
void adxl355__get_self_test_force(const adxl355_t *const hndl,
                                  bool *const  self_test_is_force);


/* Set self test force mode
 *
 * [in] hndl                - ADXL355 handle.
 * [in] self_test_is_force  −  Set true to enable self test force
*/
void adxl355__set_self_test_force(const adxl355_t *const hndl,
                                  const bool self_test_is_force);


/* Get self test mode.
 *
 * [in]  hndl                - ADXL355 handle.
 * [out] self_test_is_force  − True if self test is enabled.
*/
void adxl355__get_self_test_mode(const adxl355_t *const hndl,
                                 bool *const  self_test_is_enable);


/* Set self test mode.
 *
 * [in]  hndl               - ADXL355 handle.
 * [in] self_test_is_force  − Set true to enable self test mode
*/
void adxl355__set_self_test_mode(const adxl355_t *const hndl,
                                 const bool self_test_is_enable);


/* Resets ADXL355 device, similar to a power-on reset (POR).
 *
 * [in]  hndl  - ADXL355 handle.
*/
void adxl355__reset_dev(const adxl355_t *const hndl);



#ifdef __cplusplus
}
#endif

#endif /* __ADXL355Z_DRV_H */
