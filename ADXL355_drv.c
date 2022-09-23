/*
 * ADXL355Z_Drv.c
 *
 *  Created on: 07 april 2020
 *      Author: Alex BUdnik
 *
 * File description.
 * This file includes a description of all the registers.
 * Also full range of functions for working with the ADXL355 accelerometer.
 */


/* Includes --------------------------------------------------------------------- */

#include "ADXL355_drv.h"
#include "stdbool.h"
#include "stdint.h"


/* ADXL355 REGISTERS LIST ------------------------------------------------------- */

#define ADXL355__REG_DEVID_AD     0x00u   /* This register contains the Analog Devices ID                              */
#define ADXL355__REG_DEVID_MST    0x01u   /* This register contains the Analog Devices MEMS ID                         */
#define ADXL355__REG_PARTID       0x02u   /* This register contains the device ID                                      */
#define ADXL355__REG_REVID        0x03u   /* This register contains the product revision ID                            */
#define ADXL355__REG_STATUS       0x04u   /* This register includes bits status bits of the ADXL355                    */
#define ADXL355__REG_FIFO_ENTRIES 0x05u   /* This register indicates the number of valid data samples                  */
#define ADXL355__REG_TEMP2        0x06u   /* Four most significant bits                                                */
#define ADXL355__REG_TEMP1        0x07u   /* Eight least significant bits of the 12-bit value.                         */
#define ADXL355__REG_XDATA3       0x08u   /* Thes register contain the x-axis acceleration data.                       */
#define ADXL355__REG_XDATA2       0x09u   /* Thes register contain the x-axis acceleration data.                       */
#define ADXL355__REG_XDATA1       0x0Au   /* Thes register contain the x-axis acceleration data.                       */
#define ADXL355__REG_YDATA3       0x0Bu   /* Thes register contain the y-axis acceleration data.                       */
#define ADXL355__REG_YDATA2       0x0Cu   /* Thes register contain the y-axis acceleration data.                       */
#define ADXL355__REG_YDATA1       0x0Du   /* Thes register contain the y-axis acceleration data.                       */
#define ADXL355__REG_ZDATA3       0x0Eu   /* Thes register contain the z-axis acceleration data.                       */
#define ADXL355__REG_ZDATA2       0x0Fu   /* Thes register contain the z-axis acceleration data.                       */
#define ADXL355__REG_ZDATA1       0x10u   /* Thes register contain the z-axis acceleration data.                       */
#define ADXL355__REG_FIFO_DATA    0x11u   /* Read this register to access data stored in the FIFO.                     */
#define ADXL355__REG_OFFSET_X_H   0x1Eu   /* Thes register is x-axis offset trim register (High word)                  */
#define ADXL355__REG_OFFSET_X_L   0x1Fu   /* Thes register is x-axis offset trim register (Low word)                   */
#define ADXL355__REG_OFFSET_Y_H   0x20u   /* Thes register is y-axis offset trim register (High word)                  */
#define ADXL355__REG_OFFSET_Y_L   0x21u   /* Thes register is y-axis offset trim register (Low word)                   */
#define ADXL355__REG_OFFSET_Z_H   0x22u   /* Thes register is z-axis offset trim register (High word)                  */
#define ADXL355__REG_OFFSET_Z_L   0x23u   /* Thes register is z-axis offset trim register (Low word)                   */
#define ADXL355__REG_ACT_EN       0x24u   /* Activity enable register                                                  */
#define ADXL355__REG_ACT_THRESH_H 0x25u   /* Activity threshold register (High word)                                   */
#define ADXL355__REG_ACT_THRESH_L 0x26u   /* Activity threshold register (Low word)                                    */
#define ADXL355__REG_ACT_COUNT    0x27u   /* Number of consecutive events above threshold required to detect activity  */
#define ADXL355__REG_FILTER       0x28u   /* This register to specify parameters for the internal  filters.            */
#define ADXL355__REG_FIFO_SAMPLES 0x29u   /* This registerto to specify the number of samples to store in the FIFO.    */
#define ADXL355__REG_INT_MAP      0x2Au   /* This registerto to configures the interrupt pins.                         */
#define ADXL355__REG_SYNC         0x2Bu   /* This register to control the external timing triggers.                    */
#define ADXL355__REG_RANGE        0x2Cu   /* I2C speed, interrupt polarity, and range register                         */
#define ADXL355__REG_POWER_CTL    0x2Du   /* Power control register                                                    */
#define ADXL355__REG_SELF_TEST    0x2Eu   /* Self test register                                                        */
#define ADXL355__REG_RESET        0x2Fu   /* Reset register                                                            */



/* BITS HELPERS  ---------------------------------------------------------------- */

#define ADXL355_SET_REG_BIT(reg, enable, offs)          ((reg  & (~(0x01u << offs))) | ((enable & 0x01u) << offs))
#define ADXL355_SET_REG_DAT(reg, dat, mask ,offs)       ((reg & (~mask)) | (dat << offs))
#define ADXL355_GET_REG_DAT(reg, mask, offset)          ((reg & mask) >> offset)
#define ADXL355_CHECK_BIT(reg, nbit)                    ((reg) & (1<<(nbit)))



#define ADXL355_GET_HI_16(dat)                          ((dat & 0xff00u) >> 8)
#define ADXL355_GET_LO_16(dat)                          ((dat & 0x00FFu))

///TODO: CHECK COMBYNE BYTES
#define ADXL355_COMBINE_2BYTES(b0, b1)                  ((b0 << 8)  | b1)
#define ADXL355_COMBINE_3BYTES(low, mid, high)          ((low << 16) | (mid << 8) | (high))


/* STATUS REGISTER OPERATIONS --------------------------------------------------- */

#define ADXL355_BITS_IS_NVM_BUSY(reg)                   ADXL355_CHECK_BIT(reg, 4);
#define ADXL355_BITS_IS_ACTIVITY(reg)                   ADXL355_CHECK_BIT(reg, 3);
#define ADXL355_BITS_IS_FIFO_OVR(reg)                   ADXL355_CHECK_BIT(reg, 2);
#define ADXL355_BITS_IS_FIFO_FULL(reg)                  ADXL355_CHECK_BIT(reg, 1);
#define ADXL355_BITS_IS_DATA_RDY(reg)                   ADXL355_CHECK_BIT(reg, 0);


/* ACTIVITY ENABLE REGISTER OPERATIONS ------------------------------------------ */

#define ADXL355_BITS_IS_ACT_ENABLE_X(reg)               ADXL355_CHECK_BIT(reg, 0)
#define ADXL355_BITS_IS_ACT_ENABLE_Y(reg)               ADXL355_CHECK_BIT(reg, 1)
#define ADXL355_BITS_IS_ACT_ENABLE_Z(reg)               ADXL355_CHECK_BIT(reg, 2)

#define ADXL355_BITS_SET_ACT_ENABLE_X(reg, en)          ADXL355_SET_REG_BIT(reg, en, 0)
#define ADXL355_BITS_SET_ACT_ENABLE_Y(reg, en)          ADXL355_SET_REG_BIT(reg, en, 1)
#define ADXL355_BITS_SET_ACT_ENABLE_Z(reg, en)          ADXL355_SET_REG_BIT(reg, en, 2)


/* FILTER AND ODR SETTINGS REGISTER OPERATIONS ---------------------------------- */

#define ADXL355_BITS_GET_HPF_VAL(reg)                   ADXL355_GET_REG_DAT(reg, 0x70u, 4)
#define ADXL355_BITS_GET_ODR_LPF_VAL(reg)               ADXL355_GET_REG_DAT(reg, 0x0Fu, 0)

#define ADXL355_BITS_SET_HPF_VAL(reg, dat)              ADXL355_SET_REG_DAT(reg, dat, 0x70u, 4)
#define ADXL355_BITS_SET_ODR_LPF_VAL(reg, dat)          ADXL355_SET_REG_DAT(reg, dat, 0x0Fu, 0)


/* INTERRUPT PIN (INTx) FUNCTION MAP REGISTER OPERATIONS ------------------------ */

#define ADXL355_BITS_IS_ACT_EN2(reg)                    ADXL355_CHECK_BIT(reg, 7)
#define ADXL355_BITS_IS_OVR_EN2(reg)                    ADXL355_CHECK_BIT(reg, 6)
#define ADXL355_BITS_IS_FULL_EN2(reg)                   ADXL355_CHECK_BIT(reg, 5)
#define ADXL355_BITS_IS_RDY_EN2(reg)                    ADXL355_CHECK_BIT(reg, 4)
#define ADXL355_BITS_IS_ACT_EN1(reg)                    ADXL355_CHECK_BIT(reg, 3)
#define ADXL355_BITS_IS_OVR_EN1(reg)                    ADXL355_CHECK_BIT(reg, 2)
#define ADXL355_BITS_IS_FULL_EN1(reg)                   ADXL355_CHECK_BIT(reg, 1)
#define ADXL355_BITS_IS_RDY_EN1(reg)                    ADXL355_CHECK_BIT(reg, 0)

#define ADXL355_BITS_SET_ACT_EN2(reg, en)               ADXL355_SET_REG_BIT(reg, en, 7)
#define ADXL355_BITS_SET_OVR_EN2(reg, en)               ADXL355_SET_REG_BIT(reg, en, 6)
#define ADXL355_BITS_SET_FULL_EN2(reg, en)              ADXL355_SET_REG_BIT(reg, en, 5)
#define ADXL355_BITS_SET_RDY_EN2(reg, en)               ADXL355_SET_REG_BIT(reg, en, 4)
#define ADXL355_BITS_SET_ACT_EN1(reg, en)               ADXL355_SET_REG_BIT(reg, en, 3)
#define ADXL355_BITS_SET_OVR_EN1(reg, en)               ADXL355_SET_REG_BIT(reg, en, 2)
#define ADXL355_BITS_SET_FULL_EN1(reg, en)              ADXL355_SET_REG_BIT(reg, en, 1)
#define ADXL355_BITS_SET_RDY_EN1(reg, en)               ADXL355_SET_REG_BIT(reg, en, 0)


/* DATA SYNCHRONIZATION REGISTER OPERATIONS ------------------------------------- */

#define ADXL355_BITS_IS_EXT_CLK(reg)                    ADXL355_CHECK_BIT(reg, 3)
#define ADXL355_BITS_SET_ENABLE_EXT_CLK(reg, en)        ADXL355_SET_REG_BIT(reg, en, 3)

#define ADXL355_BITS_GET_EXT_SYNC(reg)                  ADXL355_GET_REG_DAT(reg, 0x03, 0)
#define ADXL355_BITS_SET_EXT_SYNC(reg)                  ADXL355_GET_REG_DAT(reg, 0x03, 0)


/* I2C SPEED, INTERRUPT POLARITY, AND RANGE REGISTER OPERATIONS ----------------- */

#define ADXL355_BITS_GET_RANGE_VAL(reg)                 ADXL355_GET_REG_DAT(reg, 0x03, 0)
#define ADXL355_BITS_SET_RANGE_VAL(reg, dat)            ADXL355_SET_REG_DAT(reg, dat, 0x03, 0)

#define ADXL355_BITS_GET_INT_POLARITY(reg)              ADXL355_GET_REG_DAT(reg, 0x40, 6)
#define ADXL355_BITS_SET_INT_POLARITY(reg, dat)         ADXL355_SET_REG_DAT(reg, dat, 0x40, 6)

#define ADXL355_BITS_GET_I2C_SPEED(reg)                 ADXL355_GET_REG_DAT(reg, 0x80, 7)
#define ADXL355_BITS_SET_I2C_SPEED(reg, dat)            ADXL355_SET_REG_DAT(reg, dat, 0x80, 7)


/* POWER CONTROL REGISTER ------------------------------------------------------- */

#define ADXL355_BITS_IS_DRDY_ON(reg)                    ADXL355_CHECK_BIT(reg, 2)
#define ADXL355_BITS_IS_TEMP_ON(reg)                    ADXL355_CHECK_BIT(reg, 1)
#define ADXL355_BITS_IS_STANDBY_ON(reg)                 ADXL355_CHECK_BIT(reg, 0)

#define ADXL355_BITS_SET_DRDY_ENABLE(reg, en)           ADXL355_SET_REG_BIT(reg, en, 2)
#define ADXL355_BITS_SET_TEMP_MEAS_ENABLE(reg, en)      ADXL355_SET_REG_BIT(reg, en, 1)
#define ADXL355_BITS_SET_STANDBY_MODE_ENABLE(reg, en)   ADXL355_SET_REG_BIT(reg, en, 0)


/* SELF TEST REGISTER  ---------------------------------------------------------- */

#define ADXL355_BITS_IS_ST2(reg)                        ADXL355_CHECK_BIT(reg, 1)
#define ADXL355_BITS_IS_ST1(reg)                        ADXL355_CHECK_BIT(reg, 0)

#define ADXL355_BITS_SET_ST2_ENABLE(reg, en)            ADXL355_SET_REG_BIT(reg, en, 1)
#define ADXL355_BITS_SET_ST1_ENABLE(reg, en)            ADXL355_SET_REG_BIT(reg, en, 0)




/* Scale consts for calculate measuremensts registers from sensitivity.
 */
static const float32_t adxl355_scale[3] = {
    256000.0f,
    128000.0f,
    64000.0f
};

/* Private functions ------------------------------------------------------------ */


/*
 * Two complement
*/
inline static int32_t ADXL355_TWO_COMPLEMENT (uint32_t data)
{
  int32_t volatile i32Conversion = 0;

  data = (data  >> 4);
  data = (data & 0x000FFFFF);

  if((data & 0x00080000)  == 0x00080000)
        i32Conversion = (data | 0xFFF00000);
  else i32Conversion = data;

  return i32Conversion;
}


/* Exported functions ----------------------------------------------------------- */

/* Get all general information about ADXL355.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] gen_info - General information about ADXL355
 */
void adxl355__get_general_info(const adxl355_t *const hndl,
                               adxl355__general_info_t *const gen_info)
{
    hndl->read(ADXL355__REG_DEVID_AD,  &(gen_info->dev_ad_id),   1);
    hndl->read(ADXL355__REG_DEVID_MST, &(gen_info->dev_mst_id),  1);
    hndl->read(ADXL355__REG_PARTID,    &(gen_info->dev_part_id), 1);
    hndl->read(ADXL355__REG_REVID,     &(gen_info->dev_rev_id),  1);
}


/* Get status.
 * [in]  hndl   - ADXL355 handle.
 * [out] status - Status of ADXL355
 */
void adxl355__get_status(const  adxl355_t *const hndl,
                                adxl355__status_t *const status)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_STATUS,  &data,   1);

    status->is_data_rdy  =  ADXL355_BITS_IS_DATA_RDY(data);
    status->is_fifo_full =  ADXL355_BITS_IS_FIFO_FULL(data);
    status->is_fifo_ovr  =  ADXL355_BITS_IS_FIFO_OVR(data);
    status->is_activiry  =  ADXL355_BITS_IS_ACTIVITY(data);
    status->is_nvm_nusy  =  ADXL355_BITS_IS_NVM_BUSY(data);
}


/* Get int temperature
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] temp     - Temperature.
*/
void adxl355__get_temperature(const adxl355_t *const hndl,
                              int32_t *const temp)
{
    uint8_t data[2] = {0};

    hndl->read(ADXL355__REG_TEMP2, data, 2);

    *temp = ADXL355_COMBINE_2BYTES(data[0], data[1]);
}


/* Get float temperature in c
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] temp     - Temperature.
*/
void adxl355__get_temperature_c(const adxl355_t *const hndl,
                                float *const temp)
{

    int32_t temp_data = 0;

    adxl355__get_temperature(hndl, &temp_data);

    *temp = ((float)(1852.0f - (float)temp_data) / 9.05f) + 19.21f;
}


/* Get x sample.
 *
 * [in]  hndl     - ADXL355 handle.
 * [out] sample_x - X sample(format - mg).
*/
void adxl355__get_int_x_sample(const adxl355_t *const hndl,
                               int32_t *const sample_x)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;

    hndl->read(ADXL355__REG_XDATA3, data, 3);

    axis_data = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    *sample_x = ADXL355_TWO_COMPLEMENT(axis_data);
}


/* Get y sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_y - Y sample(format - mg).
*/
void adxl355__get_int_y_sample(const adxl355_t *const hndl,
                               int32_t *const sample_y)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;

    hndl->read(ADXL355__REG_YDATA3, data, 3);

    axis_data = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    *sample_y = ADXL355_TWO_COMPLEMENT(axis_data);
}


/* Get z sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] samples_z - Z sample.
*/
void adxl355__get_int_z_sample(const adxl355_t *const hndl,
                               int32_t *const samples_z)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;

    hndl->read(ADXL355__REG_ZDATA3, data, 3);

    axis_data  = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    *samples_z = ADXL355_TWO_COMPLEMENT(axis_data);
}


/* Get x sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_x - X sample(format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_x_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_x,
                                 adxl355__meas_range_t range)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;
    int32_t  axis_res   =  0;

    hndl->read(ADXL355__REG_XDATA3, data, 3);

    axis_data = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    axis_res  = ADXL355_TWO_COMPLEMENT(axis_data);

    *sample_x = ((float32_t)axis_res) / adxl355_scale[range-1];
}


/* Get y sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_y - Y sample(format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_y_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_y,
                                 adxl355__meas_range_t range)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;
    uint32_t axis_res  =  0;

    hndl->read(ADXL355__REG_YDATA3, data, 3);

    axis_data = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    axis_res  = ADXL355_TWO_COMPLEMENT(axis_data);

    *sample_y = ((float32_t)axis_res) / adxl355_scale[range-1];
}


/* Get z sample.
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] sample_z - Z sample (format - mg).
 * [in]  range     - Measurement range.
*/
void adxl355__get_float_z_sample(const adxl355_t *const hndl,
                                 float32_t *const sample_z,
                                 adxl355__meas_range_t range)
{
    uint8_t  data[3]   = {0};
    uint32_t axis_data =  0;
    uint32_t axis_res  =  0;

    hndl->read(ADXL355__REG_ZDATA3, data, 3);

    axis_data = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    axis_res  = ADXL355_TWO_COMPLEMENT(axis_data);

    *sample_z =  ((float32_t)axis_res) / adxl355_scale[range-1];
}


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
                                 int32_t *const sample_z)
{
    uint8_t  data[9]     = {0};
    uint32_t axis_data_x =  0;
    uint32_t axis_data_y =  0;
    uint32_t axis_data_z =  0;

    hndl->read(ADXL355__REG_XDATA3, data, 9);

    axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4] ,data[5]);
    axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7] ,data[8]);

    *sample_x = ADXL355_TWO_COMPLEMENT(axis_data_x);
    *sample_y = ADXL355_TWO_COMPLEMENT(axis_data_y);
    *sample_z = ADXL355_TWO_COMPLEMENT(axis_data_z);
}


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
                                   adxl355__meas_range_t range)
{
    uint8_t   data[9]    = {0};
    uint32_t axis_data_x =  0;
    uint32_t axis_data_y =  0;
    uint32_t axis_data_z =  0;

    uint32_t axis_res_x =  0;
    uint32_t axis_res_y =  0;
    uint32_t axis_res_z =  0;

    hndl->read(ADXL355__REG_XDATA3, data, 9);

    axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
    axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4] ,data[5]);
    axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7] ,data[8]);

    axis_res_x = ADXL355_TWO_COMPLEMENT(axis_data_x);
    axis_res_y = ADXL355_TWO_COMPLEMENT(axis_data_y);
    axis_res_z = ADXL355_TWO_COMPLEMENT(axis_data_z);

    *sample_x = axis_res_x * adxl355_scale[range-1];
    *sample_y = axis_res_y * adxl355_scale[range-1];
    *sample_z = axis_res_z * adxl355_scale[range-1];
}


/* Get aviable samoles count in fifo bufer.
 *
 * [in]  hndl          - ADXL355 handle.
 * [out] samoles_count - pointer to out value.
*/
void adxl355__get_fifo_samples_count(const adxl355_t *const hndl,
                                     uint8_t *const samoles_count)
{
    hndl->read(ADXL355__REG_FIFO_ENTRIES, samoles_count, 1);
}


/* Get value when fifo full condition is triggered.
 *
 * [in]  hndl       - ADXL355 handle.
 * [out] samp_count - Pointer to out value, number of samples
 *      stored in the FIFO that triggers a FIFO_FULL condition.
 *      Values range from 1 to 96.
*/
void adxl355__get_fifo_full_condition(const adxl355_t *const hndl,
                                      uint8_t *const samp_count)
{
    hndl->read(ADXL355__REG_FIFO_SAMPLES, samp_count, 1);
}


/* Set value when fifo full condition is triggered.
 *
 * [in] hndl       - ADXL355 handle.
 * [in] samp_count - Value of number of samples
 *      stored in the FIFO that triggers a FIFO_FULL condition.
 *      Values range from 1 to 96.
*/
void adxl355__set_fifo_full_condition(const adxl355_t *const hndl,
                                      uint8_t samp_count)
{
    hndl->write(ADXL355__REG_FIFO_SAMPLES, &samp_count, 1);
}


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
                                              uint8_t *const count_samples)
{
    uint8_t fifo_count =  0;
    uint8_t data[9]    = {0};

    uint32_t axis_data_x = 0;
    uint32_t axis_data_y = 0;
    uint32_t axis_data_z = 0;

    adxl355__get_fifo_samples_count(hndl, &fifo_count);

    for(uint8_t i = 0; i < (fifo_count / 3); i++)
    {
        hndl->read(ADXL355__REG_FIFO_DATA, data, 9);

        axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
        axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4] ,data[5]);
        axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7] ,data[8]);

        samples_x[i] = ADXL355_TWO_COMPLEMENT(axis_data_x);
        samples_y[i] = ADXL355_TWO_COMPLEMENT(axis_data_y);
        samples_z[i] = ADXL355_TWO_COMPLEMENT(axis_data_z);

        *count_samples = (i+1);
    }
}


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
                                                uint8_t  *const count_samples)
{
    uint8_t fifo_count =  0;
    uint8_t data[9]    = {0};

    uint32_t axis_data_x = 0;
    uint32_t axis_data_y = 0;
    uint32_t axis_data_z = 0;

    int32_t axis_res_x = 0;
    int32_t axis_res_y = 0;
    int32_t axis_res_z = 0;

    adxl355__get_fifo_samples_count(hndl, &fifo_count);


    for(uint8_t i = 0; i < (fifo_count / 3); i++)
    {
        hndl->read(ADXL355__REG_FIFO_DATA, data, 9);

        axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
        axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4] ,data[5]);
        axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7] ,data[8]);

        axis_res_x = ADXL355_TWO_COMPLEMENT(axis_data_x);
        axis_res_y = ADXL355_TWO_COMPLEMENT(axis_data_y);
        axis_res_z = ADXL355_TWO_COMPLEMENT(axis_data_z);

        samples_x[i] = ((float32_t)axis_res_x) / adxl355_scale[range-1];
        samples_y[i] = ((float32_t)axis_res_y) / adxl355_scale[range-1];
        samples_z[i] = ((float32_t)axis_res_z) / adxl355_scale[range-1];

        *count_samples = (i+1);
    }


}


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
                                                uint8_t  *const count_samples)
{
    uint8_t  data[9]     = {0};
    uint32_t axis_data_x =  0;
    uint32_t axis_data_y =  0;
    uint32_t axis_data_z =  0;
    uint8_t  cnt         =  0;

    while(1)
    {
      hndl->read(ADXL355__REG_FIFO_DATA, data, 9);

      if((data[2] & 0x02) != 0)
       break;

      axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1] ,data[2]);
      axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4] ,data[5]);
      axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7] ,data[8]);

      samples_x[cnt] = ADXL355_TWO_COMPLEMENT(axis_data_x);
      samples_y[cnt] = ADXL355_TWO_COMPLEMENT(axis_data_y);
      samples_z[cnt] = ADXL355_TWO_COMPLEMENT(axis_data_z);

      cnt++;
    }

    *count_samples = cnt;
}


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
                                                  uint8_t  *const count_samples)
  {
      uint8_t  data[9]     = {0};
      uint32_t axis_data_x =  0;
      uint32_t axis_data_y =  0;
      uint32_t axis_data_z =  0;

      int32_t axis_res_x =  0;
      int32_t axis_res_y =  0;
      int32_t axis_res_z =  0;

      uint8_t  cnt         =  0;
      


      while(1)
      {
          hndl->read(ADXL355__REG_FIFO_DATA, data, 9);

          if((data[2] & 0x02) != 0)
            break;
          
          data[2] = data[2] & (~0x03);

          axis_data_x = ADXL355_COMBINE_3BYTES(data[0], data[1], data[2]);
          axis_data_y = ADXL355_COMBINE_3BYTES(data[3], data[4], data[5]);
          axis_data_z = ADXL355_COMBINE_3BYTES(data[6], data[7], data[8]);

          axis_res_x = ADXL355_TWO_COMPLEMENT(axis_data_x);
          axis_res_y = ADXL355_TWO_COMPLEMENT(axis_data_y);
          axis_res_z = ADXL355_TWO_COMPLEMENT(axis_data_z);

          samples_x[cnt] = (((float32_t)axis_res_x) / adxl355_scale[range-1])*1000.0f;
          samples_y[cnt] = (((float32_t)axis_res_y) / adxl355_scale[range-1])*1000.0f;
          samples_z[cnt] = (((float32_t)axis_res_z) / adxl355_scale[range-1])*1000.0f;
          cnt++;
            
          if(cnt >= 32)
            break;
      }
      
      *count_samples = cnt;
  }


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
                              int32_t *const z_offset)
{
    uint8_t x_hi_data = 0;
    uint8_t x_lo_data = 0;

    uint8_t y_hi_data = 0;
    uint8_t y_lo_data = 0;

    uint8_t z_hi_data = 0;
    uint8_t z_lo_data = 0;

    int32_t x_data = 0;
    int32_t y_data = 0;
    int32_t z_data = 0;

    hndl->read(ADXL355__REG_OFFSET_X_H, &x_hi_data, 1);
    hndl->read(ADXL355__REG_OFFSET_X_L, &x_lo_data, 1);

    hndl->read(ADXL355__REG_OFFSET_Y_H, &y_hi_data, 1);
    hndl->read(ADXL355__REG_OFFSET_Y_L, &y_lo_data, 1);

    hndl->read(ADXL355__REG_OFFSET_Z_H, &z_hi_data, 1);
    hndl->read(ADXL355__REG_OFFSET_Z_L, &z_lo_data, 1);

    x_data = ADXL355_COMBINE_2BYTES(x_hi_data, x_lo_data) >> 4;
    x_data = ADXL355_COMBINE_2BYTES(x_hi_data, x_lo_data) >> 4;
    x_data = ADXL355_COMBINE_2BYTES(x_hi_data, x_lo_data) >> 4;

    *x_offset = ADXL355_TWO_COMPLEMENT(x_data);
    *y_offset = ADXL355_TWO_COMPLEMENT(y_data);
    *z_offset = ADXL355_TWO_COMPLEMENT(z_data);
}


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
                              const int32_t z_offset)
{
    int16_t x_work_val = (x_offset >> 4);
    int16_t y_work_val = (y_offset >> 4);
    int16_t z_work_val = (z_offset >> 4);

    uint8_t x_hi_data = ADXL355_GET_HI_16(x_work_val);
    uint8_t x_lo_data = ADXL355_GET_LO_16(x_work_val);

    uint8_t y_hi_data = ADXL355_GET_HI_16(y_work_val);
    uint8_t y_lo_data = ADXL355_GET_LO_16(y_work_val);

    uint8_t z_hi_data = ADXL355_GET_HI_16(z_work_val);
    uint8_t z_lo_data = ADXL355_GET_LO_16(z_work_val);


    hndl->write(ADXL355__REG_OFFSET_X_H, &x_hi_data, 1);
    hndl->write(ADXL355__REG_OFFSET_X_L, &x_lo_data, 1);

    hndl->write(ADXL355__REG_OFFSET_Y_H, &y_hi_data, 1);
    hndl->write(ADXL355__REG_OFFSET_Y_L, &y_lo_data, 1);

    hndl->write(ADXL355__REG_OFFSET_Z_H, &z_hi_data, 1);
    hndl->write(ADXL355__REG_OFFSET_Z_L, &z_lo_data, 1);
}


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
                                const adxl355__meas_range_t range)
{
    int32_t x_offset_val = 0;
    int32_t y_offset_val = 0;
    int32_t z_offset_val = 0;

    adxl355__get_int_offsets(hndl,
                             &x_offset_val,
                             &y_offset_val,
                             &z_offset_val);

    *x_offset = ((float32_t)x_offset_val) / adxl355_scale[range-1];
    *y_offset = ((float32_t)y_offset_val) / adxl355_scale[range-1];
    *z_offset = ((float32_t)z_offset_val) / adxl355_scale[range-1];
}


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
                                adxl355__meas_range_t range)
{

    int32_t x_opposite = (int32_t)(x_offset * adxl355_scale[range-1]);
    int32_t y_opposite = (int32_t)(y_offset * adxl355_scale[range-1]);
    int32_t z_opposite = (int32_t)(z_offset * adxl355_scale[range-1]);

    adxl355__set_int_offsets(hndl,
                             x_opposite,
                             y_opposite,
                             z_opposite);
}


/* Get activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in] hndl        - ADXL355 handle.
 * [out] act_detect - detect value.
*/
void adxl355__get_int_act_detect_val(const adxl355_t *const hndl,
                                     uint32_t *const act_detect)
{
    uint8_t hi_data = 0;
    uint8_t lo_data = 0;

    hndl->read(ADXL355__REG_ACT_THRESH_H, &hi_data, 1);
    hndl->read(ADXL355__REG_ACT_THRESH_L, &lo_data, 1);

    *act_detect  = ADXL355_COMBINE_2BYTES(hi_data, lo_data) >> 4;
}


/* Set activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in] hndl        - ADXL355 handle.
 * [in] act_detect  - detect value.
*/
void adxl355__set_int_act_detect_val(const adxl355_t *const hndl,
                                      const uint32_t act_detect)
{
    uint16_t work_val = (act_detect >> 4);

    uint8_t hi_data = ADXL355_GET_HI_16(work_val);
    uint8_t lo_data = ADXL355_GET_LO_16(work_val);

    hndl->write(ADXL355__REG_ACT_THRESH_H, &hi_data, 1);
    hndl->write(ADXL355__REG_ACT_THRESH_L, &lo_data, 1);
}


/* Get activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in]  hndl        - ADXL355 handle.
 * [out] act_detect  - detect value (format in mg).
 * [in]  range       - Measurement range.
*/
void adxl355__get_float_act_detect_val(const adxl355_t *const hndl,
                                       float32_t *const act_detect,
                                       adxl355__meas_range_t range)
{
    uint32_t int_act_detect  = 0;

    adxl355__get_int_act_detect_val(hndl, &int_act_detect);

    *act_detect = ((float32_t)int_act_detect) / adxl355_scale[range-1];
}


/* Set activity detection value.
 * Value for activity detection, value work for all axes.
 *
 * [in]  hndl        - ADXL355 handle.
 * [in] act_detect   - detect value (format in mg).
 * [in]  range       - Measurement range.
*/
void adxl355__set_float_act_detect_val(const adxl355_t *const hndl,
                                       const float32_t act_detect,
                                       adxl355__meas_range_t range)
{
    uint32_t int_act_detect  = (uint32_t) (((float32_t)act_detect) * adxl355_scale[range-1]);

    adxl355__set_int_act_detect_val(hndl, int_act_detect);
}


/* Get activity detection count.
 * Number of consecutive events above threshold required to detect activity.
 *
 * [in]  hndl        - ADXL355 handle.
 * [out] act_detect  - detection count
*/
void adxl355__get_act_detect_count(const adxl355_t *const hndl,
                                   uint8_t *const act_detect_cnt)
{
   hndl->read(ADXL355__REG_ACT_COUNT, act_detect_cnt, 1);
}


/* Set activity detection count.
 * Number of consecutive events above threshold required to detect activity.
 *
 * [in]  hndl        - ADXL355 handle.
 * [in] act_detect   - detection count
*/
void adxl355__set_act_detect_count(const adxl355_t *const hndl,
                                   uint8_t act_detect_cnt)
{
    hndl->write(ADXL355__REG_ACT_COUNT, &act_detect_cnt, 1);
}


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
                                     const bool z_axis)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_ACT_EN, &data, 1);

    data = ADXL355_BITS_SET_ACT_ENABLE_X(data, x_axis);
    data = ADXL355_BITS_SET_ACT_ENABLE_Y(data, y_axis);
    data = ADXL355_BITS_SET_ACT_ENABLE_Z(data, z_axis);

    hndl->write(ADXL355__REG_ACT_EN, &data, 1);
}


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
                                     bool *const z_axis)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_ACT_EN, &data, 1);

    *x_axis = ADXL355_BITS_IS_ACT_ENABLE_X(data);
    *y_axis = ADXL355_BITS_IS_ACT_ENABLE_Y(data);
    *z_axis = ADXL355_BITS_IS_ACT_ENABLE_Z(data);
}


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
                         adxl355_odr_and_lp_filter_t *const odr_and_lpf)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_FILTER, &data, 1);

    *hp_filter   = (adxl355__hp_filter_t)ADXL355_BITS_GET_HPF_VAL(data);
    *odr_and_lpf = (adxl355_odr_and_lp_filter_t)ADXL355_BITS_GET_ODR_LPF_VAL(data);
}


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
                         const adxl355_odr_and_lp_filter_t odr_and_lpf)
{

    uint8_t data = 0;

    hndl->read(ADXL355__REG_FILTER, &data, 1);

    data = ADXL355_BITS_SET_HPF_VAL(data, (uint8_t)hp_filter);
    data = ADXL355_BITS_SET_ODR_LPF_VAL(data, (uint8_t)odr_and_lpf);

    hndl->write(ADXL355__REG_FILTER, &data, 1);
}


/* Get data ready pin mode,
 *
 * [in] hndl             - ADXL355 handle.
 * [out] drdy_is_enable  − True if DRDY output is enable
 *                         modes where it is normally signal data ready.
 *
*/
void adxl355__get_drdy_pin_mode(const adxl355_t *const hndl,
                                bool *const drdy_is_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    *drdy_is_enable = ADXL355_BITS_IS_DRDY_ON(data);
}


/* Set data ready pin mode,
 *
 * [in] hndl             - ADXL355 handle.
 * [in] drdy_is_enable   − Set true to DRDY output is 0 in
 *                         modes where it is normally signal data ready.
*/
void adxl355__set_drdy_pin_mode(const adxl355_t *const hndl,
                                const bool drdy_enable)
{
    uint8_t data  = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    data = ADXL355_BITS_SET_DRDY_ENABLE(data, drdy_enable);

    hndl->write(ADXL355__REG_POWER_CTL, &data, 1);
}


/* Get temperature measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] drdy_is_enable   − True if temperature measurement mode is enable.
*/
void adxl355__get_temp_mode(const adxl355_t *const hndl,
                            bool *const temp_meas_is_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    *temp_meas_is_enable = ADXL355_BITS_IS_TEMP_ON(data);
}


/* Get temperature measurement mode
 *
 * [in] hndl             - ADXL355 handle.
 * [in] drdy_is_enable   − Set true to enable temperature measurement mode.
*/
void adxl355__set_temp_mode(const adxl355_t *const hndl,
                            const bool temp_meas_enable)
{
    uint8_t data       = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    data = ADXL355_BITS_SET_TEMP_MEAS_ENABLE(data, temp_meas_enable);

    hndl->write(ADXL355__REG_POWER_CTL, &data, 1);
}


/* Get measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] meas_is_enable   − True if measurement mode is emabled.
*/
void adxl355__get_meas_mode(const adxl355_t *const hndl,
                            bool *const meas_is_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    *meas_is_enable = ADXL355_BITS_IS_STANDBY_ON(data);
}


/* Set measurement mode
 *
 * [in]  hndl             - ADXL355 handle.
 * [out] meas_is_enable   − Set true to emabled measurement mode.
*/
void adxl355__set_meas_mode(const adxl355_t *const hndl,
                            const bool meas_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_POWER_CTL, &data, 1);

    data = ADXL355_BITS_SET_STANDBY_MODE_ENABLE(data, !meas_enable);

    hndl->write(ADXL355__REG_POWER_CTL, &data, 1);
}


/* Get mrasurement range.
 *
 * [in]  hndl  - ADXL355 handle.
 * [out] range - Measurement range.
 *
 * return operation code.
*/
void adxl355__get_range(const adxl355_t  *const hndl,
                        adxl355__meas_range_t *const range)
{
    uint8_t data  = 0;

    hndl->read(ADXL355__REG_RANGE,  &data,   1);

    *range = (adxl355__meas_range_t) ADXL355_BITS_GET_RANGE_VAL(data);
}


/* Set mrasurement range.
 *
 * [in] hndl  - ADXL355 handle.
 * [in] range - Measurement range.
 *
 * return operation code.
*/
void adxl355__set_range(const adxl355_t *const hndl,
                        const adxl355__meas_range_t range)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_RANGE,  &data,   1);

    data = ADXL355_BITS_SET_RANGE_VAL(data, (uint8_t)range);

    hndl->write(ADXL355__REG_RANGE,  &data,   1);
}


/* Get interrupt palarity
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] int_pol   − Interupt polarity.
 *
 * return operation code.
*/
void adxl355__get_int_polarity(const adxl355_t *const hndl,
                               adxl355__int_polarity_t *const int_pol)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_RANGE, &data, 1);

    *int_pol = (adxl355__int_polarity_t)ADXL355_BITS_GET_INT_POLARITY(data);
}


/* Set interrupt palarity
 *
 * [in] hndl      - ADXL355 handle.
 * [in] int_pol   − Interupt polarity.
*/
void adxl355__set_int_polarity(const adxl355_t *const hndl,
                               const adxl355__int_polarity_t int_pol)
{
   uint8_t data = 0;

   hndl->read(ADXL355__REG_RANGE, &data, 1);

   data = ADXL355_BITS_SET_INT_POLARITY(data, (uint8_t)int_pol);

   hndl->write(ADXL355__REG_RANGE, &data, 1);
}


/* Get i2c speed
 *
 * [in] hndl    - ADXL355 handle.
 * [out] speed  − I2c speed.
*/
void adxl355__get_i2c_speed(const adxl355_t *const hndl,
                            adxl355__i2c_speed_t *const speed)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_RANGE, &data, 1);

    *speed = (adxl355__i2c_speed_t)ADXL355_BITS_GET_I2C_SPEED(data);
}


/* Set i2c speed
 *
 * [in] hndl   - ADXL355 handle.
 * [in] speed  − I2c speed.
*/
void adxl355__set_i2c_speed(const adxl355_t *const hndl,
                            const adxl355__i2c_speed_t speed)
{

    uint8_t data = 0;

    hndl->read(ADXL355__REG_RANGE, &data, 1);

    data = ADXL355_BITS_SET_I2C_SPEED(data, (uint8_t)speed);

    hndl->write(ADXL355__REG_RANGE, &data, 1);
}


/* Get interrupt map
 *
 * [in]  hndl      - ADXL355 handle.
 * [out] int_map   − Interrupt map.
*/
void adxl355__get_int_map(const adxl355_t *const hndl,
                          adxl355__interrupt_map_t *const int_map)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_INT_MAP, &data, 1);

    int_map->act_en1  = ADXL355_BITS_IS_ACT_EN1(data);
    int_map->act_en2  = ADXL355_BITS_IS_ACT_EN2(data);
    int_map->ovr_en1  = ADXL355_BITS_IS_OVR_EN1(data);
    int_map->ovr_en2  = ADXL355_BITS_IS_OVR_EN2(data);
    int_map->rdy_en1  = ADXL355_BITS_IS_RDY_EN1(data);
    int_map->rdy_en2  = ADXL355_BITS_IS_RDY_EN2(data);
    int_map->full_en1 = ADXL355_BITS_IS_FULL_EN1(data);
    int_map->full_en2 = ADXL355_BITS_IS_FULL_EN2(data);

}


/* Set interrupt map
 *
 * [in] hndl      - ADXL355 handle.
 * [in] int_map   − Interrupt map.
 *
 * return operation code.
*/
void adxl355__set_int_map(const adxl355_t *const hndl,
                          const adxl355__interrupt_map_t int_map)
{
    uint8_t data  = 0;

    data = ADXL355_BITS_SET_ACT_EN2(data,  (uint8_t)int_map.act_en2);
    data = ADXL355_BITS_SET_OVR_EN2(data,  (uint8_t)int_map.ovr_en2);
    data = ADXL355_BITS_SET_FULL_EN2(data, (uint8_t)int_map.full_en2);
    data = ADXL355_BITS_SET_RDY_EN2(data,  (uint8_t)int_map.rdy_en2);
    data = ADXL355_BITS_SET_ACT_EN1(data,  (uint8_t)int_map.act_en1);
    data = ADXL355_BITS_SET_OVR_EN1(data,  (uint8_t)int_map.ovr_en1);
    data = ADXL355_BITS_SET_FULL_EN1(data, (uint8_t)int_map.full_en1);
    data = ADXL355_BITS_SET_RDY_EN1(data,  (uint8_t)int_map.rdy_en1);

    hndl->write(ADXL355__REG_INT_MAP, &data, 1);
}


/* Get self test force mode
 *
 * [in]  hndl                - ADXL355 handle.
 * [out] self_test_is_force  − True if self test is force enabled.
*/
void adxl355__get_self_test_force(const adxl355_t *const hndl,
                                  bool *const  self_test_is_force)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_SELF_TEST, &data, 1);

    *self_test_is_force = ADXL355_BITS_IS_ST1(data);
}


/* Set self test force mode
 *
 * [in] hndl                - ADXL355 handle.
 * [in] self_test_is_force  −  Set true to enable self test force
*/
void adxl355__set_self_test_force(const adxl355_t *const hndl,
                                  const bool self_test_is_force)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_SELF_TEST, &data, 1);

    data = ADXL355_BITS_SET_ST1_ENABLE(data, (uint8_t)self_test_is_force);

    hndl->write(ADXL355__REG_SELF_TEST, &data, 1);
}


/* Get self test mode.
 *
 * [in]  hndl                - ADXL355 handle.
 * [out] self_test_is_force  − True if self test is enabled.
*/
void adxl355__get_self_test_mode(const adxl355_t *const hndl,
                                 bool *const  self_test_is_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_SELF_TEST, &data, 1);

    *self_test_is_enable = ADXL355_BITS_IS_ST2(data);
}


/* Set self test mode.
 *
 * [in]  hndl               - ADXL355 handle.
 * [in] self_test_is_force  − Set true to enable self test mode
*/
void adxl355__set_self_test_mode(const adxl355_t *const hndl,
                                 const bool self_test_is_enable)
{
    uint8_t data = 0;

    hndl->read(ADXL355__REG_SELF_TEST, &data, 1);

    data = ADXL355_BITS_SET_ST2_ENABLE(data, (uint8_t)self_test_is_enable);

    hndl->write(ADXL355__REG_SELF_TEST, &data, 1);
}


/* Resets ADXL355 device, similar to a power-on reset (POR).
 *
 * [in]  hndl  - ADXL355 handle.
*/
void adxl355__reset_dev(const adxl355_t *const hndl)
{

   uint8_t data = 0x52;

   hndl->write(ADXL355__REG_RESET, &data, 1);
}
