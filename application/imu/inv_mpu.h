/**
  ******************************************************************************
  * @file    inv_mpu.h
  * @author  Ji Chen
  * @brief   MPUxxxx sensor dirver
  ******************************************************************************
  * @attention
  ******************************************************************************
  */

#ifndef INV_MPU_H
#define INV_MPU_H

#include <stdint.h>

int mpu_reset(void);
int mpu_set_gyro_fsr(int fsr);
int mpu_set_accel_fsr(int fsr);
int mpu_set_lpf(int lpf);
int mpu_set_sample_rate(int rate);
int mpu_set_int(int enable);
int mpu_read_data_dma(uint8_t *buf);
void mpu_convert_data(uint8_t *buf, uint16_t *data);

#endif /* end of INV_MPU_H */

/******************************** END OF FILE *********************************/
