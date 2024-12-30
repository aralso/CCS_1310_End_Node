/*

 ============================================================================
 *  @file       SensorI2C.h
 *
 *  @brief      Simple interface to the TI-RTOS driver. Also manages switching
 *              between I2C-buses.
 *
 *  ============================================================================
 */
#ifndef SENSOR_I2C_H
#define SENSOR_I2C_H

/*********************************************************************
 * INCLUDES
 */
#include "stdbool.h"

/*********************************************************************
 * CONSTANTS
 */
#define SENSOR_I2C_0     0
#define SENSOR_I2C_1     1
#define SENSOR_I2C_NONE  -1

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * FUNCTIONS
 */
bool SensorI2C_open(void);
bool SensorI2C_select(uint8_t interface, uint8_t slaveAddress);
bool SensorI2C_readReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
bool SensorI2C_writeReg(uint8_t addr, uint8_t *pBuf, uint8_t nBytes);
void SensorI2C_deselect(void);
void SensorI2C_close(void);

bool SensorI2C_read(uint8_t *data, uint8_t len);
bool SensorI2C_write(uint8_t *data, uint8_t len);

bool SensorI2C_write_1_Reg(uint8_t addr, uint8_t data_val);
uint8_t SensorI2C_read_1_Reg(uint8_t addr);
void Select_addr_i2c (uint8_t address);
void SensorTest(void);


#endif /* SENSOR_I2C_H */
