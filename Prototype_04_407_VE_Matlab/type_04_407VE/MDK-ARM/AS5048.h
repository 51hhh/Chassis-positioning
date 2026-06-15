#ifndef __AS5048_H
#define __AS5048_H

#include "main.h"

#define AS5048_NUMBER 2  
#define AS5048_RAW_CALIBRATION_MODE 1

#define AS5048_ENCODER_COUNTS_PER_REV 16384
#define AS5048_ENCODER_HALF_COUNTS    (AS5048_ENCODER_COUNTS_PER_REV / 2)
#define AS5048_DATA_MASK           0x3FFFU
#define AS5048_ERROR_FLAG          0x4000U
#define AS5048_PARITY_FLAG         0x8000U
#define AS5048_READ_FLAG           0x4000U
#define AS5048_ADDRESS_MASK        0x3FFFU

#define SPI_NOP                    0x0000U
#define SPI_REG_AGC                0x3FFDU
#define SPI_REG_MAG                0x3FFEU
#define SPI_REG_DATA               0x3FFFU
#define SPI_REG_CLRERR             0x0001U
#define SPI_REG_ZEROPOS_HI         0x0016U
#define SPI_REG_ZEROPOS_LO         0x0017U

void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);
uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress);
void AS5048_getREGValue(const int AS5048_ID);
void AS5048_dataUpdate(const int AS5048_ID);
/**
 * @brief AS5048_STRUCT
 */
typedef struct {
	int AS5048_ID;
	SPI_HandleTypeDef *spi_number;       ///< SPI
	GPIO_TypeDef *GPIOx; ///<GPIO_CS
	uint16_t GPIO_Pin; ///<GPIO_PIN_CS
	int angle;
	int last_angle; //  cc_direction
	int total_angle;
	int cirle;
	int delta_dis;
	int diff_hist[3];
	uint8_t motion_state;
	uint8_t valid;
	uint8_t last_error_flags;
	uint16_t last_raw_response;
	uint32_t error_count;

} AS5048;

/**
 * @brief AS5048_OBJECT_
 */

extern AS5048 AS5048s[AS5048_NUMBER];

#endif
