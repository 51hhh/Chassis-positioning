#include "AS5048.h"
#include "spi.h"

#define AS5048_SPI_TIMEOUT_MS 2U
#define AS5048_DEADBAND_ENTER 3
#define AS5048_DEADBAND_EXIT 6

#if !AS5048_RAW_CALIBRATION_MODE
static int as5048_abs_int(int value)
{
	return (value < 0) ? -value : value;
}

static int as5048_median3(int a, int b, int c)
{
	if (a > b) {
		int tmp = a;
		a = b;
		b = tmp;
	}
	if (b > c) {
		int tmp = b;
		b = c;
		c = tmp;
	}
	if (a > b) {
		int tmp = a;
		a = b;
		b = tmp;
	}
	return b;
}
#endif

AS5048 AS5048s[AS5048_NUMBER];

static uint8_t as5048_is_valid_id(int AS5048_ID)
{
	return (AS5048_ID >= 1 && AS5048_ID <= AS5048_NUMBER) ? 1U : 0U;
}

static uint8_t as5048_even_parity_ok(uint16_t value)
{
	value ^= (uint16_t)(value >> 8);
	value ^= (uint16_t)(value >> 4);
	value ^= (uint16_t)(value >> 2);
	value ^= (uint16_t)(value >> 1);
	return ((value & 1U) == 0U) ? 1U : 0U;
}

static uint16_t as5048_make_read_cmd(uint16_t registerAddress)
{
	uint16_t cmd = AS5048_READ_FLAG | (registerAddress & AS5048_ADDRESS_MASK);

	if (as5048_even_parity_ok(cmd) == 0U) {
		cmd |= AS5048_PARITY_FLAG;
	}
	return cmd;
}

static HAL_StatusTypeDef as5048_transfer16(AS5048 *AS5, uint16_t tx, uint16_t *rx)
{
	HAL_StatusTypeDef status;
	uint16_t rx_word = 0U;

	HAL_GPIO_WritePin(AS5->GPIOx, AS5->GPIO_Pin, GPIO_PIN_RESET);
	status = HAL_SPI_TransmitReceive(AS5->spi_number,
	                                 (uint8_t *)&tx,
	                                 (uint8_t *)&rx_word,
	                                 1U,
	                                 AS5048_SPI_TIMEOUT_MS);
	HAL_GPIO_WritePin(AS5->GPIOx, AS5->GPIO_Pin, GPIO_PIN_SET);

	if (rx != 0) {
		*rx = rx_word;
	}
	return status;
}

static uint8_t as5048_clear_error(AS5048 *AS5)
{
	uint16_t ignored = 0U;
	uint16_t response = 0U;

	if (as5048_transfer16(AS5, as5048_make_read_cmd(SPI_REG_CLRERR), &ignored) != HAL_OK) {
		return 0xFFU;
	}
	if (as5048_transfer16(AS5, SPI_NOP, &response) != HAL_OK) {
		return 0xFFU;
	}
	return (uint8_t)(response & 0x0007U);
}

void AS5048_init(int AS5048_ID,SPI_HandleTypeDef *spi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	if (as5048_is_valid_id(AS5048_ID) == 0U) {
		return;
	}
	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	AS5->AS5048_ID = AS5048_ID;
	AS5->spi_number = spi;
	AS5->GPIOx = GPIOx;
	AS5->GPIO_Pin = GPIO_Pin;
	AS5->angle = 0;
	AS5->total_angle = 0;
	AS5->cirle = 0;
	AS5->last_angle = AS5->angle;
	AS5->delta_dis = 0;
	AS5->diff_hist[0] = 0;
	AS5->diff_hist[1] = 0;
	AS5->diff_hist[2] = 0;
	AS5->motion_state = 0;
	AS5->valid = 0U;
	AS5->last_error_flags = 0U;
	AS5->last_raw_response = 0U;
	AS5->error_count = 0U;

	HAL_GPIO_WritePin(AS5->GPIOx, AS5->GPIO_Pin, GPIO_PIN_SET);
}

uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress)
{
	uint16_t ignored = 0U;
	uint16_t response = 0U;
	uint16_t cmd;

	if (as5048_is_valid_id(AS5048_ID) == 0U) {
		return 0U;
	}
	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	cmd = as5048_make_read_cmd(registerAddress);

	if (as5048_transfer16(AS5, cmd, &ignored) != HAL_OK) {
		AS5->valid = 0U;
		AS5->last_error_flags = 0xFEU;
		AS5->error_count++;
		return (uint16_t)AS5->angle;
	}

	if (as5048_transfer16(AS5, SPI_NOP, &response) != HAL_OK) {
		AS5->valid = 0U;
		AS5->last_error_flags = 0xFDU;
		AS5->error_count++;
		return (uint16_t)AS5->angle;
	}

	AS5->last_raw_response = response;

	/* Detect stuck-at-zero: if response is 0x0000 AND the previous non-zero reading
	 * is far away (indicating a jump, not smooth motion through zero), reject it.
	 * A genuine angle=0 will have prior readings near 0 (e.g., 0xFFFF, 0x0001).
	 * A spurious SPI glitch reads 0x0000 when the real angle is ~7448 (encoder 1). */
	if (response == 0x0000U && AS5->angle != 0) {
		int dist = (AS5->angle < AS5048_ENCODER_HALF_COUNTS) ? AS5->angle : (AS5048_ENCODER_COUNTS_PER_REV - AS5->angle);
		if (dist > 100) {  /* >100 counts from zero = ~0.6° — unlikely smooth transition */
			AS5->valid = 0U;
			AS5->last_error_flags = 0xFBU;
			AS5->error_count++;
			return (uint16_t)AS5->angle;
		}
	}

	if (as5048_even_parity_ok(response) == 0U) {
		AS5->valid = 0U;
		AS5->last_error_flags = 0xFCU;
		AS5->error_count++;
		(void)as5048_clear_error(AS5);
		return (uint16_t)AS5->angle;
	}

	if ((response & AS5048_ERROR_FLAG) != 0U) {
		AS5->valid = 0U;
		AS5->last_error_flags = as5048_clear_error(AS5);
		AS5->error_count++;
		return (uint16_t)AS5->angle;
	}

	AS5->valid = 1U;
	AS5->last_error_flags = 0U;
	return (uint16_t)(response & AS5048_DATA_MASK);
}

void AS5048_getREGValue(const int AS5048_ID)
{
	if (as5048_is_valid_id(AS5048_ID) == 0U) {
		return;
	}
	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	uint16_t angle = AS5048_Read(AS5048_ID, SPI_REG_DATA);
	if (AS5->valid != 0U) {
		AS5->angle = (int)(angle & AS5048_DATA_MASK);
	}
}

void AS5048_dataUpdate(const int AS5048_ID)
{
	if (as5048_is_valid_id(AS5048_ID) == 0U) {
		return;
	}
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	int diff = AS5->angle - AS5->last_angle;
	int filtered_diff;
#if !AS5048_RAW_CALIBRATION_MODE
	int abs_filtered_diff;
#endif

	if (AS5->valid == 0U) {
		AS5->delta_dis = 0;
		AS5->last_angle = AS5->angle;  /* Anchor to prevent jump on valid transition */
		return;
	}

	if (diff > AS5048_ENCODER_HALF_COUNTS) {
		diff -= AS5048_ENCODER_COUNTS_PER_REV;
		AS5->cirle--;
	} else if (diff < -AS5048_ENCODER_HALF_COUNTS) {
		diff += AS5048_ENCODER_COUNTS_PER_REV;
		AS5->cirle++;
	}

#if AS5048_RAW_CALIBRATION_MODE
	filtered_diff = diff;
#else

	AS5->diff_hist[0] = AS5->diff_hist[1];
	AS5->diff_hist[1] = AS5->diff_hist[2];
	AS5->diff_hist[2] = diff;
	filtered_diff = as5048_median3(AS5->diff_hist[0], AS5->diff_hist[1], AS5->diff_hist[2]);
	abs_filtered_diff = as5048_abs_int(filtered_diff);

	if (AS5->motion_state != 0U) {
		if (abs_filtered_diff <= AS5048_DEADBAND_ENTER) {
			AS5->motion_state = 0U;
			filtered_diff = 0;
		}
	} else {
		if (abs_filtered_diff < AS5048_DEADBAND_EXIT) {
			filtered_diff = 0;
		} else {
			AS5->motion_state = 1U;
		}
	}
#endif

	AS5->delta_dis = filtered_diff;
	AS5->total_angle += filtered_diff;
	AS5->last_angle = AS5->angle;
}
