#include <sx126x_hal.h>
#include <stm32f0xx_hal.h>
#include <gpio.h>
#include <spi.h>

sx126x_hal_status_t sx126x_hal_write(const void *context, const uint8_t *command, const uint16_t command_length,
									 const uint8_t *data, const uint16_t data_length)
{
	uint8_t *datai = (uint8_t *)data;
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, datai, data_length, 1000);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return 0;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command, const uint16_t command_length,
									uint8_t *data, const uint16_t data_length)
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Receive(&hspi1, data, data_length, 1000);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return 0;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context)
{
	HAL_GPIO_WritePin(LRST_GPIO_Port, LRST_Pin, GPIO_PIN_RESET);
	return 0;
}

sx126x_hal_status_t sx126x_hal_wakeup()
{
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	// 向0xC0寄存器写入0x00 （用户实现）
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	return 0;
}

int sx126x_get_busy(void)
{
	return luat_gpio_get(BUSY_PIN);
}
