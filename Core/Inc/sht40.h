#ifndef SHT40_H
	#define SHT40_H

#include "stm32u5xx_hal.h" // cambia con il tuo MCU

#define SHT40_I2C_ADDR (0x44 << 1) // indirizzo 7bit shiftato per HAL

typedef struct {
    float temperature;
    float humidity;
} SHT40_Data;

HAL_StatusTypeDef SHT40_Read(SHT40_Data *data, I2C_HandleTypeDef *hi2c);

#endif
