
/*
 * sht40.c
 *
 *  Created on: 25 apr 2026
 *      Author: sandro stocchi
 */
#include "sht40.h"

#define SHT40_CMD_MEASURE_HIGH_PRECISION 0xFD

static uint8_t SHT40_CRC(uint8_t *data, int len)
{
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc <<= 1;
        }
    }
    return crc;
}

HAL_StatusTypeDef SHT40_Read(SHT40_Data *data, I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = SHT40_CMD_MEASURE_HIGH_PRECISION;
    uint8_t rx[6];

    // invia comando misura
    if (HAL_I2C_Master_Transmit(hi2c, SHT40_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10); // tempo conversione (~8-10 ms)

    // leggi 6 byte
    if (HAL_I2C_Master_Receive(hi2c, SHT40_I2C_ADDR, rx, 6, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;

    // verifica CRC
    if (SHT40_CRC(&rx[0], 2) != rx[2]) return HAL_ERROR;
    if (SHT40_CRC(&rx[3], 2) != rx[5]) return HAL_ERROR;

    // conversione dati
    uint16_t rawT = (rx[0] << 8) | rx[1];
    uint16_t rawRH = (rx[3] << 8) | rx[4];

    data->temperature = -45 + 175 * ((float)rawT / 65535.0f);
    data->humidity = -6 + 125 * ((float)rawRH / 65535.0f);

    return HAL_OK;
}
