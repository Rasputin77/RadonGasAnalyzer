#include "AppGas.h"
#include "ux_api.h"                     // Core USBX
#include "ux_device_stack.h"            // Device stack
#include "ux_device_class_cdc_acm.h"   // CDC-ACM
#include "ux_device_cdc_acm.h"   // CDC-ACM
#include "stm32u5xx_hal.h"
#include "sht40.h"



extern I2C_HandleTypeDef hi2c1;

#define CCS811_ADDR  (0x5A << 1 )
//#define HDC1080_ADDR (0x40 << 1)
#define BMP280_ADDR  (0x76 << 1)

// Indirizzi I2C
#define BMP280_I2C_ADDR_0   (0x76 << 1)
#define BMP280_I2C_ADDR_1   (0x77 << 1)
#define BMP280_I2C_ADDR     BMP280_I2C_ADDR_0   // usa quello che ti serve

// Registri principali
#define BMP280_REG_ID            0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_REG_STATUS        0xF3
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5

// Registri dati pressione/temperatura
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_PRESS_LSB     0xF8
#define BMP280_REG_PRESS_XLSB    0xF9

#define BMP280_REG_TEMP_MSB      0xFA
#define BMP280_REG_TEMP_LSB      0xFB
#define BMP280_REG_TEMP_XLSB     0xFC

// Registri calibrazione (0x88–0xA1)
#define BMP280_REG_CALIB_START   0x88
#define BMP280_REG_CALIB_END     0xA1

// Valori speciali
#define BMP280_CHIP_ID           0x58
#define BMP280_SOFT_RESET_CMD    0xB6

//------------------------------------

//#define HDC1080_ADDR (0x40 << 1)
#define HDC1080_TEMP_HUM_REG 0x00
#define HDC1080_REG_TEMP_HUM     0x00
#define HDC1080_REG_CONFIG       0x02

/* Register addresses */
#define HDC1080_TEMPERATURE				0x00
#define HDC1080_HUMIDITY 					0x01
#define HDC1080_CONFIG						0x02
#define HDC1080_SERIAL_ID1				0xfb
#define HDC1080_SERIAL_ID2				0xfc
#define HDC1080_SERIAL_ID3				0xfd
#define HDC1080_ID_MANU						0xfe
#define HDC1080_ID_DEV						0xff

#define HDC1080_RH_RES_14					0x00
#define HDC1080_RH_RES_11					0x01
#define HDC1080_RH_RES8						0x02

#define HDC1080_T_RES_14					0x00
#define HDC1080_T_RES_11					0x01
//#define HDC1080_ADDR 0x40


HAL_StatusTypeDef hdc1080_read_reg(I2C_HandleTypeDef *hi2c, uint16_t delay, uint8_t reg, uint16_t *val);
HAL_StatusTypeDef hdc1080_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t val);
HAL_StatusTypeDef hdc1080_measure(I2C_HandleTypeDef *hi2c,   uint8_t temp_res, uint8_t humidres, uint8_t heater, 	uint8_t *bat_stat, double *temperature,	double *humidity);
HAL_StatusTypeDef hdc1080_get_device_id(I2C_HandleTypeDef *hi2c, uint64_t *serial, uint16_t *manuf, uint16_t *device);



#include "stm32u5xx_hal.h"

#define HDC1080_ADDR   (0x40 << 1)

typedef enum {
    SENSOR_HDC1080,
    SENSOR_SHT21
} sensor_type_t;

sensor_type_t detected = SENSOR_HDC1080;

// -------------------------------------------------------------------
// Rileva se il sensore è un SHT21/Si7021 leggendo il registro 0xE7
// -------------------------------------------------------------------
static void detect_sensor(I2C_HandleTypeDef *hi2c)
{
    uint8_t cmd = 0xE7;
    uint8_t val;

    if (HAL_I2C_Master_Transmit(hi2c, HDC1080_ADDR, &cmd, 1, 50) == HAL_OK &&
        HAL_I2C_Master_Receive(hi2c, HDC1080_ADDR, &val, 1, 50) == HAL_OK)
    {
        detected = SENSOR_SHT21;
    }
    else
    {
        detected = SENSOR_HDC1080;
    }
}

// -------------------------------------------------------------------
// Misura temperatura + umidità (funzione universale)
// -------------------------------------------------------------------
HAL_StatusTypeDef sensor_measure(I2C_HandleTypeDef *hi2c,
                                 double *temperature,
                                 double *humidity)
{
    uint8_t buf[2];
    uint8_t cmd;
    uint16_t raw;

    // Autodetect una sola volta
    static uint8_t initialized = 0;
    if (!initialized) {
        detect_sensor(hi2c);
        initialized = 1;
    }

    // ---------------------------------------------------------------
    // MISURA TEMPERATURA
    // ---------------------------------------------------------------
    if (detected == SENSOR_HDC1080) {
        cmd = 0x00;   // temperatura HDC1080
    } else {
        cmd = 0xE3;   // temperatura SHT21/Si7021
    }

    if (HAL_I2C_Master_Transmit(hi2c, HDC1080_ADDR, &cmd, 1, 50) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(20);

    if (HAL_I2C_Master_Receive(hi2c, HDC1080_ADDR, buf, 2, 50) != HAL_OK)
        return HAL_ERROR;

    raw = (buf[0] << 8) | buf[1];

    if (detected == SENSOR_HDC1080)
        *temperature = ((double)raw / 65536.0) * 165.0 - 40.0;
    else
        *temperature = -46.85 + 175.72 * ((double)raw / 65536.0);

    // ---------------------------------------------------------------
    // MISURA UMIDITÀ
    // ---------------------------------------------------------------
    if (detected == SENSOR_HDC1080) {
        cmd = 0x01;   // umidità HDC1080
    } else {
        cmd = 0xE5;   // umidità SHT21/Si7021
    }

    if (HAL_I2C_Master_Transmit(hi2c, HDC1080_ADDR, &cmd, 1, 50) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(20);

    if (HAL_I2C_Master_Receive(hi2c, HDC1080_ADDR, buf, 2, 50) != HAL_OK)
        return HAL_ERROR;

    raw = (buf[0] << 8) | buf[1];

    if (detected == SENSOR_HDC1080)
        *humidity = ((double)raw / 65536.0) * 100.0;
    else
        *humidity = -6.0 + 125.0 * ((double)raw / 65536.0);

    if (*humidity < 0) *humidity = 0;
    if (*humidity > 100) *humidity = 100;

    return HAL_OK;
}





/*
 * hdc1080_get_device_id() - Get the device ID from the device
 * @i2c: handle to I2C interface
 * @serial: 40 bit serial number
 * @manuf: 16 bit manufacturer ID
 * @device: 16 bit device ID
 *
 * Returns 0 on success.
 */
HAL_StatusTypeDef hdc1080_get_device_id(I2C_HandleTypeDef *hi2c, uint64_t *serial, uint16_t *manuf, uint16_t *device)
{
  uint16_t ser[4];
	HAL_StatusTypeDef error;

	error = hdc1080_read_reg(hi2c, 10, HDC1080_SERIAL_ID1, &ser[0]);
	if (error != HAL_OK) return error;

	error = hdc1080_read_reg(hi2c, 10, HDC1080_SERIAL_ID2, &ser[1]);
	if (error != HAL_OK) return error;

	error = hdc1080_read_reg(hi2c, 10, HDC1080_SERIAL_ID3, &ser[2]);
	if (error != HAL_OK) return error;

	ser[3] = 0;
	memcpy(serial, ser, 8);

	error = hdc1080_read_reg(hi2c, 10, HDC1080_ID_MANU, manuf);
	if (error != HAL_OK) return error;

	error = hdc1080_read_reg(hi2c, 10, HDC1080_ID_DEV, device);
	if (error != HAL_OK) return error;

	return HAL_OK;  /* Success */
}
//-----------------------------------
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    int32_t  t_fine;
} BMP280_CalibData;

static BMP280_CalibData calib;

HAL_StatusTypeDef i2c_write(uint16_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Write(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}

HAL_StatusTypeDef i2c_read(uint16_t dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, dev, reg, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}


#define CCS811_REG_STATUS  0x00
#define CCS811_REG_MEAS    0x01
#define CCS811_REG_ALG     0x02
#define CCS811_REG_HW_ID   0x20
#define CCS811_CMD_APPSTART 0xF4

int CCS811_Init(void)
{
	uint8_t buf;

	// Verifica HW_ID
	i2c_read(CCS811_ADDR, CCS811_REG_HW_ID, &buf, 1);
	if (buf != 0x81)
		return -1;

	// Verifica se in application mode
	i2c_read(CCS811_ADDR, CCS811_REG_STATUS, &buf, 1);

	if (!(buf & 0x80))   // FW_MODE = 0 → serve APP_START
	{
		uint8_t cmd = CCS811_CMD_APPSTART;
		HAL_I2C_Master_Transmit(&hi2c1, CCS811_ADDR , &cmd, 1, 100);
		tx_thread_sleep(10);
	}
	tx_thread_sleep(100);

	// Imposta MEAS_MODE: 1 sample/sec
	uint8_t meas = 0x10;
	i2c_write(CCS811_ADDR, CCS811_REG_MEAS, &meas, 1);
	uint8_t check;
	i2c_read(CCS811_ADDR, CCS811_REG_MEAS, &check, 1);
	printf("MEAS_MODE = 0x%02X\n", check);

	return 0;
}

unsigned char CCS811_Read(uint16_t *eco2, uint16_t *tvoc)
{
    uint8_t buf[8];
    uint8_t status,err;
    i2c_read(CCS811_ADDR, 0x00, &status, 1);

    if (status & 0x08)
    { // dati pronti
    	i2c_read(CCS811_ADDR, 0x02, buf, 8);
    	*eco2 = (buf[0] << 8) | buf[1];
    	*tvoc = (buf[2] << 8) | buf[3];
    	return 1;
    }

    if (status & 0x01) // ERROR bit
    	{
    	i2c_read(CCS811_ADDR, 0xE0, &err, 1);
    	printf("CCS811 ERROR_ID = 0x%02X\n", err);
    	}
    return 0;

}



void BMP280_ReadRaw(int32_t *pressure)
{
    uint8_t buf[3];

    i2c_read(BMP280_ADDR, 0xF7, buf, 3);

    *pressure = ((int32_t)buf[0] << 12) |
                ((int32_t)buf[1] << 4)  |
                (buf[2] >> 4);
}

static HAL_StatusTypeDef BMP280_ReadReg(uint8_t reg, uint8_t *data, uint16_t len)
{
    return i2c_read(BMP280_I2C_ADDR, reg, data, len);
}


static HAL_StatusTypeDef BMP280_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf = value;
    return i2c_write(BMP280_I2C_ADDR, reg, &buf, 1);
}


HAL_StatusTypeDef BMP280_Init(void)
{
    uint8_t id = 0;
    BMP280_ReadReg(BMP280_REG_ID, &id, 1);

    if (id != 0x58)
        return HAL_ERROR;

    // Soft reset
    BMP280_WriteReg(BMP280_REG_RESET, 0xB6);
    HAL_Delay(10);

    // Read calibration data
    uint8_t calib_buf[24];
    BMP280_ReadReg(0x88, calib_buf, 24);

    calib.dig_T1 = (uint16_t)(calib_buf[1] << 8 | calib_buf[0]);
    calib.dig_T2 = (int16_t)(calib_buf[3] << 8 | calib_buf[2]);
    calib.dig_T3 = (int16_t)(calib_buf[5] << 8 | calib_buf[4]);

    calib.dig_P1 = (uint16_t)(calib_buf[7] << 8 | calib_buf[6]);
    calib.dig_P2 = (int16_t)(calib_buf[9] << 8 | calib_buf[8]);
    calib.dig_P3 = (int16_t)(calib_buf[11] << 8 | calib_buf[10]);
    calib.dig_P4 = (int16_t)(calib_buf[13] << 8 | calib_buf[12]);
    calib.dig_P5 = (int16_t)(calib_buf[15] << 8 | calib_buf[14]);
    calib.dig_P6 = (int16_t)(calib_buf[17] << 8 | calib_buf[16]);
    calib.dig_P7 = (int16_t)(calib_buf[19] << 8 | calib_buf[18]);
    calib.dig_P8 = (int16_t)(calib_buf[21] << 8 | calib_buf[20]);
    calib.dig_P9 = (int16_t)(calib_buf[23] << 8 | calib_buf[22]);

    // Normal mode, oversampling x1
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS, 0x27);
    BMP280_WriteReg(BMP280_REG_CONFIG, 0x00);

    return HAL_OK;
}
HAL_StatusTypeDef BMP280_ReadTemperaturePressure(float *temperature, float *pressure)
{
    uint8_t data[6];
    if (BMP280_ReadReg(BMP280_REG_PRESS_MSB, data, 6) != HAL_OK)
        return HAL_ERROR;

    int32_t adc_P = (int32_t)((data[0] << 12) | (data[1] << 4) | (data[2] >> 4));
    int32_t adc_T = (int32_t)((data[3] << 12) | (data[4] << 4) | (data[5] >> 4));

    // Temperature compensation
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib.dig_T1 << 1))) * calib.dig_T2) >> 11;
    int32_t var2 = (((((adc_T >> 4) - calib.dig_T1) * ((adc_T >> 4) - calib.dig_T1)) >> 12) * calib.dig_T3) >> 14;

    calib.t_fine = var1 + var2;
    *temperature = (calib.t_fine * 5 + 128) / 256.0f / 100.0f;

    // Pressure compensation
    int64_t pvar1 = ((int64_t)calib.t_fine) - 128000;
    int64_t pvar2 = pvar1 * pvar1 * calib.dig_P6;
    pvar2 = pvar2 + ((pvar1 * calib.dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)calib.dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * calib.dig_P3) >> 8) + ((pvar1 * calib.dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1)) * calib.dig_P1 >> 33;

    if (pvar1 == 0)
        return HAL_ERROR;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - pvar2) * 3125) / pvar1;
    pvar1 = (calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    pvar2 = (calib.dig_P8 * p) >> 19;

    p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)calib.dig_P7) << 4);

    *pressure = p / 25600.0f;

    return HAL_OK;
}
















void AppMain(void)
{

	ULONG actual_length=0;
	char rx_buffer[1024];

	UX_SLAVE_CLASS_CDC_ACM * cdc_acm_val=NULL;

	//uint16_t eco2, tvoc=0;


	//float t, p;
	unsigned long TimeRequest=0;
	//double temperature;
	//double humidity;
	//unsigned char ret;
	SHT40_Data sht;



	cdc_acm_val=get_cdc_acm();
	//CCS811_Init();
	//BMP280_Init();


 	//detect_sensor(&hi2c1);
	//TimeRequest=HAL_GetTick();
	while(1)
	{


		if ((tx_time_get()-TimeRequest)>1000)
		{
			TimeRequest=tx_time_get();

		    SHT40_Read(&sht, &hi2c1);
			if(cdc_acm_val==NULL)
			{

			}
			else
			{
				memset(rx_buffer,0,sizeof(rx_buffer));
				sprintf(rx_buffer,"\n SIXA 1.0 temp %.2f  hum %.2f",sht.temperature, sht.humidity);
				actual_length=strlen(rx_buffer);

				//if (cdc_acm_val->ux_slave_class_cdc_acm_data_dtr_state == 1)
				{
					ux_device_class_cdc_acm_write(cdc_acm_val,
												  rx_buffer,
												  actual_length,
												  &actual_length);

				}



		}



		}

	}


}

