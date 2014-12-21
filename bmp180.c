#include "bmp180.h"
#include "math.h"
#include "stdio.h"
#include "inttypes.h"
#include "utils.h"

/* Buffer of data to be received by I2C1 */
uint8_t Buffer_Rx1[255];
/* Buffer of data to be transmitted by I2C1 */
uint8_t Buffer_Tx1[2] = {0xAA};

bool bmp180_check_presence() {
	Buffer_Tx1[0] = 0xD0; // Address of device
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 1, Polling, BMP180_ADDRESS << 1);

	if (Buffer_Rx1[0] == BMP180_CHIP_ID) {
		return true;
	} else {
		return false;
	}
}

typedef enum {
	BMP180_MODE_ULTRA_LOW_POWER = 0,
	BMP180_MODE_STANDARD = 1,
	BMP180_MODE_HIGH_RESOLUTION = 2,
	BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3,
} BMP180_Mode;

inline u8 bmp180_get_delay(BMP180_Mode mode) {
	switch (mode) {
	case BMP180_MODE_ULTRA_LOW_POWER:
		return 5;
	case BMP180_MODE_STANDARD:
		return 8;
	case BMP180_MODE_HIGH_RESOLUTION:
		return 14;
	case BMP180_MODE_ULTRA_HIGH_RESOLUTION:
		return 30;
	default:
		while(1) {}
	}
}

void bmp180_get_calibration_data(CalibrationData *c) {
	Buffer_Tx1[0] = 0xAA; // Begin of calibration data, 22 bytes length
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 22, Polling, BMP180_ADDRESS << 1);

	c->AC1 = Buffer_Rx1[0] << 8 | Buffer_Rx1[1];
	c->AC2 = Buffer_Rx1[2] << 8 | Buffer_Rx1[3];
	c->AC3 = Buffer_Rx1[4] << 8 | Buffer_Rx1[5];
	c->AC4 = Buffer_Rx1[6] << 8 | Buffer_Rx1[7];
	c->AC5 = Buffer_Rx1[8] << 8 | Buffer_Rx1[9];
	c->AC6 = Buffer_Rx1[10] << 8 | Buffer_Rx1[11];
	c->B1  = Buffer_Rx1[12] << 8 | Buffer_Rx1[13];
	c->B2  = Buffer_Rx1[14] << 8 | Buffer_Rx1[15];
	c->MB  = Buffer_Rx1[16] << 8 | Buffer_Rx1[17];
	c->MC  = Buffer_Rx1[18] << 8 | Buffer_Rx1[19];
	c->MD  = Buffer_Rx1[20] << 8 | Buffer_Rx1[21];
}

void bmp180_get_uncompensated_temperature(CalibrationData* data) {
	Buffer_Tx1[0] = 0xF4; // Register to write
	Buffer_Tx1[1] = 0x2E; // Value to write (measure temperature)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 2, Polling, BMP180_ADDRESS << 1);

	delay(5 /*ms*/);

	// Read two bytes
	Buffer_Tx1[0] = 0xF6; // Register to read (temperature)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 2, Polling, BMP180_ADDRESS << 1);
	data->UT = Buffer_Rx1[0] << 8 | Buffer_Rx1[1];
}

void bmp180_get_uncompensated_pressure(CalibrationData* data) {
	Buffer_Tx1[0] = 0xF4; // Register to write
	Buffer_Tx1[1] = 0x34 | (data->oss << 6); // Value to write (measure pressure)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 2, Polling, BMP180_ADDRESS << 1);

	// Wait for reading
	delay(bmp180_get_delay(data->oss));

	// Read two bytes
	Buffer_Tx1[0] = 0xF6; // Register to read (pressure)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 3, Polling, BMP180_ADDRESS << 1);
	data->UP = (Buffer_Rx1[0] << 16 | Buffer_Rx1[1] << 8 | Buffer_Rx1[2]) >> (8 - data->oss);
}

void bmp180_calculate_true_temperature(CalibrationData* data) {
	data->X1 = ((data->UT - data->AC6) * data->AC5) >> 15;
	data->X2 = ((data-> MC) << 11) / (data->X1 + data->MD);
	data->B5 = data->X1 + data->X2;
	data->T = (data->B5 + 8) >> 4;

//	printf("X1 = %d\n\r", data->X1);
//	printf("X2 = %d\n\r", data->X2);
//	printf("B5 = %d\n\r", data->B5);
	printf("T = %d\n\r", data->T);
}

void bmp180_calculate_true_pressure(CalibrationData *data) {
	data->B6 = data->B5 - 4000;

	data->X1 = (data->B2 * (data->B6 * data->B6) >> 12) >> 11;
	data->X2 = (data->AC2 * data->B6) >> 11;
	data->X3 = data->X1 + data->X2;
	data->B3 = ((((long)data->AC1 * 4 + data->X3) << data->oss) + 2) >> 2;

	data->X1 = (data->AC3 * data->B6) >> 13;
	data->X2 = (data->B6 * data->B6) >> 12;
	data->X2 = (data->X2 * data->B1) >> 16;
	data->X3 = ((data->X1 + data->X2) + 2) >> 2;
	data->B4 = data->AC4 * (unsigned long)(data->X3 + 32768) >> 15;

	data->B7 = ((unsigned long)data->UP - data->B3) * (50000 >> data->oss);
	if (data->B7 < 0x80000000) {
		data->p = (data->B7 << 1) / data->B4;
	} else {
		data->p = (data->B7 / data->B4) << 1;
	}
	data->X1 = data->p >> 8;
	data->X1 *= data->X1;
	data->X1 = (data->X1 * 3038) >> 16;
	data->X2 = (data->p * -7357) >> 16;
	data->p = data->p + ((data->X1 + data->X2 + 3791) >> 4);

	printf("p = %d\n\r", data->p);
}

void bmp180_get_absolute_altitude(CalibrationData *data) {
	// x^y -> exp(y * log(x))
	float b = expf(1.0/5.255 * logf(data->p / 101325.0));
	float f = 44330 * (1 - b);
	printf("altitude = %d\n\r", (int)f);

//	version above doesn't require more memory
//	float F = 44330 * (1 - powf(data->p / 101325.0, 1.0/5.255));
//	printf("altitude = %d\n\r", (int)F);
}
