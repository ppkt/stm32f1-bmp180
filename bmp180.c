#include "bmp180.h"
#include "math.h"
#include "stdio.h"
#include "inttypes.h"

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

	printf("AC1 = %d\n\r", c->AC1);
	printf("AC2 = %d\n\r", c->AC2);
	printf("AC3 = %d\n\r", c->AC3);
	printf("AC4 = %d\n\r", c->AC4);
	printf("AC5 = %d\n\r", c->AC5);
	printf("AC6 = %d\n\r", c->AC6);
	printf("B1 = %d\n\r", c->B1);
	printf("B2 = %d\n\r", c->B2);
	printf("MB = %d\n\r", c->MB);
	printf("MC = %d\n\r", c->MC);
	printf("MD = %d\n\r", c->MD);
}

void bmp180_get_uncompensated_temperature(CalibrationData* data) {
	Buffer_Tx1[0] = 0xF4; // Register to write
	Buffer_Tx1[1] = 0x2E; // Value to write (measure temperature)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 2, Polling, BMP180_ADDRESS << 1);

	int i = 0;
	for ( i = 0; i < 20000; ++i) {
		asm volatile("nop");
	}
	// Read two bytes
	Buffer_Tx1[0] = 0xF6; // Register to read (temperature)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 2, Polling, BMP180_ADDRESS << 1);
	data->UT = Buffer_Rx1[0] << 8 | Buffer_Rx1[1];

	printf("UT = %d\n\r", data->UT);
}

void bmp180_get_uncompensated_pressure(CalibrationData* data) {
	Buffer_Tx1[0] = 0xF4; // Register to write
	Buffer_Tx1[1] = 0x34 | (data->oss << 6); // Value to write (measure pressure)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 2, Polling, BMP180_ADDRESS << 1);

	int i = 0;
	for ( i = 0; i < 20000; ++i) {
		asm volatile("nop");
	}
	// Read two bytes
	Buffer_Tx1[0] = 0xF6; // Register to read (pressure)
	I2C_Master_BufferWrite(I2C1, Buffer_Tx1, 1, Polling, BMP180_ADDRESS << 1);
	I2C_Master_BufferRead(I2C1, Buffer_Rx1, 3, Polling, BMP180_ADDRESS << 1);
	data->UP = (Buffer_Rx1[0] << 16 | Buffer_Rx1[1] << 8 | Buffer_Rx1[2]) >> (8 - data->oss);

	printf("UP = %d\n\r", data->UP);
}

void bmp180_calculate_true_temperature(CalibrationData* data) {
	data->X1 = (data->UT - data->AC6) * data->AC5 / pow(2, 15);
	data->X2 = data-> MC * pow(2, 11) / (data->X1 + data->MD);
	data->B5 = data->X1 + data->X2;
	data->T = (data->B5 + 8) / pow(2, 4);

	printf("X1 = %d\n\r", data->X1);
	printf("X2 = %d\n\r", data->X2);
	printf("B5 = %d\n\r", data->B5);
	printf("T = %d\n\r", data->T);
}

void bmp180_calculate_true_pressure(CalibrationData *data) {
	data->B6 = data->B5 - 4000;
	data->X1 = (data->B2 * (data->B6 * data->B6 / pow(2, 12))) / pow(2, 11);
	data->X2 = data->AC2 * data->B6 / pow(2, 11);
	data->X3 = data->X1 + data->X2;
	data->B3 = (((data->AC1 * 4 + data->X3) << data->oss) + 2) / 4;
	data->B4 = data->AC4 * (unsigned long)(data->X3 + 32768) / pow(2, 15);
	data->B7 = ((unsigned long)data->UP - data->B3) * (50000 >> data->oss);
	if (data->B7 < 0x80000000) {
		data->p = (data->B7 * 2) / data->B4;
	} else {
		data->p = (data->B7 / data->B4) * 2;
	}
	data->X1 = (data->p / pow(2, 8)) * (data->p / pow(2, 8));
	data->X1 = (data->X1 * 3038) / pow(2, 16);
	data->X2 = (-7357 * data->p) / pow(2, 16);
	data->p = data->p + (data->X1 + data->X2 + 3791) / pow(2, 4);

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
