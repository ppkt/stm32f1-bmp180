#ifndef __BMP180_H__
#define __BMP180_H__
#include "i2c_dma.h"
#include "stdbool.h"

#define BMP180_ADDRESS 0x77
#define BMP180_CHIP_ID 0x55

typedef struct {
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;
	short B1;
	short B2;
	short MB;
	short MC;
	short MD;

	long UT;
	short oss;
	long UP;

	long X1;
	long X2;
	long B5;
	long T;

	long B6;
	long X3;
	long B3;
	unsigned long B4;
	unsigned long B7;
	long p;

} CalibrationData;

typedef enum {
	BMP180_MODE_ULTRA_LOW_POWER = 0,
	BMP180_MODE_STANDARD = 1,
	BMP180_MODE_HIGH_RESOLUTION = 2,
	BMP180_MODE_ULTRA_HIGH_RESOLUTION = 3,
} BMP180_Mode;

bool bmp180_check_presence();
void bmp180_get_calibration_data(CalibrationData* data);
void bmp180_get_uncompensated_temperature(CalibrationData* data);
void bmp180_get_uncompensated_pressure(CalibrationData* data);
void bmp180_calculate_true_temperature(CalibrationData* data);
void bmp180_calculate_true_pressure(CalibrationData* data);
void bmp180_get_absolute_altitude(CalibrationData* data);

#endif // __BMP180_H__
