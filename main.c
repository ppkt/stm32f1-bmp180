#include "usart.h"
#include "i2c_dma.h"
#include "bmp180.h"
#include "stdio.h"
#include "utils.h"

void GPIO_Configuration(void);
void NVIC_Configuration(void);

int main(void)
{
	USART2_Init(9600);

	NVIC_Configuration();
	I2C_LowLevel_Init(I2C1);

	// Tick every 1 ms
	if (SysTick_Config(SystemCoreClock / 1000))  while (1);

	printf("Ready\n\r");
	if (bmp180_check_presence()) {
		printf("Sensor is present\n\r");
	} else {
		printf("Sensor is NOT present\n\r");
		while(1){}
	}

	CalibrationData data;
	data.oss = 3;
	bmp180_get_calibration_data(&data);
	bmp180_get_uncompensated_temperature(&data);
	bmp180_get_uncompensated_pressure(&data);
	bmp180_calculate_true_temperature(&data);
	bmp180_calculate_true_pressure(&data);
	bmp180_get_absolute_altitude(&data);

    while(1)
    {

    }
}

void NVIC_Configuration(void)
{

    /* 1 bit for pre-emption priority, 3 bits for subpriority */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    NVIC_SetPriority(I2C1_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C1_EV_IRQn);

    NVIC_SetPriority(I2C1_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C1_ER_IRQn);


    NVIC_SetPriority(I2C2_EV_IRQn, 0x00);
    NVIC_EnableIRQ(I2C2_EV_IRQn);

    NVIC_SetPriority(I2C2_ER_IRQn, 0x01);
    NVIC_EnableIRQ(I2C2_ER_IRQn);

}


void SysTick_Handler(void)
{
  delay_decrement();
}

