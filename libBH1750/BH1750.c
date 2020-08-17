#include "BH1750.h"
uint8_t buff[2];

void delay_ms( uint32_t _time );

/*****************************************************************************/
void I2C_WriteBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf)
{
	while(
			HAL_I2C_Master_Transmit(
					&hi
					, (uint16_t)DEV_ADDR
					, (uint8_t*) &aTxBuffer
					, (uint16_t)sizebuf
					, (uint32_t)1000
					)!= HAL_OK
		){
		if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF){
			//sprintf(str, "Buffer error");
		}
	}
}
/*****************************************************************************/
void I2C_ReadBuffer(I2C_HandleTypeDef hi, uint8_t DEV_ADDR, uint8_t sizebuf)
{
	while(
			HAL_I2C_Master_Receive(
					&hi
					, (uint16_t)DEV_ADDR
					, (uint8_t*) &aTxBuffer
					, (uint16_t)sizebuf
					, (uint32_t)1000
					) != HAL_OK
		) {
		if (HAL_I2C_GetError(&hi) != HAL_I2C_ERROR_AF){
			//sprintf(str, "Buffer error");
		}
	}
}
/*****************************************************************************/
void BH1750_Init()
{
	//I2C_start(I2C1,BH1750Address<<1,I2C_Direction_Transmitter);
	I2C_WriteBuffer(BH1750_I2C_PORT,BH1750_I2C_ADDR,0x10);
	//I2C_stop(I2C1);
}

/*****************************************************************************/
uint16_t BH1750_Read()
{
	uint16_t val=0;
	I2C_start(I2C1, BH1750Address<<1,I2C_Direction_Transmitter);
	buff[1]=I2C_read_ack(I2C1);
	buff[2]=I2C_read_nack(I2C1);
	I2C_stop(I2C1);
	val=((buff[0]<<8)|buff[1])/1.2;
	return val;
}

/*****************************************************************************/
void delay_ms( uint32_t _time )
{
	_time = _time * 420;
	while( _time-- ){
	}
}

/*****************************************************************************/
