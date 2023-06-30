/*
 * NRF24.c
 *
 *  Created on: 16 окт. 2022 г.
 *      Author: serad
 */


#include "NRF24.h"

//------------------------------------------------

extern SPI_HandleTypeDef hspi1;

extern uint32_t pressure[4];

//------------------------------------------------

#define TX_ADR_WIDTH 3 // Address width

#define RX_ADR_WIDTH 1 // Address width

#define TX_PLOAD_WIDTH 5 // Data width

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0xC1,0xb3,0xb4};
uint8_t RX_ADDRESS_P1[RX_ADR_WIDTH] = {0xC2};
uint8_t RX_ADDRESS_P2[RX_ADR_WIDTH] = {0xC3};
uint8_t RX_ADDRESS_P3[RX_ADR_WIDTH] = {0xC4};
uint8_t RX_ADDRESS_P4[RX_ADR_WIDTH] = {0xC5};
uint8_t RX_ADDRESS_P5[RX_ADR_WIDTH] = {0xC6};


uint8_t RX_BUF[TX_PLOAD_WIDTH] = {0};

//extern char str1[150];


uint8_t ErrCnt_Fl = 0;

//------------------------------------------------

__STATIC_INLINE void DelayMicro(__IO uint32_t micros)

{

  micros *= (SystemCoreClock / 1000000) / 6;

  /* Wait till done */

  while (micros--) ;

}

//--------------------------------------------------

uint8_t NRF24_ReadReg(uint8_t addr)

{

  uint8_t dt=0, cmd;

  CS_ON;

  HAL_SPI_TransmitReceive(&hspi1,&addr,&dt,1,1000);

  if (addr!=STATUS)//если адрес равен адрес регистра статус то и возварщаем его состояние

  {

    cmd=0xFF;

    HAL_SPI_TransmitReceive(&hspi1,&cmd,&dt,1,1000);

  }

  CS_OFF;

  return dt;

}

//------------------------------------------------

void NRF24_WriteReg(uint8_t addr, uint8_t dt)

{

  addr |= W_REGISTER;//включим бит записи в адрес

  CS_ON;

  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину

  HAL_SPI_Transmit(&hspi1,&dt,1,1000);//отправим данные в шину

  CS_OFF;

}

//------------------------------------------------

void NRF24_ToggleFeatures(void)

{

  uint8_t dt[1] = {ACTIVATE};

  CS_ON;

  HAL_SPI_Transmit(&hspi1,dt,1,1000);

  DelayMicro(1);

  dt[0] = 0x73;

  HAL_SPI_Transmit(&hspi1,dt,1,1000);

  CS_OFF;

}

//-----------------------------------------------

void NRF24_Read_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)

{

  CS_ON;

  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину

  HAL_SPI_Receive(&hspi1,pBuf,bytes,1000);//отправим данные в буфер

  CS_OFF;

}

//------------------------------------------------

void NRF24_Write_Buf(uint8_t addr,uint8_t *pBuf,uint8_t bytes)

{

  addr |= W_REGISTER;//включим бит записи в адрес

  CS_ON;

  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину

  DelayMicro(1);

  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//отправим данные в буфер

  CS_OFF;

}

//------------------------------------------------

void NRF24_FlushRX(void)

{

  uint8_t dt[1] = {FLUSH_RX};

  CS_ON;

  HAL_SPI_Transmit(&hspi1,dt,1,1000);

  DelayMicro(1);

  CS_OFF;

}

//------------------------------------------------

void NRF24_FlushTX(void)

{

  uint8_t dt[1] = {FLUSH_TX};

  CS_ON;

  HAL_SPI_Transmit(&hspi1,dt,1,1000);

  DelayMicro(1);

  CS_OFF;

}

//------------------------------------------------

void NRF24L01_RX_Mode(void)

{

  uint8_t regval=0x00;

  regval = NRF24_ReadReg(CONFIG);

  //разбудим модуль и переведём его в режим приёмника, включив биты PWR_UP и PRIM_RX

  regval |= (1<<PWR_UP)|(1<<PRIM_RX);

  NRF24_WriteReg(CONFIG,regval);

  CE_SET;

  DelayMicro(150); //Задержка минимум 130 мкс

  // Flush buffers

  NRF24_FlushRX();

  NRF24_FlushTX();

}
//------------------------------------------------

void NRF24L01_TX_Mode(uint8_t *pBuf)

{

  NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);

  CE_RESET;

  // Flush buffers

  NRF24_FlushRX();

  NRF24_FlushTX();

}

//------------------------------------------------

void NRF24_Transmit(uint8_t addr,uint8_t *pBuf,uint8_t bytes)

{

  CE_RESET;

  CS_ON;

  HAL_SPI_Transmit(&hspi1,&addr,1,1000);//отправим адрес в шину

  DelayMicro(1);

  HAL_SPI_Transmit(&hspi1,pBuf,bytes,1000);//отправим данные в буфер

  CS_OFF;

  CE_SET;

}

//------------------------------------------------

void NRF24L01_Send(uint8_t *pBuf)
{
	uint8_t status=0x00, regval=0x00;
	NRF24L01_TX_Mode(pBuf);
	regval = NRF24_ReadReg(CONFIG);

	//если модуль ушел в спящий режим, то разбудим его, включив бит PWR_UP и выключив PRIM_RX
	regval |= (1<<PWR_UP);
	regval &= ~(1<<PRIM_RX);
	NRF24_WriteReg(CONFIG,regval);
	DelayMicro(150); //Задержка минимум 130 мкс

	//Отправим данные в воздух
	NRF24_Transmit(WR_TX_PLOAD, pBuf, TX_PLOAD_WIDTH);
	CE_SET;
	DelayMicro(15); //minimum 10us high pulse (Page 21)
	CE_RESET;
}

//------------------------------------------------
/*
uint8_t NRF24L01_Send_Complete()
{

	uint8_t regval=0x00;

	regval = NRF24_ReadReg(OBSERVE_TX);
	//Уходим в режим приёмника
	NRF24L01_RX_Mode();
	return regval;

}
*/
//------------------------------------------------

void NRF24L01_Receive(void)

{
  //uint8_t status=0x0;

  //status = NRF24_ReadReg(STATUS);
  NRF24_Read_Buf(RD_RX_PLOAD,RX_BUF,TX_PLOAD_WIDTH);
    //if(status & 0x2){
    	pressure[1] = ((uint32_t)(RX_BUF[2]<<16))+((uint32_t)(RX_BUF[1]<<8))+((uint32_t)(RX_BUF[0]));
    	pressure[2] = (uint32_t)(RX_BUF[3]);
    	pressure[3] = (uint32_t)(RX_BUF[4]);
}

//--------------------------------------------------

void NRF24L01_IT(void)
{
	uint8_t status=0x00;
	status = NRF24_ReadReg(STATUS);
	if((status & RX_DR)&&(status & TX_DS))
	{
			NRF24_WriteReg(STATUS, RX_DR);
			NRF24L01_Receive();
			NRF24_WriteReg(STATUS, TX_DS);
			pressure[2] = (NRF24_ReadReg(OBSERVE_TX) & 0xF);
			NRF24L01_RX_Mode();

	}
	else if(status & RX_DR)
	{
		NRF24_WriteReg(STATUS, RX_DR);
		NRF24L01_Receive();
	}
	else if(status & TX_DS)
	{
	 	NRF24_WriteReg(STATUS, TX_DS);
	 	pressure[2] = (NRF24_ReadReg(OBSERVE_TX) & 0xF);
	 	NRF24L01_RX_Mode();
	}
	else if(status & MAX_RT)
	{
		NRF24_WriteReg(STATUS, MAX_RT);
		NRF24_FlushTX();
		pressure[2] = (NRF24_ReadReg(OBSERVE_TX) & 0xF);
		NRF24L01_RX_Mode();
	}
}

//--------------------------------------------------
void NRF24_ini(void)

{
	  CE_RESET;
	  DelayMicro(5000);
	  NRF24_WriteReg(CONFIG, 0x0a); 						// Set PWR_UP bit, enable CRC(1 byte) &Prim_RX:0 (Transmitter)
	  DelayMicro(5000);
	  NRF24_WriteReg(EN_AA, 0x01); 							// Enable Pipe0
	  NRF24_WriteReg(EN_RXADDR, 0x01); 						// Enable Pipe0
	  NRF24_WriteReg(SETUP_AW, 0x01); 						// Setup address width=3 bytes
	  NRF24_WriteReg(SETUP_RETR, 0x5F); 					// 1500us, 15 retrans
	  NRF24_ToggleFeatures();
	  NRF24_WriteReg(FEATURE, 0x6);							//Enable ACK payload (require dynamic payload)
	  NRF24_WriteReg(DYNPD, 0x1);							//Dynamic payload pipe 0
	  NRF24_WriteReg(STATUS, 0x70); 						//Reset flags for IRQ
	  NRF24_WriteReg(RF_CH, 70); 							// частота 2470 MHz
	  NRF24_WriteReg(RF_SETUP, 0x26); 						//TX_PWR:0dBm, Datarate:250kbps
	  NRF24_Write_Buf(TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);
	  NRF24_Write_Buf(RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);
	  NRF24_WriteReg(RX_PW_P0, TX_PLOAD_WIDTH); 			//Number of bytes in RX payload in data pipe 0
	  NRF24L01_RX_Mode();
}

//--------------------------------------------------
