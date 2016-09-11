#include "stm32f0xx.h"
#include "A7105.h"


static __inline uint8_t SPISend (uint8_t data)
{
	while ((SPI1->SR & SPI_SR_TXE) == 0){}
	*(uint8_t *)&(SPI1->DR) = (uint8_t) data;
	while ((SPI1->SR & SPI_SR_RXNE) == 0){} 	
	return (uint8_t) SPI1->DR;		
}



void A7105Strobe (a7105_state_t st)
{
	A7105_SEL();
	SPISend((uint8_t)st);
	while ((SPI1->SR & SPI_SR_BSY) != 0){}
	A7105_DESEL();
}


void A7105WriteReg (a7105_reg_t reg, uint8_t data)
{
	A7105_SEL();
	SPISend((uint8_t)reg);
	SPISend(data);
	while ((SPI1->SR & SPI_SR_BSY) != 0){}
	A7105_DESEL();
} 



uint8_t A7105ReadReg (a7105_reg_t reg)
{
	uint8_t res;
	
	A7105_SEL();
	SPISend(((uint8_t)reg) | 0x40);	

	GPIOA->MODER &= ~GPIO_MODER_MODER7_1;
	res = SPISend(0xFF);
	A7105_DESEL();
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
	
	return res;
} 



static void A7105WriteRegMul (a7105_reg_t reg, uint8_t *data, uint16_t num)
{
	uint16_t i;
	
	A7105_SEL();
	SPISend((uint8_t)reg);
	for (i = 0; i < num; i++)
	{
		SPISend(*data++);
	}
	while ((SPI1->SR & SPI_SR_BSY) != 0){}
	A7105_DESEL();
}


static void A7105ReadRegMul (a7105_reg_t reg, uint8_t *data, uint16_t num)
{
	uint16_t i;
	uint8_t tmp;
	
	A7105_SEL();
	SPISend(((uint8_t)reg) | 0x40);	

	GPIOA->MODER &= ~GPIO_MODER_MODER7_1;
	for (i = 0; i < num; i++)
	{
		tmp = SPISend(0xFF);
		*data++ = tmp;
	}	

	A7105_DESEL();
	GPIOA->MODER |= GPIO_MODER_MODER7_1;
}


void A7105ReadFIFO (uint8_t *data, uint8_t num)
{
	if(num > 64) num = 64;
	A7105ReadRegMul (A7105_05_FIFO_DATA, data, num);
}


void A7105WriteID(uint32_t ida)
{
	uint8_t data[4];
	data[0] = (ida>>24)&0xff;
	data[1] = (ida>>16)&0xff;
	data[2] = (ida>>8)&0xff;
	data[3] = (ida>>0)&0xff;
	A7105WriteRegMul (A7105_06_ID_DATA, &data[0], sizeof(data));
}

uint32_t A7105ReadID (void)
{
	uint32_t id;
	uint8_t data[4];
	
	A7105ReadRegMul (A7105_06_ID_DATA, &data[0], sizeof(data));
	id  = data[0] << 24;
	id |= data[1] << 16;
	id |= data[2] << 8;
	id |= data[3] << 0;
	return id;
}


void A7105SoftReset (void)
{
	A7105WriteReg(A7105_00_MODE, 0x00);
}


void A7105SetChannel (uint8_t chan)
{
	A7105Strobe(A7105_STANDBY);
	A7105Strobe(A7105_RST_RDPTR);
	A7105WriteReg(A7105_0F_CHANNEL, chan);
	A7105Strobe(A7105_RX);
}












