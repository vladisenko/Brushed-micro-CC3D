#include "stm32f0xx.h"
#include "flash.h"

uint8_t SaveTXID (uint8_t *data, uint16_t num)
{
	uint8_t res = 1;
	
	if (num <= FLASH_PAGE_SIZE)
	{
		uint16_t num_to_write;
		uint16_t tmp;
		__IO uint16_t *pflash = (__IO uint16_t*)FLASH_LAST_PAGE;
		
		/* flash unlock*/
		while ((FLASH->SR & FLASH_SR_BSY) != 0) {}
		if ((FLASH->CR & FLASH_CR_LOCK) != 0)
		{
			FLASH->KEYR = FLASH_FKEY1;
			FLASH->KEYR = FLASH_FKEY2;
		}

		/* erase last page */
		FLASH->CR |= FLASH_CR_PER;
		FLASH->AR = FLASH_LAST_PAGE;
		FLASH->CR |= FLASH_CR_STRT;
		while ((FLASH->SR & FLASH_SR_BSY) != 0)	{}
		if ((FLASH->SR & FLASH_SR_EOP) != 0)
		{
			FLASH->SR |= FLASH_SR_EOP;
		}
		FLASH->CR &= ~FLASH_CR_PER;
		
		num_to_write = (num + 1) >> 1;

		while(num_to_write--)
		{
			tmp = *data++;
			num--;
			if (num)
			{
				tmp |= (*data++) << 8;
				num--;
			}
			
			FLASH->CR |= FLASH_CR_PG;
			*pflash++ = tmp; 
			while ((FLASH->SR & FLASH_SR_BSY) != 0){}
			if ((FLASH->SR & FLASH_SR_EOP) != 0) 
			{
				FLASH->SR |= FLASH_SR_EOP;
			}
			FLASH->CR &= ~FLASH_CR_PG;
		}
		
		/* flash lock*/
		FLASH->CR |= FLASH_CR_LOCK;
		res = 0;
	}
	return res;
}




uint8_t LoadTXID (uint8_t *data, uint16_t num)
{
	uint8_t res = 1;
	
	if (num <= FLASH_PAGE_SIZE)
	{
		uint16_t num_to_read;
		uint16_t tmp;
		__IO uint16_t *pflash = (__IO uint16_t*)FLASH_LAST_PAGE;
		
		num_to_read = (num + 1) >> 1;

		while (num_to_read--)
		{
			tmp = *pflash++;
			*data++ = tmp & 0x00FF;
			num--;
			if (num)
			{
				*data++ = (tmp >>8) & 0x00FF;
				num--;
			}
		}
		res = 0;
	}
	return res;
} 

