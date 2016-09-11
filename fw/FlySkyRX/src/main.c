/*
FlySky compatible receiver with DSM-like output. Tested with LibrePilot CC3D,
Cleanflight and Betaflight.
After the first turn on the receiver automatically switches to bind mode.
For manual binding pin PA13 and PA14 must be closed before power up. 

Sources:
	http://www.rcgroups.com/forums/showthread.php?t=1921870
	http://www.parkflyer.ru/ru/blogs/view_entry/12740/
	https://developer.mbed.org/users/d34d/code/A7105TxRx/
*/


#include "stm32f0xx.h"
#include "A7105.h"
#include "dsm.h"
#include "flash.h"

#define FLYSKY_PACKET_SIZE 		21

#define TICK_US					100
#define US(us)					(((us) + TICK_US/2)/TICK_US)
#define MS(ms)					(US(1000UL*ms))	

#define LED_ON()				do { GPIOA->BSRR = GPIO_BSRR_BR_1; } while(0)
#define LED_OFF()				do { GPIOA->BSRR = GPIO_BSRR_BS_1; } while(0)


uint8_t DSMFrame[DSM_FRAME_LENGTH];

const uint8_t A7105_regs[] = {
  0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff , 0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
  0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
  0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
  0x01, 0x0f, 0xff,
};

const uint8_t tx_channels[16][16] = {
  {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
  {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
  {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
  {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
  {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
  {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
  {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
  {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
  {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
  {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
  {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
  {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};


volatile uint32_t Tick = 0;

uint8_t rcv[FLYSKY_PACKET_SIZE];
uint8_t txid[5];
uint8_t chanrow;
uint8_t chancol;
uint8_t chanoffset;
uint8_t channel;

void HWInit (void);
void DelayTick (uint32_t t);

uint8_t CheckJumper (void);
uint32_t GetTick (void);

void FlySkyInit (void);
void FlySkyBind (void);
void NextChannel (void);


int main (void)
{
	uint32_t time_last_dms = 0;
	uint32_t time_last_pkg_flysky = 0;
	uint32_t time_led = 0;
	uint32_t id;
	
	uint32_t timeout_flysky = US(1600);
	uint16_t lost_pkg_flysky = 0;
		
	uint8_t bind;
	uint8_t have_new_pkg_flysky = 0;
	uint8_t led_state = 0, led_flashing = 0;
	
	
	HWInit();
	
	bind = CheckJumper();
	
	DelayTick(MS(100));
	DSMInit();
	FlySkyInit();
	
	LoadTXID(txid, sizeof(txid));
	
	if (bind || (txid[0] != 0xAA))
	{
		FlySkyBind();
		SaveTXID(txid, sizeof(txid));
		while(1);
	}
  
	id = (txid[1] | ((uint32_t)txid[2] << 8) | ((uint32_t)txid[3] << 16) | ((uint32_t)txid[4] << 24));
	chanrow = id % 16;
	chanoffset = (id & 0xff) / 16;
	chancol = 0;
	if (chanoffset > 9) chanoffset = 9; //from sloped soarer findings, bug in flysky protocol
	
	time_last_pkg_flysky = GetTick();
	
	while (1)
	{
	
		if (GIO_GET() == 0)
		{
			if ((A7105ReadReg(A7105_00_MODE) & (1 << 5)) != 0)			// CRC error
			{
				A7105Strobe(A7105_RST_RDPTR);
				A7105Strobe(A7105_RX);			
			}
			else
			{
				A7105ReadFIFO(&rcv[0], FLYSKY_PACKET_SIZE);
				if ((rcv[1] == txid[1]) && (rcv[2] == txid[2]) && (rcv[3] == txid[3]) && (rcv[4] == txid[4]))
				{
					time_last_pkg_flysky = GetTick();
					NextChannel();
					have_new_pkg_flysky = DSM_TIMEOUT;
					lost_pkg_flysky = 0;
					timeout_flysky = (US(1600));
				}
			}
		}
		
			
		if ((GetTick() - time_last_pkg_flysky) >= timeout_flysky)
		{
			NextChannel();	
			time_last_pkg_flysky = GetTick();
				
			if (lost_pkg_flysky < UINT16_MAX) lost_pkg_flysky++;
			timeout_flysky = (lost_pkg_flysky < 120) ? (US(1500)) : (US(1600));
		}
		
		
		if ((GetTick() - time_last_dms) > MS(11))
		{
			if (have_new_pkg_flysky)
			{
				uint8_t ch;
				
				have_new_pkg_flysky--;
				
				time_last_dms = GetTick();
			
				DSMFrame[0] = 0;
				DSMFrame[1] = 0xB2;
				
				for (ch = 0; ch < DSM_CHANNELS_PER_FRAME; ch++)
				{
					uint32_t tmp;
					time_last_dms = GetTick();
						
					tmp = (rcv[2*ch + 6] << 8) | rcv[2*ch + 5];
					if (tmp > 1000) tmp -= 1000; else tmp = 0;									// todo: check value
#if DSM_11BIT
					tmp = (tmp * 2047 + 500)/1000;				
					tmp = (tmp & 0x07FF) | (ch << 11);
#else
					tmp = (tmp * 1023 + 500)/1000;				
					tmp = (tmp & 0x03FF) | (ch << 10);				
#endif
					DSMFrame[2*ch + 2]  = (tmp >> 8) & 0x00FF;
					DSMFrame[2*ch + 3]  = tmp & 0x00FF;
				}
				DSMSend(&DSMFrame[0], sizeof(DSMFrame));
			}
			else
			{
				led_flashing = 1;
			}
			
		}
		
		
		if (led_flashing && (!led_state))
		{
			led_flashing = 0;
			led_state = 1;
		}
		
		
		switch(led_state)
		{
			case 0:
			break;

			case 1:
				LED_ON();
				time_led = GetTick();
				led_state++;
			break;
			
			case 2:
				if ((GetTick() - time_led) > MS(100)) led_state++;
			break;	
				
			case 3:
				LED_OFF();
				time_led = GetTick();
				led_state++;
			break;
				
			case 4:
				if ((GetTick() - time_led) > MS(500)) led_state = 0;
			break;
			
			default:
			break;		
		}

		__WFI();
	}
}


void SysTick_Handler (void)
{
	Tick++;
}


void HWInit (void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/(1000000UL/TICK_US));
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER =	(1U << 1*2) |			// PA1 - LED - GPout,
					(1U << 4*2) |			// PA4 - CS A7105 - GPout,
					(2U << 5*2) |			// PA5 - SCK A7105 - AF,
					(2U << 6*2) |			// PA6 - DIO A7105 - AF,
					(2U << 7*2) |			// PA7 - DIO A7105 - AF,
					(2U << 13*2) |			// PA13 - SDWIO - AF,
					(2U << 14*2) ;			// PA14 - SWCLK - AF.
	
	GPIOA->OTYPER = (1 << 1);				// PA1 - LED - open drain
	
	GPIOA->PUPDR =	(2U << 0*2) |			// PA0 - Pull-down,
					(2U << 2*2) |			// PA2 - Pull-down,
					(2U << 3*2) |			// PA3 - Pull-down,
					(2U << 6*2) |			// PA6 - Pull-down,
					(2U << 8*2) |			// PA8 - Pull-down,
					(2U << 10*2) |			// PA10 - Pull-down,
					(2U << 11*2) |			// PA11 - Pull-down,
					(2U << 12*2) |			// PA12 - Pull-down,
					(2U << 15*2);			// PA15 - Pull-down.
					
	GPIOA->OSPEEDR =(3U << 5*2) |			// PA5 - SCK A7105 - high speed,
					(3U << 7*2) |			// PA7 - DIO A7105 - high speed,
					(3U << 13*2);			// PA13 - SDWIO - high speed.
	

	GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_5;
	
	
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	GPIOB->PUPDR =	(1U << 1*2);			// PB1 - Pull-up.

	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 = 0;
	SPI1->CR2 = SPI_CR2_FRXTH |				// RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
				SPI_CR2_DS_0 |				// Data Size: 0b0111 - 8 bit
				SPI_CR2_DS_1 |
				SPI_CR2_DS_2;
	SPI1->CR1 = SPI_CR1_MSTR |
				SPI_CR1_SPE |				// Peripheral enabled
				SPI_CR1_SSI |				// Internal slave select
				SPI_CR1_SSM ;				// Software slave management
}



uint8_t CheckJumper (void)
{
	uint8_t res = 0;
	
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
	GPIOA->MODER = 0x10000000 | (GPIOA->MODER & ~0x3C000000);	// PA13 - GP in,
																// PA14 - GP out.
	if ((GPIOA->IDR & (1<<13)) == 0) res = 1;
	
	GPIOA->MODER = 0x28000000 | (GPIOA->MODER & ~0x3C000000);	// PA13 - SDWIO - AF,
																// PA14 - SWCLK - AF.
	return res;
}



void DelayTick (uint32_t t)
{
	uint32_t start = Tick;
	while ( (uint32_t)(Tick - start) < t);
}



uint32_t GetTick (void)
{
	return Tick;
}	



void FlySkyBind (void)
{
	uint16_t binded = 0;
	uint16_t led_state = 0;
	uint32_t time_led_bl = 0;
	
	A7105SetChannel(0x00);												// binding listen on channel 0

	while(binded == 0)
	{
					
		if (GIO_GET() == 0)
		{
			if ((A7105ReadReg(A7105_00_MODE) & (1 << 5)) != 0)			// CRC error
			{
				A7105Strobe(A7105_RST_RDPTR);
				A7105Strobe(A7105_RX);			
			}
			else
			{
				uint16_t i;
				
				A7105ReadFIFO(&rcv[0], FLYSKY_PACKET_SIZE);
				for(i = 0; i < sizeof(txid); i++)
				{
					txid[i] = rcv[i];
				}
				binded = 1;
			}
		}	
		
		if ((GetTick() - time_led_bl) > MS(1000))
		{
			time_led_bl = GetTick();
			led_state = !led_state;
			if (led_state)
				LED_ON();
			else
				LED_OFF();
			
		}
    }
	LED_ON();
}



void FlySkyInit (void)
{
	uint16_t i;
	
	A7105SoftReset();
	A7105WriteID(0x5475c52A);
	
	for (i = 0; i < 0x33; i++)
	{
		if (A7105_regs[i] != 0xff)
		{
			A7105WriteReg((a7105_reg_t)i, A7105_regs[i]);
		}
	}
 	A7105Strobe(A7105_STANDBY);
	A7105WriteReg(A7105_02_CALC, 0x01);
	while (A7105ReadReg(A7105_02_CALC) != 0){}
	A7105ReadReg(A7105_22_IF_CALIB_I);
		
	A7105WriteReg(A7105_24_VCO_CURCAL, 0x13);
	A7105WriteReg(A7105_25_VCO_SBCAL_I, 0x09);
	A7105Strobe(A7105_STANDBY);
}



void NextChannel (void)
{
	channel = tx_channels[chanrow][chancol] - chanoffset;
	channel--;
	chancol = (chancol + 1) % 16;
	A7105SetChannel(channel);
}
