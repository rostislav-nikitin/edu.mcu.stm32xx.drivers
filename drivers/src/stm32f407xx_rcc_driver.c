/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: Sep 11, 2020
 *      Author: s0lid
 */

#include "stm32f407xx_rcc_driver.h"

uint32_t RCC_GetPLLOutputClock(void)
{
	return 0;
}


uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t AHB_PreScalar[9] = {1, 2, 4, 8, 16, 64, 128, 256, 512};

	uint32_t APB1_PreScalar[5] = {1, 2, 4, 8, 16};

	uint32_t pcklk1, SystemClk;

	uint8_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	int32_t hpre = ((RCC->CFGR >> 4) & 0xf);
	int32_t ahb_pscalar_idx = hpre - 7;
	if(ahb_pscalar_idx < 0)
	{
		ahb_pscalar_idx = 0;
	}
	int32_t ahb_pscalar = AHB_PreScalar[ahb_pscalar_idx];
	//uint32_t ahb_clck = SystemClk / ahb_pscalar;

	int32_t ppre1 = ((RCC->CFGR >> 10) & 0x7);
	int32_t apb1_pscalar_index = ppre1 - 3;
	if(apb1_pscalar_index < 0)
	{
		apb1_pscalar_index = 0;
	}

	int32_t apb1_pscalar = APB1_PreScalar[apb1_pscalar_index];

	pcklk1 = (SystemClk / ahb_pscalar) / apb1_pscalar;

	return pcklk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t AHB_PreScalar[9] = {1, 2, 4, 8, 16, 64, 128, 256, 512};

	uint32_t APB2_PreScalar[5] = {1, 2, 4, 8, 16};

	uint32_t pcklk2, SystemClk;

	uint8_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	if(clksrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(clksrc == 1)
	{
		SystemClk = 8000000;
	}
	else
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	int32_t hpre = ((RCC->CFGR >> 4) & 0xf);
	int32_t ahb_pscalar_idx = hpre - 7;
	if(ahb_pscalar_idx < 0)
	{
		ahb_pscalar_idx = 0;
	}
	int32_t ahb_pscalar = AHB_PreScalar[ahb_pscalar_idx];
	//uint32_t ahb_clck = SystemClk / ahb_pscalar;

	int32_t ppre2 = ((RCC->CFGR >> 13) & 0x7);
	int32_t apb2_pscalar_index = ppre2 - 3;
	if(apb2_pscalar_index < 0)
	{
		apb2_pscalar_index = 0;
	}

	int32_t apb2_pscalar = APB2_PreScalar[apb2_pscalar_index];

	pcklk2 = (SystemClk / ahb_pscalar) / apb2_pscalar;

	return pcklk2;
}
