#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx.h"
//#include "main.h"
#include "stm32f1xx_ll_adc.h"
#include "DMAInit.h"


//uint16_t ADC_Result;


void DMA_Init(uint32_t ADC_Result_ADDR){
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), ADC_Result_ADDR, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);

}
