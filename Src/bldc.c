
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_tim.h"
#include "string.h"
#include "DMAInit.h"

#include "bldc.h"

uint8_t Prev_Bridge_State[6] = {0,0,0,0,0,0,};
const uint8_t STOP_Bridge_State[6] = {0,0,0,0,0,0,};


uint16_t ADC_Result;

static const uint8_t CW_BRIDGE_STATE[8][6] = 
//{
////	Blue	Yellow	Red
//	{ 0,0	,	0,0	,	0,0}, //000
//	{ 0,1	,	1,0	,	0,0}, //001 "ST3", BlueH, RedL
//	{ 0,0	,	0,1	,	1,0}, //010 "ST1", AH, BL, RH, YL
//	{ 1,0	,	0,1	,	0,0}, //011
//	{ 1,0	,	0,0	,	0,1}, //100
//	{ 0,0	,	1,0	,	0,1}, //101
//	{ 0,1	,	0,0	,	1,0}, //110
//	{ 0,0	,	0,0	,	0,0} //111
//};
{
	{ 0,0,0,0,0,0 },  // 0 //000   
  { 0,1,0,0,1,0 },  // 001
  { 1,0,0,1,0,0 },  // 010
  { 0,0,0,1,1,0 },  // 011
  { 0,0,1,0,0,1 },  // 100
  { 0,1,1,0,0,0 },  // 101
  { 1,0,0,0,0,1 },  // 110
  { 0,0,0,0,0,0 },  // 0 //111};
};

static const uint8_t CCW_BRIDGE_STATE[8][6] = 
//{
////	Blue	Yellow	Red
//	{ 0,0	,	0,0	,	0,0}, //000
//	{ 0,1	,	1,0	,	0,0}, //001 "ST3", BlueH, RedL
//	{ 0,0	,	0,1	,	1,0}, //010 "ST1", AH, BL, RH, YL
//	{ 1,0	,	0,1	,	0,0}, //011
//	{ 1,0	,	0,0	,	0,1}, //100
//	{ 0,0	,	1,0	,	0,1}, //101
//	{ 0,1	,	0,0	,	1,0}, //110
//	{ 0,0	,	0,0	,	0,0} //111
//};
{
	{ 0,0,0,0,0,0 },  // 0 //000   
  { 1,0,0,0,0,1 },  // 001
  { 0,1,1,0,0,0 },  // 010
  { 0,0,1,0,0,1 },  // 011
  { 0,0,0,1,1,0 },  // 100 -> 101?
  { 1,0,0,1,0,0 },  // 101
  { 0,1,0,0,1,0 },  // 110
  { 0,0,0,0,0,0 },  // 0 //111};
//	{ 0,0,0,0,0,0 },  // 0 //000   
//  { 0,0,0,1,1,0 },  // 100 -> 001
	
	
};

uint8_t HallState = 0;

//uint8_t HallState = 000;

void MotorCommutation(){
	HallState = GetHallState();
	uint16_t SpinVal;
	uint8_t Dir;
	if (ADC_Result > 2090){
		SpinVal = (ADC_Result - 2048) >> 2;
		Dir = CW;
	} else if (ADC_Result < 2000){
		SpinVal = (2047 - ADC_Result) >> 2;
		Dir = CCW;
	} else{
		SpinVal = 0;
		Dir = STOP;
		//memcpy(Prev_Bridge_State, STOP_Bridge_State, sizeof(Prev_Bridge_State)); // ???
	}
	
	SetTimer(SpinVal);
	SetSpin(HallState, Dir);
}

void SetTimer(uint16_t value){
	TIM1->CCR1 = value;
	TIM1->CCR2 = value;
	TIM1->CCR3 = value;
	
}

void MotorInit(){
	LL_TIM_EnableCounter(TIM1);
	LL_TIM_EnableAutomaticOutput(TIM1);
	
	LL_DMA_DisableChannel(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_ClearFlag_TC1(DMA1);
	LL_DMA_ClearFlag_HT1(DMA1);
	LL_DMA_ClearFlag_TE1(DMA1);
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);
	LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1, LL_ADC_DMA_GetRegAddr(ADC1, LL_ADC_DMA_REG_REGULAR_DATA), (uint32_t) &ADC_Result, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);
//	LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableIT_TE(DMA1, LL_DMA_CHANNEL_1);
	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	
	LL_ADC_Enable(ADC1);
	
	uint32_t wait_loop_index = 0;
	
	wait_loop_index = ((LL_ADC_DELAY_ENABLE_CALIB_ADC_CYCLES * 32) >> 1);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }
  LL_ADC_StartCalibration(ADC1);
	while (LL_ADC_IsCalibrationOnGoing(ADC1) != 0){}
				LL_ADC_REG_StartConversionSWStart(ADC1);
}

uint8_t GetHallState(){
	//HallState = GPIOB ->IDR >> 3;
	//HallState = HallState & 7;
	HallState = (GPIOB -> IDR & (LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7)) >> 5;
	return HallState;
}


void SetSpin(uint8_t HallState, uint8_t Dir){
switch(Dir){
	case CW:
	for( uint8_t i = 0; i <= 5; i++){
	if (CW_BRIDGE_STATE[HallState][i] != Prev_Bridge_State[i]){
		if(!CW_BRIDGE_STATE[HallState][i]){
		// 1, 3, 5
/*
			1: TIM1->CCER->Bit0
			3: TIM1->CCER->Bit4
			5: TIM1->CCER->Bit8
			i=1
			
			(i-1)*2 = 1-1 = 0
			i = 3
			3-1 *2 = 2*2 = 4
			
			i = 5
			
			5-1 *2 = 4*2 = 8
			
			
			
			// 2,4,8
			*/			
			switch(i){
				case BH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
									//TIM1->CCER->CC1E = 1;
									break;

				case YH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
									break;

				case RH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
									break;

				case BL: 	LL_GPIO_ResetOutputPin(BL_GPIO_PORT, BL_GPIO_PIN);
									break;

				case YL: 	LL_GPIO_ResetOutputPin(YL_GPIO_PORT, YL_GPIO_PIN);
									break;

				case RL: 	LL_GPIO_ResetOutputPin(RL_GPIO_PORT, RL_GPIO_PIN);
									break;
			} for(uint16_t u = 0; u <= DDTime; u++);
		} else	
		if(CW_BRIDGE_STATE[HallState][i]){
				
			switch(i){
				case BH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
									break;

				case YH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
									break;

				case RH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
									break;

				case BL: 	LL_GPIO_SetOutputPin(BL_GPIO_PORT, BL_GPIO_PIN);
									break;

				case YL: 	LL_GPIO_SetOutputPin(YL_GPIO_PORT, YL_GPIO_PIN);
									break;

				case RL: 	LL_GPIO_SetOutputPin(RL_GPIO_PORT, RL_GPIO_PIN);
									break;
			}
		}
	}

	Prev_Bridge_State[i] = CW_BRIDGE_STATE[HallState][i];
}
	break;
				case CCW:
					for( uint8_t i = 0; i <= 5; i++){
	if (CCW_BRIDGE_STATE[HallState][i] != Prev_Bridge_State[i]){
		if(!CCW_BRIDGE_STATE[HallState][i]){
				
			switch(i){
				case BH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
									break;

				case YH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
									break;

				case RH: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
									break;

				case BL: 	LL_GPIO_ResetOutputPin(BL_GPIO_PORT, BL_GPIO_PIN);
									break;

				case YL: 	LL_GPIO_ResetOutputPin(YL_GPIO_PORT, YL_GPIO_PIN);
									break;

				case RL: 	LL_GPIO_ResetOutputPin(RL_GPIO_PORT, RL_GPIO_PIN);
									break;
			} for(uint16_t u = 0; u <= DDTime; u++);
		} else	
		if(CCW_BRIDGE_STATE[HallState][i]){
				
			switch(i){
				case BH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
									break;

				case YH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
									break;

				case RH: 	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH3);
									break;

				case BL: 	LL_GPIO_SetOutputPin(BL_GPIO_PORT, BL_GPIO_PIN);
									break;

				case YL: 	LL_GPIO_SetOutputPin(YL_GPIO_PORT, YL_GPIO_PIN);
									break;

				case RL: 	LL_GPIO_SetOutputPin(RL_GPIO_PORT, RL_GPIO_PIN);
									break;
			}
		}
	}

	Prev_Bridge_State[i] = CCW_BRIDGE_STATE[HallState][i];
}
	break;
				case STOP: 	LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH1);
										LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH2);
										LL_TIM_CC_DisableChannel(TIM1, LL_TIM_CHANNEL_CH3);
			
										LL_GPIO_ResetOutputPin(BL_GPIO_PORT, BL_GPIO_PIN);
										LL_GPIO_ResetOutputPin(YL_GPIO_PORT, YL_GPIO_PIN);
										LL_GPIO_ResetOutputPin(RL_GPIO_PORT, RL_GPIO_PIN);
										
										memcpy(Prev_Bridge_State, STOP_Bridge_State, sizeof(Prev_Bridge_State)); // ???
										
										break;
}
	
}
			
