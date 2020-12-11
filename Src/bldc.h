
//#include "bldc.c"
#include "stm32f1xx.h"



#define DDTime 12

// Rise = 15 + 60 = 80 ns = 140 ns
// Fall = 45 + 20 = 65 ns + 60ns IR2101
// FR = 72 * 10^6 -> Ticks = FR * tau = 72*10^6 * 140*10^-9 = 72*0,14 ~ 10-11 ticks;

#define BH 0
#define BL 1
#define YH 2
#define YL 3
#define RH 4
#define RL 5

#define BL_GPIO_PORT GPIOB
#define YL_GPIO_PORT GPIOB
#define RL_GPIO_PORT GPIOB
#define BL_GPIO_PIN LL_GPIO_PIN_13
#define YL_GPIO_PIN LL_GPIO_PIN_14
#define RL_GPIO_PIN LL_GPIO_PIN_15




#define CW 1
#define CCW 2
#define STOP 0


void SetTimer(uint16_t value);
uint8_t GetHallState(void);
void SetSpin(uint8_t HallState, uint8_t Dir);
void MotorCommutation(void);
void MotorInit(void);
