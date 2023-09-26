#include "../system/include/cmsis/stm32f4xx.h"

#include "./mcu1.h"

//#include "../system/include/stm32f4-hal/stm32f4xx_hal.h"
//#include "../system/include/stm32f4/stm32f4xx_rcc.h"
//#include "../system/include/stm32f4/stm32f4xx_gpio.h"
//#include "../system/include/stm32f4/stm32f4xx_i2c.h"
//#include "../system/include/stm32f4/stm32f4xx_dma.h"
//#include "../system/include/stm32f4/stm32f4xx_dcmi.h"
//#include "../system/include/stm32f4/misc.h"

void init();

//===========================================================================================================
/*                                  main                                                                   */
//===========================================================================================================
int main(void) {
    init();
	while(1) {

	}
}

void init() {
    SystemCoreClockUpdate();
   // SysTick_Config(SystemCoreClock / 1000);
    MCO1_init();
}
