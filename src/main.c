#include "../system/include/cmsis/stm32f4xx.h"
//#include "../system/include/stm32f4-hal/stm32f4xx_hal.h"
#include "../system/include/stm32f4/stm32f4xx_rcc.h"
#include "../system/include/stm32f4/stm32f4xx_gpio.h"
#include "../system/include/stm32f4/stm32f4xx_i2c.h"
#include "../system/include/stm32f4/stm32f4xx_dma.h"
#include "../system/include/stm32f4/stm32f4xx_dcmi.h"
#include "../system/include/stm32f4/misc.h"

#include "MySysTimeConfig.h"

//===========================================================================================================
#include "./OV2640_register.h"

uint16_t Targetbuffer[160*120];


//===========================================================================================================
/*                              OV2640_init                                                                */
//===========================================================================================================

//===========================================================================================================
/*                              DCMI + DMA + DMCI_gpio                                                     */
//===========================================================================================================
//===========================================================================================================
void print_CameraData() {
    int line, column;
	for (line = 0; line < VERTICAL_RESOLUTION; ) {
		for (column = 0; column < HORIZONTAL_RESOLUTION;) {
			if ( Targetbuffer[line*HORIZONTAL_RESOLUTION+column] > 0x00 && Targetbuffer[line*HORIZONTAL_RESOLUTION+column] < 0x4000 ) {
			    G_ON();
            }
			if ( Targetbuffer[line*HORIZONTAL_RESOLUTION+column] >= 0x4000 && Targetbuffer[line*HORIZONTAL_RESOLUTION+column] < 0x8000 ) {
			    O_ON();
            }
			if ( Targetbuffer[line*HORIZONTAL_RESOLUTION+column] >= 0x8000 && Targetbuffer[line*HORIZONTAL_RESOLUTION+column] < 0xb000 ) {
                R_ON();
			}
			if ( Targetbuffer[line*HORIZONTAL_RESOLUTION+column] >= 0xb000 && Targetbuffer[line*HORIZONTAL_RESOLUTION+column] < 0xffff ) {
                B_ON();
			}

            G_OFF(); O_OFF(); R_OFF(); B_OFF(); 
            Delay(1000);
			column = column + 2;
		}
		line = line + 4;
	}
}
//===========================================================================================================
/*                                  main                                                                   */
//===========================================================================================================
int main(void) {
    SysTick_Config(SystemCoreClock / 1000);

    my_GPIO_init();
    MCO1_init();
    MySystemInit();

    OV2640_HW_Init();

	/* Print camera Id */
	OV2640_IDTypeDef camera_id;
	OV2640_ReadID(&camera_id);
	OV2640_QQVGAConfig();
	OV2640_Init(BMP_QQVGA);

    DMA_Cmd(DMA2_Stream1, ENABLE);
	while ( DMA_GetCmdStatus(DMA2_Stream1) != ENABLE );
    DCMI_Cmd(ENABLE);
    DCMI_CaptureCmd(ENABLE);

	while(1) {
        G_ON();
        while (DCMI_GetFlagStatus(DCMI_FLAG_FRAMERI) == RESET);
        G_OFF();
		print_CameraData();
	}
}
//===========================================================================================================
/*                      Инициализация функций дополнительных                                               */
//===========================================================================================================


