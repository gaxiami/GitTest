/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
//#include "peripherals.h"
//#include "pin_mux.h"
//#include "board.h"
//#include "fsl_gpio.h"
//#include "fsl_usart.h"
//#include "fsl_mrt.h"
//#include "fsl_swm_connections.h"
//#include "fsl_swm.h"
//#include "fsl_ftm.h"
//#include "fsl_i3c.h"
//#include "fsl_power.h"
//#include "fsl_clock.h"
//#include "clock_config.h"
#include "test_usart.h"
#include "test_clk.h"
#include "test_gpio.h"
#include "test_mrt.h"
#include "test_mux.h"
#include "test_spi.h"  
#include "test_i2c.h"
#include "test_crc.h"
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;
volatile uint32_t g_SysTick = 0;

uint8_t data[]=  
"Hello LPC860.\r\nBoard will send back received characters.\r\nNow, please input any character:\r\n";
 
/*******************************************************************************
 * Code
 ******************************************************************************/


void SysTick_Handler(void)
{
   static uint32_t timeval=0;
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
    g_SysTick++;
    if((g_SysTick%1000) == 1)  ;
  
    
    timeval++;
//    if(timeval>=1000)
//    {
//      timeval=0;
//       b=~b;
//       //GPIO_PinWrite(GPIO, 0, 15,b);
//    }
  
}


add func2
 int main(void)
{ 
  add func3
    static uint8_t b=0;
    uint8_t DataArr[]={0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
  
    Test_MuxInit();//开关矩阵复用初始化
    
    Test_SysClkInit();//测试系统时钟初始化
// 
    __enable_irq();//打开中断
    
    Test_I2cInit();
   
    Test_UsartInit();
  
    Test_GpioInit();
  
    Test_MrtInit();
    
    Test_SPI0init();
    
    //SysTick_Config(SystemCoreClock / 24000U); //24000000/24000=1000 =1ms 
 
    
    //Test_Usart0SendString(data,98);
   // Test_Usart0SendString(data);
    Test_Usart0SendData("zzxok\r\n",7);
    
      Test_CrcDisplay();
    while(1)
    {
        if(Test_MrtTimeFlag)
        {
          b=~b;
          GPIO_PinWrite(GPIO, 0, 15,b);
          Test_MrtTimeFlag = 0;
//          Test_Spi0SendData(DataArr,8);
          //Test_I2c0SendData(DataArr,8);
        
          
        }
    }
}




