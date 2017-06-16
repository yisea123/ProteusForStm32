/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal_uart.h"
#include "enc28j60.h"
#include "tapdev.h"
#include "uip.h"
#include "uip_arp.h"
#include "timer.h"	
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */ 
/* Private variables ---------------------------------------------------------*/
#define ARP_LEN 60
u32 uip_timer=0;//uip 计时器，每10ms增加1.
#define BUF ((struct uip_eth_hdr *)&uip_buf[0])	 	
extern uint16_t uip_len, uip_slen;
extern uint8_t uip_buf[UIP_BUFSIZE + 2]; 
uint8_t arpdata[ARP_LEN]={//mac 0x04,0x02,0x35,0x00,0x00,0x01 self ip:0xc0,0xa8,0x5f,0xc8 query ip :0xc0,0xa8,0x0a,0x01
	0xff,0xff,0xff,0xff,0xff,0xff,
	0x04,0x02,0x35,0x00,0x00,0x01,
//	0x01,0x00,0x00,0x35,0x02,0x04,
	
	0x08,0x06,0x00,0x01,0x08,0x00,0x06,0x04,0x00,0x01,
	
	0x04,0x02,0x35,0x00,0x00,0x01,
//	0x01,0x00,0x00,0x35,0x02,0x04,
	0xc0,0xa8,0x38,0xc8,
	0x00,0x00,0x00,0x00,0x00,0x00,
	0xc0,0xa8,0x38,0x01,
	 
	0x00,0x00,0x00,0x00,0x00,0x00
,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

extern FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	HAL_UART_Transmit(&huart1,(uint8_t *)&ch,1,10);
//	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
//    USART1->DR = (u8) ch;      
	return ch;
}
#endif 
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	static struct timer periodic_timer, arp_timer;
	uint8_t sptdata=0x84;
	uint8_t spid[2]={0x23,0xff},spidr[2];
	uint16_t i;
	u8 timer_ok=0;	 
	uint32_t tickstart = 0, tickend=0;
  uip_ipaddr_t ipaddr;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */ 
  MX_GPIO_Init();
  MX_SPI1_Init();  
  MX_USART1_UART_Init();
  MX_TIM1_Init(); 

  /* USER CODE BEGIN 2 */ 
	
	printf("Athis is proteus demo\r\ntime:%s\r\n",__TIME__);
//	ENC28J60_Reset();
//	while(0){ 
//		sptdata = ENC28J60_Read(ESTAT);
//		printf("ESTAT:%x \r\n",sptdata);
//		HAL_Delay(10);
//	}  
	while(tapdev_init())
	{
		printf("tapdev_init error\r\n");
	}
	printf("tapdev_init \r\n");
	HAL_Delay(100);
// 	uip_ipaddr(ipaddr, 192,168,95,16);	//设置本地设置IP地址
//	uip_sethostaddr(ipaddr);					    
//	uip_ipaddr(ipaddr, 192,168,95,1); 	//设置网关IP地址(其实就是你路由器的IP地址)
//	uip_setdraddr(ipaddr);						 
//	uip_ipaddr(ipaddr, 255,255,255,0);	//设置网络掩码
//	uip_setnetmask(ipaddr);
  /* USER CODE END 2 */
 
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) 
  {
  /* USER CODE END WHILE */ 

  /* USER CODE BEGIN 3 */
//		HAL_GPIO_WritePin(GPIOA,TESTIO_Pin,GPIO_PIN_RESET);
//		HAL_Delay(100);
//		HAL_GPIO_WritePin(GPIOA,TESTIO_Pin,GPIO_PIN_SET);
//		HAL_Delay(100);

		uip_len=tapdev_read();	//从网络设备读取一个IP包,得到数据长度.uip_len在uip.c中定义
		if(uip_len>0) 			//有数据
		{
			printf("rec a packet uip_len %d:",uip_len);
			for(i=0;i<uip_len;i++){ 
				if(i%16 == 0)
					printf("\r\n");
				printf("%x ",uip_buf[i]);

			}
			printf("\r\nover\r\n");
		}
//		for(i=0;i<500;i++);
		HAL_Delay(1);
//		uip_len = ARP_LEN;
//		printf("send a packet:");
//		for(i=0;i<uip_len;i++){
//			uip_buf[i] = arpdata[i];
//			if(i%16 == 0)
//					printf("\r\n");
//			printf("%x ",uip_buf[i]);
//		}
//		printf("\r\nover\r\n");
//		tapdev_send();
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_RESET);
//		HAL_SPI_Transmit(&hspi1,&sptdata,1,10);
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_SET);
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_RESET);
//		HAL_SPI_Receive(&hspi1,&sptdata,1,10);
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_SET);
		
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_RESET);
//		HAL_SPI_TransmitReceive(&hspi1,spid,spidr,2,10);
//		HAL_GPIO_WritePin(GPIOA,SPI_NSS_Pin,GPIO_PIN_SET);
//		printf("%x %x\n\r",spidr[0],spidr[1]);
//		if(timer_ok==0)
//		{
//			timer_ok=1;
//			timer_set(&periodic_timer,CLOCK_SECOND/2);  //创建1个0.5秒的定时器 
//			timer_set(&arp_timer,CLOCK_SECOND*10);	   	//创建1个10秒的定时器 
//			tickstart = HAL_GetTick();
//			tickend = tickstart;
//		}
//		tickstart = HAL_GetTick();
//		if(tickstart - tickend > 10)
//		{
//			tickend = tickstart;
//			uip_timer++;//uip计时器增加1	
//		}
//		uip_len=tapdev_read();	//从网络设备读取一个IP包,得到数据长度.uip_len在uip.c中定义
//		if(uip_len>0) 			//有数据
//		{   
//			//处理IP数据包(只有校验通过的IP包才会被接收) 
//			if(BUF->type == htons(UIP_ETHTYPE_IP))//是否是IP包? 
//			{
//				uip_arp_ipin();	//去除以太网头结构，更新ARP表
//				uip_input();   	//IP包处理
//				//当上面的函数执行后，如果需要发送数据，则全局变量 uip_len > 0
//				//需要发送的数据在uip_buf, 长度是uip_len  (这是2个全局变量)		    
//				if(uip_len>0)//需要回应数据
//				{
//					uip_arp_out();//加以太网头结构，在主动连接时可能要构造ARP请求
//					tapdev_send();//发送数据到以太网
//				}
//			}else if (BUF->type==htons(UIP_ETHTYPE_ARP))//处理arp报文,是否是ARP请求包?
//			{
//				uip_arp_arpin();
//				//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
//				//需要发送的数据在uip_buf, 长度是uip_len(这是2个全局变量)
//				if(uip_len>0)tapdev_send();//需要发送数据,则通过tapdev_send发送	 
//			}
//		}else if(timer_expired(&periodic_timer))	//0.5秒定时器超时
//		{
//			timer_reset(&periodic_timer);		//复位0.5秒定时器 
//			//轮流处理每个TCP连接, UIP_CONNS缺省是40个  
//			for(i=0;i<UIP_CONNS;i++)
//			{
//				uip_periodic(i);	//处理TCP通信事件  
//				//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
//				//需要发送的数据在uip_buf, 长度是uip_len (这是2个全局变量)
//				if(uip_len>0)
//				{
//					uip_arp_out();//加以太网头结构，在主动连接时可能要构造ARP请求
//					tapdev_send();//发送数据到以太网
//				}
//			}
//	#if UIP_UDP	//UIP_UDP 
//			//轮流处理每个UDP连接, UIP_UDP_CONNS缺省是10个
//			for(i=0;i<UIP_UDP_CONNS;i++)
//			{
//				uip_udp_periodic(i);	//处理UDP通信事件
//				//当上面的函数执行后，如果需要发送数据，则全局变量uip_len>0
//				//需要发送的数据在uip_buf, 长度是uip_len (这是2个全局变量)
//				if(uip_len > 0)
//				{
//					uip_arp_out();//加以太网头结构，在主动连接时可能要构造ARP请求
//					tapdev_send();//发送数据到以太网
//				}
//			}
//	#endif 
//			//每隔10秒调用1次ARP定时器函数 用于定期ARP处理,ARP表10秒更新一次，旧的条目会被抛弃
//			if(timer_expired(&arp_timer))
//			{
//				timer_reset(&arp_timer);
//				uip_arp_timer();
//			}
//		}
  /* USER CODE END 3 */
	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16; 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */ 
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */ 
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, enc28j60_rst_Pin|SPI_NSS_Pin|TESTIO_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : enc28j60_rst_Pin SPI_NSS_Pin TESTIO_Pin */
  GPIO_InitStruct.Pin = enc28j60_rst_Pin|SPI_NSS_Pin|TESTIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
