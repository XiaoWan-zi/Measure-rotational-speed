/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stm32f1xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
    float e;
    float to;
    float ta;
    float bo;
} gyir;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t aTxStartMessage[] = "\r\n****UART-Hyperterminal communication based on IT ****\r\n";
/* Buffer used for reception */
uint8_t aRxBuffer[20];

uint8_t sum=0,i=0;
  int16_t data=0;
  gyir my_ir= {0,0,0,0};							//���У�������ʾGCV��Ŀ���¶ȵȡ�
	
	float RPM = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	int i = 0;	//����ѭ������

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)aTxStartMessage, sizeof(aTxStartMessage));
  //HAL_UART_Receive_IT(&huart1, (uint8_t *)aRxBuffer, 10);
	
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//TIM1 CH1 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_UART_Receive_IT(&huart3,usart_rx_data,12); //������ת���ж���ɵ��޷�������ȡ����	��UART3���ȼ���������ߣ���ok�ˡ���

		
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_15);


		printf("RPM:%f,count == %d\r\n",RPM/count,count);//��ӡ�ܵĸߵ�ƽʱ��
		count = 0;
		//RPM_stat = 0;
		RPM = 0;
		
	 if(Receive_ok)//���ڽ������
		{
			printf("found successfully\n"); 
		for(sum=0,i=0; i<(usart_rx_data[3]+4); i++) //rgb_data[3]=3
			sum+=usart_rx_data[i];
		if(sum==usart_rx_data[i])//У����ж�
		{
			my_ir.to=(usart_rx_data[5]<<8)|usart_rx_data[6];
			my_ir.ta=(usart_rx_data[7]<<8)|usart_rx_data[8];
			my_ir.bo=(usart_rx_data[9]<<8)|usart_rx_data[10];
			my_ir.e=usart_rx_data[4];
			printf("E: %.2f,",(float)my_ir.e/100);
			printf("  to: %.2f,",(float)my_ir.to/100);
			printf("  ta: %.2f,",(float)my_ir.ta/100);
			printf(" bo %.2f\r\n ",(float)my_ir.bo/100);
		}
		else
		{
			printf(" sum %d\r\n ",sum);
			printf(" count %d\r\n ",usart_rx_data[3]+4);
		}
			Receive_ok=0;//����������ϱ�־
		}
		else
		{
			printf("not found\n");
		}
  /* USER CODE END 3 */
	}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
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
}

/* USER CODE BEGIN 4 */
/**
  * @brief Rx Transfer completed callbacks
  * @param huart: uart handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//printf("HAL_UART_RxCpltCallback\n");
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
   
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file
   */
	if(huart->Instance==USART3)
	{
		//static uint8_t i=0,rebuf[30]= {0};					//ʹ�þ�̬������ʹ�ÿ���������ȡ���ݣ�������
		uint8_t flag = 1;
    {
				
			//printf("rebug[%d] = %x  \n",i-1,rebuf[i-1]);
        //rebuf[i++]=USART_ReceiveData(USART3);//��ȡ�������ݣ�ͬʱ����ձ�־
			  //HAL_UART_Transmit_IT(&huart1, &rebuf[i-1], 1);//USART_SendData(USART1,rebuf[i-1]);	//֮ǰ���ڵ����������������õ����ݡ�
        if (usart_rx_data[0]!=add)//֡ͷ����
				{
					for(int j = 0;j<12;j++)
					{
						printf("usart_rx_data[%d]=%x ",j,usart_rx_data[j]);
					}
					printf("\n");
					printf("֡ͷ0����\n");
					flag = 0;						//ʹ��flag���б�ʶ��ʹ��return�Ļ���Ҫ�ٴ��жϽ���
					/* ���־λ���������������ݺ��޷����͵����� */
					__set_FAULTMASK(1);//�������ж�
					NVIC_SystemReset();//����
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_PE);//???
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
				}
            
        else if ((usart_rx_data[1]!=0x03))//֡ͷ���ԣ�03����� �Ĵ���
				{
					printf("֡ͷ1����\n");
					flag = 0;
					/* ���־λ���������������ݺ��޷����͵����� */
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_PE);//???
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
				}
            
        else if ((usart_rx_data[2]>16))//֡ͷ���ԣ���ʼ�Ĵ���λ��
        {
					printf("֡ͷ2����\n");
					flag = 0;
					/* ���־λ���������������ݺ��޷����͵����� */
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_PE);//???
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
				}
        else if ((usart_rx_data[3]>16))//֡ͷ���ԣ��Ĵ�������
        {
					printf("֡ͷ3����\n");
					flag = 0;
					/* ���־λ���������������ݺ��޷����͵����� */
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_PE);//???
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_FE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_NE);
					__HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_ORE);
				}
        if(flag == 1)//i����4ʱ���Ѿ����յ��������ֽ�rebuf[3]
        {
            //if(i<21)
            {
                if(12==(usart_rx_data[3]+5))//�ж��Ƿ����һ֡�������
                {

                    if(!Receive_ok)//�����ݴ�����ɺ�Ž����µ�����
                    {
                        //memcpy(usart_rx_data,rebuf,i);//�������յ�������
                        Receive_ok=1;//������ɱ�־
											printf("Receive_ok\n");
                    }
                    //i=0;//������0

                }
            }
						//else	//�����洢��Χ��Ϊʲô��21��
						//i=0;
        }
    }
		HAL_UART_Receive_IT(&huart3,usart_rx_data,12);
	}
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//printf("HAL_TIM_PeriodElapsedCallback\n");
  if(htim->Instance == TIM1)
	{
		TIM1CH1_CAPTURE_STA++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	
	static unsigned char j = 0;
	unsigned int time = 0;

	//printf("������\n");
	j++;
	if(j == 1)						//����㷨��ʵ���Ե���ÿ��j��0��1��ʱ�䣬ת��Խ�죬���ԵĴ�����Խ�࣬����õĺ͵Ĵ�����Խ�٣����Ⱦ���Ա�͡����԰�j=4����һ�����j=1��Ȼ��ѭ����ʼ ��ʱ������ j++ ;
	{
		__HAL_TIM_SET_COUNTER(&htim1,0);
		TIM1CH1_CAPTURE_STA = 0;
		TIM1CH1_CAPTURE_VAL = 0;
	}
	
	if(j == 4)	//���Ĵβ�����������
	{
		count++;
		TIM1CH1_CAPTURE_VAL = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
		time= TIM1CH1_CAPTURE_VAL + TIM1CH1_CAPTURE_STA*65536;
		//printf("time/100000 = %f\n",(float)time/1000000);
		//printf("j=%d\n",j);
		//	printf("RPM ԭʼ = %f\n",60/((float)time/1000000));
		RPM += 60/((float)time/1000000);	//��main��ӡ֮ǰ�ۼ�RPM
		time = 0;
		j=0;
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
