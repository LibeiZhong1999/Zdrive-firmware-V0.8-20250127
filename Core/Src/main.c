/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "motorcontrol.h"
#include "arm_math.h"
#include "as5047p.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern const float one_div_sq_three;
extern const uint32_t ARR;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	AS5047P_Instance encInstanceA = {0};
  AS5047P_Result errorcode;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t tmp=0,tmp_1=0,tmp_2=0;
int16_t tmp_int16;
float f_e=0,a,b,c,x,y,mo;
bool guzhang=false;
//uint32_t adc_v[100];
typedef union
{
	float v_float;
	uint8_t v_u8[4];
} vofa_data;
vofa_data test_vofa;
vofa_data vofa_tail;
vofa_data data_len[100];
uint8_t len=3;
uint8_t state=0;

uint32_t adc_data[8],cnt;

float theta_m_big,theta_m_big_pre=-1,vel_m_big,del_pos_sum,del_theta;
bool first=true;
float iq_zero_vel[2048];
int index_pos=0;
volatile bool is_still=false;int still_time=0;
int status=0;	//0 

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	
  AS5047P_Init(&encInstanceA, 0); // Bind encoder with id = 0
	(&encInstanceA)->zeroPosCalibrated = true;
//	Load_tx_buffer();
//	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, 0x00);
	
	/*PWM PARA
	//period 				160us
	//Dead time			2us
	*/
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,	TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1,	TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,	TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1,	TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,	TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1,	TIM_CHANNEL_3);
	
	__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_3,1500);//U
	__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_2,1500);//V
	__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_1,1500);//W
	
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	
	
	vofa_tail.v_u8[0]=0x00;
	vofa_tail.v_u8[1]=0x00;
	vofa_tail.v_u8[2]=0x80;
	vofa_tail.v_u8[3]=0x7f;
//	float test_f=0;
	
//	id_ref=1.5;
//	iq_ref=0;
	float band_width_cur=10000;	//10000		rise-time 1ms
	kp_d=Ld*band_width_cur;
	ki_d=R*band_width_cur;
	kp_q=Lq*band_width_cur;
	ki_q=R*band_width_cur;
	
	float band_width_vel=600;
	kp_vel=band_width_vel;
	ki_vel=kp_vel*band_width_vel*0.2;
	vel_ref=20*2*pi/60;//10rpm
	
	kp_pos=5;
	pos_ref=0;			//rad
	del_pos_sum=0;	//rad
	index_pos=0;		//pulse  2048max
	//zero pos init
//	if(encPositionA<1024)
//		pos_ref=(0-encPositionA)*pulse_to_rad;
//	else
//		pos_ref=(2048-encPositionA)*pulse_to_rad;
//	pos_abs+=pos_ref;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//	HAL_Delay(10);		//initial pos hatch
//	status=1;
//	while(1)
//	{
//		if(is_still)
//			break;
//		status=99;
//	}
//		status=1;

	del_pos_sum=(0-encPositionA)*pulse_to_rad;
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
//////		//get anticogging table
//////		if(is_still&&index_pos<2048)
//////		{
//////				int ccw_dis=(encPositionA-index_pos+2048)%2048;
////////				if(ccw_dis<=1||ccw_dis>=2047)
//////				if(ccw_dis==0)
//////				{
//////					iq_zero_vel[index_pos]=iq;
//////					index_pos++;
//////					if(index_pos<2048)
//////						del_pos_sum=(index_pos-encPositionA)*pulse_to_rad;
//////				}
//////				else
//////				{
//////					del_pos_sum=(index_pos-encPositionA)*pulse_to_rad;
//////				}
//////		}
//////		else if(index_pos>=2048&&status==0)
//////		{
//////			HAL_TIM_Base_Stop(&htim1);
//////			HAL_TIM_Base_Stop(&htim1);
//////			HAL_TIM_PWM_Stop(&htim1,	TIM_CHANNEL_1);
//////			HAL_TIMEx_PWMN_Stop(&htim1,	TIM_CHANNEL_1);
//////			HAL_TIM_PWM_Stop(&htim1,	TIM_CHANNEL_2);
//////			HAL_TIMEx_PWMN_Stop(&htim1,	TIM_CHANNEL_2);
//////			HAL_TIM_PWM_Stop(&htim1,	TIM_CHANNEL_3);
//////			HAL_TIMEx_PWMN_Stop(&htim1,	TIM_CHANNEL_3);
//////			for(int i=0;i<2048;++i)
//////			{
//////				data_len[0].v_float=i;
//////				data_len[1].v_float=iq_zero_vel[i];
//////				data_len[2].v_float=0;
//////				data_len[3]=vofa_tail;
//////				state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);
//////				HAL_Delay(5);
//////			}
//////			HAL_Delay(30000);
////////			status=1;
//////		}
//////		
//////		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_RESET);

		
//		IA=((float)adc_v[0]/4096*3.3-1.65)*25;
//		IB=((float)adc_v[1]/4096*3.3-1.65)*25;
//		IC=((float)adc_v[2]/4096*3.3-1.65)*25;
//		UA=((float)adc_v[3]/4096*3.3)*11;
//		UB=((float)adc_v[4]/4096*3.3)*11;
//		UC=((float)adc_v[5]/4096*3.3)*11;
//		Tem=((float)adc_v[6]/4096*3.3);
//		DC_BUS=((float)adc_v[7]/4096*3.3)*11;
//		data_len[0].v_float=IA;
//		data_len[1].v_float=IB;
//		data_len[2].v_float=IC;
//		data_len[3].v_float=UA;
//		data_len[4].v_float=UB;
//		data_len[5].v_float=UC;
//		data_len[6].v_float=Tem;
//		data_len[7].v_float=DC_BUS;
//		data_len[8]=vofa_tail;
//		state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);
//		test_vofa.v_float=test_f;
//		state=CDC_Transmit_FS(test_vofa.v_u8, 4);
//		test_f+=0.1;
//		if(test_f>500)
//			test_f=0;			
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
//injected adc conversion finished call back
//timer1 update trigs adc 
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	cnt=__HAL_TIM_GET_COUNTER(&htim1);
//	if(tmp<(1<<20))
		tmp++;
	
//	if(tmp<40000)//about 1s
//		vel_ref=0;
//	else
//		vel_ref=60*2*pi/60;//10rpm
	
	//downflow  measuring current and FOC
	if(cnt<(htim1.Init.Period >>1))
	{
		tmp_1++;
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_SET);
		ia_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
		ib_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
		ic_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
		
//		ia=((float)ia_u32/4096*3.3-1.65)*25-(-13.555);
//		ib=((float)ib_u32/4096*3.3-1.65)*25-(-11.279);
//		ic=((float)ic_u32/4096*3.3-1.65)*25-(-12.528);
		ia=(3.3*ia_u32/4096-1.65)*25-(ia_carli);
		ib=(3.3*ib_u32/4096-1.65)*25-(ib_carli);
		ic=(3.3*ic_u32/4096-1.65)*25-(ic_carli);
		ia*=-1.0;ib*=-1.0;ic*=-1.0;
		
//		encPositionA = AS5047P_ReadPosition(&encInstanceA, AS5047P_OPT_ENABLED);
//		theta_m=(float)encPositionA;
		
//		theta_e=0;
//		theta_e+=0.005;
//		if(theta_e>=2*pi)
//			theta_e-=2*pi;
		
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_SET);
//		encPositionA = AS5047P_ReadPosition(&encInstanceA, AS5047P_OPT_ENABLED);
//		theta_m=((float)encPositionA)*pulse_to_rad;//rad
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_RESET);
		

		if((tmp_1&0x0000000F)==0)
		{
//						if(del_pos_sum>pi)		del_pos_sum-=2*pi;
//						if(del_pos_sum<-pi)		del_pos_sum+=2*pi;
//						pos_ref=del_pos_sum;
//						del_pos_sum=0;
//						pos_loop();
						speed_loop();
		}
		current_loop();
		
		//0rad---0x21DE		polarity same
//		tmp_u16=encPositionA;
//		encPositionA = AS5047P_GetAngle_Dir(&encInstanceA);
		
//		data_len[0].v_float=(float)encPositionA;
//		data_len[1].v_float=theta_m;
//		data_len[2].v_float=vel_fil;
//		data_len[3]=vofa_tail;
//		state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_RESET);
	}
	//upflow  measuring carlibration current
	else
	{
				tmp_2++;
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_RESET);
		ia_carli_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_1);
		ib_carli_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_2);
		ic_carli_u32=HAL_ADCEx_InjectedGetValue(&hadc1,ADC_INJECTED_RANK_3);
//		
		ia_carli=(3.3*ia_carli_u32/4096-1.65)*25;
		ib_carli=(3.3*ib_carli_u32/4096-1.65)*25;
		ic_carli=(3.3*ic_carli_u32/4096-1.65)*25;
		
		encPositionA = AS5047P_ReadPosition(&encInstanceA, AS5047P_OPT_ENABLED);
		encPositionA = encPositionA>>3;
		if( AS5047P_ErrorPending(&encInstanceA) )
     {
			 errorcode=AS5047P_GetError(&encInstanceA).errorCode;
     }
		 
		 theta_m=(float)encPositionA*pulse_to_rad;	//11bit  2048pulse/r
		
		 //iq average	10num
		 
		 
		 if((tmp_2&(0x0000000F))==0) //trigger period 16*cycle
		 {
			 if(theta_m_pre!=-1)
			{
			 del_theta=theta_m-theta_m_pre;
			 //cross zero point
			 if(del_theta>pi)		del_theta-=2*pi;
			 if(del_theta<-pi)	del_theta+=2*pi;
			 
				//cal if is still
			
				if(pos_ref!=0)
				{
					is_still=false;
					still_time=0;
				}
				else
				{
					if(still_time<=100)
					{
						is_still=false;
						still_time++;
					}
					else
					{
						is_still=true;
					}
				}
				
			 vel_m=del_theta*one_div_cycle/16;//rad/s  T=160us  decrease high frequency noise
			 
			 //vel filter
			 //wc=1000Hz  a=Ts*wc=0.16
			 float a=0.6;
			 vel_fil=a*vel_m+(1-a)*vel_fil_pre;
			 vel_fil_pre=vel_fil; 
			}
			theta_m_pre=theta_m;
		 }
		 
		if((tmp_2&(0x0000000F))==0) //trigger period 16*cycle
		{
//			HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);
		data_len[0].v_float=vel_ref;
		data_len[1].v_float=vel_fil;
		data_len[2].v_float=iq;
		data_len[3]=vofa_tail;
		state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);
		}
//		HAL_GPIO_WritePin(GPIO1_GPIO_Port, GPIO1_Pin,GPIO_PIN_RESET);
	}
	
}

// timer(pwm) interrupt callback function
// control code
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//  /* Prevent unused argument(s) compilation warning */
//  HAL_GPIO_TogglePin(GPIO1_GPIO_Port, GPIO1_Pin);
//	tmp++;
////	HAL_ADC_Start_DMA(&hadc1,adc_data,8);
//	
//	
////	//check clark-park
////	theta_e+=0.01;//2^20
////	if(theta_e>=2*pi)
////		theta_e-=2*pi;
////	float ia,ib,ic;
////	ia=cos(theta_e);
////	ib=cos(theta_e-2*pi/3);
////	ic=cos(theta_e+2*pi/3);
////	
////	struct tri_vector cur_abc_v1={ia,ib,ic,true};
////	struct tri_vector cur_dq_v1;
////	cur_dq_v1=Clark_Park_Tf(cur_abc_v1,theta_e);
////	data_len[0].v_float=cur_abc_v1.fir;
////	data_len[1].v_float=cur_abc_v1.sec;
////	data_len[2].v_float=cur_abc_v1.thi;
////	data_len[3].v_float=cur_dq_v1.fir;
////	data_len[4].v_float=cur_dq_v1.sec;
////	data_len[5]=vofa_tail;
////	state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);	
//	
//	/*V-F mode
//	if(f_e<15)
//		f_e+=0.000125;
//	theta_e+=0.36*f_e;//2^20
//	if(theta_e>=360.0)
//		theta_e-=360;
//	float theta_e_rad=theta_e*2*pi/360;
//	mo=0.02*f_e;
//	x=(float)cos(theta_e_rad)*mo;
//	y=(float)sin(theta_e_rad)*mo;
//	struct tri_vector v=SVPWM( x,y);
//	a=v.fir;b=v.sec;c=v.thi;
//	if(v.sta)
//	{
//			__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_3,(uint32_t)(ARR*(1-v.fir)));//U
//			__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_2,(uint32_t)(ARR*(1-v.sec)));//V
//			__HAL_TIM_SetCompare(&htim1,	TIM_CHANNEL_1,(uint32_t)(ARR*(1-v.thi)));//W
//	}
//	else
//		guzhang=true;
//	
//		data_len[0].v_float=a;
//		data_len[1].v_float=b;
//		data_len[2].v_float=c;
//		data_len[3]=vofa_tail;
//		state=CDC_Transmit_FS((uint8_t*)data_len, (len+1)*4);	
//		*/
//		
//		
//		
//  /* NOTE : This function should not be modified, when the callback is needed,
//            the HAL_TIM_IC_CaptureCallback could be implemented in the user file
//   */
//}
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
