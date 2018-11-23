/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include <stdlib.h>
#include <math.h>
#include "FindRoad.h"
#include "decodemessage.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
uint8_t receive[1];
uint8_t temptext[75];
volatile uint8_t text[70];
volatile uint8_t refreshed=0;
int current=0;  //position of the newest data in text[]
uint8_t pitext[5];
volatile uint8_t pirefreshed=0;
uint8_t fre=10;  //编码器频率
volatile int leftu=0,rightu=0;  //当前速度(pwm表示)



struct _vector{
	int x,y;
	float angle;  //degree
};
typedef struct _vector vector; 

float cal_vecangle(vector* a){
	//(-180,180]
	if (a->x==0){
		if (a->y>=0)a->angle=90;
		else a->angle=-90;
	}
	else{
		if (a->x>0) a->angle=atan(a->y*1.0/a->x)*180/3.1416;
		else{
			if (a->y>=0) a->angle=atan(a->y*1.0/a->x)*180/3.1416 + 180;
			else a->angle=atan(a->y*1.0/a->x)*180/3.1416 - 180;
		}
	}
	return a->angle;
}

float cal_myangle(int* x,int* y,int maxn){
	//(-180,180]
	int xi=0,x2=0,yi=0,xy=0;
	float k;
	for(int i = 0; i < maxn; i++){
		xi += x[i];
		x2 += x[i] * x[i];
		yi += y[i];
		xy += x[i] * y[i];
	}
	if (xi * xi - x2 * maxn==0) return 90;
	else{
		k = (yi * xi - xy * maxn)*1.0 / (xi * xi - x2 * maxn);
		printf("\nk=%f\n",k);
		printf("\natan(k)=%f",atan(k));
		if (x[0]-x[maxn-1]>=0) return atan(k)*180/3.1416;
		else{
			if (y[0]-y[maxn-1]>=0) return atan(k)*180/3.1416 + 180;
			else return atan(k)*180/3.1416 - 180;
		}
	}
}

void go(uint8_t l,uint8_t r){
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,r);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,l);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6,GPIO_PIN_SET);
}

struct _PID{
	int setvalue,error_1,error_2;
	float Kp,Ki,Kd;
};
typedef struct _PID PID;

int PIDCal(PID *pp, int error){
	int perror,ierror,derror;
	float du;
	perror=error-pp->error_1;
	ierror=error;
	derror=error-2*(pp->error_1)+pp->error_2;
	du=pp->Kp*perror + pp->Ki*ierror + pp->Kd*derror;
	pp->error_2=pp->error_1;
	pp->error_1=error;
	return (int)du;
}

void PIDInit(PID *pp){
	pp->error_1=0;
	pp->error_2=0;
	pp->setvalue=0;
	pp->Kp=0;
	pp->Ki=0.1;
	pp->Kd=0;
}

/*-----MPU Part Start-----*/
//MPU6050寄存器地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样频率,典型值:0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率,典型值:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围,典型值:0x18(不自检,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速度计自检、测量范围、高通滤波频率,典型值:0x01(不自检,2G,5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B  //电源管理，典型值：0x00（正常启用）
#define	WHO_AM_I		0x75    //IIC地址寄存器（默认数值0x68，只读）
#define SlaveAddress 0xD0 //通过I2C向MPU6050写入数据时的“设备地址”（+1为读取） （具体含义参见I2C相关教程，MPU6050设备地址为0xD0）

#define FS_SEL_2000 0x18
#define FS_SEL_1000 0x10
#define FS_SEL_500 0x8
#define FS_SEL_250 0x0
#define FS_SEL FS_SEL_2000
#define GYRO_SCALE_RANGE 2000
//现选用的是满量程2000度（即加速度计输出32767时表示角速度2000度/s），如有必要可改量程，改量程时将上面最后两行的2000改为所需量程即可（其他可选量程：1000,500,250）

int16_t data; //从MPU6050直接读取的数据（注意：signed int 而非unsigned！）
float angle_speed; //由角度根据“满量程”值(Full Scale Range)计算得到的角速度
float angle=0.0; //积分得到的角度
float gyro_z_offset=0.0; //z轴角速度测量值偏移量(见下面InitMPU6050()函数)

void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_Data) //REG_Address 数据类型 uint8_t/uint16_t?
{
	HAL_I2C_Mem_Write(&hi2c1, SlaveAddress, REG_Address ,1 ,&REG_Data, 1, 1000);
	//注：第4个参数 MemAddSize 表示要写入的寄存器的大小
}

uint8_t Single_ReadI2C(uint8_t REG_Address) //REG_Address 数据类型 uint8_t/uint16_t?
{
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, SlaveAddress, REG_Address ,1 ,&data, 1, 1000);
	return data;
}


uint16_t Get_MPU_Data(uint8_t REG_Address) //要读取的2字节的数据的高字节地址
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return ((uint16_t)H)*256+L;
}


void InitMPU6050()
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//解除休眠状态
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, FS_SEL);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);
	
	//下面的代码：消除z轴角速度偏移量
	int i;
	int16_t _data;
	HAL_Delay(1000); //等MPU稳定之后再测 
	for(i=1;i<=10;i++)
	{
		_data=Get_MPU_Data(GYRO_ZOUT_H);
		//不要把Get_MPU_Data返回的int16_t类型数据直接赋值给float类型数据，单片机不会进行类型转换！要通过中间变量赋值！！ 
		gyro_z_offset+=_data;
		HAL_Delay(100); //100ms这样更准一些 
	}
	gyro_z_offset/=10;
	printf("offset:%f",gyro_z_offset);
	/*注：MPU6050测得的原始数据存在偏移量（角速度为0时从MPU6050的寄存器中读出的数据不为0）。
	可通过取若干次平均的方法求出偏移量，然后每次测量读取原始数据之后减去偏移量*/
	
}
/*-----MPU Part end-----*/

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	InitMPU6050();
	MessageInfo* message=(MessageInfo*)malloc(sizeof(MessageInfo));
	CarMove* carmove=(CarMove*)malloc(sizeof(CarMove));
	uint8_t isA=(HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_2)?1:0);
	
	int x[5],y[5];  //current and previous position
	for (int i=0;i<5;i++){
		x[i]=0;y[i]=0;
	}
	float angle_error;
	vector* myvector=(vector*)malloc(sizeof(vector));
	vector* desvector=(vector*)malloc(sizeof(vector)); 

	int delta=0;
	int leftV=90,rightV=90,leftv=0,rightv=0;  //V-平均速度, v-当前理论速度
	int leftpwm=0,rightpwm=0;  //当前pwm
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	uint8_t a[]="--This is a test message.--\r\n";
	printf("%s",a);  //send a test message
	HAL_UART_Receive_IT(&huart1,pitext,4);
	/*
	HAL_UART_Transmit(&huart3,"AT\r\n",4,100);
	HAL_UART_Transmit(&huart2,"AT\r\n",4,100);
	HAL_Delay(3000);
	
	HAL_UART_Transmit(&huart2,"AT+CWMODE=3\r\n",13,100);
	HAL_UART_Transmit(&huart3,"AT+CWMODE=3\r\n",13,100);
	HAL_Delay(3000);
  
	HAL_UART_Transmit(&huart2,"AT+RST\r\n",8,100);
	HAL_UART_Transmit(&huart3,"AT+RST\r\n",8,100);
	HAL_Delay(3000);
	*/
	HAL_UART_Transmit(&huart2,"AT+CWJAP=\"EDC20\",\"12345678\"\r\n",30,100);
	HAL_UART_Transmit(&huart3,"AT+CWJAP=\"EDC20\",\"12345678\"\r\n",30,100);
	HAL_Delay(5000);
	
  HAL_UART_Transmit(&huart2,"AT+CIPSTART=\"TCP\",\"192.168.1.105\",20000\r\n",41,100);
	HAL_UART_Transmit(&huart3,"AT+CIPSTART=\"TCP\",\"192.168.1.105\",20000\r\n",41,100);
	HAL_Delay(3000);
	
  HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	HAL_UART_Receive_IT(&huart2,receive,1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (pirefreshed==1){
			if (pitext[0]==48){
				delta = (pitext[1]-48)*100 + (pitext[2]-48)*10 + (pitext[3]-48);
			}
			else{
				delta = -((pitext[1]-48)*100 + (pitext[2]-48)*10 + (pitext[3]-48));
			}
			pirefreshed=0;
		}
		uint8_t pretype=carmove->type;
		if (refreshed==1){
			msgrefresh((char*)text,message,isA);
			refreshed=0;
			printf("\nmy_x=%d my_y=%d oppo_x=%d oppo_y=%d passengerNum=%d\n",
			message->my_x,message->my_y,message->oppo_x,message->oppo_y,message->passengerNum);
			for (int i=0;i<message->passengerNum;i++){
				printf("pass_status=%hhd xs_pos=%d ys_pos=%d xe_pos=%d ye_pos=%d\n",
				message->pass_status[i],message->xs_pos[i],message->ys_pos[i],message->xe_pos[i],message->ye_pos[i]);
			}
			for (int i=4;i>0;i--){
				x[i]=x[i-1];
				y[i]=y[i-1];
			}
			x[0]=message->my_x;
			y[0]=message->my_y;
			
			*carmove=GetNextMove(*message);
			
			//carmove->type=1;
			if (carmove->type==0){
				go(0,0);
			}
			if (carmove->type==1){  //go straight
				if (delta>=0){
					leftv = leftV - delta*0.08;
					rightv = rightV - delta*0.08 - delta*0.1;
				}
				else{
					leftv = leftV + delta*0.08 + delta*0.1;
					rightv = rightV + delta*0.08;
				}
				//printf("leftv=%d rightv=%d",leftv,rightv);
				leftpwm = leftv + (leftv - leftu)*0.5;
				rightpwm = rightv + (rightv - rightu)*0.5;
				if (leftpwm>100) leftpwm=100;
				if (leftpwm<30) leftpwm=30;
				if (rightpwm>100) rightpwm=100;
				if (rightpwm<30) rightpwm=30;
				//printf("leftpwm=%d rightpwm=%d",leftpwm,rightpwm);
				go(leftpwm,rightpwm);
				//calculate angle
				if (pretype!=1){
					for (int i=0;i<5;i++){
						x[i]=0;y[i]=0;
					}
				}

				desvector->x = carmove->dest_x - message->my_x;
				desvector->y = carmove->dest_y - message->my_y;
				cal_vecangle(desvector);	
				uint8_t maxn=4;
				while(x[maxn]==0) maxn--;
				maxn++;
				if (maxn==1) myvector->angle = desvector->angle;
				else myvector->angle = cal_myangle(x,y,maxn);
				angle_error = myvector->angle - desvector->angle;
				if (angle_error<=-180) angle_error+=360;
				if (angle_error>180) angle_error-=360;
				printf("myangle=%f\ndesangle=%f\nerror=%f\n\n",myvector->angle,desvector->angle,angle_error);
				
				if (abs(angle_error)==0||carmove->r<10){
					go(98,100);
				}
				else if ((angle_error>0)&&(angle_error<=5)){  
					go(98,95);
				}
				else if ((angle_error<0)&&(angle_error>=-5)){ 
					go(93,100);
				}
				else if ((angle_error>5)&&(angle_error<=15)){ 
					go(98,85);
				}
				else if ((angle_error<-5)&&(angle_error>=-15)){ 
					go(83,100);
				}
				else if (angle_error>15){
					go(100,70);
				}
				else if(angle_error<-15){
					go(70,100);
				}
			}
			if (carmove->type==2){
				if (carmove->angle<0){  //turn left
					go(70,100);
					angle=0;
						while(carmove->angle<angle && -1*carmove->angle>angle){
							data=Get_MPU_Data(GYRO_ZOUT_H);
							angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE*1.4/32768.0;
							angle+=angle_speed*0.01;
							HAL_Delay(10);
						}
					go(100,100);
				}
				else{  //turn right
					go(100,70);
					angle=0;
						while(carmove->angle>angle && -1*carmove->angle<angle){
							data=Get_MPU_Data(GYRO_ZOUT_H);
							angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE*1.4/32768.0;
							angle+=angle_speed*0.01;
							HAL_Delay(10);
						}
					go(100,100);
					}
				}
			}
		}
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 799;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 15;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 99;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	if (huart->Instance==USART2){
		temptext[current]=receive[0];
		current++;
		if (temptext[current-1]==0x0A && temptext[current-2]==0x0D){
			for(int i=0;i<64;i++){
				text[i]=temptext[current-64+i];
			}
		  current=0;
			refreshed=1;
		}
		HAL_UART_Receive_IT(&huart2,receive,1);
	}
	if (huart->Instance==USART1){
		pirefreshed=1;
		HAL_UART_Receive_IT(&huart1,pitext,3);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance==TIM1){
		rightu = (int)(__HAL_TIM_GET_COUNTER(&htim3))*fre*98/(160*8);
		leftu = (int)(__HAL_TIM_GET_COUNTER(&htim2))*fre*98/(160*8);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);	
	}	
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_0){  //right wheel, GPIO_PIN_0=1
		//printf("right edge\n");
		//l=90;
	}
	if (GPIO_Pin == GPIO_PIN_1){  //left wheel, GPIO_PIN_1=2
		//printf("left edge\n");
		//r=90;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
