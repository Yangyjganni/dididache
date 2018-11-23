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
#include "utils.h"
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
uint8_t fre=10;  //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æµï¿½ï¿½
volatile int leftu=0,rightu=0;  //ï¿½ï¿½Ç°ï¿½Ù¶ï¿½(pwmï¿½ï¿½Ê¾)

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

short cal_myangle(int* x,int* y,int maxn){
	//(-180,180]
	short _angle;
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
		//printf("\nk=%f\n",k);
		//printf("\natan(k)=%f",atan(k));
		if (x[0]-x[maxn-1]>=0) _angle = atan(k)*180/3.1416;
		else{
			if (y[0]-y[maxn-1]>=0) _angle = atan(k)*180/3.1416 + 180;
			else _angle = atan(k)*180/3.1416 - 180;
		}
	}
	if(_angle >= 360) _angle-= 360;
	else if(_angle < 0) _angle+= 360;
	return _angle;
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
//MPU6050ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ö·
#define	SMPLRT_DIV		0x19	//ï¿½ï¿½ï¿½ï¿½ï¿½Ç²ï¿½ï¿½ï¿½Æµï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½Öµ:0x07(125Hz)
#define	CONFIG			0x1A	//ï¿½ï¿½Í¨ï¿½Ë²ï¿½Æµï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½Öµ:0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ô¼ì¼°ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î§,ï¿½ï¿½ï¿½ï¿½Öµ:0x18(ï¿½ï¿½ï¿½Ô¼ï¿½,2000deg/s)
#define	ACCEL_CONFIG	0x1C	//ï¿½ï¿½ï¿½Ù¶È¼ï¿½ï¿½Ô¼ì¡¢ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Î§ï¿½ï¿½ï¿½ï¿½Í¨ï¿½Ë²ï¿½Æµï¿½ï¿½,ï¿½ï¿½ï¿½ï¿½Öµ:0x01(ï¿½ï¿½ï¿½Ô¼ï¿½,2G,5Hz)
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
#define	PWR_MGMT_1		0x6B  //ï¿½ï¿½Ô´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½0x00ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ã£ï¿½
#define	WHO_AM_I		0x75    //IICï¿½ï¿½Ö·ï¿½Ä´ï¿½ï¿½ï¿½ï¿½ï¿½Ä¬ï¿½ï¿½ï¿½ï¿½Öµ0x68ï¿½ï¿½Ö»ï¿½ï¿½ï¿½ï¿½
#define SlaveAddress 0xD0 //Í¨ï¿½ï¿½I2Cï¿½ï¿½MPU6050Ð´ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½Ä¡ï¿½ï¿½è±¸ï¿½ï¿½Ö·ï¿½ï¿½ï¿½ï¿½+1Îªï¿½ï¿½È¡ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ï¿½åº¬ï¿½ï¿½Î¼ï¿½I2Cï¿½ï¿½Ø½Ì³Ì£ï¿½MPU6050ï¿½è±¸ï¿½ï¿½Ö·Îª0xD0ï¿½ï¿½

#define FS_SEL_2000 0x18
#define FS_SEL_1000 0x10
#define FS_SEL_500 0x8
#define FS_SEL_250 0x0
#define FS_SEL FS_SEL_2000
#define GYRO_SCALE_RANGE 2000
//ï¿½ï¿½Ñ¡ï¿½Ãµï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½2000ï¿½È£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶È¼ï¿½ï¿½ï¿½ï¿½32767Ê±ï¿½ï¿½Ê¾ï¿½ï¿½ï¿½Ù¶ï¿½2000ï¿½ï¿½/sï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ð±ï¿½Òªï¿½É¸ï¿½ï¿½ï¿½ï¿½Ì£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ê±ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ðµï¿½2000ï¿½ï¿½Îªï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¼ï¿½ï¿½É£ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ñ¡ï¿½ï¿½ï¿½Ì£ï¿½1000,500,250ï¿½ï¿½

int16_t data; //ï¿½ï¿½MPU6050Ö±ï¿½Ó¶ï¿½È¡ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½×¢ï¿½â£ºsigned int ï¿½ï¿½ï¿½ï¿½unsignedï¿½ï¿½ï¿½ï¿½
float angle_speed; //ï¿½É½Ç¶È¸ï¿½ï¿½Ý¡ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ì¡ï¿½Öµ(Full Scale Range)ï¿½ï¿½ï¿½ï¿½Ãµï¿½ï¿½Ä½ï¿½ï¿½Ù¶ï¿½
float angle=0.0; //ï¿½ï¿½ï¿½ÖµÃµï¿½ï¿½Ä½Ç¶ï¿½
float gyro_z_offset=0.0; //zï¿½ï¿½ï¿½ï¿½Ù¶È²ï¿½ï¿½ï¿½ÖµÆ«ï¿½ï¿½ï¿½ï¿½(ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½InitMPU6050()ï¿½ï¿½ï¿½ï¿½)

void Single_WriteI2C(uint8_t REG_Address, uint8_t REG_Data) //REG_Address ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ uint8_t/uint16_t?
{
	HAL_I2C_Mem_Write(&hi2c1, SlaveAddress, REG_Address ,1 ,&REG_Data, 1, 1000);
	//×¢ï¿½ï¿½ï¿½ï¿½4ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ MemAddSize ï¿½ï¿½Ê¾ÒªÐ´ï¿½ï¿½Ä¼Ä´ï¿½ï¿½ï¿½ï¿½Ä´ï¿½Ð¡
}

uint8_t Single_ReadI2C(uint8_t REG_Address) //REG_Address ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ uint8_t/uint16_t?
{
	uint8_t data;
	HAL_I2C_Mem_Read(&hi2c1, SlaveAddress, REG_Address ,1 ,&data, 1, 1000);
	return data;
}


uint16_t Get_MPU_Data(uint8_t REG_Address) //Òªï¿½ï¿½È¡ï¿½ï¿½2ï¿½Ö½Úµï¿½ï¿½ï¿½ï¿½ÝµÄ¸ï¿½ï¿½Ö½Úµï¿½Ö·
{
	uint8_t H,L;
	H=Single_ReadI2C(REG_Address);
	L=Single_ReadI2C(REG_Address+1);
	return ((uint16_t)H)*256+L;
}


void InitMPU6050()
{
	Single_WriteI2C(PWR_MGMT_1, 0x00);	//ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×´Ì¬
	Single_WriteI2C(SMPLRT_DIV, 0x07);
	Single_WriteI2C(CONFIG, 0x06);
	Single_WriteI2C(GYRO_CONFIG, FS_SEL);
	Single_WriteI2C(ACCEL_CONFIG, 0x01);
	
	//ï¿½ï¿½ï¿½ï¿½Ä´ï¿½ï¿½ë£ºï¿½ï¿½ï¿½ï¿½zï¿½ï¿½ï¿½ï¿½Ù¶ï¿½Æ«ï¿½ï¿½ï¿½ï¿½
	int i;
	int16_t _data;
	HAL_Delay(1000); //ï¿½ï¿½MPUï¿½È¶ï¿½Ö®ï¿½ï¿½ï¿½Ù²ï¿½ 
	for(i=1;i<=10;i++)
	{
		_data=Get_MPU_Data(GYRO_ZOUT_H);
		//ï¿½ï¿½Òªï¿½ï¿½Get_MPU_Dataï¿½ï¿½ï¿½Øµï¿½int16_tï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ö±ï¿½Ó¸ï¿½Öµï¿½ï¿½floatï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý£ï¿½ï¿½ï¿½Æ¬ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×ªï¿½ï¿½ï¿½ï¿½ÒªÍ¨ï¿½ï¿½ï¿½Ð¼ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Öµï¿½ï¿½ï¿½ï¿½ 
		gyro_z_offset+=_data;
		HAL_Delay(100); //100msï¿½ï¿½ï¿½ï¿½ï¿½ï¿½×¼Ò»Ð© 
	}
	gyro_z_offset/=10;
	printf("offset:%f",gyro_z_offset);
	/*×¢ï¿½ï¿½MPU6050ï¿½ï¿½Ãµï¿½Ô­Ê¼ï¿½ï¿½ï¿½Ý´ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶ï¿½Îª0Ê±ï¿½ï¿½MPU6050ï¿½Ä¼Ä´ï¿½ï¿½ï¿½ï¿½Ð¶ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Ý²ï¿½Îª0ï¿½ï¿½ï¿½ï¿½
	ï¿½ï¿½Í¨ï¿½ï¿½È¡ï¿½ï¿½ï¿½É´ï¿½Æ½ï¿½ï¿½ï¿½Ä·ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½Æ«ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½È»ï¿½ï¿½Ã¿ï¿½Î²ï¿½ï¿½ï¿½ï¿½ï¿½È¡Ô­Ê¼ï¿½ï¿½ï¿½ï¿½Ö®ï¿½ï¿½ï¿½È¥Æ«ï¿½ï¿½ï¿½ï¿½*/
	
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
	setInitHandle(&huart3);
	//InitMPU6050();
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
	int leftV=90,rightV=90,leftv=0,rightv=0;  //V-Æ½ï¿½ï¿½ï¿½Ù¶ï¿½, v-ï¿½ï¿½Ç°ï¿½ï¿½ï¿½ï¿½ï¿½Ù¶ï¿½
	int leftpwm=0,rightpwm=0;  //ï¿½ï¿½Ç°pwm
	
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
	
	HAL_UART_Transmit(&huart3,"AT\r\n",4,100);
	HAL_UART_Transmit(&huart2,"AT\r\n",4,100);
	HAL_Delay(3000);
	
	HAL_UART_Transmit(&huart2,"AT+CWMODE=3\r\n",13,100);
	HAL_UART_Transmit(&huart3,"AT+CWMODE=3\r\n",13,100);
	HAL_Delay(3000);
  
	HAL_UART_Transmit(&huart2,"AT+RST\r\n",8,100);
	HAL_UART_Transmit(&huart3,"AT+RST\r\n",8,100);
	HAL_Delay(3000);
	
	HAL_UART_Transmit(&huart2,"AT+CWJAP=\"EDC20\",\"12345678\"\r\n",30,100);
	HAL_UART_Transmit(&huart3,"AT+CWJAP=\"EDC20\",\"12345678\"\r\n",30,100);
	HAL_Delay(5000);
	
  HAL_UART_Transmit(&huart2,"AT+CIPSTART=\"TCP\",\"192.168.1.100\",20000\r\n",41,100);
	HAL_UART_Transmit(&huart3,"AT+CIPSTART=\"TCP\",\"192.168.1.100\",20000\r\n",41,100);
	HAL_Delay(3000);
	
  HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	HAL_UART_Receive_IT(&huart2,receive,1);
	
	if(1==isA) 
	{
		myvector->x=0;
		myvector->y=0;
		myvector->angle=0;
	}
	else
	{
		myvector->x=0;
		myvector->y=0;
		myvector->angle=0;
	}
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
			
			short _angle;
			uint8_t maxn=4;
			while(x[maxn]==0) maxn--;
			maxn++;
			if (maxn==0||maxn==1) _angle = -1;
			else _angle = cal_myangle(x,y,maxn);
			_angle=-1;
			*carmove=GetNextMoveWithAngle(*message,_angle);
			if (carmove->type==0){
				printf("type=0");
			}
			if (carmove->type==1){
				printf("type=1 x=%d y=%d dis=%f  mm",carmove->dest_x,carmove->dest_y, carmove->dis);
			}
			if (carmove->type==2){
				printf("type=2 angle=%d dis=%f mm",carmove->angle, carmove->dis);
			}
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
			}

			if(carmove->type==2)//turning without camera
			{
				//value
				float type2_feedback=1.4;
				double type2_angle=0;
				int type2_radius1=15;
				int type2_radius2=28;
				int type2_a,type2_b;
				//radius
				if(carmove->r<type2_radius1) {type2_a=30; type2_b=98;}
				else if(carmove->r>type2_radius1) {type2_a=55; type2_b=98;}
				else {type2_a=85; type2_b=98;}
				//angle
				type2_angle=0;
				if(carmove->angle>=0){
					go(type2_b,type2_a);
					while(type2_angle<carmove->angle && type2_angle>carmove->angle*-1){
						data=Get_MPU_Data(GYRO_ZOUT_H);
						angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
						type2_angle+=angle_speed*0.05*type2_feedback;
						HAL_Delay(50);
					}
				}
				else {
					go(type2_a,type2_b);
					while(type2_angle<carmove->angle*-1 && type2_angle>carmove->angle){
						data=Get_MPU_Data(GYRO_ZOUT_H);
						angle_speed=(data-gyro_z_offset)*GYRO_SCALE_RANGE/32768.0;
						type2_angle+=angle_speed*0.05*type2_feedback;
						HAL_Delay(50);
					}
				}
				go(100,100);
			}
//éœ€è°ƒæ•´é‡ï¼šradius1,radius2,ä»¥åŠå…¶å¯¹åº”çš„å ç©ºæ¯”,feedback,ç§¯åˆ†é—´éš”
			
			else if(carmove->type==3) //point 18b to 26s / point 27s to 19a
			{
				int8_t i,max_i=-1; //max_i±íÊ¾ÔÚxºÍyÊý×éÖÐ´æ´¢ÁËÊý¾ÝµÄ×î´óÏÂ±ê
				for(i=0;i<=4;i++){
					x[i]=0; y[i]=0;
				}
				#define SLOWWHEELSPEED 83
				#define FASTWHEELSPEED 98
				//¿ìÂÖ¡¢ÂýÂÖÄ¬ÈÏ×ªËÙ£¨µ÷²Î¾ö¶¨£©
				//#define ABS_MINUS(a,b) (((a)>=(b))?((a)-(b)):((b)-(a)))
				#define POWER2(x) ((x)*(x))
				int center_x,center_y; //×ªÍäÖÐÐÄ
				float average_offset; //µ½Ô²ÖÜÉÏµÄÆ½¾ùÆ«ÒÆÁ¿
				
				if(116==carmove->dest_x) //point 18b to 26s 
				{
					center_x=55; center_y=290;
					go(SLOWWHEELSPEED,FASTWHEELSPEED);
					
					while(message->my_x<=116)
					{
						if(0==refreshed)continue;
						msgrefresh((char*)text,message,isA);
						refreshed=0;
						for(i=4;i>0;i--){
							x[i]=x[i-1];
							y[i]=y[i-1];
						}
						if(max_i<4) max_i++;
						x[0]=message->my_x;
						y[0]=message->my_y;
											
						if(4*x[0]+3*y[0] >= 1090) //½øÈëµÚ¶þ¶ÎÔ²»¡
						{
							center_x=115;
							center_y=210;
							max_i=-1; //ÖØÐÂ¼ÇÂ¼ÐÐÊ»Í¾ÖÐ¸÷µã
							for(i=0;i<=4;i++){
								x[i]=0; y[i]=0;
							}	
						}
						
						average_offset=0.0;
						for(i=0;i<=max_i;i++)
							average_offset+=sqrt(1.0*(POWER2(x[i]-center_x)+POWER2(y[i]-center_y)))-50; //×ªÍä°ë¾¶50
						average_offset/=(max_i+1);
						if(average_offset<-6.0) //¿¿½üÔ²ÐÄ
						{
							if(55==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED+10,FASTWHEELSPEED); //×óÂÖ¼ÓËÙ
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED,SLOWWHEELSPEED+10); //ÓÒÂÖ¼ÓËÙ
							}
						}
						else if(average_offset>6.0)
						{
							if(55==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED,FASTWHEELSPEED+10); //ÓÒÂÖ¼ÓËÙ
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED+10,SLOWWHEELSPEED); //×óÂÖ¼ÓËÙ
							}
						}
						else {
							if(55==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED,FASTWHEELSPEED); //ÓÒÂÖ¼ÓËÙ
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED,SLOWWHEELSPEED); //×óÂÖ¼ÓËÙ
							}
						}
					}
				}
				else //point 27s to 19a
				{
					center_x=210; center_y=115;
					go(FASTWHEELSPEED,SLOWWHEELSPEED);
					
					while(message->my_y>=58)
					{
						if(0==refreshed)continue;
						msgrefresh((char*)text,message,isA);
						refreshed=0;
						for(i=4;i>0;i--){
							x[i]=x[i-1];
							y[i]=y[i-1];
						}
						if(max_i<4) max_i++;
						x[0]=message->my_x;
						y[0]=message->my_y;
						if(max_i>=1) myvector->angle=cal_myangle(x,y,max_i);
						
						if(3*x[0]+4*y[0] <= 1090) //½øÈëµÚ¶þ¶ÎÔ²»¡
						{
							center_x=290;
							center_y=55;
							max_i=-1; //ÖØÐÂ¼ÇÂ¼ÐÐÊ»Í¾ÖÐ¸÷µã
							for(i=0;i<=4;i++){
								x[i]=0; y[i]=0;
							}
						}
						
						average_offset=0.0;
						for(i=0;i<=max_i;i++)
							average_offset+=sqrt(1.0*(POWER2(x[i]-center_x)+POWER2(y[i]-center_y)))-50; //×ªÍä°ë¾¶50
						average_offset/=(max_i+1);
						if(average_offset<-6.0) //¿¿½üÔ²ÐÄ
						{
							if(210==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED,SLOWWHEELSPEED+10);
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED+10,FASTWHEELSPEED);
							}
						}
						else if(average_offset>6.0)
						{
							if(210==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED+10,SLOWWHEELSPEED);
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED,FASTWHEELSPEED+10);
							}
						}
						else
						{
							if(210==center_x) //Ç°°ë¶ÎÔ²»¡
							{
								go(FASTWHEELSPEED,SLOWWHEELSPEED);
							}
							else //ºó°ë¶ÎÔ²»¡
							{
								go(SLOWWHEELSPEED,FASTWHEELSPEED);
							}
						}
					}
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
