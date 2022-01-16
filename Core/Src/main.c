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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_CHANNEL TIM_CHANNEL_4			//Set channel for PWM
#define GPIO_PIN_DIRECTION GPIO_PIN_10		//Set pin for direction


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t NO_KALMAN = 1;
uint8_t Prev_NO_KALMAN = 1;

uint64_t _micros = 0;				//Keep track of time
uint64_t Time_Sampling_Stamp = 0;	//Control loop time stamp
uint64_t Time_Velocity_Stamp = 0;
uint64_t Time_Trajectory_Stamp = 0;	//Time to calculate in "Trajectory_Generataion()"
uint16_t Encoder_Resolution = 8192; //2048*4
uint16_t Encoder_Overflow = 4096;	//8192/2
float pi = 3.14159265359;			//value of pi
const uint8_t Station_List[10] = {0,0,0,0,0,0,0,0,0,0};
uint16_t Goal_List[20] = {0};
uint16_t GO = 0;

uint16_t Distance_Length = 0;		//Long or Short distance
uint16_t Distance_Calculated = 0;	//Calculate distance yet?

//Velocity Control
float Velocity_Read_Encoder = 0;  		//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in pulse per second
float Velocity_Now_RPM = 0;			//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in RPM
float Velocity_Want_RPM = 0;		//Velocity calculated for control motor at the time
float Velocity_K_P = 2000;			//K_P of "Velocity_Control()"
float Velocity_K_I = 1;				//K_I of "Velocity_Control()"
float Velocity_K_D = 0;				//K_D of "Velocity_Control()"
float Velocity_Error = 0;			//Velocity error from Velocity_Want_RPM
float Velocity_Error_Sum = 0;		//Sum of Velocity_Error that will be use in integrate part of "Velocity_Control()"
float Velocity_Error_Diff = 0;		//Difference of Velocity_Error and Velocity_Error_Prev that will be use in differential part of "Velocity_Control()"
float Velocity_Error_Prev = 0;		//last Velocity_Error
int16_t PWM_Out = 0;				//PWM for motor

//Position Control
float Position_Read_Encoder = 0;  		//Encoder's now position in CNT
float Position_Now_Degree = 0;		//Encoder's now position in degree
float Position_Want_Degree = 0;		//Position of the end point  (actually same as Point_Stop)
float Position_Prev_Degree = 0;		//Check that Position_Want_Degree changed or not
float Position_K_P = 228.62;		//K_P of "Position_Control()"
float Position_K_I = 0;			    //K_I of "Position_Control()"
float Position_K_D = 7.1;			//K_D of "Position_Control()"
float Position_Error = 0;			//Position error from Position_Want_Degree
float Position_Error_Sum = 0;		//Sum of Position_Error that will be use in integrate part of "Position_Control()"
float Position_Error_Diff = 0;		//Difference of Position_Error and Position_Error_Prev that will be use in differential part of "Position_Control()"
float Position_Error_Prev = 0;		//last Position_Error

//Trajectory Generation
float Accel_Max = 0.5;			    //Max acceleration & fixed at 0.5
float Velocity_Now_Rad = 0;			//Encoder's velocity Calculated from "Encoder_Velocity_Update()" in radian per second
float Velocity_Max_RPM = 10;		//Max velocity in RPM & wanted Velocity that system try to achieve & can be change by UART mode 4
float Velocity_Max_Rad = 0;			//Max velocity in radian per second
float Velocity_Achieve_RPM = 0;	    //Velocity limit in RPM motor can achieve in short distance
float Velocity_Achieve_Rad = 0; 	//Velocity limit in radian per second motor can achieve in short distance
float Distance_Degree_Set = 0;		//Distance from Point_Start to Point_Stop in degree
float Distance_Radian_Set = 0;		//Distance from Point_Start to Point_Stop in radian
float Distance_Blend = 0;			//Calculated distance for when a != 0 and check that have enough distance and time to achieve Velocity_Max_RPM or not
float Distance_Center = 0;			//Distance when Velocity_Want = Velocity_Max_RPM
float Position_Start = 0;			//Start position
float Position_Now_Rad = 0;				//Encoder's now position in radian
float Time_Blend = 0;				//Calculated time for when a != 0 and have enough distance and time to achieve Velocity_Max_RPM
float Time_All = 0;					//Calculated time for all distance
float Time_Center = 0;
uint64_t Time_Start = 0;			//Start time
float Time_Blend_Micro = 0;			//Time_Blend in microsecond
float Time_All_Micro = 0;			//Time_All in microsecond
float Time_Center_Micro = 0;
uint16_t Trajectory_Flag = 0;		//For keeping Position_Start & Time_Start or Finished

//Kalman_Filter
float Sigma_a = 14;
float Sigma_w = 0.8;
float Q = 0;
float R = 0;
float Position_Kalman = 0;
float Velocity_Kalman = 0;
float p11 = 0;
float p12 = 0;
float p21 = 0;
float p22 = 0;
const float CON_T = 0.001;

//debugging
float Flag4 = 0;
float P1 = 0;
float P2 = 0;
float P3 = 0;
float Distance_Traveled = 0;

//Mai's part debug
//quintic polynomial
//requirement
float angle_rad_start = 0; //rad
float angle_rad_stop = 0; //rad
float omega_max = 1; // 1 rad/s = 10 rpm
float alpha_max = 0.5; // rad/s^2
//find time duration for each viapoint
float tau_max = 0; // sec
uint64_t time_initial = 0; //initial time of each viapoint
//coeffient parameter
float c_0 = 0;
float c_1 = 0;
float c_2 = 0;
float c_3 = 0;
float c_4 = 0;
float c_5 = 0;
//parameter
uint8_t initial = 1;
uint64_t tau = 0; //sec
//position control
float desired_position = 0;
float error_position = 0;
float error_position_diff = 0;
float error_position_int = 0;
float error_position_prev = 0;
float position_kp = 0;
float position_ki = 0;
float position_kd = 0;
float position_bias = 0;
//velocity control
float desired_velocity = 0;
float error_velocity = 0;
float error_velocity_diff = 0;
float error_velocity_int = 0;
float error_velocity_prev = 0;
float velocity_kp = 0;
float velocity_ki = 0;
float velocity_kd = 0;
float velocity_bias = 0;
//kalman filter
float theta_predict = 0;
float omega_predict = 0;
float theta_estimate = 0;
float omega_estimate = 0;
float p_predict11 = 0 ;
float p_predict12 = 0 ;
float p_predict21 = 0 ;
float p_predict22 = 0 ;
float p_estimate11 = 1 ;
float p_estimate12 = 0 ;
float p_estimate21 = 0 ;
float p_estimate22 = 1 ;
float z_predict = 0;
float s = 0;
float k11 = 0;
float k21 = 0;


//UART Protocol
typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;
uint8_t UART_Ack1[2] = {88,117};
uint8_t UART_Ack2[2] = {70,110};
UARTStucrture UART2 = {0};

typedef enum
{
	Start_Mode,
	N_Station,
	Data_Frame,
	Check_Sum,
}UART_State;

typedef enum
{
	Test_Command = 145,
	Connect_MCU,
	Disconnect_MCU,
	Velocity_Set,
	Position_Set,
	Goal_1_Set,
	Goal_N_Set,
	Go_to_Goal,
	Station_Request,
	Position_Request,
	Velocity_Request,
	Gripper_On,
	Gripper_Off,
	Home_Set
}UART_Mode;

UART_State State = Start_Mode;
UART_Mode Mode = 144;
uint8_t Frame;
uint8_t N;
uint8_t Data;
uint8_t Sum;
uint8_t len;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
float Encoder_Velocity_Update();	//Read Encoder position and calculate into velocity
void Motor_Drive_PWM();				//Supply PWM and control motor
void Velocity_Control();			//PID velocity control and calculate PWM for "Motor_Drive_PWM()"
void Position_Control();			//PID position control for short distance
void Trajectory_Generation();		//Calculate velocity base on Accel_Max for long distance
void Distance_Calculation();		//Calculate distance if it short or long
void Kalman_Filter();

void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void UART_Protocol(UARTStucrture *uart, int16_t dataIn);
void UART_Do_Command();

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);					//micros()
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);   //Start reading encoder
  HAL_TIM_Base_Start(&htim3);						//Start TIM3
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);			//Start PWM TIM3

  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


	  	int16_t inputChar = UARTReadChar(&UART2);
	  	if (inputChar != -1)
	  	{
	  		UART_Protocol(&UART2, inputChar);
			len+=1;

	  	}

	  if (micros() - Time_Velocity_Stamp >= 100)
	  {
		  Time_Velocity_Stamp = micros();
		  Velocity_Read_Encoder = (Velocity_Read_Encoder*9999 + Encoder_Velocity_Update())/(float)10000; //pulse per sec
		  Velocity_Now_RPM = (Velocity_Read_Encoder*60)/Encoder_Resolution;	//Convert Velocity_Read_Encoder (Encoder's velocity at the moment) to RPM
		  Velocity_Now_Rad = (Velocity_Now_RPM*2*pi)/60;

		  //read position
		  Position_Read_Encoder = htim1.Instance->CNT;
		  Position_Now_Rad = (Position_Read_Encoder*2*pi)/Encoder_Resolution;  //rad
	  }
	  if (micros() - Time_Sampling_Stamp >= 1000)	  //Control loop
	  {

			Time_Sampling_Stamp = micros();

			if (initial == 1 && angle_rad_stop - angle_rad_start != 0)
			{
				//calculate tau
				//short if condition
				tau_max = 15/8*(angle_rad_stop - angle_rad_start)/omega_max >= sqrtf(((10*pow(3+sqrt(3),1))-(5*powf(3+sqrtf(3),2))+(5/9*powf(3+sqrtf(3),3)))*(angle_rad_stop-angle_rad_start)/alpha_max) ? 15/8*(angle_rad_stop - angle_rad_start)/omega_max : sqrtf(((10*powf(3+sqrtf(3),1))-(5*powf(3+sqrtf(3),2))+(5/9*powf(3+sqrtf(3),3)))*(angle_rad_stop-angle_rad_start)/alpha_max);
				//calculate coeffient
				c_0 = angle_rad_start;
				c_1 = 0;
				c_2 = 0;
				c_3 = 10*((angle_rad_stop - angle_rad_start)/(pow(tau_max,3)));
				c_4 = 15*((angle_rad_start - angle_rad_stop)/(pow(tau_max,4)));
				c_5 = 6*((angle_rad_stop - angle_rad_start)/(pow(tau_max,5)));
				//save initial time
				//change microsec to second
				time_initial = micros()/1000000.0;
				initial = 0;

				//initial parameter in kalman filter
				theta_estimate = desired_position;
				omega_estimate = 0;

			}
			else if (initial == 0 &&  angle_rad_stop - angle_rad_start != 0)
			{
				//at the final point
				//tau = (micros()/1000000.0)-time_initial ; in second unit
				if ((micros()/1000000.0)-time_initial >= tau_max)
				{
					initial = 1;
					angle_rad_start = angle_rad_stop;
				}
				else //on going to final point
				{
					//tau = real time - initial time (duration in second unit)
					tau = micros()/1000000.0 - time_initial;
					desired_position = c_0*powf(tau,0) + c_1*powf(tau,1) + c_2*powf(tau,2) + c_3*powf(tau,3) + c_4*powf(tau,4) + c_5*powf(tau,5);

					//kalman filter
					//predict state
					//x_predict = A*x_estimate
					//x_predict = [theta_predict; omega_predict]
					//A = [1 dt; 0 1]
					//x_estimate = [theta_estimate; omega_estimate]
					theta_predict = theta_estimate + omega_estimate*CON_T;
					omega_predict = omega_estimate;

					//p_predict = A*p_estimate*transpose(A) + G*Q*transpose(G)
					//p_predict = [p_predict11 p_predict12 ; p_predict21 p_predict22]
					//p_estimate = [p_estimate11 p_estimate12 ; p_estimate21 p_estimate22] -> initial [1 0; 0 1]
					//G = [0.5*dt^2 ; dt]
					//Q = Sigma_a^2
					Q = powf(Sigma_a,2);
					p_predict11 = p_estimate11 + (p_estimate12 + p_estimate21)*CON_T + p_estimate22*powf(CON_T,2)+powf(CON_T,4)*Q/4.0;
					p_predict12 = p_estimate12 + p_estimate22*CON_T + powf(CON_T,3)*Q/2.0;
					p_predict21 = p_estimate21 + p_estimate22*CON_T + powf(CON_T,3)*Q/2.0;
					p_predict22 = p_estimate22 + powf(CON_T,2)*Q;

					//update
					//z_predict = z - C*x_predict
					//z_predict = theta_error
					//z = sensor_theta_input
					//C = [1 0]
					z_predict = Position_Now_Rad - theta_predict;

					//S = C*p_predict*transpose(C) + R
					//R = Sigma_w^2
					R = powf(Sigma_w,2);
					s = p_predict11 + R;

					//K = p_predict*transpose(C)*inv(S)
					//K = [k11;k21]
					k11 = p_predict11/s;
					k21 = p_predict21/s;

					//x_estimate = x_predict + K*z_predict
					theta_estimate = theta_predict + k11*z_predict;
					omega_estimate = omega_predict + k21*z_predict;

					//p_estimate = (I - K*C)*p_predict
					//I = [1 0; 0 1]
					p_estimate11 = k11*R;
					p_estimate12 = p_predict12*R/s;
					p_estimate21 = -k21*p_predict11 + p_predict21;
					p_estimate22 = p_predict22 - k21*p_predict12;

					//use estimate theta
					Position_Now_Rad = theta_estimate;

					//position control
					error_position = desired_position - Position_Now_Rad;
					error_position_diff = (error_position - error_position_prev)*1000.0;
					error_position_int = error_position_int + error_position/1000.0;
					desired_velocity = position_kp*error_position + position_ki*error_position_int + position_kd*error_position_diff + position_bias;
					error_position_prev = error_position;

					//limitter velocity
					if (desired_velocity > 1)
					{
						desired_velocity = 1;
					}
					else if (desired_velocity < -1)
					{
						desired_velocity = -1;
					}

					//velocity control
					error_velocity = desired_velocity - Velocity_Now_Rad;
					error_velocity_diff = (error_velocity - error_velocity_prev)*1000.0;
					error_velocity_int = error_velocity_int + error_velocity/1000.0;
					PWM_Out = velocity_kp*error_velocity + velocity_ki*error_velocity_int + velocity_kd*error_velocity_diff + velocity_bias;
					error_velocity_prev = error_velocity;

					//limitter pwm
					if (PWM_Out > 10000)
					{
						PWM_Out = 10000;
					}
					else if (PWM_Out < -10000)
					{
						PWM_Out = -10000;
					}

					//control motor direction
					if (PWM_Out < 0)
					{
						__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, -PWM_Out);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_RESET);
					}
					else if (PWM_Out >= 0)
					{
						__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, PWM_Out);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_SET);
					}

				}
			}




//			if(NO_KALMAN)
//			{
//				Position_Read_Encoder = htim1.Instance->CNT; //Read Encoder
//				Position_Now_Degree = (Position_Read_Encoder*360)/Encoder_Resolution; //Convert Encoder CNT to degree
//				NO_KALMAN=Prev_NO_KALMAN;
//			}
//			else
//			{
//				if(NO_KALMAN!=Prev_NO_KALMAN)
//				{
//					Position_Read_Encoder = htim1.Instance->CNT; //Read Encoder
//					Position_Now_Degree = (Position_Read_Encoder*360)/Encoder_Resolution; //Convert Encoder CNT to degree
//					Position_Kalman = Position_Now_Degree*pi/180;
//				}
//
//				Kalman_Filter();
//				Position_Now_Rad = Position_Kalman;
//				Velocity_Now_Rad = Velocity_Kalman;
//				Position_Now_Degree = Position_Now_Rad*180/pi;
//			}
//
//
//		if ((Distance_Calculated == 0 )&& (Position_Now_Degree != Position_Want_Degree)) //Distance not calculated and not arrive at next station
//			{
//				Distance_Calculation();		//Calculate distance
//			}
//			else if ((Distance_Calculated == 1) && (Position_Now_Degree != Position_Want_Degree) && (Trajectory_Flag < 5)) //Distance calculated and not arrive at next station
//			{
//				Trajectory_Generation();	//Get Velocity_Want_RPM
//				Velocity_Control();
//				Motor_Drive_PWM();			//Drive
//
//				if(Trajectory_Flag == 4)
//				{
//					/*if(Distance_Length == 0)
//					{
//						Trajectory_Flag = 5;
//					}
//					else if (Distance_Length == 1)
//					{
//						Distance_Calculation();
//						Trajectory_Flag = 0;
//					}*/
//
//					Trajectory_Flag = 5;
//
//				}
//
//
//			}
//
//
//			if (Trajectory_Flag == 5)		//Reach next station
//			{
//
//				if (Position_Prev_Degree != Position_Want_Degree)	//Change goal
//				{
//					Trajectory_Flag = 0;	//Reset flag
//					Distance_Calculated = 0;//Reset distance
//					Velocity_Want_RPM = 0;  //Reset Velocity_Want_RPM
//					Velocity_Error_Sum = 0;
//				}
//				Velocity_Want_RPM = 0;
//				Velocity_Control();
//				PWM_Out = 0;
//				Motor_Drive_PWM();			//Drive
//
//			}
//			Position_Prev_Degree = Position_Want_Degree; //Check that Position_Want_Degree change or not



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8191;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_Input_Proxreal_Pin Encoder_X_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_Proxreal_Pin|Encoder_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_Input_Proxtest_Pin */
  GPIO_InitStruct.Pin = GPIO_Input_Proxtest_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_Input_Proxtest_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
float Encoder_Velocity_Update()  //Lecture code DON'T TOUCH!
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	//read data
	uint32_t EncoderNowPosition = htim1.Instance->CNT; //pulse
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//compensate overflow and underflow
	if (EncoderPositionDiff >= Encoder_Overflow)
	{
		EncoderPositionDiff -= Encoder_Resolution;
	}
	else if (-EncoderPositionDiff >= Encoder_Overflow)
	{
		EncoderPositionDiff += Encoder_Resolution;
	}

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity
	//EncoderTimeDiff is in uS
	return (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;
}

#define PWM_CHANNEL TIM_CHANNEL_4			//Set channel for PWM
#define GPIO_PIN_DIRECTION GPIO_PIN_10		//Set pin for direction
void Motor_Drive_PWM()	//Motor drive
{
	if (PWM_Out > 10000)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		PWM_Out = 10000;		//Run with Velocity_Max_RPM
	}
	else if (PWM_Out < -10000)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		PWM_Out = -10000;		//Run with Velocity_Max_RPM
	}

	if (PWM_Out < 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, -PWM_Out);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_RESET);
	}

	else if (PWM_Out >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, PWM_CHANNEL, PWM_Out);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_DIRECTION, GPIO_PIN_SET);
	}
}

void Kalman_Filter()
{
	Q = powf(Sigma_a, 2);
	R = powf(Sigma_w, 2);

	float Position_Kalman_New = Position_Kalman + (Velocity_Kalman*CON_T);
	float Velocity_Kalman_New = 0 + Velocity_Kalman;
	float ye = Velocity_Now_Rad - Velocity_Kalman_New;

	p11 = p11 + (CON_T*p21) + (Q*powf(fabs(CON_T),4))/4 + (powf(fabs(CON_T),2)*(p12 + CON_T*p22))/CON_T;
	p12 = p12 + (CON_T*p22) + (Q*CON_T*powf(fabs(CON_T),2))/2;
	p21 = (2*CON_T*p21 + Q*powf(fabs(CON_T),4) + 2*p22*powf(fabs(CON_T),2))/(2*CON_T);
	p22 = Q*powf(fabs(CON_T),2) + p22;

	Position_Kalman_New = Position_Kalman_New + (p12*ye)/(R+p22);
	Velocity_Kalman_New = Velocity_Kalman_New + (p22*ye)/(R+p22);

	p11 = p11 - (p12*p21)/(R+p22);
	p12 = p12 - (p12*p22)/(R+p22);
	p21 = -p21*(p22/(R+22)-1);
	p22 = -p22*(p22/(R+22)-1);

	Position_Kalman = Position_Kalman_New;
	Velocity_Kalman = Velocity_Kalman_New;
}

void Velocity_Control()  //Velocity Control PID
{

	if(NO_KALMAN)
	{
		Velocity_Now_RPM = (Velocity_Read_Encoder*60)/Encoder_Resolution;	//Convert Velocity_Read_Encoder (Encoder's velocity at the moment) to RPM
		Velocity_Now_Rad = (Velocity_Now_RPM*2*pi)/60;
	}

	else
	{
		Velocity_Now_Rad = Velocity_Kalman;
		Velocity_Now_RPM = Velocity_Now_Rad*60/(2*pi);
	}

	if (Velocity_Want_RPM > Velocity_Max_RPM)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		Velocity_Want_RPM = Velocity_Max_RPM;		//Run with Velocity_Max_RPM
	}
	else if (Velocity_Want_RPM < -Velocity_Max_RPM)		//If Velocity_Want_RPM exceed Velocity_Max_RPM
	{
		Velocity_Want_RPM = -Velocity_Max_RPM;		//Run with Velocity_Max_RPM
	}

	Velocity_Error = Velocity_Want_RPM - Velocity_Now_RPM;
	Velocity_Error_Sum = Velocity_Error_Sum + Velocity_Error;
	Velocity_Error_Diff = Velocity_Error - Velocity_Error_Prev;
	Velocity_Error_Prev = Velocity_Error;

	PWM_Out = (Velocity_K_P*Velocity_Error) + (Velocity_K_I*Velocity_Error_Sum) + (Velocity_K_D*(Velocity_Error_Diff));

}
void Distance_Calculation()	//Calculate that distance is short or long
{
	//acceleration is fixed at 0.5 radian per second^2
	if(NO_KALMAN)
	{
		Position_Now_Degree = (Position_Read_Encoder*360)/Encoder_Resolution;  //degree
	}
	else
	{
		Position_Now_Degree = Position_Now_Rad*180/pi;
	}

	Distance_Degree_Set = Position_Want_Degree - Position_Now_Degree;  //Get distance from  EndPoint - StartPoint in degree
	if (Distance_Degree_Set < 0)
	{
		Distance_Degree_Set += 360;
	}
	Distance_Radian_Set = (Distance_Degree_Set*pi)/180;				   //Change Distance_Degree_Set to radian

	Velocity_Max_Rad = (Velocity_Max_RPM*2*pi)/60;					   //Change max velocity to radian per second

	Time_Blend = Velocity_Max_Rad*2;								   //Time used for motor to reach Velocity_Max_Rad with a=0.5 radian per second^2
	Time_Blend_Micro = Time_Blend*1000000;							   //Change from second to microsecond

	Distance_Blend = 2*(powf(Velocity_Max_Rad, 2));					   //Distance used for motor to reach Velocity_Max_Rad with a=0.5 radian per second^2


	if ((2*Distance_Blend) < Distance_Radian_Set)					   //Distance_Radian_Set is long enough to achieve Velocity_Max_Rad
	{
		Distance_Length = 1;
		Distance_Center = Distance_Radian_Set - (2*Distance_Blend);	   //Distance when a=0 radian per second^2
		Time_Center = Distance_Center/Velocity_Max_Rad;
		Time_Center_Micro = Time_Center*1000000;
		Time_All = (2*Time_Blend) + (Time_Center);//Time use to reach next station
		Time_All_Micro = Time_All*1000000;							   //Change from second to microsecond
	}

	else if ((2*Distance_Blend) >= Distance_Radian_Set)				   //Distance_Radian_Set is not long enough to achieve Velocity_Max_Rad
	{
		Distance_Length = 0;
		Time_Blend = sqrtf(Distance_Radian_Set*2);					   //Time used for motor to reach Velocity_Achieve_Rad with a=0.5 radian per second^2
		Time_Blend_Micro = Time_Blend*1000000;						   //Change from second to microsecond
		Time_All = (2*Time_Blend);
		Time_All_Micro = Time_All * 1000000;
		Velocity_Achieve_Rad = sqrtf(Distance_Radian_Set/2);		   //Top limit velocity that motor can achieve in short distance
		Velocity_Achieve_RPM = (Velocity_Achieve_Rad*60)/(2*pi);	   //Change from radian per second to RPM
	}

	Distance_Calculated = 1;
}
void Position_Control()  //Position Control PID
{

	if(NO_KALMAN)
	{
		Position_Read_Encoder = htim1.Instance->CNT;
		Position_Now_Degree = (Position_Read_Encoder*360)/Encoder_Resolution;  //degree

	}

	else
	{
		Position_Now_Degree = Position_Now_Rad*180/pi;
	}

	Position_Error = Position_Want_Degree - Position_Now_Degree;
	Position_Error_Sum = Position_Error_Sum + Position_Error;
	Position_Error_Diff = Position_Error - Position_Error_Prev;
	Position_Error_Prev = Position_Error;

	Velocity_Want_RPM = (Position_K_P*Position_Error) + (Position_K_I*Position_Error_Sum) + (Position_K_D*(Position_Error_Diff));

}
void Trajectory_Generation()  //Position Control with Trajectory Generation
{
	if(NO_KALMAN)
	{
		Position_Read_Encoder = htim1.Instance->CNT;
			Position_Now_Rad  = (Position_Read_Encoder*2*pi)/Encoder_Resolution;  //radian

	}
	else
	{
		Position_Now_Rad = Position_Kalman;
	}

	if (Trajectory_Flag == 0)
	{
		Time_Start = micros();
		Position_Start = Position_Now_Rad;
		Trajectory_Flag = 1;
	}
	Time_Trajectory_Stamp = micros();
	Distance_Traveled = Position_Now_Rad-Position_Start;
	if (Distance_Length == 1)
	{
		if ((Time_Trajectory_Stamp-Time_Start) <= Time_Blend_Micro)
		{
			Velocity_Want_RPM = Velocity_Max_RPM*((Time_Trajectory_Stamp-Time_Start)/Time_Blend_Micro);
			Trajectory_Flag = 2;
			P1 = Distance_Traveled;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) > (Time_Blend_Micro) )
				&& (Time_Trajectory_Stamp-Time_Start < Time_All_Micro-Time_Blend_Micro))
		{
			Velocity_Want_RPM = Velocity_Max_RPM;
			P2 = Distance_Traveled-P1;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) >= (Time_All_Micro-Time_Blend_Micro))
				&& (Time_Trajectory_Stamp-Time_Start <= Time_All_Micro) )
		{
			Velocity_Want_RPM = (-Velocity_Max_RPM)*((((Time_Trajectory_Stamp-Time_Start)-(Time_All_Micro-Time_Blend_Micro))/Time_Blend_Micro)-1);
			Trajectory_Flag = 3;

			P3 = Distance_Traveled-P2-P1;

		}
		else if ((Time_Trajectory_Stamp-Time_Start) >= Time_All_Micro)
		{
			Velocity_Want_RPM = 0;
			Trajectory_Flag = 4;
		}
	}

	else if (Distance_Length == 0)
	{
		if ((Time_Trajectory_Stamp-Time_Start) <= Time_Blend_Micro)
		{
			Velocity_Want_RPM = Velocity_Achieve_RPM*((Time_Trajectory_Stamp-Time_Start)/Time_Blend_Micro);
			Trajectory_Flag = 2;
		}
		else if (((Time_Trajectory_Stamp-Time_Start) >= Time_Blend_Micro)
				&& ((Time_Trajectory_Stamp-Time_Start) < (2*Time_Blend_Micro)))
		{
			Velocity_Want_RPM = (-Velocity_Achieve_RPM)*((((Time_Trajectory_Stamp-Time_Start)-Time_Blend_Micro)/Time_Blend_Micro)-1);
			Trajectory_Flag = 3;
		}
		else if ((Time_Trajectory_Stamp-Time_Start) >= (2*Time_Blend_Micro))
		{
			Velocity_Want_RPM = 0;
			Trajectory_Flag = 4;
		}
	}


}
void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;

}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}
uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}
int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;

}
void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}
void UART_Protocol(UARTStucrture *uart, int16_t dataIn)
{

	switch (State)
	{
	case Start_Mode:
		Mode = dataIn;
		switch (Mode)
		{
		case Test_Command:
			Frame = 2;
			State = Data_Frame;
			break;
		case Connect_MCU:
			Frame = 1;
			State = Check_Sum;
			break;
		case Disconnect_MCU:
			Frame = 1;
			State = Check_Sum;
			break;
		case Velocity_Set:
			Frame = 2;
			State = Data_Frame;
			break;
		case Position_Set:
			Frame = 2;
			State = Data_Frame;
			break;
		case Goal_1_Set:
			Frame = 2;
			State = Data_Frame;
			break;
		case Goal_N_Set:
			Frame = 3;
			State = N_Station;
			break;
		case Go_to_Goal:
			Frame = 1;
			State = Check_Sum;
			break;
		case Station_Request:
			Frame = 1;
			State = Check_Sum;
			break;
		case Position_Request:
			Frame = 1;
			State = Check_Sum;
			break;
		case Velocity_Request:
			Frame = 1;
			State = Check_Sum;
			break;
		case Gripper_On:
			Frame = 1;
			State = Check_Sum;
			break;
		case Gripper_Off:
			Frame = 1;
			State = Check_Sum;
			break;
		case Home_Set:
			Frame = 1;
			State = Check_Sum;
			break;
		default:
			State = Start_Mode;
			Mode = 144;
			Frame = 0;
			Data = 0;
			Sum = 0;
			N = 0;
			len = 0;
			break;
		break;
		}
		break;
	case N_Station:
		N = dataIn;
		State = Data_Frame;
		break;
	case Data_Frame:
		Data = dataIn;
		switch (Mode)
		{
		case Test_Command:
			State = Check_Sum;
			break;
		case Velocity_Set:
			State = Check_Sum;
			break;
		case Position_Set:
			State = Check_Sum;
			break;
		case Goal_1_Set:
			State = Check_Sum;
			break;
		case Goal_N_Set:
			N -= 1;
			if (N == 0)
			{
				State = Check_Sum;
			}
			break;
		break;
		}

		break;
	case Check_Sum:
		Sum = dataIn;
		switch (Frame)
		{
		case 1:
			if (Sum == (uint8_t)~Mode)
			{
				UART_Do_Command();
			}
			else
			{
				State = Start_Mode;
				Mode = 144;
				Frame = 0;
				Data = 0;
				Sum = 0;
				N = 0;
				len = 0;
			}
			break;
		case 2:
			if (Sum == (uint8_t)~(Mode+Data))
			{
				UART_Do_Command();
			}
			else
			{
				State = Start_Mode;
				Mode = 144;
				Frame = 0;
				Data = 0;
				Sum = 0;
				N = 0;
				len = 0;
			}
			break;
		case 3:

			if (Sum == (uint8_t)~(Mode))
			{
				UART_Do_Command();
			}
			else
			{
				State = Start_Mode;
				Mode = 144;
				Frame = 0;
				Data = 0;
				Sum = 0;
				N = 0;
				len = 0;
			}
			break;
		break;
		}

		State = Start_Mode;
		break;

	break;
	}

}
void UART_Do_Command()
{
	if (Frame == 1)
	{
		uint8_t Test[] = {0, 0};
		Test[0] = Mode;
		Test[1] = Sum;
		UARTTxWrite(&UART2, Test, len);
		len = 0;
	}
	else if (Frame == 2)
	{
		uint8_t Test[] = {0, 0, 0};
		Test[0] = Mode;
		Test[1] = Data;
		Test[2] = Sum;
		UARTTxWrite(&UART2, Test, len);
		len = 0;
	}
	else if (Frame == 3)
	{


	}

	switch (Mode)
	{
	case Test_Command:
		break;
	case Connect_MCU:
		break;
	case Disconnect_MCU:
		break;
	case Velocity_Set:
		break;
	case Position_Set:
		break;
	case Goal_1_Set:
		break;
	case Goal_N_Set:
		break;
	case Go_to_Goal:
		break;
	case Station_Request:
		break;
	case Position_Request:
		break;
	case Velocity_Request:
		break;
	case Gripper_On:
		break;
	case Gripper_Off:
		break;
	case Home_Set:
		break;
	}




}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}
uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
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
