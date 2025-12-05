/* USER CODE BEGIN Header */
/**
  ***************************************************************************
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include <stdbool.h>

#include <stdio.h>
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LENGTH_ARRAY_SIZE 3  // Define the size of the length array
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float p1 = 200;//behind left
float p2 = 220;//front left
float p3 = 250;//front right
float p4 = 250;//behind right
uint8_t rxdata1[12]={0,0,0,0,0,0,0,0,0,0,0,0};
float setpoint = 0.0; // Desired position
float pv = 0.0; // Process Variable (current position)
float dt = 0.2; // Time difference
float control = 0.0;
int c = -1; //count
bool flag = true;
bool flag2 = true;
bool flag3 = true;
bool no_color = false;

PID myPID;

uint32_t val= 0;
float Length = 50;
float Length2 = 0;

float lengths[LENGTH_ARRAY_SIZE] = {100, 100, 100};  // Initialize all elements to 100 cm
int index = 0;                     // Current index in the lengths array
int count_under = 0;            // Counter for lengths under 90 cm
bool assign_pass_dog = false;
bool pass_dog = false;
bool pass_obs = false;
bool arrow_jurdge = true;
bool traffic_light = false;
bool trigger = false;
bool greens = true;
bool dog_stop = false;
int countU = 0, countL = 0, countR = 0;
int recentU[3] = {0}, recentL[3] = {0}, recentR[3] = {0};  // Arrays to keep track of recent occurrences
int indexU = 0, indexL = 0, indexR = 0;  // Index for updating the recent occurrences

uint32_t aTxBuffer[30];
uint8_t  TIM3_IC_STA;
uint16_t TIM3_IC_VAL;
uint8_t  TIM2_IC_STA;
uint16_t TIM2_IC_VAL;

uint32_t delayDuration = 1000;  // Delay duration in milliseconds
uint32_t lastTick = 0;          // Last tick time
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
  HAL_Delay(1000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  PID_Init(&myPID, 4,0.4 , 0.8);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);


  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);

 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);//...............巡线
 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p4);

  HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);   // ?????启TIM3的捕�????????? ? �?????????1，并且开启捕获中 ?????
  __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);   //使能更新中断
  HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_1);   // ?????启TIM3的捕�????????? ? �?????????1，并且开启捕获中 ?????
  __HAL_TIM_ENABLE_IT(&htim2,TIM_IT_UPDATE);
  HAL_Delay(500);
  /* USER CODE END 2 */

  /* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (trigger) {
			Trig_Capture();
			if (TIM3_IC_STA & 0x80) {  // Successfully captured a pulse
				val = TIM3_IC_STA & 0x3F;   // Get the overflow count
				val *= 65536;       // Convert the overflow times to timer ticks
				val += TIM3_IC_VAL;         // Add the last captured timer value

				Length = val * 342.62 * 100 / 2000000; // Calculate the length in cm

				// Store the length in the array
				lengths[index] = Length;
				index = (index + 1) % LENGTH_ARRAY_SIZE; // Move to the next index, wrapping around if necessary

				// Check if the array is filled and act if necessary
				count_under = 0;

				// Count how many lengths are under 90 cm
				for (int i = 0; i < LENGTH_ARRAY_SIZE; i++) {
					if (lengths[i] < 40) {
						count_under++;
					}
				}
			}
		}
		if (!pass_dog) {
			// If 5 or more lengths are under 90 cm and the jump flag is not set, perform the action once
			if (count_under >= 2) {
				// Perform the desired action here
				//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
				//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
				//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
				//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);

				flag2 = false;
				assign_pass_dog = true;

				//jump = true; // Set jump to true to prevent re-triggering
				//HAL_Delay(2000);
			} else {
				flag2 = true;
				if (assign_pass_dog) {
					pass_dog = true;
					dog_stop = true;
				}

			}
		}
		// Reset the capture status
		TIM3_IC_STA = 0;

		// Prepare and send the UART message
		char aTxBuffer[50];
		sprintf(aTxBuffer,
				"UltraSonic1 Length: %f cm\r\n Count_under: %d \r\n pass_dog: %d",
				Length, count_under, pass_dog);
		HAL_UART_Transmit(&huart2, (uint8_t*) aTxBuffer, strlen(aTxBuffer),200);
		sprintf(aTxBuffer,"\r\n flag: %d  \r\n flag2:  %d \r\n flag3: %d \r\n",flag,flag2,flag3);
		HAL_UART_Transmit(&huart2, (uint8_t*) aTxBuffer, strlen(aTxBuffer),200);
	//Trig2_Capture();
	//if(TIM2_IC_STA & 0x80)	//成功捕获 ?个脉 ?
	//{
	//val = TIM2_IC_STA & 0x3F;	//获取溢出次数
	//val *= 65536;		//获得溢出的时�????????? ??
	//val += TIM2_IC_VAL;	//加上 ?后一次取得的 ?
	//Length2 = val * 342.62*100/2000000;

	//TIM2_IC_STA = 0;
	//sprintf(aTxBuffer,"UltraSonic2 Length: %f cm\r\n", Length2);
	//HAL_UART_Transmit(&huart2,aTxBuffer,strlen((const char*)aTxBuffer),200);
	//}

	if (p1 <= 205 && p1 >= 150) {
		p1 -= control;
	}
	if (p1 > 205) {
		p1 = 205;
	} else if (p1 < 150) {
		p1 = 150;
	}
	if (p2 <= 225 && p2 >= 180) {
		p2 -= control;
	}
	if (p2 > 225) {
		p2 = 225;
	} else if (p2 < 180) {
		p2 = 180;
	}
	if (p3 <= 300 && p3 >= 200) {
		p3 += 1.3 * control;
	}
	//if (p3 < 200) {
	//	p3 = 200;
	//} else if (p3 > 300) {
	//	p3 = 300;
	//}
	if (p4 <= 300 && p4 >= 200) {
		p4 += 1.3 * control;
	}
	if (p4 > 300) {
		p4 = 300;
	} else if (p4 < 200) {
		p4 = 200;
	}
	sprintf(aTxBuffer, "p1: %f p2: %f p3: %f p4:%f \r\n", p1, p2, p3, p4);
	HAL_UART_Transmit(&huart2, aTxBuffer, strlen((const char*) aTxBuffer), 200);
	if (flag && flag2 && flag3) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, p1);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, p2);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, p3);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, p4);
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	}

	HAL_UART_Receive(&huart3, rxdata1, 12, 0xFFFF);
	// Transmit received data back (optional, if needed)
	HAL_UART_Transmit(&huart3, rxdata1, 12, 0xFFFF);
	for (int i = 0; i < 12; i++) {
		if (rxdata1[i] == 97) {
			c = i;
			break;
		}
	}
	if (c != -1 && rxdata1[c % 12] == 97) {
		if (rxdata1[(c + 1) % 12] == 97) {
			if (rxdata1[(c + 2) % 12] == 97) {
				pv = -((rxdata1[(c + 3) % 12] - 48)
						+ (0.1 * (rxdata1[(c + 5) % 12] - 48)));
				//HAL_UART_Transmit(&huart3, rxdata2, 1, 0xFFFF);
			} else if (rxdata1[(c + 2) % 12] == 45) {
				pv = ((rxdata1[(c + 3) % 12] - 48)
						+ (0.1 * (rxdata1[(c + 5) % 12] - 48)));
			} else {
				pv = -((rxdata1[(c + 2) % 12] - 48) * 10
						+ (rxdata1[(c + 3) % 12] - 48)
						+ ((rxdata1[(c + 5) % 12] - 48) * 0.1));
			}
		} else {
			pv = ((rxdata1[(c + 2) % 12] - 48) * 10
					+ (rxdata1[(c + 3) % 12] - 48)
					+ (rxdata1[(c + 5) % 12] - 48) * 0.1);
		}
		control = PID_Update(&myPID, setpoint, pv, dt);
	}

	// turn the car for a certain angle so that it faces the arrow/traffic light
	// change 8 logic prots in spin manner for l298n
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);

	//set pwm for 4 wheels
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
	if (!arrow_jurdge) {
		if (rxdata1[(c + 6) % 12] == 98) {
			if (rxdata1[(c + 7) % 12] == 85) {  // Check for UP
				recentU[indexU] = 1;  // Mark the occurrence
				indexU = (indexU + 1) % 5; // Update the index in a circular manner
				countU = recentU[0] + recentU[1] + recentU[2]+recentU[3]+recentU[4];  // Sum up the occurrences
				if (countU >= 2) {
					// Execute action for UP direction
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 270); //...........直线
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 280);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 340);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 350);
					HAL_Delay(1200);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 270);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 280);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 310);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 320);
					HAL_Delay(5000);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
					//HAL_Delay(10000);
					//  flag3 = false;
					// Reset tracking after action is taken
					recentU[0] = recentU[1] = recentU[2] = recentU[3] = recentU[4] = 0;
					countU = 0;
					greens = true;
					arrow_jurdge= true;
				}
			} else if (rxdata1[(c + 7) % 12] == 76) {  // Check for LEFT
				recentL[indexL] = 1;
				indexL = (indexL + 1) % 5;
				countL = recentL[0] + recentL[1] + recentL[2] + recentL[3]+ recentL[4];
				if (countL >= 2) {
					// Execute action for LEFT direction
					//..............................................................向左转
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);// turn left
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 260);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 390);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 400);
					HAL_Delay(1500);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 275);//go in circle
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 285);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 230);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 240);
					HAL_Delay(3800);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 240);//go back to road
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 250);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 390);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 400);
					HAL_Delay(1300);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,0);//stay
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4,0);
					// HAL_Delay(10000);
					//flag3 = false;
					// Reset tracking after action is taken
					recentL[0] = recentL[1] = recentL[2] = recentL[3] =recentL[4] = 0;
					countL = 0;
					greens = true;
					arrow_jurdge= true;
				}
			} else if (rxdata1[(c + 7) % 12] == 82) {  // Check for right
				recentR[indexR] = 1;
				indexR = (indexR + 1) % 5;
				countR = recentR[0] + recentR[1] + recentR[2] + recentR[3]+ recentR[4];
				if (countR >= 2) {
					// Execute action for right direction
					//..........................................................向右转
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);//go to the center
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 260);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 330);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 340);
					HAL_Delay(1200);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 280);//turn right
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 290);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 260);
					HAL_Delay(1200);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 220);//go in circle
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 230);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 355);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 365);
					HAL_Delay(3800);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 340);//go back to road
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 350);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 280);
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 290);
					HAL_Delay(1300);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);	//stay
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
					//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
					//HAL_Delay(10000);
					//flag3 = false;
					// Reset tracking after action is taken
					recentR[0] = recentR[1] = recentR[2] = recentR[3] =recentR[4] = 0;
					countR = 0;
					greens = true;
					arrow_jurdge= true;
				}
			} else {
				// Reset the entry for other directions if needed
				recentU[indexU] = 0;
				recentL[indexL] = 0;
				recentR[indexR] = 0;
			}
			//if other two directions...
		}
	}
	sprintf(aTxBuffer, "countL: %d\r\n", countL);
	HAL_UART_Transmit(&huart2, aTxBuffer, strlen((const char*) aTxBuffer), 200);
	if (greens) {
			if (!traffic_light) {
				if (rxdata1[(c + 8) % 12] == 99) {
					if (rxdata1[(c + 9) % 12] == 82) {
						flag = false;
					}
				}
				// else if(rxdata1[(c+9)%10] == 71){
				//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
				//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
				//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
				//	 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
				//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
				//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
				//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
				//	 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);

				//	 p1 = 1000;
				//	 p2 = 0;
				//	 p3 = 1000;
				//	 p4 = 0;
				//	 HAL_Delay(1000);
				//	 flag = true;

				if (rxdata1[(c + 9) % 12] == 71) {
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
					//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
					//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
					flag = true;
					no_color = 1;
					trigger = true;
					traffic_light = true;
				}
			}
		}
	if(dog_stop){
		if (!pass_obs) {
		if (Length <= 25) {

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);      // LEFT
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 350);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 380);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 410);
			HAL_Delay(1500);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);       // UP
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 270);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 280);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 320);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 330);
			HAL_Delay(1500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);       // right
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 300);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 310);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 355);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 365);
			HAL_Delay(1500);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);       // UP
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
			HAL_Delay(500);
			trigger = false;
			pass_obs = true;
		}
	}
	}
	sprintf(aTxBuffer, "flag: %d no_color = %d\r\n", flag, no_color);
	HAL_UART_Transmit(&huart2, aTxBuffer, strlen((const char*) aTxBuffer), 200);
	if(!trigger){
	if (rxdata1[(c + 10) % 12] == 100) {
		if (rxdata1[(c + 11) % 12] == 103) {
			//flag=false;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
			//open the gate for 2 sec
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
			HAL_Delay(2000);
		} else if (rxdata1[(c + 11) % 12] == 115) {
			 HAL_Delay(1000);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 1);
			HAL_Delay(5000);
			//flag = false;
		}
	}
}
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);     //  RIGHT
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 330);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 350);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 400);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 410);
	//HAL_Delay(1400);

	//if detect obstacles, stop for 1 sec
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 0);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 0);
	//HAL_Delay(1000);

	//Left shift for 1 sec
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, 1);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, 1);

	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 900);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 900);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 900);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 900);
	//HAL_Delay(1000);

	//Run forward, side ultra detect, if detect, run for ? sec
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 250);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 300);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 250);
	//__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 300);

}
}
//HAL_UART_Transmit(&huart2,rxdata1, 5, 0xffff);
/* USER CODE END WHILE */

/* USER CODE BEGIN 3 */
//}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */

void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 50;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void Trig_Capture(void) {
	uint32_t i;

	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_SET);
	for (i = 0; i < 72 * 40; i++)
		__NOP();                      //利用指令耗时计算延时
	HAL_GPIO_WritePin(Trig_GPIO_Port, Trig_Pin, GPIO_PIN_RESET);
}

void Trig2_Capture(void) {
	uint32_t i;

	HAL_GPIO_WritePin(Trig2_GPIO_Port, Trig2_Pin, GPIO_PIN_SET);
	for (i = 0; i < 72 * 40; i++)
		__NOP();                      //利用指令耗时计算延时
	HAL_GPIO_WritePin(Trig2_GPIO_Port, Trig2_Pin, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim_baseHandle) {
	if (tim_baseHandle->Instance == TIM3) {
		if ((TIM3_IC_STA & 0X80) == 0)		//还未捕获成功
				{
			if (TIM3_IC_STA & 0X40)		//捕获到一个下降沿
					{
				if ((TIM3_IC_STA & 0X3F) == 0X3F)	//高电平时间太长了
						{
					TIM3_IC_STA |= 0X80;	//标记成功捕获 ????? ?????
					TIM3_IC_VAL = 0XFFFF;	//
				}

				else
					TIM3_IC_STA++;			//否则标记溢出数加 ?????
			}
		}
	} else if (tim_baseHandle->Instance == TIM2) {
		if ((TIM2_IC_STA & 0X80) == 0)		//还未捕获成功
				{
			if (TIM2_IC_STA & 0X40)		//捕获到一个下降沿
					{
				if ((TIM2_IC_STA & 0X3F) == 0X3F)	//高电平时间太长了
						{
					TIM2_IC_STA |= 0X80;	//标记成功捕获 ????? ?????
					TIM2_IC_VAL = 0XFFFF;	//
				}

				else
					TIM2_IC_STA++;			//否则标记溢出数加 ?????
			}
		}
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *tim_baseHandle) {
	if (tim_baseHandle->Instance == TIM3) {
		if ((TIM3_IC_STA & 0x80) == 0)	//还未捕获成功
				{
			if (TIM3_IC_STA & 0x40)	//成功捕获到一个下降沿
					{
				TIM3_IC_STA |= 0X80;	//标记成功，捕获到 ?????次高电平完成
				TIM3_IC_VAL = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);	//获取当前捕获 ?????
				TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1);		//清除原来设置
				TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
						TIM_ICPOLARITY_RISING);	//捕获到下降沿后，将捕获复位到捕获上升 ?????
			}

			else							//捕获到一个上升沿
			{
				TIM3_IC_STA = 0;
				TIM3_IC_VAL = 0;
				TIM3_IC_STA |= 0x40;	//第六位标记为捕获到上升沿
				__HAL_TIM_DISABLE(&htim3);		//关闭定时 ?????
				__HAL_TIM_SET_COUNTER(&htim3, 0);//定时器初�????????? ? 设置�?????????0
				TIM_RESET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1);
				TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_1,
						TIM_ICPOLARITY_FALLING);	//捕获到上升沿之后，将捕获设置为下降沿
				__HAL_TIM_ENABLE(&htim3);
			}
		}
	} else if (tim_baseHandle->Instance == TIM2) {
		if ((TIM2_IC_STA & 0x80) == 0)	//还未捕获成功
				{
			if (TIM2_IC_STA & 0x40)	//成功捕获到一个下降沿
					{
				TIM2_IC_STA |= 0X80;	//标记成功，捕获到 ?????次高电平完成
				TIM2_IC_VAL = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);	//获取当前捕获 ?????
				TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1);		//清除原来设置
				TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
						TIM_ICPOLARITY_RISING);	//捕获到下降沿后，将捕获复位到捕获上升 ?????
			}

			else							//捕获到一个上升沿
			{
				TIM2_IC_STA = 0;
				TIM2_IC_VAL = 0;
				TIM2_IC_STA |= 0x40;	//第六位标记为捕获到上升沿
				__HAL_TIM_DISABLE(&htim2);		//关闭定时 ?????
				__HAL_TIM_SET_COUNTER(&htim2, 0);//定时器初�????????? ? 设置�?????????0
				TIM_RESET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1);
				TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1,
						TIM_ICPOLARITY_FALLING);	//捕获到上升沿之后，将捕获设置为下降沿
				__HAL_TIM_ENABLE(&htim2);
			}
		}
	}
}

void adjust_motor_control(float *p1, float *p2, float *p3, float *p4,
		float control) {
	char aTxBuffer[100];  // Increase buffer size if necessary

	// Adjust p1 based on control
	if (*p1 <= 205 && *p1 >= 150) {
		*p1 -= control;
	}
	if (*p1 > 205) {
		*p1 = 205;
	} else if (*p1 < 150) {
		*p1 = 150;
	}

	// Adjust p2 based on control
	if (*p2 <= 225 && *p2 >= 180) {
		*p2 -= control;
	}
	if (*p2 > 225) {
		*p2 = 225;
	} else if (*p2 < 180) {
		*p2 = 180;
	}

	// Adjust p3 based on control
	if (*p3 <= 300 && *p3 >= 200) {
		*p3 += 1.3 * control;
	}
	if (*p3 < 200) {
		*p3 = 200;
	} else if (*p3 > 300) {
		*p3 = 300;
	}

	// Adjust p4 based on control
	if (*p4 <= 300 && *p4 >= 200) {
		*p4 += 1.3 * control;
	}
	if (*p4 > 300) {
		*p4 = 300;
	} else if (*p4 < 200) {
		*p4 = 200;
	}

	// Format and send status message
	sprintf(aTxBuffer, "p1: %f p2: %f p3: %f p4: %f\r\n", *p1, *p2, *p3, *p4);
	HAL_UART_Transmit(&huart2, (uint8_t*) aTxBuffer, strlen(aTxBuffer), 200);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, *p1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, *p2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, *p3);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, *p4);
	HAL_UART_Receive(&huart3, rxdata1, 12, 0xFFFF);
	// Transmit received data back (optional, if needed)
	HAL_UART_Transmit(&huart3, rxdata1, 12, 0xFFFF);
	for (int i = 0; i < 12; i++) {
		if (rxdata1[i] == 97) {
			c = i;
			break;
		}
	}
	if (c != -1 && rxdata1[c % 12] == 97) {
		if (rxdata1[(c + 1) % 12] == 97) {
			if (rxdata1[(c + 2) % 12] == 97) {
				pv = -((rxdata1[(c + 3) % 12] - 48)
						+ (0.1 * (rxdata1[(c + 5) % 12] - 48)));
				//HAL_UART_Transmit(&huart3, rxdata2, 1, 0xFFFF);
			} else if (rxdata1[(c + 2) % 12] == 45) {
				pv = ((rxdata1[(c + 3) % 12] - 48)
						+ (0.1 * (rxdata1[(c + 5) % 12] - 48)));
			} else {
				pv = -((rxdata1[(c + 2) % 12] - 48) * 10
						+ (rxdata1[(c + 3) % 12] - 48)
						+ ((rxdata1[(c + 5) % 12] - 48) * 0.1));
			}
		} else {
			pv = ((rxdata1[(c + 2) % 12] - 48) * 10
					+ (rxdata1[(c + 3) % 12] - 48)
					+ (rxdata1[(c + 5) % 12] - 48) * 0.1);
		}
		control = PID_Update(&myPID, setpoint, pv, dt);
	}
}

// Add function to print messages
// UART Receive Complete Callback

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
