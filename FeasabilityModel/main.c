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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */


//USART VARIABLES
uint8_t txd_msg_buffer[128] = {0};
uint8_t tx_print[2];
volatile uint8_t rcv_intpt_flag = 00;

volatile int curr_zone = 0;
volatile int curr_pwm = 0;
volatile uint16_t curr_hcsr04 = 50;

//BCD DISPLAY
uint8_t digit_a = 0;
uint8_t digit_b = 0;

//RPM Sensor
volatile uint32_t rpm = 0;
volatile uint32_t rpm_tick_count = 0;
volatile uint32_t rpm_time_out = 0;

//SETUP MODE
int connect_1, pwm_1, connect_2, pwm_2, connect_3, pwm_3, connect_4, pwm_4;
int curr_wctime, inlet_wcstart, inlet_wcstop, zone1_wcstart, zone1_wcstop, zone2_wcstart, zone2_wcstop, zone3_wcstart, zone3_wcstop ;
volatile uint8_t pb_press_flag = 0;

//ADC INPUT
uint8_t ADC_CH9;

//WALL CLK TEST
volatile uint8_t clock_hours = 0;
volatile uint8_t clock_mins = 0;
volatile uint8_t clock_secs = 0;
volatile int hour_flg = 0;

//HCSR04
volatile uint16_t hcsr04_Rx_flag = 0; // reset the interrupt flag for the HCSR04
volatile uint16_t time_diff = 0;
volatile uint16_t distance = 0;
volatile uint16_t first_edge = 0;	 // set it back to false
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;
volatile uint16_t res_empty_flg = 0;
//min, max distances
const uint16_t max_dist = 30;
const uint16_t min_dist = 2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ADC_Select_CH(int CH)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	switch(CH)
	{

	case 2:
		sConfig.Channel = ADC_CHANNEL_2;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 3:
		sConfig.Channel = ADC_CHANNEL_3;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 5:
		sConfig.Channel = ADC_CHANNEL_5;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	case 9:
		sConfig.Channel = ADC_CHANNEL_9;
		sConfig.Rank = 1;
		if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
		break;
	default:
		break;
	}
}

void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B)
{
	uint8_t DIGITA_VAL = 0x0F & DIGIT_A; //mask off higher4 bits
	int Abit0 = (DIGITA_VAL ) & 1;  	// extract Abit0 of the 4-bit value
	int Abit1 = (DIGITA_VAL >> 1) & 1;  // extract Abit1 of the 4-bit value
	int Abit2 = (DIGITA_VAL >> 2) & 1;  // extract Abit2 of the 4-bit value
	int Abit3 = (DIGITA_VAL >> 3) & 1;  // extract Abit3 of the 4-bit value

	uint8_t DIGITB_VAL = 0x0F & DIGIT_B; //mask off higher4 bits
	int Bbit0 = (DIGITB_VAL ) & 1;  	// extract Bbit0 of the 4-bit value
	int Bbit1 = (DIGITB_VAL >> 1) & 1;  // extract Bbit1 of the 4-bit value
	int Bbit2 = (DIGITB_VAL >> 2) & 1;  // extract Bbit2 of the 4-bit value
	int Bbit3 = (DIGITB_VAL >> 3) & 1;  // extract Bbit3 of the 4-bit value

	if (Abit0 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin, GPIO_PIN_SET);

	}
	if (Abit1 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A1_Pin, GPIO_PIN_SET);

	}
	if (Abit2 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A2_Pin, GPIO_PIN_SET);

	}
	if (Abit3 == (0))
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOB, DIGIT_A3_Pin, GPIO_PIN_SET);

	}


	if (Bbit0 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin, GPIO_PIN_SET);

	}
	if (Bbit1 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B1_Pin, GPIO_PIN_SET);

	}
	if (Bbit2 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B2_Pin, GPIO_PIN_SET);

	}
	if (Bbit3 == (0))
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, DIGIT_B3_Pin, GPIO_PIN_SET);

	}
}

int pwm_zoom(int input) {
	if (input == 1) {
		return 450 + (550*0.7);
	}
	else if (input == 2) {
		return 450 + (550*0.85);
	}
	else if (input == 3) {
		return 450 + (550*0.9);
	}
	else {
		return 1200;
	}
}

void get_rpm(){
	rpm = (rpm_tick_count*60)/300;
	rpm = rpm_tick_count/20;
}

void get_distance(){

	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2); // stop timer 5
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_CC2);
	hcsr04_Rx_flag = 0; // reset the interrupt flag for the HCSR04

	/* flags set by interrupt routine */
	first_edge = 0;
	time_edge1 = 0;
	time_edge2 = 0;

	/* calculations from flags */
	time_diff = 0;
	distance = 0;

	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // start timer 5 again

	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
	for (int j = 0; j != 15; j++) {}; // 11 micro seconds
	HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);

	while(hcsr04_Rx_flag == 0){};

	HAL_TIM_IC_Stop_IT(&htim5, TIM_CHANNEL_2); // stop timer 5 again

	/* enter below your difference in time between the edges here*/
	if(time_edge2 < time_edge1){
		time_edge1 = 65535-time_edge1;
		time_diff = time_edge2 + time_edge1;
	}
	else{
		time_diff = time_edge2 - time_edge1;
	}

	/* enter below your distance calculation here that uses HCSR04 datasheet information */
	distance = time_diff/58;

	if(distance <= min_dist){
		curr_hcsr04 = 99;
	}
	else if (distance > max_dist){
		curr_hcsr04 = 0;
		res_empty_flg = 1;
	}
	else{
		curr_hcsr04 = 100 - ( ((distance-min_dist) * 100) / (max_dist-min_dist));
	}

}

void set_led(int zone){
	if(zone == 1){
		HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET); // RED PIN
	}
	else if(zone == 2){
		HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_SET); //GREEN PIN
	}
	else if(zone == 3){
		HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_SET); //BLUE PIN
	}
	else{
		HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);	// reset RED PIN
		HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_RESET);	// reset GRN PIN
		HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_RESET);	// reset BLU PIN
	}
}


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
	MX_USART2_UART_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM5_Init();
	MX_USART6_UART_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	/* SETUP MODE */

	sprintf((char*)txd_msg_buffer, "\r\n\tSETUP MODE");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	/*Initialize DC Motor Variables, PUMP MOTOR OFF*/
	HAL_TIM_Base_Init(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	TIM3->CCR1 = 0;
	TIM3->CCR3 = 0;
	HAL_Delay(500);


	/* GET PIPELINE INFO */

	/* FIRST ZONE CHOICE */
	sprintf((char*)txd_msg_buffer, "\r\nPIPELINE (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intpt_flag == (00)) {};
	connect_1 = atoi((char*)&tx_print);
	sprintf((char*)txd_msg_buffer, "\r\nRECEIVED VALUE : %d", connect_1);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPump PMW (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	pwm_1 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* SECOND ZONE CHOICE */
	sprintf((char*)txd_msg_buffer, "\r\nPIPELINE (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	connect_2 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPump PMW (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	pwm_2 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* THIRD ZONE CHOICE */
	sprintf((char*)txd_msg_buffer, "\r\nPIPELINE (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	connect_3 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPump PMW (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	pwm_3 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* FOURTH ZONE CHOICE */
	sprintf((char*)txd_msg_buffer, "\r\nPIPELINE (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	connect_4 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPump PWM (options: 0 to 3): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 1);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	pwm_4 = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;


	/* GET PIPELINE FIRST/LAST HOUR INFO */

	/* CURRENT WALL CLK TIME */
	curr_wctime = 0;

	/* INLET WALL CLK TIME */
	sprintf((char*)txd_msg_buffer, "\r\nPipeline 0 Pump FIRST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	inlet_wcstart = (atoi((char*)&tx_print));
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 0 Pump LAST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	inlet_wcstop = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* ZONE 1 WALL CLK TIME */
	sprintf((char*)txd_msg_buffer, "\r\nPipeline 1 Pump FIRST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone1_wcstart = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 1 Pump LAST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone1_wcstop = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* ZONE 2 WALL CLK TIME */
	sprintf((char*)txd_msg_buffer, "\r\nPipeline 2 Pump FIRST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone2_wcstart = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 2 Pump LAST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone2_wcstop = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* ZONE 3 WALL CLK TIME */
	sprintf((char*)txd_msg_buffer, "\r\nPipeline 3 Pump FIRST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone3_wcstart = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 3 Pump LAST HOUR (options: 00 to 23): ");
	HAL_UART_Receive_IT(&huart6, tx_print, 2);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	while (rcv_intpt_flag == (00)) {};
	zone3_wcstop = atoi((char*)&tx_print);
	rcv_intpt_flag = 00;

	/* PRINT OUT SET UP PARAMETERS */
	sprintf((char*)txd_msg_buffer, "\r\n\tPrinting SETUP Parameters\n\n");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\nCURRENT WALL CLOCK HOUR %d\n", curr_wctime);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\n\nPIPELINE: %d\t Pump PMW: %d\t Pump FIRST HOUR: %d\t Pump LAST HOUR: %d\n", connect_1, pwm_1, inlet_wcstart, inlet_wcstop);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\n\nPIPELINE: %d\t Pump PMW: %d\t Pump FIRST HOUR: %d\t Pump LAST HOUR: %d\n", connect_2, pwm_2, zone1_wcstart, zone1_wcstop);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\n\nPIPELINE: %d\t Pump PMW: %d\t Pump FIRST HOUR: %d\t Pump LAST HOUR: %d\n", connect_3, pwm_3, zone2_wcstart, zone2_wcstop);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\n\nPIPELINE: %d\t Pump PMW: %d\t Pump FIRST HOUR: %d\t Pump LAST HOUR: %d\n", connect_4, pwm_4, zone3_wcstart, zone3_wcstop);
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	/* FLASH LD2 */
	sprintf((char*)txd_msg_buffer, "\r\nSETUP is done. Press blue button for RUN MODE ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	while (pb_press_flag == 0)
	{
		HAL_GPIO_WritePin(GPIOA, LED_Mode_Pin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOA, LED_Mode_Pin, GPIO_PIN_RESET);
		HAL_Delay(250);
	}
	HAL_GPIO_WritePin(GPIOA, LED_Mode_Pin, GPIO_PIN_SET);

	/* INITIALIZE WALL CLK */
	clock_hours = 0;
	clock_mins = 0;
	clock_secs = 0;

	sprintf((char*)txd_msg_buffer, "\r\n\tRUN MODE: ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

	sprintf((char*)txd_msg_buffer, "\r\nCLOCK : PIPE : PWM : RPM : DEPTH : ");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	sprintf((char*)txd_msg_buffer, "\r\n-----------------------------------------------");
	HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	HAL_TIM_Base_Init(&htim4);
	HAL_TIM_Base_Start_IT(&htim4);


	/* INITIALIZE SERVO MOTOR VARIABLES */
	int inlet_servo = 500;
	int zone1_servo = 750;
	int zone2_servo = 1000;
	int zone3_servo = 1250;

	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	TIM2->PSC = 16-1;
	TIM2->ARR = 20000-1;
	TIM2->CCR1 = inlet_servo;


	/* INITIALIZE HCSR04 VARIABLES */;
	hcsr04_Rx_flag = 0;
	first_edge = 0;
	time_edge1 = 0;
	time_edge2 = 0;
	time_diff = 0;
	distance = 0;
	HAL_TIM_Base_Init(&htim5); // start up timer 5 again

	DIGITS_Display( curr_hcsr04/10, curr_hcsr04%10);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (clock_hours < 24 && !res_empty_flg)
	{
		/* --------------------- Filling up the Inlet -------------------- */
		while ( inlet_wcstart <= clock_hours && clock_hours < inlet_wcstop ) {
			curr_zone = connect_1;

			if (res_empty_flg) {
				break;
			}
			/* ----- 1. LED TURNS PURPLE */
			HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET); // RED PIN
			HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_SET); // BLU PIN

			/* Servo turns to 0 position */
			TIM2->CCR1 = inlet_servo;

			//			sprintf((char*)txd_msg_buffer, "\r\n%d ", clock_mins);
			//			HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

			if(pwm_1 == 0){
				ADC_Select_CH(9);
				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 1000);
				ADC_CH9 = HAL_ADC_GetValue(&hadc1);
				curr_pwm = ADC_CH9;
				HAL_ADC_Stop(&hadc1);
				TIM3->CCR1 = 0;
				TIM3->CCR3 = (curr_pwm*100);
				HAL_Delay(1000);
				/*PRINT OUT THE 8 bit digitized value of the external analog voltage*/
				//				  sprintf((char*)txd_msg_buffer, "DIGITALIZED ANALOG VALUE FOR CH9: %d \r \n", ADC_CH9);
				//				  HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			}
			else{
				curr_pwm = pwm_zoom(pwm_1);
				/* DC Motor goes brrr */
				TIM3->CCR1 = 0;
				TIM3->CCR3 = curr_pwm;
				HAL_Delay(1000);

			}

			get_distance();
			get_rpm();
			DIGITS_Display( curr_hcsr04/10, curr_hcsr04%10);

		}

		/* Reset the LEDs */
		HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);	// reset RED PIN
		HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_RESET);	// reset BLU PIN

		/* The DC motor turns off*/
		TIM3->CCR1 = 0;
		TIM3->CCR3 = 0;
		HAL_Delay(1000);


		//		break;

		/* --------- ZONE 1 --------- */
		while ( zone1_wcstart <= clock_hours && clock_hours < zone1_wcstop ) {
			curr_zone = connect_2;
			curr_pwm = pwm_zoom(pwm_2);

			if (res_empty_flg) {
				break;
			}

			/* LED TURNS RED */
			set_led(curr_zone);

			/* Servo turns to Zone 1 position */
			TIM2->CCR1 = zone1_servo;

			/* DC Motor switches directions for the rest of the zones */
			TIM3->CCR1 = curr_pwm;
			TIM3->CCR3 = 0;
			HAL_Delay(1000);

			get_rpm();
			get_distance();
			DIGITS_Display( curr_hcsr04/10, curr_hcsr04%10);

		}

		/* Reset the LED */
		set_led(0);
		/* The DC motor turns off*/
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;

		/* --------- ZONE 2 --------- */
		while ( zone2_wcstart <= clock_hours && clock_hours < zone2_wcstop ) {
			curr_zone = connect_3;
			curr_pwm = pwm_zoom(pwm_3);
			if (res_empty_flg) {
				break;
			}

			/* LED TURNS GREEN */
			set_led(curr_zone);

			/* Servo turns to Zone 2 position */
			TIM2->CCR1 = zone2_servo;

			/* DC Motor go brrr*/
			TIM3->CCR1 = curr_pwm;
			TIM3->CCR3 = 0;
			HAL_Delay(1000);

			get_rpm();
			get_distance();
			DIGITS_Display( curr_hcsr04/10, curr_hcsr04%10);

		}

		/* Reset the LED */
		set_led(0);

		/* The DC motor turns off*/
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;
		HAL_Delay(1000);

		/* --------- ZONE 3 --------- */
		while ( zone3_wcstart <= clock_hours && clock_hours <= zone3_wcstop) {
			curr_zone = connect_4;
			curr_pwm = pwm_zoom(pwm_4);

			if (res_empty_flg) {
				break;
			}
			/* LED TURNS BLUE */
			set_led(curr_zone);

			/* Servo turns to Zone 3 position */
			TIM2->CCR1 = zone3_servo;

			/* DC Motor go brrr*/
			TIM3->CCR1 = curr_pwm;
			TIM3->CCR3 = 0;
			HAL_Delay(1000);

			get_rpm();
			get_distance();
			DIGITS_Display( curr_hcsr04/10, curr_hcsr04%10);

		}

		/* Reset the LED */
		set_led(0);

		/* The DC motor turns off*/
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;
		HAL_Delay(1000);

		/* USER CODE END WHILE */
	}

	/* USER CODE BEGIN 3 */
	HAL_TIM_Base_Stop_IT(&htim4);

	/* Empty */
	if (res_empty_flg) {
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;

		sprintf((char*)txd_msg_buffer, "\r\n-------------------------------------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-                                   -");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n---------- RESERVOIR EMPTY ----------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-                                   -");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-------------------------------------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);

		/* Blink LED White*/
		while (1){

			HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_SET);	// set RED PIN
			HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_SET);	// set GRN PIN
			HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_SET);	// set BLU PIN
			HAL_Delay(500);

			HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);	// reset RED PIN
			HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_RESET);	// reset GRN PIN
			HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_RESET);	// reset BLU PIN
			HAL_Delay(500);
		}
	}

	/* DONE */
	if (clock_hours <= 24 && res_empty_flg) {
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;

		TIM2->CCR1 = inlet_servo;
		//	  HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOA, RED_Pin, GPIO_PIN_RESET);	// reset RED PIN
		HAL_GPIO_WritePin(GPIOA, GRN_Pin, GPIO_PIN_RESET);	// reset GRN PIN
		HAL_GPIO_WritePin(GPIOA, BLU_Pin, GPIO_PIN_RESET);	// reset BLU PIN


		sprintf((char*)txd_msg_buffer, "\r\n-------------------------------------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-                                   -");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-------- IRRIGATION COMPLETE --------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-                                   -");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
		sprintf((char*)txd_msg_buffer, "\r\n-------------------------------------");
		HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
	}


} // end of main

/* USER CODE END 3 */

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 20000-1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 500-1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */
	HAL_TIM_MspPostInit(&htim2);

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2000-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	sConfigOC.Pulse = 1200-1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.Pulse = 0;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
	htim4.Init.Prescaler = 53;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 1000-1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
	HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);  // Set priority
	HAL_NVIC_EnableIRQ(TIM4_IRQn);  // Enable interrupt
	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 16-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 65536-1;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */
	HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);  // Set priority
	HAL_NVIC_EnableIRQ(TIM5_IRQn);  // Enable interrupt
	/* USER CODE END TIM5_Init 2 */

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
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
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
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, HCSR04_TRIG_Pin|LED_Mode_Pin|BLU_Pin|GRN_Pin
			|RED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin|DIGIT_B3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|DIGIT_A3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB_Mode_Pin */
	GPIO_InitStruct.Pin = PB_Mode_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PB_Mode_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : HCSR04_TRIG_Pin LED_Mode_Pin BLU_Pin GRN_Pin
                           RED_Pin */
	GPIO_InitStruct.Pin = HCSR04_TRIG_Pin|LED_Mode_Pin|BLU_Pin|GRN_Pin
			|RED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : RPM_Tick_Pin */
	GPIO_InitStruct.Pin = RPM_Tick_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(RPM_Tick_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : DIGIT_B0_Pin DIGIT_B1_Pin DIGIT_B2_Pin DIGIT_B3_Pin */
	GPIO_InitStruct.Pin = DIGIT_B0_Pin|DIGIT_B1_Pin|DIGIT_B2_Pin|DIGIT_B3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : DIGIT_A0_Pin DIGIT_A1_Pin DIGIT_A2_Pin DIGIT_A3_Pin */
	GPIO_InitStruct.Pin = DIGIT_A0_Pin|DIGIT_A1_Pin|DIGIT_A2_Pin|DIGIT_A3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI2_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* EXTI interrupt init*/

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART6) {
		HAL_UART_Transmit(&huart6, tx_print, 2, 100);
		rcv_intpt_flag = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PB_Mode_Pin) {
		pb_press_flag = 1;
	}

	if (GPIO_Pin == RPM_Tick_Pin) {
		rpm_tick_count += 1;

	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* only interrupts from TIM5 Channel 1 are processed */
	if ((htim->Instance == TIM5) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)){
		if (first_edge == 0) {
			/* first_edge = 1 when first edge occurs */
			time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			first_edge = 1;
		}
		else {
			time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			__HAL_TIM_SET_COUNTER(htim, 0); //reset timer counter
			hcsr04_Rx_flag = 1;
			/* flag is only set once both edges have been captured */
		}
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if((htim->Instance == TIM4))
	{
		//CALCULATRE RPM OVER '
		hour_flg = 0; // screen updates occur hourly on the half-hour
		clock_secs += 1;
		if(clock_secs == 60)
		{
			clock_secs = 0;
			clock_mins += 1;

			if(clock_mins == 60)
			{
				clock_mins = 0;
				clock_hours += 1;
				hour_flg = 1; // screen updates occur hourly on the hour

				// "\r\nCLOCK : PIPE : PWM : RPM : DEPTH :
				sprintf((char*)txd_msg_buffer, "\r\n%d   :   %d   :   %d   :   %d   :   %d", clock_hours, curr_zone, curr_pwm, rpm, curr_hcsr04 );
				rpm = 0;
				rpm_tick_count = 0;

				HAL_UART_Transmit(&huart6, txd_msg_buffer, strlen((char*)txd_msg_buffer), 100);
			}
		}
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
