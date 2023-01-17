/* Autor: Leonhard Kirchberger
 * Date:  17.01.2023
 * Functions to control a robot hand with Motors and Sensors
 */

#include <hand_functions.h>

/////////////////////////// Variables /////////////////////////////

uint32_t ASensor[] = { A_SENSOR1, A_SENSOR2, A_SENSOR3, A_SENSOR4 };
uint32_t TSensor[] = { T_SENSOR1, T_SENSOR2, T_SENSOR3, T_SENSOR4 };

// Motor1 struct
const TIM_OC_InitTypeDef sConfigOC_Motor1 = { .OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH, .OCFastMode = TIM_OCFAST_DISABLE, .OCIdleState = TIM_OCIDLESTATE_RESET, .OCNIdleState = TIM_OCNIDLESTATE_RESET, };
MOTOR_T Motor1 = { .htim = &htim1, .Channel = TIM_CHANNEL_1, .sConfigOC_Motor = sConfigOC_Motor1, .M_GPIOx = GPIOA, .M_GPIO_Pin = MOTOR1_PH_Pin, };

// Motor2 struct
const TIM_OC_InitTypeDef sConfigOC_Motor2 = { .OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH, .OCFastMode = TIM_OCFAST_DISABLE, .OCIdleState = TIM_OCIDLESTATE_RESET, .OCNIdleState = TIM_OCNIDLESTATE_RESET, };
MOTOR_T Motor2 = { .htim = &htim1, .Channel = TIM_CHANNEL_3, .sConfigOC_Motor = sConfigOC_Motor2, .M_GPIOx = GPIOB, .M_GPIO_Pin = MOTOR3_PH_Pin, };

// Motor3 struct
const TIM_OC_InitTypeDef sConfigOC_Motor3 = { .OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH, .OCFastMode = TIM_OCFAST_DISABLE, .OCIdleState = TIM_OCIDLESTATE_RESET, .OCNIdleState = TIM_OCNIDLESTATE_RESET, };
MOTOR_T Motor3 = { .htim = &htim1, .Channel = TIM_CHANNEL_2, .sConfigOC_Motor = sConfigOC_Motor3, .M_GPIOx = GPIOB, .M_GPIO_Pin = MOTOR2_PH_Pin, };

// Motor4 struct
const TIM_OC_InitTypeDef sConfigOC_Motor4 = { .OCMode = TIM_OCMODE_PWM1, .Pulse = 0, .OCPolarity = TIM_OCPOLARITY_HIGH, .OCFastMode = TIM_OCFAST_DISABLE, .OCIdleState = TIM_OCIDLESTATE_RESET, .OCNIdleState = TIM_OCNIDLESTATE_RESET, };
MOTOR_T Motor4 = { .htim = &htim16, .Channel = TIM_CHANNEL_1, .sConfigOC_Motor = sConfigOC_Motor4, .M_GPIOx = GPIOB, .M_GPIO_Pin = MOTOR4_PH_Pin, };

////////////////////////// Callback functions ///////////////////////////

/* DMA Callback function					*
 * sets adcComplet to 1 when called			*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	adcComplete = 1;
	return;
}

/* Timer 6 Callback function				*
 * sets timestep to 1 when called			*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	timestep = 1;
	return;
}

/* UART Callback function						*
 * sends Echo									*
 * is called when 10 characters are received	*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	// Echo
	HAL_UART_Transmit(&huart2, rxBuffer, sizeof(rxBuffer), 0xFFFF);
	uint8_t backspace[] = "\n";
	HAL_UART_Transmit(&huart2, backspace, sizeof(backspace), 0xFFFF);

	// request new receive
	HAL_UART_Receive_IT(&huart2, rxBuffer, 10);

	// receive indicator to 1
	receive = 1;

	return;
}

//////////////////////////////// functions //////////////////////////////////

/* Initialization of Timers
 */
void hand_functions_init() {
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	return;
}

/* Every Motor is set to its pwm value and time is reduced by 1				*
 * if the time is 1 or 0 the Motor is stopped 								*
 * if the pwm value is negative the motor will spin the other direction 	*/
void SetMotors(int32_t pwm[], uint32_t time[]) {

	// Finger 1
	if (time[0] > 1) {
		time[0] -= 1;
		if (pwm[0] >= 0)
			Motor(Motor1, CLOCKWISE, (uint16_t) pwm[0]);
		else
			Motor(Motor1, CCLOCKWISE, (uint16_t) (-1 * pwm[0]));
	} else if (time[0] == 1) {
		Motor(Motor1, CLOCKWISE, 0);
		time[0] = 0;
	}

	// Finger 2
	if (time[1] > 1) {
		time[1] -= 1;
		if (pwm[1] >= 0)
			Motor(Motor2, CLOCKWISE, (uint16_t) pwm[1]);
		else
			Motor(Motor2, CCLOCKWISE, (uint16_t) (-1 * pwm[1]));
	} else if (time[1] == 1) {
		Motor(Motor2, CLOCKWISE, 0);
		time[1] = 0;
	}

	// Finger 3
	if (time[2] > 1) {
		time[2] -= 1;
		if (pwm[2] >= 0)
			Motor(Motor3, CLOCKWISE, (uint16_t) pwm[2]);
		else
			Motor(Motor3, CCLOCKWISE, (uint16_t) (-1 * pwm[2]));
	} else if (time[2] == 1) {
		Motor(Motor3, CLOCKWISE, 0);
		time[2] = 0;
	}

	// Finger 4
	if (time[3] > 1) {
		time[3] -= 1;
		if (pwm[3] >= 0)
			Motor(Motor4, CLOCKWISE, (uint16_t) pwm[3]);
		else
			Motor(Motor4, CCLOCKWISE, (uint16_t) (-1 * pwm[3]));
	} else if (time[3] == 1) {
		Motor(Motor4, CLOCKWISE, 0);
		time[3] = 0;
	}

	return;
}

/* Motor control for motor						*
 * Gets direction (1 or 0) and speed (pwm)		*
 */
void Motor(MOTOR_T motor, uint8_t dir, uint16_t speed) {

	if (speed > 0) {

		if (dir == CLOCKWISE) {

			HAL_GPIO_WritePin(motor.M_GPIOx, motor.M_GPIO_Pin, GPIO_PIN_RESET); //Motor direction
			user_pwm_setvalue(motor, speed);

		} else {

			HAL_GPIO_WritePin(motor.M_GPIOx, motor.M_GPIO_Pin, GPIO_PIN_SET); //Motor direction
			user_pwm_setvalue(motor, speed); //Motor speed
		}

	} else {
		HAL_GPIO_WritePin(motor.M_GPIOx, motor.M_GPIO_Pin, GPIO_PIN_RESET); //Motor direction
		user_pwm_setvalue(motor, 0); //Motor speed //Motor speed
	}

	return;
}

/* Sets the Compare register of the PWM Pin	of motor	*
 * Values between 0 and 999 can be submitted			*
 * a Value of "100" correspond to 10% PWM Signal		*/
void user_pwm_setvalue(MOTOR_T motor, uint16_t value) {

	if (value >= 999)
		value = 999;

	motor.sConfigOC_Motor.Pulse = value;

	HAL_TIM_PWM_Stop(motor.htim, motor.Channel);
	HAL_TIM_PWM_ConfigChannel(motor.htim, &motor.sConfigOC_Motor, motor.Channel);
	HAL_TIM_PWM_Start(motor.htim, motor.Channel);

	return;
}

/* Returns Sensor value for given Sensor */
uint16_t get_sensor(SENSOR sensor) {
	uint16_t result = 0;
	volatile uint16_t adcResultsDMA[SENSOR_COUNT];
	adcComplete = 0;

	// pull all ADC Data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, SENSOR_COUNT);
	while (adcComplete == 0)
		;

	// write Sensor value to result
	if (sensor < SENSOR_COUNT) {
		result = adcResultsDMA[sensor];
	}

	return (result);
}

/* writes Sensor values in Result array		*/
void get_all_sensors(int32_t result[]) {

	volatile uint16_t adcResultsDMA[SENSOR_COUNT];
	adcComplete = 0;

	// pull all ADC Data
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adcResultsDMA, SENSOR_COUNT);
	// wait for new sensor values in DMA
	while (adcComplete == 0)
		;

	// write Sensor value to result
	for (int8_t i = 0; (i < SENSOR_COUNT) || (i < ((sizeof(result)) / sizeof(result[0]))); i++) {
		result[i] = adcResultsDMA[i];
	}

	return;
}

/* updates command, pwm and time from the received Serial Message
 * the message should be 10 characters long
 * command	direction	pwm		unused	time
 * n		(+ or -)	nnnn	x		nnn
 * Example: 3+0700t100 -> command 3 with 70% pwm for 100 cycles
 */
void read_msg_pwm(uint16_t *pcom, int32_t *ppwm, uint16_t *ptime) {
	*ppwm = 0;
	*ptime = 0;
	// write command
	uint8_t command[10];

	for (uint8_t i = 0; i < 10; i++) {
		command[i] = rxBuffer[i];
	}

	// which Fingers are supposed to move first Character
	if (command[0] >= 48 && command[0] <= 57) {
		*pcom = (uint16_t) command[0] - 48;

	} else if (command[0] >= 65 && command[0] <= 70) {
		*pcom = (uint16_t) command[0] - 55;

	} else {
		snprintf(txBuf, 64, "Command not valid i = %d \r\n", command[0]);
		HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
	}

	// What Power is should be applied ?
	for (uint8_t j = 2; j <= 5; j++) {
		if (command[j] >= 48 && command[j] <= 57) {
			*ppwm += (command[j] - 48) * (uint32_t) pow(10, (5 - j));

		} else {
			snprintf(txBuf, 64, "Command not valid character %d \r\n", (j - 1));
			HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
		}
	}
	if (command[1] == 45)
		*ppwm = *ppwm * -1;
	else if (command[1] != 43) {
		snprintf(txBuf, 64, "Wrong sign \r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
	}

	// how long should the Motor work?
	for (uint8_t j = 7; j <= 9; j++) {
		if (command[j] >= 48 && command[j] <= 57) {
			*ptime += (command[j] - 48) * (uint32_t) pow(10, (9 - j));

		} else {
			snprintf(txBuf, 64, "Command not valid character %d \r\n", (j - 1));
			HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
		}
	}

	return;
}

/* updates command, force and time from the received Serial Message
 * the message should be 10 characters long
 * command	State	force	unused	time
 * n		(-)		nnnn	x		nnn
 * Example: 3s5700t100 -> command 3 with 5700 mN for 100 cycles
 * Example: 3-5700t100 -> command 13 open for 100 cycles
 */
void read_msg_force(uint16_t *pcom, int32_t *pforce, uint16_t *ptime) {
	*pforce = 0;
	*ptime = 0;
	// write command
	uint8_t command[10];

	for (uint8_t i = 0; i < 10; i++) {
		command[i] = rxBuffer[i];
	}

	// which Fingers are supposed to move first Character
	if (command[0] >= 48 && command[0] <= 57) {
		*pcom = (uint16_t) command[0] - 48;

	} else if (command[0] >= 65 && command[0] <= 70) {
		*pcom = (uint16_t) command[0] - 55;

	} else {
		snprintf(txBuf, 64, "Command not valid i = %d \r\n", command[0]);
		HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
	}

	// What Power is should be applied
	for (uint8_t j = 2; j <= 5; j++) {
		if (command[j] >= 48 && command[j] <= 57) {
			*pforce += (command[j] - 48) * (int32_t) pow(10, (5 - j));

		} else {
			snprintf(txBuf, 64, "Command not valid character %d \r\n", (j - 1));
			HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
		}
	}

	// State is open?
	if (command[1] == 45)
		*pcom += 10;

	// how long should the Motor work?
	for (uint8_t j = 7; j <= 9; j++) {
		if (command[j] >= 48 && command[j] <= 57) {
			*ptime += (command[j] - 48) * (uint32_t) pow(10, (9 - j));

		} else {
			snprintf(txBuf, 64, "Command not valid character %d \r\n", (j - 1));
			HAL_UART_Transmit(&huart2, (uint8_t*) txBuf, strlen(txBuf), HAL_MAX_DELAY);
		}
	}

	return;
}

/* calculates the force in mN from the sensor data 	*/
uint32_t sensor_to_force(int32_t sensordata, SENSOR sensornumber) {
	uint32_t force;

	force = (ASensor[sensornumber] * sensordata * sensordata) / 256 + TSensor[sensornumber];

	return (force);
}

/* calculates the forces in mN from all the sensor data 	*/
void sensors_to_forces(int32_t *sensordata, int32_t force[]) {

	for (uint8_t i = 0; i < 4; i++) {
		force[i] = (ASensor[i] * sensordata[i] * sensordata[i]) / 256 + TSensor[i];
	}

	return;
}

/* calculates the corresponding sensor value to given force 	*/
uint32_t force_to_sensor(uint32_t force, SENSOR sensornumber) {
	uint32_t sensor;

	if (force >= TSensor[(uint8_t) sensornumber]) {
		sensor = (uint32_t) sqrt(256 * (force - TSensor[(uint8_t) sensornumber]) / (double) ASensor[sensornumber]);
	} else
		sensor = 0;
	return (sensor);
}

