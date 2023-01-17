/* Autor: Leonhard Kirchberger
 * Date:  17.01.2023
 * Functions to control a 3D printed hand with Gear Motors and Force sensors
 */

#ifndef HAND_FUNCTIONS_H
#define HAND_FUNCTIONS_H

//////////////////////////// include ////////////////////////////
#include "main.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

/////////////////////////// define //////////////////////////////
#define CLOCKWISE  ((uint8_t)0x0000)
#define CCLOCKWISE  ((uint8_t)0x0001)
#define SENSOR_COUNT 4

/////////////////////////// variables //////////////////////////
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;
extern UART_HandleTypeDef huart2;

// interrupt indicators
volatile uint8_t adcComplete;
volatile uint8_t timestep;
volatile uint8_t receive;

// UART array
uint8_t rxBuffer[10];
char txBuf[64];

// Motor struct
typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	TIM_OC_InitTypeDef sConfigOC_Motor;
	GPIO_TypeDef *M_GPIOx;
	uint16_t M_GPIO_Pin;
} MOTOR_T;

// Sensor names
typedef enum {
	SENSOR1, SENSOR2, SENSOR3, SENSOR4,
} SENSOR;

//////////////////////////// Empirical values ////////////////////////


// Empirical time values for opening the hand form closed [Cycles]
#define FTIME1 30
#define FTIME2 30
#define FTIME3 30
#define FTIME4 22

/* Sensor calibration F = (a/256) * sensor_data^2 + t		*
 * Measuring Sensor output at 1000 mN and 10000 mN			*/
#define A_SENSOR1 5
#define A_SENSOR2 5
#define A_SENSOR3 5
#define A_SENSOR4 5

#define T_SENSOR1 0
#define T_SENSOR2 0
#define T_SENSOR3 0
#define T_SENSOR4 0

///////////////////////////// functions //////////////////////////////

/* Initialization of Timers		*/
void hand_functions_init();

/* Every Motor is set to its pwm value and time is reduced by 1				*
 * if the time is 1 or 0 the Motor is stopped 								*
 * if the pwm value is negative the motor will spin the other direction 	*/
void SetMotors(int32_t pwm[], uint32_t time[]);

/* Motor control for motor						*
 * Gets direction (1 or 0) and speed (pwm)		*
 */
void Motor(MOTOR_T motor, uint8_t dir, uint16_t speed);

/* Sets the Compare register of the PWM Pin	of motor	*
 * Values between 0 and 999 can be submitted			*
 * a Value of "100" correspond to 10% PWM Signal		*/
void user_pwm_setvalue(MOTOR_T motor, uint16_t value);

/* Returns Sensor value for given Sensor */
uint16_t get_sensor(SENSOR sensor);

/* writes Sensor values in Result array		*/
void get_all_sensors(int32_t result[]);

/* updates command, pwm and time from the received Serial Message
 * the message should be 10 characters long
 * command	direction	pwm		unused	time
 * n		(+ or -)	nnnn	x		nnn
 * Example: 3+0700t100 -> command 3 with 70% pwm for 100 cycles
 */
void read_msg_pwm(uint16_t *pcom, int32_t *ppwm, uint16_t *ptime);

/* updates command, force and time from the received Serial Message
 * the message should be 10 characters long
 * command	State	force	unused	time
 * n		(-)		nnnn	x		nnn
 * Example: 3s5700t100 -> command 3 with 5700 mN for 100 cycles
 * Example: 3-5700t100 -> command 13 open for 100 cycles
 */
void read_msg_force(uint16_t *pcom, int32_t *pforce, uint16_t *ptime);

/* calculates the force in mN from the sensor data 	*/
uint32_t sensor_to_force(int32_t sensordata, SENSOR sensornumber);

/* calculates the forces in mN from all the sensor data 	*/
void sensors_to_forces(int32_t *sensordata, int32_t force[]);

/* calculates the corresponding sensor value to given force 	*/
uint32_t force_to_sensor(uint32_t force, SENSOR sensornumber);

#endif
