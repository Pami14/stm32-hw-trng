/*
	Matrix Tastatur 4x4 fuer STM32
*/

//Header files
#include "stm32f4xx_hal.h"

// Verzoegerung zum Einschwingen der Pins

// DELAY_VALUE muss ggf. je nach verwendeter CPU bzw. Taktfrequenz
// angepasst werden, um ein sicheres Einschwingen der Tasten
// zu garantieren.
// 55 passt fuer STM32F411 mit 100MHz

#define DELAY_VALUE		65

#define KEYPAD_DELAY 	for (uint32_t i = 0; i < DELAY_VALUE; i++)\
	{\
		__NOP();\
	}

// GPIO pin definitions for keypad columns (must on the same GPIO,
// configure as output open drain with pull-up)
#define KEYPAD_GPIO_COL				GPIOD
#define KEYPAD_PIN_COL0				GPIO_PIN_8
#define KEYPAD_PIN_COL1				GPIO_PIN_9
#define KEYPAD_PIN_COL2				GPIO_PIN_10
#define KEYPAD_PIN_COL3				GPIO_PIN_11

// GPIO pin definitions for keypad rows (must on the same GPIO,
// configure as input with pull-up)
#define KEYPAD_GPIO_ROW				GPIOD
#define KEYPAD_PIN_ROW0				GPIO_PIN_0
#define KEYPAD_PIN_ROW1				GPIO_PIN_1
#define KEYPAD_PIN_ROW2				GPIO_PIN_6
#define KEYPAD_PIN_ROW3				GPIO_PIN_3

/** Public function prototypes ----------------------------------------------
    Parameter must point to char array large enough to hold no of
    keys to be pressed simulaneously and terminating '\0', i.e. max 17 chars */
uint16_t KeypadGetKey(char *);

