/*
    Matrix Tastatur 4x4 f�r STM32
		=============================

    Original:
    http://embeddedsystemengineering.blogspot.com/2016/03/arm-cortex-m3-stm32f103-tutorial-4x4.html
		
		Angepasst f�r STM32F411, erweitert und vereinfacht: 
		Herbert Paulis, FH Campus Wien, 2018, 2022
		
*/

#include "Matrix_Keypad.h"

// Parameter "text" must point to char array large enough to hold no of
// keys to be pressed simulaneously and terminating '\0', i.e. max 17 chars
uint16_t KeypadGetKey(char * text)
{
	uint16_t index = 0;
	
	// Scan column 0 (column 0 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	KEYPAD_DELAY;
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
		text[index++] = '1';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
		text[index++] = '4';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW2))
		text[index++] = '7';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
		text[index++] = '*';
		
	// Scan column 1 (column 1 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	KEYPAD_DELAY;
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
		text[index++] = '2';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
		text[index++] = '5';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW2))
		text[index++] = '8';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
		text[index++] = '0';
		
	// Scan column 2 (column 2 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL3, GPIO_PIN_SET);
	KEYPAD_DELAY;
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
		text[index++] = '3';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
		text[index++] = '6';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW2))
		text[index++] = '9';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
		text[index++] = '#';
		
	// Scan column 3 (column 3 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL3, GPIO_PIN_RESET);
	KEYPAD_DELAY;
	// Read rows
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW0))
		text[index++] = 'A';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW1))
		text[index++] = 'B';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW2))
		text[index++] = 'C';
	if (!HAL_GPIO_ReadPin(KEYPAD_GPIO_ROW, KEYPAD_PIN_ROW3))
		text[index++] = 'D';
	
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(KEYPAD_GPIO_COL, KEYPAD_PIN_COL3, GPIO_PIN_RESET);

	text[index] = 0;
	return index;
}
