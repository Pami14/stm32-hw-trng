/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"
#include "i2c.h"
#include "sdio.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <string.h>

#include "Matrix_Keypad.h"
#include "i2c-lcd.h"
#include "i2c-mux.h"
#include "i2c-poti.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define I2C_LCD_ADR   32
#define I2C_Multi_ADR 112
#define I2C_MUX_BIT12 0
#define I2C_MUX_BIT34 1
#define I2C_MUX_BIT56 2
#define I2C_MUX_BIT78 3

#define RDAC1  0
#define RDAC2  1

#define SAMPLE_COUNT 512

#define SCHMITT_TOLERANZ 2
#define STABIL_LIMIT 5

#define NUM_MEASUREMENTS 50
#define NUM_GENERATORS 32

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} GPIO_Byte;

volatile uint8_t RNG_buffer[SAMPLE_COUNT];
volatile uint32_t cycles;
uint8_t RG_status[32];

uint32_t count1      = 0;
uint16_t count_diag  = 0;

uint8_t active_mode   = 0;
uint8_t resetflag     = 0;
uint8_t Calib_flag    = 0;
uint8_t Analysis_flag = 0;
uint8_t Diag_flag     = 0;
uint8_t SDERROR_flag  = 0;
uint8_t SD_not_found  = 0;
uint8_t USBorSD       = 3; 						// 0: USB / 1: SD / 3: init value
uint8_t StoragLock    = 0; 				    	// 0: not locked / 1: locked (for Run Mode)

uint32_t CalibrationSamples     = 1000000;
uint32_t CalibrationAccuracy    = 15000;

uint8_t Schmit;
uint8_t Noise1;

uint8_t TR1 = 254; // 2N3904 11,0V
uint8_t TR2 = 205; // 2N2222 12,1V
uint8_t TR3 = 182; // 2N5551 10,7V
uint8_t TR4 = 197; // S8050  11,2V

GPIO_Byte MUX_GPIOs[8] = {
    {MuxOut_1_GPIO_Port, MuxOut_1_Pin},
    {MuxOut_2_GPIO_Port, MuxOut_2_Pin},
    {MuxOut_3_GPIO_Port, MuxOut_3_Pin},
    {MuxOut_4_GPIO_Port, MuxOut_4_Pin},
    {MuxOut_5_GPIO_Port, MuxOut_5_Pin},
    {MuxOut_6_GPIO_Port, MuxOut_6_Pin},
    {MuxOut_7_GPIO_Port, MuxOut_7_Pin},
    {MuxOut_8_GPIO_Port, MuxOut_8_Pin}
};

i2c_poti_t RG[8] = {
    { .hi2c = &hi2c1, .addr_offset = 15 },  	// RG0
    { .hi2c = &hi2c1, .addr_offset = 14 },  	// RG1
    { .hi2c = &hi2c1, .addr_offset = 12 },  	// RG2
    { .hi2c = &hi2c1, .addr_offset = 11 },  	// RG3
    { .hi2c = &hi2c1, .addr_offset = 10 },  	// RG4
    { .hi2c = &hi2c1, .addr_offset = 8 },   	// RG5
    { .hi2c = &hi2c1, .addr_offset = 3 },   	// RG6
    { .hi2c = &hi2c1, .addr_offset = 2 }    	// RG7
};

i2c_mux_t mux = {
		.hi2c        = &hi2c1,                  // Pointer to the I2C handle
		.rst_port    = I2C_Reset_GPIO_Port,   	// GPIO port for the reset pin
		.rst_pin     = I2C_Reset_Pin,          	// GPIO pin number for the reset pin
		.addr_offset = 0                   		// Address offset for the I2C multiplexer
};

FATFS FatFs;    								// FatFS-Objekt
FIL   fil;      								// Datei-Objekt

char filename[32];
UINT bytes_written;

char SD_Storage[10];

volatile uint8_t newKey = 0;
char KeyPad    = 'x';
char KeyPadold = 'y';

char LCD_line_1[21] = "MODE    SAVE: -/-   ";
char LCD_line_2[21] = "A START    D KALIB. ";
char LCD_line_3[21] = "C STORAGE  # ANALYSE";
char LCD_line_4[21] = "FREI: -/- STATE INIT";

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

uint8_t SD_get_free_space_GB(void)
{
	FATFS *pfs;
	DWORD fre_clust, fre_sect;
	uint64_t fre_bytes;

	SDERROR_flag = f_mount(&FatFs, "", 1);

    if (SDERROR_flag != FR_OK) {
        return 1;
    }

	SDERROR_flag = f_getfree("", &fre_clust, &pfs);

	if ( SDERROR_flag == FR_OK)
    {
        fre_sect = fre_clust * pfs->csize;

        fre_bytes = (uint64_t)fre_sect * 512;

        uint8_t free_gb = fre_bytes / (1024UL * 1024UL * 1024UL);

        return free_gb;
    }

    f_mount(NULL, "", 1);

    return 0;
}

void SDwriteRandomByte( uint8_t* data, uint16_t length )
{
	unsigned int bytesWritten;

	SDERROR_flag = f_lseek(&fil, f_size(&fil));

    SDERROR_flag = f_write(&fil, data, length, &bytesWritten);

    if (SDERROR_flag == FR_OK && bytesWritten == 512) {
    	// Success
    	SDERROR_flag = f_sync(&fil);
    }
}

void update_LCD( char cmd ){

	if (cmd == 'i')                                                	// INIT : i
	{
		lcd_clear_display();

		lcd_locate(1, 1);
		lcd_printf(LCD_line_1);
		lcd_locate(2, 1);
		lcd_printf(LCD_line_2);
		lcd_locate(3, 1);
		lcd_printf(LCD_line_3);
		lcd_locate(4, 1);
		lcd_printf(LCD_line_4);
	}
	else if( cmd == 'f' ){ 											// update free storage of SD : f

		if( SD_not_found == 0) {

			uint8_t SD_GB_free = SD_get_free_space_GB();

			if (SD_GB_free != 0) {
				sprintf(SD_Storage, "%dGB", SD_GB_free);
			} else {
				sprintf(SD_Storage, "ERR ");
			}
		} else {
			sprintf(SD_Storage, "NotF");
		}



	    memcpy(&LCD_line_4[5], SD_Storage, 4);

	    lcd_locate(4, 1);
	    lcd_printf(LCD_line_4);

	} else if( cmd == 'r'){									     	// Mode = RUN : r

		memcpy(&LCD_line_2[0], "B STOP ", 7);
		lcd_locate(2, 1);
		lcd_printf(LCD_line_2);

		memcpy(&LCD_line_4[16], "RUN ", 4);
		lcd_locate(4, 1);
		lcd_printf(LCD_line_4);

	} else if( cmd == 's' ){									   // Mode = STOP : s

		memcpy(&LCD_line_2[0], "A START", 7);
		lcd_locate(2, 1);
		lcd_printf(LCD_line_2);

		memcpy(&LCD_line_4[16], "STOP", 4);
		lcd_locate(4, 1);
		lcd_printf(LCD_line_4);

	} else if( cmd == 'd' ){									    // Storage = SD : d

		memcpy(&LCD_line_1[14], "SD ", 3);
		lcd_locate(1, 1);
		lcd_printf(LCD_line_1);

	} else if( cmd == 'u' ){										// Storage = USB : u

		memcpy(&LCD_line_1[14], "USB", 3);
		lcd_locate(1, 1);
		lcd_printf(LCD_line_1);

	} else if( cmd == 'e' ){                                        // Storage = USB : u

		char SD_ERR[12];

		sprintf(SD_ERR, "SD-ERR: %d", SDERROR_flag );
		memcpy(&LCD_line_1[8], SD_ERR , 11);
		lcd_locate(1, 1);
		lcd_printf(LCD_line_1);
	}

}

void SelectRGBits( uint8_t RGBits, i2c_mux_t* mux ){

	if(RGBits == 0){

		i2c_mux_select(mux, 0);

		HAL_GPIO_WritePin(S0_OutSel_GPIO_Port, S0_OutSel_Pin, 0);
		HAL_GPIO_WritePin(S1_OutSel_GPIO_Port, S1_OutSel_Pin, 0);

	} else if( RGBits == 1 ){
		i2c_mux_select(mux, 1);

		HAL_GPIO_WritePin(S0_OutSel_GPIO_Port, S0_OutSel_Pin, 1);
		HAL_GPIO_WritePin(S1_OutSel_GPIO_Port, S1_OutSel_Pin, 0);

	} else if( RGBits == 2 ){
		i2c_mux_select(mux, 2);

		HAL_GPIO_WritePin(S0_OutSel_GPIO_Port, S0_OutSel_Pin, 0);
		HAL_GPIO_WritePin(S1_OutSel_GPIO_Port, S1_OutSel_Pin, 1);

	} else if( RGBits == 3 ){
		i2c_mux_select(mux, 3);

		HAL_GPIO_WritePin(S0_OutSel_GPIO_Port, S0_OutSel_Pin, 1);
		HAL_GPIO_WritePin(S1_OutSel_GPIO_Port, S1_OutSel_Pin, 1);
	}
	HAL_Delay(100);
}

int CalibrateNoisGenerator(i2c_poti_t* RG, GPIO_Byte* RG_Pin, uint8_t Transistor)
{
	int count = 0;
	int done  =  0;
	int cout1 = 0;

	uint8_t schmitt_history[STABIL_LIMIT] = {0};
	uint8_t index = 0;

	uint8_t Noise = Transistor;
	uint8_t Schmitt;

	//write start values
	Schmitt = i2c_poti_READ_EEPROM(RG, RDAC1);

	i2c_poti_set(RG, RDAC1, &Schmitt);
	i2c_poti_set(RG, RDAC2, &Noise);

	while(done == 0){

		uint32_t LowCount  = 0;
		uint32_t HighCount = 0;

		while( count < CalibrationSamples)
		{
			if( HAL_GPIO_ReadPin(RG_Pin->port, RG_Pin->pin) == 0)
			{
			  LowCount++;

			} else {

			  HighCount++;
			}
			count++;
		}
		count = 0;

		lcd_locate(3, 1);
		lcd_printf("L: %d      ", LowCount);
		lcd_locate(4, 1);
		lcd_printf("H: %d      ", HighCount);
		lcd_locate(3, 11);
		lcd_printf("NOISE: %d", Noise );
		lcd_locate(4, 11);
		lcd_printf("SCH:   %d", Schmitt );

		if (HighCount < (CalibrationSamples/2) + (CalibrationAccuracy/3) && HighCount > (CalibrationSamples/2) - (CalibrationAccuracy/3)){
		  done = 1;
		} else {

			i2c_poti_set(RG, RDAC1, &Schmitt);

			if (HighCount < (CalibrationSamples/2) ){
			  Schmitt--;
			}

			if (HighCount > (CalibrationSamples/2) ){
			  Schmitt++;
			}

			schmitt_history[index] = Schmitt;
			index = (index + 1) % STABIL_LIMIT;

			int min = schmitt_history[0];
			int max = schmitt_history[0];

			for (int i = 1; i < STABIL_LIMIT; i++) {
			  if (schmitt_history[i] < min) min = schmitt_history[i];
			  if (schmitt_history[i] > max) max = schmitt_history[i];
			}

			if ((max - min) <= SCHMITT_TOLERANZ) {

				if (HighCount < (CalibrationSamples/2) + CalibrationAccuracy && HighCount > (CalibrationSamples/2) - CalibrationAccuracy){
				  done = 1;
				}

				if( cout1 == 5 ){ done = 1; }

				cout1++;
			}
		}
	}
	//write final values to EEPROM
	i2c_poti_RDAC_to_eeprom(RG, RDAC1);
	i2c_poti_RDAC_to_eeprom(RG, RDAC2);

	return 0;
}

int ManualCalibrateNoisGenerator(i2c_poti_t* RG, GPIO_Byte* RG_Pin)
{
	int count = 0;

	uint32_t LowCount  = 0;
	uint32_t HighCount = 0;

	while( count < CalibrationSamples)
	{
		if( HAL_GPIO_ReadPin(RG_Pin->port, RG_Pin->pin) == 0)
		{
			LowCount++;

		} else {

			HighCount++;
		}
		count++;
	}
	count = 0;

	lcd_locate(1, 1);
	lcd_printf("L:%d      ", LowCount);
	lcd_locate(1, 10);
	lcd_printf("H:%d      ", HighCount);

	return 0;
}

void AnalyseNoisGenerator( void ){

	uint32_t LowCount  = 0;
	uint32_t HighCount = 0;

	int count = 0;

	int u = Analysis_flag / 8;   //Get actual group
	int i = Analysis_flag % 8;	 //Get actual generator

	SelectRGBits(u, &mux);

	while( count < 1000 ){

	  if( HAL_GPIO_ReadPin( MUX_GPIOs[i].port, MUX_GPIOs[i].pin ) == 0) {
		  LowCount++;
	  } else {
		  HighCount++;
	  }
	  count++;
	}

	if (HighCount < 200 || HighCount > (800) ){

		RG_status[Analysis_flag] = 1;

		lcd_locate(4, 17);
		lcd_printf("ERR ");

	} else {
		RG_status[Analysis_flag] = 0;
	}

	Analysis_flag ++;

	if(Analysis_flag == 31) {

		Analysis_flag = 0;
	}
}

void calibrate_all_RG( int mode ){

	uint8_t Transistor;

	for( int u = 0; u < 4; u++ ){							// select RG group 1 - 4

		int y = u + 1;

		lcd_locate(2, 8);
		lcd_printf("%d", y);

		if( mode == 1 ){

			if (u == 0) { Transistor = TR1; }
			if (u == 1) { Transistor = TR2; }
			if (u == 2) { Transistor = TR3; }
			if (u == 3) { Transistor = TR4; }

		} else if ( mode == 0 ) {

			if (u == 0) { Transistor = TR1; }
		}

		SelectRGBits(u, &mux);

		for( int i = 0; i < 8; i++ ){						// select RG generator 1 - 8

			y = i + 1;

			lcd_locate(2, 20);
			lcd_printf("%d", y);

			CalibrateNoisGenerator( &RG[i], &MUX_GPIOs[i], Transistor );
		}
	}
}

uint8_t sample_gpio( void ){

    uint16_t pe = GPIOE->IDR; 	//faster then HAL function (30 clock cycle vs 42 per HAL_GPIO execution)
    uint16_t pb = GPIOB->IDR;   //faster then HAL function (30 clock cycle vs 42 per HAL_GPIO execution)

    uint8_t value = ((pe >> 6) & 0xFE) | ((pb >> 2) & 0x01); // make one byte out of PE Pins and PB Pin

    return value;
}

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

void sample_RNG( void ){

	DWT->CYCCNT = 0;

    for( int i = 0; i < SAMPLE_COUNT; i++ ){     // 1 measurement takes 64 clock cycles -> sampling rate= 48MHz/64 = 0,75MHz

        RNG_buffer[i] = sample_gpio();
    }

    cycles = DWT->CYCCNT;
}

void get_measurement(uint8_t gen, uint32_t* high, uint32_t* low) {

    int count = 0;
    GPIO_Byte RG_Pin;

    // Reset outputs
    *high = 0;
    *low  = 0;

    // group based on generator number
    if (gen >= 1 && gen <= 8) {
        SelectRGBits(0, &mux);
    } else if (gen >= 9 && gen <= 16) {
        SelectRGBits(1, &mux);
    } else if (gen >= 17 && gen <= 24) {
        SelectRGBits(2, &mux);
    } else if (gen >= 25 && gen <= 32) {
        SelectRGBits(3, &mux);
    } else {
        return;
    }

    // Get MUX pin
    RG_Pin = MUX_GPIOs[(gen - 1) % 8];

    //Count CalibrationSamples
    while (count < 100000 ) {
        if (HAL_GPIO_ReadPin(RG_Pin.port, RG_Pin.pin) == GPIO_PIN_RESET) {
            (*high)++;
        } else {
            (*low)++;
        }
        count++;
    }
}

void AnalysisDistNoiseGen (void ){

	 if (f_mount(&FatFs, "", 1) == FR_OK) {

		// Create and 32 files with header
		for (int i = 0; i < NUM_GENERATORS; ++i) {
			sprintf(filename, "NoiseGen%d.csv", i + 1);

			if (f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
				f_printf(&fil, "Measurement;HighValue;LowValue\r\n");
				f_close(&fil);
			} else {
				lcd_locate(2,1); lcd_printf("SD-fail 1");
			}
		}

		if (f_open(&fil, "NoiGenGes.csv", FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
		    f_printf(&fil, "Measurement;NoiseGenerator;HighValue;LowValue\n");
		    f_close(&fil);
		}else {
			lcd_locate(2,1); lcd_printf("SD-fail 2");
		}

		//100 measurement cycles
		for (int m = 1; m <= NUM_MEASUREMENTS; ++m) {

			lcd_locate(1,1); lcd_printf("Measurement: %d", m);

			for (int i = 0; i < NUM_GENERATORS; ++i) {

				uint32_t high, low;
				get_measurement(i + 1, &high, &low);

				sprintf(filename, "NoiseGen%d.csv", i + 1);

				if (f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
					f_printf(&fil, "%d;%lu;%lu\n", m, high, low);
					f_close(&fil);
				} else {
					lcd_locate(2,1); lcd_printf("SD-fail 11");
				}

				if (f_open(&fil, "NoiGenGes.csv", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
					f_printf(&fil, "%d;%d;%lu;%lu\n", m, i + 1, high, low);
					f_close(&fil);
				} else {
					lcd_locate(2,1); lcd_printf("SD-fail 22");
				}
			}
		}

		f_mount(NULL, "", 1);  // Unmount SD card
	}
}


void ManualCTRLNoisGenerator( void ){

	uint8_t genreator = 0;

	int group   = 0;
	int select  = 0;
	int active  = 1;
	int steps   = 0;

	char KeyPadValue;
	char KeyPadold;

	lcd_locate(1, 1);
	lcd_printf("Group: * Noisgen.:  ");
	lcd_locate(4, 1);
	lcd_printf("OK press #");

	while( active ){

		if( newKey ){

			KeypadGetKey( &KeyPadValue );

			if ( KeyPadold != KeyPadValue ){

				if( KeyPadValue == '#' && select != 3 ){ 									//#: select group and generator to control

					KeyPadValue = 'x';

					if( select == 0 &&  genreator == 0 ){

						select = 1;
						lcd_locate(1,  8);

						if( group == 0 ){ lcd_printf(" ");}

						lcd_locate(1,  20); lcd_printf("*");

						lcd_locate(4, 1);
						lcd_printf("OK press 0");

					} else if( select == 1 && group == 0 ){

						select = 0;
						lcd_locate(1, 20);

						if( genreator == 0 ){ lcd_printf(" "); }

						lcd_locate(1, 8); lcd_printf("*");
					}

					HAL_Delay(500);
				}

				int Value = KeyPadValue - '0';

				if( select == 0 && Value >= 1 && Value <= 4 ){					// select group value: values from 1 - 4 are permitted

					lcd_locate(1, 8);
					lcd_printf("%d", Value);

					group = Value;
				}

				if( select == 1 && Value >= 1 && Value <= 8 ){					// select generator value: values from 1 - 8 are permitted

					lcd_locate(1, 20);
					lcd_printf("%d", Value);

					genreator = Value;
				}
			}

			if( (KeyPadValue == '1' || KeyPadValue == '2' || KeyPadValue == '4' || KeyPadValue == '5') && select == 3 ){

				KeyPadold = KeyPadValue;
			}

			if( KeyPadValue == '0' && genreator >= 1 && group >= 1 && select != 3 ){ 			//0: Start manual mode of selected generator

				SelectRGBits(group - 1, &mux); 									//group - 1: 1-4 needed to be converted to 0-3 due to array

				select = 3;

				lcd_locate(2, 1);
				lcd_printf("1- 2+  Noise  =   %d", Noise1);

				lcd_locate(3, 1);
				lcd_printf("4- 5+  Schmit.=   %d", Schmit);

				lcd_locate(4, 1);
				lcd_printf("A: x1* B:x10  C:test");

				i2c_poti_set( &RG[genreator - 1], RDAC2, &Noise1  );	   		//Generator - 1: 1-8 needed to be converted to 0-7 due to array
				i2c_poti_set( &RG[genreator - 1], RDAC1, &Schmit );
			}

			if( select == 3 ){

				if( KeyPadValue == '1' || KeyPadValue == '2' ){									//1 or 2: set noise voltage

					if( KeyPadValue == '1' ){ if( steps ){ Noise1-=10;} else{ Noise1--; }}
					else if ( KeyPadValue == '2' ){ if( steps ){ Noise1+=10;} else{ Noise1++; }}

					if( Noise1   > 255 ){ Noise1   = 0;   }
					if( Noise1   < 0   ){ Noise1   = 255; }

					lcd_locate(2, 17);
					lcd_printf("%3d", Noise1);

					i2c_poti_set( &RG[genreator - 1], RDAC2, &Noise1 );

					HAL_Delay(500);
				}

				if( KeyPadValue == '4' || KeyPadValue == '5' ){									//4 or 5: set schmitt trigger voltage

					if( KeyPadValue == '4' ){ if( steps ){ Schmit-=10;} else{ Schmit--; }}
					else if ( KeyPadValue == '5' ){ if( steps ){ Schmit+=10;} else{ Schmit++; }}

					if( Schmit > 255 ){ Schmit = 0;   }
					if( Schmit < 0   ){ Schmit = 255; }

					lcd_locate(3, 17);
					lcd_printf("%3d", Schmit);

					i2c_poti_set( &RG[genreator - 1], RDAC1, &Schmit );

					HAL_Delay(500);
				}

				if( KeyPadValue == 'A'){														//A: change increment = 1x
					lcd_locate(4,6);
					lcd_printf("*");
					lcd_locate(4,13);
					lcd_printf(" ");

					steps = 0;
				}

				if( KeyPadValue == 'B'){														//B: change increment = 10x
					lcd_locate(4,13);
					lcd_printf("*");
					lcd_locate(4,6);
					lcd_printf(" ");

					steps = 1;
				}

				if( KeyPadValue == 'C'){														//C: test distribution

					ManualCalibrateNoisGenerator( &RG[genreator - 1], &MUX_GPIOs[genreator - 1]);

					lcd_locate(4, 15);
					lcd_printf("D:safe");
				}

				if( KeyPadValue == 'D'){														//D: store values to eeprom and exit

					i2c_poti_RDAC_to_eeprom( &RG[genreator - 1], RDAC1 );
					i2c_poti_RDAC_to_eeprom( &RG[genreator - 1], RDAC2 );

					active = 0;
				}
			}

			if( KeyPadValue == '*' ) 															//*: Exit
			{
				active = 0;
			}
			newKey = 0;
		}

	}
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void init_sd(void) {

	SDERROR_flag = f_mount(&FatFs, "", 1);

	if (SDERROR_flag != FR_OK) {
		return;
	}

	SDERROR_flag = f_open(&fil, "hexdata.bin",  FA_CREATE_ALWAYS | FA_WRITE );
	if (SDERROR_flag == FR_OK) {
		f_close(&fil);
	}

	f_mount(NULL, "", 1);
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
  DWT_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  if( HAL_GPIO_ReadPin(SD_CD_GPIO_Port, SD_CD_Pin) == 0 ){

	  init_sd();

  } else {

	  SD_not_found = 1;
  }


  HAL_Delay(500);

  lcd_init(&hi2c3, (I2C_LCD_ADR << 1));
  lcd_clear_display();

  update_LCD('i');
  update_LCD('f');

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if( newKey ){

		KeypadGetKey(&KeyPad);

		if ( KeyPadold != KeyPad)
		{
		  if( KeyPad == 'A' ) 							//A: START
		  {
			  if (USBorSD != 3) {

				  StoragLock = 1;

				  if ( USBorSD == 1 )
				  {
					  if (f_mount(&FatFs, "", 1) == FR_OK) {

						  SDERROR_flag = f_open(&fil, "hexdata.bin", FA_WRITE | FA_OPEN_ALWAYS);

						  if ( SDERROR_flag == FR_OK) {

						  }
					  }
				  }

				  update_LCD('r');

				  active_mode = 1;
			  }
		  }

		  if( KeyPad == 'B' )							//B: STOP
		  {
			  if (USBorSD != 3) {

				  StoragLock = 0;

				  if ( USBorSD == 1 )
				  {
					  f_close(&fil);

					  f_mount(NULL, "", 1);
				  }

				  update_LCD('s');

				  active_mode = 0;
			  }
		  }
		  	  	  	  	  	  	  	  	  	  	  	  	//C: STORAGE : SD | USB
		  if( KeyPad == 'C' && StoragLock == 0 && Calib_flag == 0 && Diag_flag == 0 )
		  {
			  if(USBorSD)
			  {
				  update_LCD('u');

				  USBorSD = 0;							// USB

			  } else {

				  update_LCD('d');

				  if( SD_not_found == 0 ){
					  USBorSD = 1;						// SD
				  } else {
					  update_LCD('u');
				  }
			  }


			  KeyPadold = 'x';
		  }

		  if( KeyPad == 'D' && active_mode == 0 )		//D: Calibration :
		  {
			  lcd_clear_display();

			  lcd_locate(1, 1);
			  lcd_printf("Select Calibration:");
			  lcd_locate(2, 1);
			  lcd_printf("1: Manual");
			  lcd_locate(3, 1);
			  lcd_printf("2: Auto");

			  Calib_flag = 1;
		  }

		  if( Calib_flag == 1 )						      //Calibration
		  {
			  if( KeyPad == '1' )					      //1: Manual Calibration :
			  {
				  lcd_clear_display();

				  HAL_Delay(500);

				  ManualCTRLNoisGenerator();

				  Calib_flag = 0;

				  update_LCD('i');
			  }

			  if( KeyPad == '2' )						  //2: Auto Calibration :
			  {
				  lcd_clear_display();

				  lcd_locate(1, 1);
				  lcd_printf("Select Calibration:");
				  lcd_locate(2, 1);
				  lcd_printf("3: Normal   CFG");
				  lcd_locate(3, 1);
				  lcd_printf("4: Analysis CFG");


				  Calib_flag = 2;
			  }
		  }

		  if( Calib_flag == 2 )						      //Calibration
		  {
			  if( KeyPad == '3' )					      //3: Normal CFG (transistor: same type is used as noise source)
			  {
				  lcd_clear_display();

				  lcd_locate(1, 1);
				  lcd_printf("Normal CFG");
				  lcd_locate(2, 1);
				  lcd_printf("Group:   Noisgen.:  ");

				  calibrate_all_RG(0);

				  Calib_flag = 0;
				  update_LCD('i');
			  }

			  if( KeyPad == '4' )						  //4: Analysis CFG (transistor: four different types are used as noise source)
			  {
				  lcd_clear_display();

				  lcd_locate(1, 1);
				  lcd_printf("Analysis CFG");
				  lcd_locate(2, 1);
				  lcd_printf("Group:   Noisgen.:  ");

				  calibrate_all_RG(1);

				  Calib_flag = 0;
				  update_LCD('i');
			  }
		  }

		  if( KeyPad == '#' )								//#: Check Error State
		  {
			  Diag_flag++;

			  if (Diag_flag == 1) {

				  lcd_clear_display();

				  for (int i = 0; i < 32 ; i++) {

					  AnalyseNoisGenerator();
				  }

				  lcd_locate(1, 1);
				  lcd_printf("GRP1: %d%d%d%d%d%d%d%d",  RG_status [0], RG_status [1], RG_status [2], RG_status [3], RG_status [4], RG_status [5], RG_status [6], RG_status [7]);
				  lcd_locate(2, 1);
				  lcd_printf("GRP2: %d%d%d%d%d%d%d%d",  RG_status [8], RG_status [9], RG_status[10], RG_status[11], RG_status[12], RG_status[13], RG_status[14], RG_status[15]);
				  lcd_locate(3, 1);
				  lcd_printf("GRP3: %d%d%d%d%d%d%d%d",  RG_status[16], RG_status[17], RG_status[18], RG_status[19], RG_status[20], RG_status[21], RG_status[22], RG_status[23]);
				  lcd_locate(4, 1);
				  lcd_printf("GRP4: %d%d%d%d%d%d%d%d",  RG_status[24], RG_status[25], RG_status[26], RG_status[27], RG_status[28], RG_status[29], RG_status[30], RG_status[31]);
			  }

			  if (Diag_flag == 2) {

				  update_LCD('i');

				  Diag_flag = 0;
			  }

			  KeyPadold = 'x';

			  HAL_Delay(500);
		  }

		  if( KeyPad == '0' )								//0: Analysis mode --> 100x write all noise generator distributions to SD
		  {
			  lcd_clear_display();

			  if( SD_not_found == 0 ){

				  AnalysisDistNoiseGen();

			  } else {

				  lcd_locate(1,1); lcd_printf("SD not found");
				  HAL_Delay(1000);
			  }

			  update_LCD('i');
		  }

		  if( KeyPad == '*' || resetflag == 1 )				//* + 1: Software-Reset
		  {
			  if( KeyPad == '1' )							//*: Software-Reset
			  {
				  NVIC_SystemReset();
			  }

			  resetflag = 1;
		  }

		  if( KeyPad != 0){
			  KeyPadold = KeyPad;
		  }

		  if ( SDERROR_flag != 0 )
		  {
			 update_LCD('e');
		  }
		}

		newKey = 0;
	}

	if( active_mode == 1 ){

		sample_RNG();

		if ( USBorSD == 1 ){
			SDwriteRandomByte(&RNG_buffer[0], 512);

			count1 ++;

			if( SDERROR_flag != 0){

				f_mount(NULL, "", 1);

				update_LCD('s');
				update_LCD('e');

				active_mode = 0;
			}

			if( count1 == 2000000 ){

				f_close(&fil);

				update_LCD('f');
				count1 = 0;

				SDERROR_flag = f_open(&fil, "hexdata.bin", FA_WRITE | FA_OPEN_ALWAYS);
			}
		} else if ( USBorSD == 0 ){

			CDC_Transmit_FS( &RNG_buffer[0], 512);
		}

		count_diag++;

		if( count_diag == 1000 ){

			AnalyseNoisGenerator();
			count_diag = 0;
		}
	}

	HAL_Delay(1);
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0 || GPIO_Pin == GPIO_PIN_1 || GPIO_Pin == GPIO_PIN_3 || GPIO_Pin == GPIO_PIN_6)
	{
		newKey = 1;
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
