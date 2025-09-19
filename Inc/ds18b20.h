/******************************************************************************************* */
/*                                                                                           */
/* DS18B20.h                                                                                 */
/*                                                                                           */
/* Minimal driver to interface DS18B20 temperature sensors with STM32H7 microcontroller      */
/*                                                                                           */
/* Florian TOPEZA & Merlin KOOSHMANIAN - 2025                                                */
/*                                                                                           */
/******************************************************************************************* */

#ifndef INC_DS18B20_H_
// Header guard to prevent multiple inclusions
#define INC_DS18B20_H_

/******************************* INCLUDES BEGIN ******************************************** */

/** Include standard libraries */
#include <stdbool.h> 		// Required to use booleans
#include <stdint.h> 		// Required to use uint8_t and uint16_t
#include <stdlib.h>			// To dynamically allocate memory

/** Include STM32 HAL */
#include "stm32h7xx_hal.h" 	// HAL functions for GPIO and Timers used here are declared
                            // in the HAL files

/** Include error type */
#include "errors.h"

/******************************* INCLUDES END ********************************************** */

/******************************* DEFINE BEGIN ********************************************** */

// Debug DS18B20
#ifdef DEBUG_DS18B20
#include "console.h"
#define log_ds18b20(...)  printf(__VA_ARGS__)
#else
#define log_ds18b20(...)
#endif


//DS18B20 ROM Commands
#define SEARCH_ROM		0xF0
#define READ_ROM		0x33
#define MATCH_ROM		0x55
#define SKIP_ROM		0xCC
#define ALARM_SEARCH	0xEC


//DS18B20 Function Commands
#define CONVERT_T			0x44
#define WRITE_SCRATCHPAD	0x4E
#define READ_SCRATCHPAD		0xBE
#define COPY_SCRATCHPAD		0x48
#define RECALL_EE			0xB8
#define READ_PWR_SUPPLY		0xB4

/*********************************** DEFINE END ******************************************** */

/******************************** TYPEDEF BEGIN ******************************************** */

/* DS18B20 structure */
typedef struct
{
    TIM_HandleTypeDef htim;             // Timer TypeDef instance for the timer to use delays in microseconds
    TIM_TypeDef* timer_instance;        // Timer instance
    void (*timer_clk_enable)(void);		// Pointer to the clock enable function for the timer

    GPIO_TypeDef * gpio_port;   // GPIO Port for the onewire pin of the sensor
    uint16_t gpio_pin;          // GPIO Pin number for the onewire of the sensor
    void (*gpio_clk_enable)(void);   // Pointer to the clock enable function for the chosen pin for the onewire

} DS18B20_t;

/* Search state structure */
typedef struct {

    // The highest bit position where a bit was ambiguous and a zero was written
    int8_t last_zero_branch;

    // Internal flag to indicate if the search is complete
    // This flag is set once there are no more branches to search
    bool done;

    // Discovered 64-bit device address (LSB first)
    // After a successful search, this contains the found device address.
    // During a search this is overwritten LSB-first with a new address.
    uint8_t address[8];

} onewire_search_state_t;

/* ROM Code Address structure */
typedef struct {

	uint8_t ROM_code[8];
	void *p_parent_ROM_code_address;

} ROM_Code_Address_t;

/******************************** TYPEDEF END ********************************************** */

/************************** FUNCTION PROTOTYPES BEGIN ************************************** */

error_t DS18B20_Init(DS18B20_t* sensor);

//ROM_Code_Address_t* DS18B20_Search(DS18B20_t* sensor);

uint8_t DS18B20_Search(DS18B20_t* sensor, uint64_t ROM_Codes_array[]);

//int DS18B20_GetTemp(DS18B20_t *sensor, ROM_Code_Address_t *p_ROM_code_address,
 //                 uint16_t *temperature);

uint8_t DS18B20_GetTemp(DS18B20_t *sensor, const uint64_t ROM_Codes_array[], uint16_t *temperature);

/************************** FUNCTION PROTOTYPES END **************************************** */

 #endif /* INC_DS18B20_H_ */

/********************************** END OF FILE ******************************************** */
