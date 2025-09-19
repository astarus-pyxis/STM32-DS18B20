/******************************************************************************************* */
/*                                                                                           */
/* DS18B20.h                                                                                 */
/*                                                                                           */
/* Minimal driver to interface DS18B20 temperature sensors with STM32H7 microcontroller      */
/*                                                                                           */
/* Florian TOPEZA & Merlin KOOSHMANIAN - 2025                                                */
/*                                                                                           */
/******************************************************************************************* */

#include "ds18b20.h"

/******************************* STATIC FUNCTIONS BEGIN ************************************ */

static error_t DS18B20_Timer_Init(DS18B20_t *sensor);

static uint8_t DS18B20_Start(DS18B20_t *sensor);

static void DS18B20_delay(DS18B20_t *sensor, uint16_t us);

static uint8_t DS18B20_writeData(DS18B20_t *sensor, uint8_t data);
static uint8_t DS18B20_write1(DS18B20_t *sensor);
static uint8_t DS18B20_write0(DS18B20_t *sensor);

static uint8_t DS18B20_readByte(DS18B20_t *sensor);
static uint8_t DS18B20_read(DS18B20_t *sensor);

static uint8_t onewireSearchInit(onewire_search_state_t *state);
static uint8_t searchNext(DS18B20_t *sensor, onewire_search_state_t *state);
static uint8_t searchDevices(DS18B20_t *sensor, uint8_t command, onewire_search_state_t *state);
static uint8_t crcGenerator(uint8_t initial_crc, uint8_t input);
static uint8_t addressValid(const uint8_t ROM_code[]);

/******************************* STATIC FUNCTIONS END ************************************** */

/******************************* IO FUNCTIONS BEGIN **************************************** */

/* Sensor init function */
error_t DS18B20_Init(DS18B20_t *sensor)
{
	error_t result = OK;

	/* Enable CS GPIO clock using the provided function pointer */
	if (sensor->gpio_clk_enable != NULL)
	{
		sensor->gpio_clk_enable();
	}

	// Initialize the GPIO pin for the sensor
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = sensor->gpio_pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(sensor->gpio_port, &GPIO_InitStruct);

	// Initialize the timer for the sensor
	if (DS18B20_Timer_Init(sensor) != 0)
	{
		result = ERROR_OTHER;
	}

	return result;
}

/* Timer init function */
error_t DS18B20_Timer_Init(DS18B20_t *sensor)
{
	error_t result = OK;

	// Vérifier que le timer_instance n'est pas NULL
	if (sensor->timer_instance == NULL)
	{
		result = NULL_POINTER; // Error: Invalid timer instance
	}

	/* Enable timer clock using the provided function pointer */
	if (sensor->timer_clk_enable != NULL)
	{
		sensor->timer_clk_enable();
	}

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	RCC_ClkInitTypeDef clkconfig = {0};
	uint32_t pFLatency = 0u;
	HAL_RCC_GetClockConfig(&clkconfig, &pFLatency);

	// Get APB1 prescaler, because ABP1 timers clock is either
	// - Equal to APB1 peripheral clock if the prescaler equals 1
	// - Equal to 2 x APB1 peripheral clock if the prescaler is greater than 1
	uint32_t APB1_prescaler = clkconfig.APB1CLKDivider;
	uint32_t APB1_timers_clock = 0u;
	if (APB1_prescaler == RCC_HCLK_DIV1)
	{
		APB1_timers_clock = HAL_RCC_GetPCLK1Freq();
	}
	else
	{
		APB1_timers_clock = 2UL * HAL_RCC_GetPCLK1Freq();
	}

	// Get prescaler for 1MHz
	uint32_t prescaler = (uint32_t)((APB1_timers_clock / 1000000u) - 1u);

	sensor->htim.Instance = sensor->timer_instance;
	sensor->htim.Init.Prescaler = prescaler;
	sensor->htim.Init.Period = -1U; // Max value (we don't care because not a periodic timer)
	sensor->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	sensor->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&sensor->htim) != HAL_OK)
	{
		result = ERROR_OTHER; // Error
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&sensor->htim, &sClockSourceConfig) != HAL_OK)
	{
		result = ERROR_OTHER; // Error
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&sensor->htim, &sMasterConfig) != HAL_OK)
	{
		result = ERROR_OTHER; // Error
	}

	if (HAL_TIM_Base_Start(&sensor->htim) != HAL_OK) // Start timer
	{
		result = ERROR_OTHER; // Error
	}

	return result;
}

/* Count us microseconds */
void DS18B20_delay(DS18B20_t *sensor, uint16_t us)
{

	__HAL_TIM_SET_COUNTER(&sensor->htim, 0); // set the counter value at 0
	while (__HAL_TIM_GET_COUNTER(&sensor->htim) < us)
		; // wait for the counter to reach the us input in the parameter
}

/* Transaction initialization sequence of the DS18B20 */
uint8_t DS18B20_Start(DS18B20_t *sensor)
{
	uint8_t response = 0;

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_RESET); // pull the pin low
	DS18B20_delay(sensor, 480);												// wait at least 480µs low according to datasheet

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_SET); // release the pin

	DS18B20_delay(sensor, 80); // wait 80µs to receive presence pulse according to datasheet
							   // 80µs = 60µs max for transition + 20µs to receive presence pulse

	if (!(HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin)))
	{ // check if pin is low

		response = 1; // if the pin is low i.e the presence pulse is detected
	}
	else
	{
		response = 0;
	}

	DS18B20_delay(sensor, 400); // at least 480 us DS18B20_delay totally, according to datasheet

	return response;
}

/* DS18B20_read a bit from the sensor */
uint8_t DS18B20_read(DS18B20_t *sensor)
{

	uint8_t response = 0;

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_RESET); // set pin low
	DS18B20_delay(sensor, 3);												// wait at least 1µs low according to datasheet

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_SET); // release the pin

	DS18B20_delay(sensor, 10); // data is valid for 15µs max after master releases the data line
							   // here we wait 8µs before DS18B20_reading data, to remain under the 15µs

	if (HAL_GPIO_ReadPin(sensor->gpio_port, sensor->gpio_pin))
	{					 // if the sensor pulled the line to high
		response = 1; // we DS18B20_read 1, otherwise 0
	}

	DS18B20_delay(sensor, 52); // wait another 50µs to complete the 60µs of a DS18B20_read time slot
							   // and another 2µs to ensure the minimum 1µs recovery between DS18B20_read slots
	return response;
}

/* DS18B20_read a byte from the sensor */
uint8_t DS18B20_readByte(DS18B20_t *sensor)
{

	uint8_t value = 0;

	for (uint8_t i = 0; i < 8; i++)
	{

		uint8_t bit = DS18B20_read(sensor); // DS18B20_read a bit
		value |= bit << i;
	}

	return value;
}

/* Write a 0 to the sensor */
uint8_t DS18B20_write0(DS18B20_t *sensor)
{

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_RESET); // pull the pin low

	DS18B20_delay(sensor, 60); // wait at least 60µs low according to datasheet to write 0

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_SET); // release the pin

	DS18B20_delay(sensor, 5); // wait another 1µs to for recovery between write time slots

	return 0; // OK
}

/* Write a 1 to the sensor */
uint8_t DS18B20_write1(DS18B20_t *sensor)
{

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_RESET); // pull the pin low

	DS18B20_delay(sensor, 5); // wait less than 15µs low according to datasheet to write 1

	HAL_GPIO_WritePin(sensor->gpio_port, sensor->gpio_pin, GPIO_PIN_SET); // release the pin

	DS18B20_delay(sensor, 60); // wait another 60µs to wait 65µs totally, according to datasheet (write time slot + recovery)

	return 0; // OK
}

/* Write a byte to the sensor*/
uint8_t DS18B20_writeData(DS18B20_t *sensor, uint8_t data)
{

	for (uint8_t i = 0; i < 8; i++)
	{

		if ((data & (1 << i)) == (1 << i))
		{ // if the bit is high

			DS18B20_write1(sensor); // write 1
		}
		else
		{

			DS18B20_write0(sensor); // write 0 otherwise
		}
	}

	return 0; // OK
}

/* Reset a search state for use in a search */
uint8_t onewireSearchInit(onewire_search_state_t *state)
{
	state->last_zero_branch = -1;
	state->done = false;

	// Zero-fill the address
	for (int i = 0; i < 8; i++)
	{

		state->address[i] = 0;
	}

	return 0; // OK
}

uint8_t searchNext(DS18B20_t *sensor, onewire_search_state_t *state)
{
	// States of ROM search DS18B20_reads
	enum
	{
		kConflict = 0x00,
		kZero = 0x02,
		kOne = 0x01,
	};

	// Value to write to the current position
	uint8_t bitValue = 0;

	// Keep track of the last zero branch within this search
	// If this value is not updated, the search is complete
	int8_t locallast_zero_branch = -1;

	for (uint8_t bitPosition = 0; bitPosition < 64; bitPosition++)
	{

		// Calculate bitPosition as an index in the address array
		// This is written as-is for DS18B20_readability. Compilers should reduce this to bit shifts and tests
		uint8_t byteIndex = bitPosition / 8;
		uint8_t bitIndex = bitPosition % 8;

		// DS18B20_read the current bit and its complement from the bus
		uint8_t DS18B20_reading = 0;
		DS18B20_reading |= DS18B20_read(sensor);	  // Bit
		DS18B20_reading |= DS18B20_read(sensor) << 1; // Complement of bit (negated)

		switch (DS18B20_reading)
		{
		case kZero:
		case kOne:
			// Bit was the same on all responding devices: it is a known value
			// The first bit is the value we want to write (rather than its complement)
			bitValue = (DS18B20_reading & 0x1);
			break;

		case kConflict:
			// Both 0 and 1 were written to the bus
			// Use the search state to continue walking through devices
			if (bitPosition == state->last_zero_branch)
			{
				// Current bit is the last position the previous search chose a zero: send one
				bitValue = 1;
			}
			else if (bitPosition < state->last_zero_branch)
			{
				// Before the last_zero_branch position, repeat the same choices as the previous search
				bitValue = state->address[byteIndex] & (1 << bitIndex);
			}
			else
			{
				// Current bit is past the last_zero_branch in the previous search: send zero
				bitValue = 0;
			}

			// Remember the last branch where a zero was written for the next search
			if (bitValue == 0)
			{
				locallast_zero_branch = bitPosition;
			}

			break;

		default:
			// If we see "11" there was a problem on the bus (no devices pulled it low)
			return false;
		}

		// Write bit into address
		if (bitValue == 0)
		{
			state->address[byteIndex] &= ~(1 << bitIndex);
		}
		else
		{
			state->address[byteIndex] |= (bitValue << bitIndex);
		}

		// Write bit to the bus to continue the search
		if (bitValue == 0)
		{
			DS18B20_write0(sensor);
		}
		else
		{
			DS18B20_write1(sensor);
		}
	}

	// If the no branch points were found, mark the search as done.
	// Otherwise, mark the last zero branch we found for the next search
	if (locallast_zero_branch == -1)
	{
		state->done = true;
	}
	else
	{
		state->last_zero_branch = locallast_zero_branch;
	}

	// DS18B20_read a whole address - return OK
	return 0;
}

/* Search for devices on the 1-Wire bus */
uint8_t searchDevices(DS18B20_t *sensor, uint8_t command, onewire_search_state_t *state)
{
	uint8_t result = 0;
	// Bail out if the previous search was the end or if no sensor on the bus
	if ( (state->done) || (DS18B20_Start(sensor) == 0) )
	{
		result = 1;
	}
	else
	{
		DS18B20_writeData(sensor, command);
		result = searchNext(sensor, state);
	}

	return result;	// returns 0 if OK, 1 otherwise
}

/* Generate CRC for a given input byte */
uint8_t crcGenerator(uint8_t initial_crc, uint8_t input)
{

	// Input is supposed to be a Byte where the most left bit is MSB (input not reversed).

	// 1. Calculate the CRC of data 0x00 to 0xFF and store them into one array in order.
	const uint8_t crc8_table[256] = {
		0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
		0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
		0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
		0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
		0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
		0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
		0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
		0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
		0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
		0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
		0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
		0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
		0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
		0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
		0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
		0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35};

	// 2. Do one XOR operation with the input data byte and the initial byte.
	uint8_t index = input ^ initial_crc;

	// 3. Use the calculated result in above second step as the array index to retrieve
	// its CRC value from the CRC array built in first step.

	uint8_t final_crc = crc8_table[index];

	return final_crc;
}

/* Check if the address is valid by computing the CRC */
uint8_t addressValid(const uint8_t ROM_code[])
{
	uint8_t result = 0;

	// Initially, the CRC is equal to 0 according to datasheet
	uint8_t initial_crc = 0x00;

	// Variable to store the computed CRC
	uint8_t computed_crc = 0x00;

	// Compute the CRC by iterating on the CRC computed for each data byte
	for (int i = 0; i < 7; i++)
	{
		computed_crc = crcGenerator(initial_crc, ROM_code[i]);
		initial_crc = computed_crc;
	}

	// Compare computed CRC with the stored one, sent by the sensor.
	if (computed_crc != ROM_code[7])
	{
		result = 1;
	}

	return result;	// returns 0 if OK, 1 otherwise
}

/* Search all sensors on the 1-Wire bus and return a chained list of ther ROM Codes */
//!\ This function uses a chained list to search for all the sensors on the bus when the
//!\ exact number of sensors present is unknown. Though this is an interesting programming
//!\ exercise, it is not recommended to use it in embedded systems applications where memory
//!\ usage are critical and malloc forbidden. What is more, in practice, the number of sensors
//!\ on the bus is known, so no need to use such a functions. Instead, use the function
//!\ DS18B20_Search after this one, which takes into account the (known) number of sensors on the bus.
/* ROM_code_Address_t *DS18B20_Search(DS18B20_t *sensor)
{
	// Searching ROM codes of all sensors is more complicated than just sending
	// the Search ROM Command and wait for responses.
	// It needs to perform a Search ROM Cycle that includes sending and receiving
	// data through a certain process.

	onewire_search_state_t search_state;
	onewireSearchInit(&search_state);

	// We will use a chained list to save all the detected ROM codes.

	// Allocate memory for the first ROM code to be detected.
	ROM_code_Address_t *p_ROM_code_address = malloc(sizeof(ROM_code_Address_t));

	// As this is the first element of the chain, its parent is NULL.
	p_ROM_code_address->p_parent_ROM_code_address = NULL;

	uint8_t count = 0;

	log_ds18b20("Searching devices...\n\r");

	while (searchDevices(sensor, SEARCH_ROM, &search_state))
	{

		// Store the detected ROM code in the field ROM code of the current
		// ROM_code_Address structure.

		for (int i = 0; i < 8; i++)
		{

			p_ROM_code_address->ROM_code[i] = search_state.address[i];
		}

		// Check is the received ROM code is valid

		if (addressValid(p_ROM_code_address->ROM_code))
		{
			// Display through Serial
			log_ds18b20("Received valid ROM code !\n\r");
		}
		else
		{
			// Display through Serial
			log_ds18b20("Received ROM code not valid !\n\r");
		}

		count += 1;

		// Display the detected ROM Code of the sensor through serial.
		log_ds18b20("ROM Code for sensor %u: %x %x %x %x %x %x %x %x\n\r", count,
					search_state.address[0], search_state.address[1], search_state.address[2],
					search_state.address[3], search_state.address[4], search_state.address[5],
					search_state.address[6], search_state.address[7]);

		// Allocate memory for a new ROM_code_Address structure, to prepare
		// detection of another ROM code.
		ROM_code_Address_t *p_new_ROM_code_address = malloc(sizeof(ROM_code_Address_t));

		// Save the memory address of this new structure in the
		// p_next_ROM_code_address of the current ROM_code_Address structure.
		p_new_ROM_code_address->p_parent_ROM_code_address = p_ROM_code_address;

		// Move on to the next memory allocation for the next detected ROM code.
		p_ROM_code_address = p_new_ROM_code_address;
	}

	// Set the last ROM code to 0 to signal the end of the search.
	for (int i = 0; i < 8; i++)
	{

		p_ROM_code_address->ROM_code[i] = 0;
	}

	return p_ROM_code_address; // Return the last Rom Code address
} */

/* Search all sensors on the 1-wire bus */
uint8_t DS18B20_Search(DS18B20_t *sensor, uint64_t ROM_codes_array[])
{
	// Searching ROM codes of all sensors is more complicated than just sending
	// the Search ROM Command and wait for responses.
	// It needs to perform a Search ROM Cycle that includes sending and receiving
	// data through a certain process.

	onewire_search_state_t search_state;
	onewireSearchInit(&search_state);

	// The detected ROM Codes will be stored in an array of uint64_t, provided as argument
	// of the function (ROM_codes_array). Its lenght shall be greater than the number of
	// sensors on the bus.

	uint8_t index = 0;
	uint8_t count = 0;

	log_ds18b20("Searching devices...\n\r");

	while (searchDevices(sensor, SEARCH_ROM, &search_state))
	{

		// Store the detected ROM code in the field ROM code of the current
		// ROM_code_Address structure.

		ROM_codes_array[index] = 0ULL;

		for (int i = 0; i < 8; i++)
		{
			// ROM codes are stored MSB first
			ROM_codes_array[index] |= search_state.address[i] << 8*(7-i);
		}

		// Check is the received ROM code is valid
		if (addressValid(&ROM_codes_array[index]))
		{
			// Display through Serial
			log_ds18b20("Received valid ROM code !\n\r");
		}
		else
		{
			// Display through Serial
			log_ds18b20("Received ROM code not valid !\n\r");
		}

		count++;

		// Display the detected ROM Code of the sensor through serial.
		log_ds18b20("ROM Code for sensor %u: %x \n\r", count, ROM_codes_array[index]);

		index++;
	}

	return 0; // OK
}

/* Get the temperature of all the detected sensors on the 1-Wire bus */
//!\ This function uses the DS18B20_Search function that uses a chained list.
//!\ Its is not recommended to use it for an embedded system application.
//!\ Use instead the DS18B20_GetTemp function after this one.
/* uint8_t DS18B20_GetTemp(DS18B20_t *sensor, ROM_code_Address_t *p_ROM_code_address, uint16_t *temperature) // p_ROM_code_address is returned by DS18B20_Search.
{

	// We will use the chained list of ROM codes memory addresses structures,
	// each of them containing the ROM code of one sensor and the memory address of the
	// parent structure.

	// This chained list is obtained with the function DS18B20_Search.

	// We will ask the temperature to each sensor and display it through Serial
	// if the console is enabled.

	uint8_t ROM_code_byte = {0};
	uint8_t count = 0;

	// We will go through the chained list, from bottom to top, get the ROM Codes
	// and ask each sensor to send temperature data.
	while (p_ROM_code_address->p_parent_ROM_code_address != NULL)
	{

		// Step up to the last ROM_code_Address structure with a valid ROM Code.
		p_ROM_code_address = p_ROM_code_address->p_parent_ROM_code_address;

		DS18B20_Start(sensor);				  // Initiate the transaction on the 1-Wire bus
											  // sensor can be any of the sensors of the bus
		DS18B20_writeData(sensor, MATCH_ROM); // Match ROM

		for (int i = 0; i < 8; i++)
		{ // Send the ROM Code

			ROM_code_byte = p_ROM_code_address->ROM_code[i];
			DS18B20_writeData(sensor, ROM_code_byte);
		}

		DS18B20_writeData(sensor, CONVERT_T); // Temperature conversion

		DS18B20_Start(sensor);				  // Initiate the transaction
		DS18B20_writeData(sensor, MATCH_ROM); // Match ROM

		for (int i = 0; i < 8; i++)
		{ // Send the ROM Code

			ROM_code_byte = p_ROM_code_address->ROM_code[i];
			DS18B20_writeData(sensor, ROM_code_byte);
		}

		DS18B20_writeData(sensor, READ_SCRATCHPAD); // Read data

		uint8_t Temperature_byte_1 = DS18B20_readByte(sensor); // First byte of data
		uint8_t Temperature_byte_2 = DS18B20_readByte(sensor); // Second byte of data

		// Temperature data is made of the concatenation of the two data bytes.
		// The sensor sends the data LSB first.
		// For DS18S20+, the point is after the Bit 1,
		// we therefore have to divide the 16-bit result by 2^1 = 2.
		uint16_t Temperature = (((Temperature_byte_2 << 8)) | Temperature_byte_1) >> 4;
		temperature[count] = Temperature;

		count += 1; // Increment the sensor count

		// Display the temperature of the sensor through Serial
		log_ds18b20("Temperature of sensor %i: %d\n\r", count, Temperature);
	}

	return 0; // OK
} */

/* Get the temperature of all the detected sensors on the 1-Wire bus */
uint8_t DS18B20_GetTemp(DS18B20_t *sensor, const uint64_t ROM_codes_array[], uint16_t *temperature)
{

	// We will ask the temperature to each sensor and display it through Serial
	// if the console is enabled.

	uint8_t ROM_code_byte = 0;
	uint8_t count = 0;

	// We will go through the array of ROM Codes
	while (ROM_codes_array[count] != 0)
	{
		DS18B20_Start(sensor);				  // Initiate the transaction on the 1-Wire bus
											  // sensor can be any of the sensors of the bus
		DS18B20_writeData(sensor, MATCH_ROM); // Match ROM

		for (uint8_t i = 0; i < 8; i++)
		{ // Send the ROM Code MSB first
			ROM_code_byte = ( ROM_codes_array[count] >> 8*(7-i) ) & 0xFF;
			DS18B20_writeData(sensor, ROM_code_byte);
		}

		DS18B20_writeData(sensor, CONVERT_T); // Temperature conversion

		DS18B20_Start(sensor);				  // Initiate the transaction
		DS18B20_writeData(sensor, MATCH_ROM); // Match ROM

		for (uint8_t i = 0; i < 8; i++)
		{ // Send the ROM Code MSB first again

			ROM_code_byte = ( ROM_codes_array[count] >> 8*(7-i) ) & 0xFF;
			DS18B20_writeData(sensor, ROM_code_byte);
		}

		DS18B20_writeData(sensor, READ_SCRATCHPAD); // Read data

		uint8_t Temperature_byte_1 = DS18B20_readByte(sensor); // First byte of data
		uint8_t Temperature_byte_2 = DS18B20_readByte(sensor); // Second byte of data

		// Temperature data is made of the concatenation of the two data bytes.
		// The sensor sends the data LSB first.
		// For DS18S20+, the point is after the Bit 1,
		// we therefore have to divide the 16-bit result by 2^1 = 2.
		uint16_t Temperature = (((Temperature_byte_2 << 8)) | Temperature_byte_1) >> 4;
		temperature[count] = Temperature;

		count += 1; // Increment the sensor count

		// Display the temperature of the sensor through Serial
		log_ds18b20("Temperature of sensor %i: %d\n\r", count, Temperature);
	}

	return 0; // OK
}

/********************************** END OF FILE ******************************************** */
