
#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

// SPI Definitions

#define SPI_DDR DDRB
#define SPI_PORT PORTB
#define SPI_MISO PINB4
#define SPI_MOSI PINB3
#define SPI_SCK PINB5
#define SPI_CS1 PIND2
#define SPI_CS2 PIND3

#define VCC 5					// Voltage supplied to ACS712
#define BURDEN_RES 470.0
#define SAMPLES 100				// Number of samples for RMS calculation
#define SAMPLE_DELAY 0.1			// Delay in ms for 100Hz sampling rate (1/100 = 1ms)
#define VREF 5.0			// Reference voltage
#define ADC_MAX 4096			// Maximum ADC value (12-bit resolution)

#define BUFFER_SIZE 16
#define BASE_ADDRESS 0x20

#define BAUD 9600
#define MY_UBRR F_CPU/16/BAUD-1

uint8_t pins[8] = {PIND3, PIND4, PIND5, PIND6, PIND7, PINB0, PINB1, PINB2};
	
	

volatile uint8_t  i2c_buffer[BUFFER_SIZE];
volatile uint8_t received_command = 0;		// Last received I2C command
volatile uint8_t data_ready = 0;			// Data ready flag for I2C
volatile uint8_t byte_count = 0;


void init_pins();
void SPI_init(void);
void I2C_store_data(uint8_t channel, float current);// to store the current into one buffer // so it will easy to transfer current data
void i2c_dynamic_address(void);// dynamic addres setting for the i2c
void I2C_init(void);
//uint16_t MCP3208_read(uint8_t channel);
uint16_t MCP3208_read(uint8_t ADC_select,uint8_t ch);
//float calculate_rms_current(uint8_t channel);
float calculate_rms_current(uint8_t ADC_select,uint8_t channel);
void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
int USART_putchar(char c, FILE *stream);

// Setup stdout to use USART

FILE mystdout = FDEV_SETUP_STREAM(USART_putchar, NULL, _FDEV_SETUP_WRITE);

// Main Function

int main(void) {
	unsigned int ubrr = MY_UBRR;
	USART_Init(ubrr);		// Initialize USART
	stdout = &mystdout;		// Redirect printf to USART
	init_pins();  
	I2C_init();               
	SPI_init();
	
	while (1) {

// 		float rms_current = calculate_rms_current(0);
// 		//rms_current_values[channel] = rms_current;
// 		printf("RMS Current: %.6f A\n", rms_current);
		//_delay_ms(1000); // Delay for stability
		uint8_t index=0;
		for(uint8_t channel=0; channel<8;channel=channel+2)
		{
			float Actual_current= calculate_rms_current(1,channel);
			printf("adc1_current_channel[%d]: %.6f\n",index,Actual_current);
			I2C_store_data(index, Actual_current);
			index++;
		}
		
		 for(uint8_t channel=0; channel<8;channel=channel+2)
		 {
			 float Actual_current= calculate_rms_current(2,channel);
			 printf("adc2_current_channel[%d]: %.6f\n",index,Actual_current);
			 I2C_store_data(index, Actual_current);
			 index++;
		 }
		
	}

	return 0;

}

void init_pins() {

	for (int i = 0; i < 8; i++){
		if (i < 5) {
			DDRD |= (1 << pins[i]);
				PORTD &= ~(1 << pins[i]);
			}
			 else {
			DDRB |= (1 << pins[i]);
				PORTB &= ~(1 << pins[i]);
			}
		}
}


// SPI Initialization

void SPI_init(void) {
	SPI_DDR |= (1 << SPI_MOSI) | (1 << SPI_SCK);				// MOSI, SCK, and CS as output
	SPI_DDR &= ~(1 << SPI_MISO);                                // MISO as input
	DDRD |= (1 << SPI_CS1);	                                    // set as output
	DDRD |= (1 << SPI_CS2);										// Set CS as output on Port D
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR0);              // Enable SPI, set as Master
	PORTD |= (1 << SPI_CS1);                                // set cs high
	PORTD |= (1 << SPI_CS2);                                  // Set CS high
	printf("SPI initialized\n");
}

// MCP3208 Read function

uint16_t MCP3208_read(uint8_t ADC_select,uint8_t ch) {
	uint8_t reg[3] = {0b00000100, 0x00, 0x00}; // MCP3208 command format
	reg[0] |= (0b00000001 & (ch >> 2));  // Set the channel in the first byte
	reg[1] = 0b11000000 & (ch << 6);     // Set the channel in the second byte
	uint8_t buf[3] = {0}; // Buffer to store received data
		if(ADC_select==1){
			PORTD &= ~(1 << SPI_CS1); // Pull CS low

			for (uint8_t i = 0; i < 3; i++) {
				SPDR = reg[i]; // Load data to be sent into SPDR

				// while (!(SPSR & (1 << SPIF))); // Wait for transmission to complete
				uint16_t timeout = 1000;
				while (!(SPSR & (1 << SPIF)) && --timeout);
				if (timeout == 0) {
					printf("SPI timeout at byte %d\n", i);
					PORTD |= (1 << SPI_CS1); // Pull CS high
					return 0;

				}
				buf[i] = SPDR; // Read received data
			}
			
			PORTD |= (1 << SPI_CS1); // Pull CS high
			return ((buf[1] & 0b00001111) << 8) | buf[2];
			
		}
		else{
	       PORTD &= ~(1 << SPI_CS2); // Pull CS low
	       for (uint8_t i = 0; i < 3; i++) {
		SPDR = reg[i]; // Load data to be sent into SPDR

		// while (!(SPSR & (1 << SPIF))); // Wait for transmission to complete
		uint16_t timeout = 1000;
		while (!(SPSR & (1 << SPIF)) && --timeout);
		if (timeout == 0) {
			printf("SPI timeout at byte %d\n", i);
			PORTD |= (1 << SPI_CS2); // Pull CS high
			return 0;

		}
		buf[i] = SPDR; // Read received data
	}
	
	PORTD |= (1 << SPI_CS2); // Pull CS high
	return ((buf[1] & 0b00001111) << 8) | buf[2]; // Combine the high and low bytes
		}

}

// Function to calculate RMS current

float calculate_rms_current(uint8_t ADC_select,uint8_t channel) {
	float sum_of_squares = 0;
	printf("calculate_rms before for\n");
	for (uint8_t i = 0; i < SAMPLES; i++) {
		uint16_t adc_value = MCP3208_read(ADC_select,channel);
		//printf( " the raw value %d",adc_value);
		float v_adc = ((float)(adc_value * VREF) /(float) ADC_MAX);  // Convert ADC to voltage
		//printf( " the VADC value %F", v_adc);
		float irms_Rb=v_adc/BURDEN_RES;
		//printf("v_adc[%d]: %.4f", i, v_adc);
		sum_of_squares += irms_Rb*irms_Rb ;
		//printf("sumof_square %f\n",sum_of_squares);
		_delay_ms(SAMPLE_DELAY);                      
	}

	float irms_A=sqrt(sum_of_squares / SAMPLES);
	printf(" irms %f\n",irms_A);
	float Actual_current =irms_A*1500;
	// return irms_A;
	return Actual_current;
}

void I2C_store_data(uint8_t channel, float rms_current) {
	printf("I2C_store_data begin\n");
	uint8_t offset = channel * 2;											// Each channel uses 2 bytes and offset mainly give the position of storeing data into buffer
	i2c_buffer[offset] = (uint8_t)((uint16_t)(rms_current * 100) & 0xFF);       // LSB store into buffer
	printf("the stored data %d\n",i2c_buffer[offset] );
	i2c_buffer[offset + 1] = (uint8_t)((uint16_t)(rms_current * 100) >> 8);		// MSB store into buffer
	printf("the stored data %d\n",i2c_buffer[offset+1] );
	//printf("I2C_store_data done\n");
}

//i2c interrupt communication
void i2c_dynamic_address(void)
{
	DDRC &= ~((1<<PINC2) | (1<<PINC3));// pin set as input  (for address setting in i2c)
	PORTC |= (1 << PINC2) | (1 << PINC3);// pull up

	uint8_t jumper_state = ((PINC & (1 << PINC2)) >> PINC2) << 1;  // Read PC2
	jumper_state |= (PINC & (1 << PINC3)) >> PINC3;                // Read PC3
	
	//uint8_t dip_switch = ((PINC & (1<<PINC2)) >> PINC2) | ((PINC & (1<<PINC3)) >> (PINC3 - 1));

	uint8_t slave_address = BASE_ADDRESS + jumper_state;
	//printf("slave addr: %d", slave_address);
	TWAR =(slave_address)<<1;
	
}


void I2C_init(void)
{
	i2c_dynamic_address();
	TWCR = (1 << TWEN) |			// Enable TWI (I2C)
	(1 << TWEA) |			// Enable acknowledgment
	(1 << TWIE);				// Enable interrupt
	sei();							// Enable global interrupts
	printf("I2c initialized\n");
}


ISR(TWI_vect) {
	static uint8_t response_buffer[16]; // Buffer for all 16 bytes
	static uint8_t byte_index = 0; // Tracks the byte being sent
	static uint8_t bytes_to_send = 2; // Default number of bytes to send
	
	switch (TWSR & 0xF8)
	{
		case 0x60: // Own SLA+W received, ACK returned
		case 0x68: // Arbitration lost, SLA+W received
		byte_count = 0;
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;
		
		case 0x80: // Data byte received, ACK returned
		if (byte_count == 0)
		{
			data_ready = 1;
			received_command = TWDR;
		}
		byte_count++;
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;
		
		case 0xA8: // SLA+R received, ACK returned
		if (data_ready==1)
		{
			switch (received_command)
			{
				case 0x01: // Send all 16 bytes
				for (uint8_t i = 0; i < 16; i++)
				{
					response_buffer[i] = i2c_buffer[i];
				}
				bytes_to_send = 16;
				break;

				default: // Unknown command
				response_buffer[0] = 0xFF;
				bytes_to_send = 1;
				break;
			}

			byte_index = 0;
			TWDR = response_buffer[byte_index++]; // Send first byte
		}
		else
		{
			TWDR = 0xFF; // If no valid data, send default response
		}
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;

		case 0xB8: // Data byte transmitted, ACK received (Master wants more data)
		if (byte_index < bytes_to_send)
		{
			TWDR = response_buffer[byte_index++]; // Send next byte
		}
		else
		{
			TWDR = 0xFF; // If master requests extra bytes, send default
		}
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;

		case 0xC0: // Data transmitted, NACK received (Master does not want more data)
		case 0xC8: // Last data byte transmitted, ACK received
		data_ready = 0; // Reset data_ready flag after transmission
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;

		default: // Handle unexpected states
		// printf("Unexpected I2C status: 0x%02X\n", TWSR & 0xF8);
		TWCR = (1 << TWEN) | (1 << TWEA) | (1 << TWINT) | (1 << TWIE);
		break;
	}
}
// Initialize USART for communication

void USART_Init(unsigned int ubrr) {
	// Set baud rate
	UBRR0H = (unsigned char)(ubrr >> 8);  // Set high byte of UBRR
	UBRR0L = (unsigned char)ubrr;         // Set low byte of UBRR
	// Enable transmitter
	UCSR0B = (1 << TXEN0);
	// Set frame format: 8 data bits, 1 stop bit
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// Function to transmit data via USART

void USART_Transmit(unsigned char data) {
	while (!(UCSR0A & (1 << UDRE0)));  // Wait for the transmit buffer to be empty
	UDR0 = data;  // Put data into the transmit buffer, sending the data
}

// Redirect printf to USART

int USART_putchar(char c, FILE *stream) {
	if (c == '\n') {
		USART_Transmit('\r');  // Send a carriage return before newline
	}

	USART_Transmit(c);  // Send the character
	return 0;
}
