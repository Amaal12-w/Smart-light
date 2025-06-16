
#define F_CPU 16000000UL   // Define clock frequency as 16 MHz

#include <avr/io.h>        // AVR input/output definitions
#include <avr/interrupt.h> // AVR interrupt support
#include <util/delay.h>    // For _delay_ms and other delay functions
#include <stdlib.h>        // For atoi and itoa
#include <stdbool.h>       // For boolean types

#define LED_PIN PD6          // Define PD6 (OC0A) as LED output pin
#define BUTTON_PIN PD2       // Define PD2 (INT0) as button input pin
#define POT_CHANNEL 0        // Define analog channel 0 (ADC0) for potentiometer

volatile uint8_t mode = 0;           // Operating mode flag: 0 = Automatic, 1 = Manual
volatile uint8_t brightness = 0;     // Brightness level (0-100%)

volatile uint32_t last_interrupt_time = 0;  // Stores last interrupt time for debouncing

// UART initialization
void UART_init() {
	UBRR0H = 0;                         // Set high byte of baud rate to 0
	UBRR0L = 103;                       // Set low byte of baud rate for 9600 bps with 16MHz clock
	UCSR0B = (1 << RXEN0) | (1 << TXEN0); // Enable RX and TX
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // Set data frame: 8 data bits, 1 stop bit
}

// Send single character
void UART_sendChar(char data) {
	while (!(UCSR0A & (1 << UDRE0))); // Wait until data register is empty
	UDR0 = data;                      // Send character
}

// Send string
void UART_sendString(const char* str) {
	while (*str) {                    // Loop until null terminator
		UART_sendChar(*str++);       // Send each character
	}
}

// Receive ASCII string until '\n'
void UART_receiveString(char* buffer, uint8_t maxLength) {
	uint8_t i = 0;
	char c;

	while (1) {
		while (!(UCSR0A & (1 << RXC0))); // Wait for character to be received
		c = UDR0;                        // Read received character

		if (c == '\n') {                // If newline, finish input
			break;
			} else if (c == '\r') {
			continue;                   // Ignore carriage return
		}

		if (i < maxLength - 1) {         // Check buffer limit
			buffer[i++] = c;            // Store character
			UART_sendChar(c);           // Echo back character
		}
	}
	buffer[i] = '\0';                // Null terminate the string
	UART_sendString("\r\n");         // Echo newline
}

// ADC initialization
void ADC_init() {
	ADMUX = (1 << REFS0);                         // AVcc as reference voltage
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // Enable ADC, set prescaler to 64
}

// ADC read
uint16_t ADC_read() {
	ADMUX = (ADMUX & 0xF8) | POT_CHANNEL;         // Select ADC channel 0
	ADCSRA |= (1 << ADSC);                        // Start conversion
	while (!(ADCSRA & (1 << ADIF)));              // Wait until conversion complete
	return ADC;                                   // Return ADC value
}

// Map ADC value to 0–100
uint8_t map_ADC_to_percentage(uint16_t value) {
	return (value * 100UL) / 1023;                // Scale ADC 0-1023 to 0-100%
}

// PWM setup on OC0A (PD6)
void init_PWM() {
	DDRD |= (1 << LED_PIN);                            // Set PD6 as output
	TCCR0A = (1 << WGM01) | (1 << WGM00) | (1 << COM0A1); // Fast PWM mode, non-inverting
	TCCR0B = (1 << CS00);                              // No prescaler
}

void set_PWM(uint8_t percent) {
	if (percent == 0) {
		TCCR0A &= ~(1 << COM0A1);          // Disable PWM output
		PORTD &= ~(1 << LED_PIN);          // Set LED low
		} else if (percent > 100) {
		TCCR0A |= (1 << COM0A1);           // Enable PWM
		OCR0A = 255;                       // Max duty cycle
		} else {
		TCCR0A |= (1 << COM0A1);           // Enable PWM
		OCR0A = (percent * 255UL) / 100;   // Map 0-100% to 0-255
	}
}

// Toggle mode and display
void toggleMode() {
	mode = !mode;                             // Toggle between automatic and manual
	if (mode == 0) {
		UART_sendString("Switched to Automatic Mode\r\n");
		PORTB = 0x01;                         // Indicate mode with LED on PB0
		} else {
		UART_sendString("Switched to Manual Mode\r\n");
		PORTB = 0x02;                         // Indicate mode with LED on PB1
	}
}

// External interrupt (INT0) for button press with debounce
ISR(INT0_vect) {
	uint32_t current_time = TCNT1;                        // Read current timer1 value
	if ((current_time - last_interrupt_time) > 10000) {   // Debounce delay (~10ms)
		toggleMode();
		last_interrupt_time = current_time;             // Update last interrupt time
	}
}

// Initialize button and INT0
void Button_init() {
	DDRB = 0xFF;                            // Set PORTB as output (for mode indicators)
	DDRD &= ~(1 << BUTTON_PIN);            // Set PD2 as input (button)
	PORTD |= (1 << BUTTON_PIN);            // Enable pull-up resistor on PD2

	EICRA |= (1 << ISC01);                 // Trigger INT0 on falling edge
	EIMSK |= (1 << INT0);                  // Enable INT0

	TCCR1B |= (1 << CS11);                 // Start Timer1 with prescaler 8
	sei();                                 // Enable global interrupts
}

// Main function
int main(void) {
	_delay_ms(500);                                 // Small delay for Serial Monitor

	UART_init();                                    // Initialize UART
	ADC_init();                                     // Initialize ADC
	init_PWM();                                     // Initialize PWM
	Button_init();                                  // Initialize button and INT0

	PORTB = 0x01;                                   // Default to automatic mode indicator
	UART_sendString("System Initialized in Automatic Mode\r\n");

	char buffer[8];                                 // Buffer to store UART input

	while (1) {
		if (mode == 0) {
			uint16_t adc = ADC_read();                 // Read potentiometer value
			brightness = map_ADC_to_percentage(adc);   // Convert to percentage
			set_PWM(brightness);                       // Set LED brightness

			itoa(brightness, buffer, 10);              // Convert number to string
			UART_sendString("Brightness set to: ");
			UART_sendString(buffer);
			UART_sendString("%\r\n");
			_delay_ms(100);

			} else {
			if (UCSR0A & (1 << RXC0)) {                // If data available on UART
				UART_receiveString(buffer, sizeof(buffer));
				int val = atoi(buffer);                // Convert string to integer
				if (val >= 0) {
					UART_sendString("You entered: ");
					UART_sendString(buffer);
					UART_sendString("\r\n");

					brightness = val;
					set_PWM(brightness);               // Set LED brightness manually

					UART_sendString("Brightness is now set to: ");
					itoa(brightness, buffer, 10);
					UART_sendString(buffer);
					UART_sendString("%\r\n");
					} else {
					UART_sendString("Invalid input. Enter 0-100.\r\n");
				}
			}
		}
	}
}