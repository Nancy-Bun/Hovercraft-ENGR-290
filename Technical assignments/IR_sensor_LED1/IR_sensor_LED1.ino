#include <avr/io.h>
#include <util/delay.h>
#include <math.h>

#define IR_SENSOR_PIN 0  // ADC0 (A0)
#define LED_PIN PB3      // OC2A (Pin 11)
#define YELLOW_LED_PIN PB5  // 'L' LED (Pin 13)
#define WINDOW_SIZE 7    // Take 5 last raw values for average

// Distance thresholds for LED brightness
const float min_distance = 12.0;  
const float max_distance = 46.0;  

//LED brightness constants
const int min_brightness = 26;    // 10% brightness
const int max_brightness = 255;   // 100% brightness

// Array to store raw values
int rawValues[WINDOW_SIZE];
int valueIndex = 0;

// Function prototypes
void setup(void);
void L_LED(float distance_cm);
int calculateLEDBrightness(float distance);
void initADC(void);
uint16_t readADC(uint8_t channel);
void initPWM(void);
void setPWM(uint8_t duty);
void initUART(void);
void UART_sendChar(char data);
void UART_sendString(const char* str);
void intToString(int value, char* buffer);
void floatToString(float value, char* buffer, int decimalPlaces);

int main(void) {
   setup();

    while (1) {
        // Read ADC value and store in the array
        rawValues[valueIndex] = readADC(IR_SENSOR_PIN);
        valueIndex = (valueIndex + 1) % WINDOW_SIZE;

        // Calculate the average of the last WINDOW_SIZE values
        long sum = 0;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            sum += rawValues[i];
        }
        float avgRawValue = (float)sum / WINDOW_SIZE;

        // Convert raw ADC value to voltage
        float voltage = avgRawValue * (5.0 / 1023.0);

        // Convert voltage to distance (cm)
        float distance_cm = 29.988 * pow(voltage, -1.2134);

        // Calculate LED brightness and set PWM
        int ledBrightness = calculateLEDBrightness(distance_cm);
        setPWM(ledBrightness);

        // Control the yellow LED with blinking logic
        L_LED(distance_cm);

        // Prepare buffers for UART output
        char adcBuffer[10], distanceBuffer[10];
        intToString((int)avgRawValue, adcBuffer);
        floatToString(distance_cm, distanceBuffer, 1);

        // Print ADC value and distance to UART
        UART_sendString("ADC Value: ");
        UART_sendString(adcBuffer);
        UART_sendString(" | Distance: ");
        UART_sendString(distanceBuffer);
        UART_sendString(" cm \r\n");

        _delay_ms(250);  // for stability
    }
}

void setup(void) {
    DDRB |= (1 << LED_PIN) | (1 << YELLOW_LED_PIN);  // Set LED pins as output
    initADC();
    initPWM();
    initUART();
    for (int i = 0; i < WINDOW_SIZE; i++) {
        rawValues[i] = readADC(IR_SENSOR_PIN);
    }
}

void L_LED(float distance_cm) {
    if (distance_cm < min_distance || distance_cm > max_distance) {
        PORTB ^= (1 << YELLOW_LED_PIN);
    } else {
        PORTB &= ~(1 << YELLOW_LED_PIN);
    }
}

int calculateLEDBrightness(float distance) {
    if (distance <= min_distance) 
      distance = max_brightness;
    if (distance >= max_distance) 
      distance = min_brightness;
    else return max_distance - (max_brightness - min_brightness) * (max_distance - distance) / (max_distance - min_distance);
}

void initADC(void) {
    ADMUX = (1 << REFS0);
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t readADC(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

void initPWM(void) {
    DDRB |= (1 << LED_PIN);
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
}

void setPWM(uint8_t duty) {
    OCR2A = duty;
}

void initUART(void) {
    uint16_t ubrr_value = 103;
    UBRR0H = (ubrr_value >> 8);
    UBRR0L = ubrr_value;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void UART_sendChar(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_sendString(const char* str) {
    while (*str) {
        UART_sendChar(*str++);
    }
}

void intToString(int value, char* buffer) {
    char temp[10];
    int i = 0;
    if (value < 0) {
        buffer[i++] = '-';
        value = -value;
    }
    do {
        temp[i++] = (value % 10) + '0';
        value /= 10;
    } while (value > 0);
    int j = 0;
    while (i > 0) {
        buffer[j++] = temp[--i];
    }
    buffer[j] = '\0';
}

void floatToString(float value, char* buffer, int decimalPlaces) {
    int intPart = (int)value;
    float fractionalPart = value - intPart;
    intToString(intPart, buffer);
    while (*buffer) buffer++;
    *buffer++ = '.';
    for (int i = 0; i < decimalPlaces; i++) {
        fractionalPart *= 10;
        int digit = (int)fractionalPart;
        *buffer++ = digit + '0';
        fractionalPart -= digit;
    }
    *buffer = '\0';
}