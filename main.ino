/*
 * 7-Segment Display
 * Datasheet: https://www.mouser.com/datasheet/2/143/ELF511GWA2-1166007.pdf
 * 
 * // datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 */
#include <avr/interrupt.h>
#include <avr/io.h>

#define DEADBAND_VARIANCE_NEAR  100 //ask Angel about what this value should be
#define DEADBAND_VARIANCE_FAR   300 //ask Angel about what this value should be
#define PORTD_DEFAULT   B00000011
#define PORTB_DEFAULT   B11000000
#define DP      10
/* Pin Connections */
#define SEG_A   PD2
#define SEG_B   PD3
#define SEG_C   PD4
#define SEG_D   PD5
#define SEG_E   PD6
#define SEG_F   PD7
#define SEG_G   PB0
#define SEG_DP  PB1
#define SEG_D1  PB2
#define SEG_D2  PB3
#define SEG_D3  PB4
#define SEG_D4  PB5
#define RLED    PC0     //Error: THE ORDERING OF THESE 3 IS INCORRECT
#define GLED    PC1
#define YLED    PC2

/* analogVal is stored as a value 0-5000 representing 0-5V */
volatile uint32_t sensor_val;
volatile uint8_t ISR_generated = 0;
uint16_t sensor_val_sum = 0;
uint16_t sensor_val_avg = 0;
volatile uint8_t counter = 0;
const uint8_t sensor_arr_length = 8;
uint16_t sensor_val_arr[sensor_arr_length] = {};
enum digits {digit1, digit2, digit3, digit4};

/* 
 * Displays numbers on LED 7-Segment Display.
 * Parameters: number - 0-9 or DP. Pace: 0-3 or digit1, digit2, digit3, digit4
 */
void display_seven_segment(unsigned char number, unsigned char place) {
    PORTD &= PORTD_DEFAULT;
    PORTB &= PORTB_DEFAULT;
    PORTB |= (1 << place + 2) | (place == digit1 ? 1 << SEG_DP : 0); 
  
    /* Segments A-F are on D and G-D4 are on B */
    switch(number) {
        case 0:
            PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_C) | (1<<SEG_D) | (1<<SEG_E) | (1<<SEG_F);
            break;
        case 1:
            PORTD |= (1<<SEG_B) | (1<<SEG_C);
            break;
        case 2:
            PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_D) | (1<<SEG_E);
            PORTB |= (1<<SEG_F) | (1<<SEG_G);
            break;
        case 3:
            PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_C) | (1<<SEG_D);
            PORTB |= (1<<SEG_G);
            break;
        case 4:
            PORTD |= (1<<SEG_B) | (1<<SEG_C) | (1<<SEG_F);
            PORTB |= (1<<SEG_G);
            break;
        case 5:
            PORTD |= (1<<SEG_A) | (1<<SEG_C) | (1<<SEG_D) | (1<<SEG_F);
            PORTB |= (1<<SEG_G);
            break;
        case 6:
            PORTD |= (1<<SEG_A) | (1<<SEG_C) | (1<<SEG_D) | (1<<SEG_E) | (1<<SEG_F);
            PORTB |= (1<<SEG_G);
            break;
        case 7:
            PORTD |= PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_C);
            break;
        case 8:
            PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_C) | (1<<SEG_D) | (1<<SEG_E) | (1<<SEG_F);
            PORTB |= (1<<SEG_G);
            break;
        case 9:
            PORTD |= (1<<SEG_A) | (1<<SEG_B) | (1<<SEG_C) | (1<<SEG_F);
            PORTB |= (1<<SEG_G);
            break;
        case DP:
            PORTD |= 1<<SEG_DP;
            break;
    }
}

/******************************************************************************/

/* 
 * DDR Port Registers
 * https://www.arduino.cc/en/Reference/PortManipulation
 * -15 digital outputs needed. 2 must be analog output pins
 * _D: digital pins 0-7. bits 0/1 TX, RX. Do not use
 * _B: digital pins 8-15. 2 high bits 6/7 for crystal. do not use
 * _C: analog pins 0-7. analog pins 0-2 used as digital out for this application
 * DDRD/DDRB/DDRC (R/W): port direction register. 1 output, 0 input
 * PORTD/PORTB/PORTC (R/W): port data register. Output. 1-high, 0-low. 0 initial val
 * PIND/PINB/PINC (R): port input pin register. Read only, stores value.
 * 
 * 
 * ADC - 10 bit
 * Example code (with ISR): https://forum.arduino.cc/index.php?topic=43169.0
 * Startup:
 *      1. Write 0 to ADC power reduction bit PRADC 
 *      2. ADEN bit on ADCSRA to 1 (0 to turn off)
 *      3. Write 1 to ADSC bit. Write to 1 after every conversion cycle
 *      4. ADIF (bit?) raised high when conversion is complete
 * ADMUX - selects analog input channel
 *      -bit 7:6 choose Vref. 00-AREF, internal Vref off. 01-AVcc at AREF pins
 *       11 internal 1.1Vref. 01 fits arduino layout
 *      -bit 5: 
 *      -bit 4: reserved
 *      -bit 3:0 Channel select - 0000-0111 for ADC0-ADC7. 1000 ADC8/temp sensor
 * ADCSRA
 *      -bit 7: ADEN ADC enable. 1 to enable. 0 to stop
 *      -bit 6: ADSC ADC start conversion
 *      -bit 5: auto trigger enable
 *      -bit 4: ADIF ADC interrupt flag (R/W but stick to read)
 *      -bit 3: ADC interrupt enable
 *      -bit 2:0: determine division factor between clk freq and ADC. division
 *       factor: 2^(bits), 2^0->2 not 1
 * ADCL/ADCH
 *      -read ADCL before ADCH
 *      -10-bit
 *      -ADLAR=0 (use this) right ended, else left ended
 * ADCSRB
 *      -bits 7, 5:3: reserved
 *      -bits 2:0 - ADC trigger sources don't matter if auto-trigger off
 * DIDR0 - digital input disable register 0
 *      -bits 7:6: reserved
 *      -bits 5:0: ADC5...0 input disable [ADC0-2 needed as digital inputs]
 *       writing to 1 disables the digital input buffer
 * 
 * If ADATE and ADEN are 1, an interrupt can occur at any time
 * ADC complete interrupt generated if ADIE and the I-bit are set
 */
void setup() {
    Serial.begin(9600);
    /* Digital Input Pin Configuration */
    DDRD |= B11111100;
    DDRB |= B00111111;
    DDRC  = B00000111; //LEDs
    
    /* ADC Configuration */
    PRR     &= B11111110; //Bit 0: ADC Power reduction
    ADMUX   &= B11011111; //Set ADLAR to 0 to right adjust result. ADCL lower 8 bits, ADCH higher 2
    ADMUX   |= B01000000; //Set bits 6-7 to set reference voltage to gnd (01)
    ADMUX   &= B11110000; //clear its 0-3 to set analog input
    ADMUX   |= B00000111; //set 0-3 to 0111 to use ADC7
    ADCSRA  |= B10000000; //Set ADEN to enable ADC. Takes 12 ADC clock cycles
    //ADCSRA  |= B00100000; //set ADATE in to enable auto triggering. should i stop this?
    ADCSRB  &= B11111000; //clear ADTS2..0 in ADCSRB to set trigger mode to free running
    ADCSRA  |= B00000111; //sets ADC prescaler to 128 - 16000KHz/128=125KHz
    ADCSRA  |= B00001000; //sets ADIE to enable ADC interrupt
#ifndef Arduino_h
    sei(); //enables global interrupts
#endif
    //start conversion
    ISR_generated = 0;
    sensor_val = 0;
    ADCSRA  |= B01000000; //set ADSC to 1 to start ADC conversion

    DIDR0   = B00111111; //this could be an issue

    /* Get an initial value to display but get a sum that can be averaged */
    while(counter < sensor_arr_length) {
        if(ISR_generated) {
            sensor_val_sum += sensor_val_arr[counter%sensor_arr_length] = sensor_val;
            counter++;
        }
    }
}

/******************************************************************************/

/*
 * Generated once Analog->Digital Conversion Complete
 */
ISR(ADC_vect) {
    /* Read ADCL before ADCH */
    sensor_val = ((ADCL) | ADCH << 8)/**5000/1024*/; //low must be read first
    
    sensor_val_sum -= sensor_val_arr[counter%sensor_arr_length];
    sensor_val_arr[counter%sensor_arr_length] = sensor_val;
    sensor_val_sum += sensor_val;
    sensor_val_avg = sensor_val_sum/sensor_arr_length;
    counter++;

    ISR_generated = 1;

    ADCSRA |= B01000000; //reset ADSC ADC start conversion bit. not needed bc code is free running
}

/******************************************************************************/
uint8_t iter = 0;
uint8_t loop_counter = 0;
//Do i want to use a moving average or exponential filter on sensor_val
void loop() {
    //if (ISR_generated) {
    //    ISR_generated = 0;
        
        // sensor_val_sum -= sensor_val_arr[counter%sensor_arr_length];
        // sensor_val_arr[counter%sensor_arr_length] = sensor_val;
        // sensor_val_sum += sensor_val;
        // sensor_val_avg = sensor_val_sum/sensor_arr_length;
        // counter++;

        /* turn on the correct LED */
        //could potentially use some clever bit shifts, boolean algebra, and arithmetic to eliminate ternary
        // PORTC = sensor_val > 2500 + DEADBAND_VARIANCE_FAR  && 
        //         sensor_val < 2500 - DEADBAND_VARIANCE_FAR  ? B00000010 : //red
        //         sensor_val > 2500 + DEADBAND_VARIANCE_NEAR && 
        //         sensor_val < 2500 - DEADBAND_VARIANCE_NEAR ? B00000100 : //yellow
        //                                                      B00000001;  //green
        //these 4 lines could be 1 but idk if it would be more efficient or not
    //}
    //ADCSRA &= B10111111; //locks the adc out temporarily
    /* A switch statement should even out the flickering */
    //display_seven_segment(sensor_val_avg%1000/100, digit4);
    //display_seven_segment(sensor_val_avg%10, digit2);
    uint16_t dig4 = sensor_val_avg%10;
    uint16_t dig3 = sensor_val_avg%100/10;
    uint16_t dig2 = sensor_val_avg%1000/100;
    uint16_t dig1 = sensor_val_avg/1000;
    switch(loop_counter%4) {
        case 0:
            display_seven_segment(dig4, digit4);
            break;
        case 1:
            display_seven_segment(dig3, digit3);
            break;
        case 2:
            display_seven_segment(dig2, digit2);
            break;
        case 3:
            display_seven_segment(dig1, digit1);
            break;
    }
    //ADCSRA |= B01000000; //unlocks the ADC
    loop_counter++;
}
