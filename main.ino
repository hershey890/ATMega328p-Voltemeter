/*
 * 7-Segment Display
 * Datasheet: https://www.mouser.com/datasheet/2/143/ELF511GWA2-1166007.pdf
 * 
 * // datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 */

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
#define SEG_D4  PD5
#define RLED    PC0     //Error: THE ORDERING OF THESE 3 IS INCORRECT
#define GLED    PC1
#define YLED    PC2

/* sensor_val is stored as a value 0-5000 representing 0-5V */
volatile uint32_t sensor_val = 0;
volatile bool ISR_generated = false;
volatile unsigned char counter = 0;
enum digits {digit1, digit2, digit3, digit4};

/* 
 * Displays numbers on LED 7-Segment Display.
 * Parameters: number - 0-9 or DP. Pace: 0-3 or digit1, digit2, digit3, digit4
 */
void display_seven_segment(unsigned char number, unsigned char place) {
    PORTD &= PORTD_DEFAULT;
    PORTB &= PORTB_DEFAULT;
    PORTB |= 1 << place + 3;

    /*switch(place) {
        case digit1: 
            //turn off DP2, 3, 4
            PORTD &= B00000011;
            PORTB &= B11000000;
            //turn on DP1.
            PORTB |=  B00000100; //D10
            break;
        case digit2: 
            //turn off DP1, 3, 4
            PORTD &= B00000011;
            PORTB &= B11000000;
            //turn on DP2
            PORTB |= B00001000; //D11
            break;
        case digit3: 
            //turn off DP1, 2, 4
            PORTD &= B00000011;
            PORTB &= B11000000;
            //turn on DP3
            PORTB |= B00010000; //D12
            break;
        case digit4: 
            //turn off DP1, 2, 3
            PORTD &= B00000011;
            PORTB &= B11000000;
            //turn on DP4
            PORTB |= B00100000; //D13
            break;
    }*/

    switch(number) {
        case 0:
            PORTD |= () | () | () | () | () | ();
            PORTB |= B00______;
            break;
        case 1:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 2:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 3:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 4:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 5:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 6:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 7:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 8:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case 9:
            PORTD |= B______00;
            PORTB |= B00______;
            break;
        case DP:
            PORTD |= 1 << PD7;
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
    /* Digital Input Pin Configuration */
    DDRD |= B11111100;
    DDRB |= B00111111;
    DDRC  = B00000111; //LEDs
    
    /* ADC Configuration */
    // ADMUX  = B01100111; //ADC7
    // ADCSRB = B00000000;
    // ADCSRA = B11001111;
    // DIDR0  = B00111111;
}

/******************************************************************************/

/*
 * Generated once Analog->Digital Conversion Complete
 */
// ISR(ADC_vect) {
//     /* Only respond to updates to Channel ADC7 */
//     if (ADMUX & B00000111 && B00000111) {
//         sensor_val = (ADCH << 8 | ADCL)*5000/1024;
//         ADCSRA |= B01000000; /* Reset ADC start conversion bit*/
//         ISR_generated = true;
//     }
// }

/******************************************************************************/

//Do i want to use a moving average or exponential filter on sensor_val
void loop() {
    // if (ISR_generated) {
    //     ISR_generated = false;
    //     /* turn on the correct LED */
    //     //could potentially use some clever bit shifts, boolean algebra, and arithmetic to eliminate ternary
    //     PORTC = sensor_val > 2500 + DEADBAND_VARIANCE_FAR  && 
    //             sensor_val < 2500 - DEADBAND_VARIANCE_FAR  ? B00000001 : //red
    //             sensor_val > 2500 + DEADBAND_VARIANCE_NEAR && 
    //             sensor_val < 2500 - DEADBAND_VARIANCE_NEAR ? B00000010 : //yellow
    //                                                          B00000100;  //green
    // }
    sensor_val = 4888; //temporary
    //these 4 lines could be 1 but idk if it would be more efficient or not
    display_seven_segment(sensor_val%10, digit1); //digit 1
    display_seven_segment(sensor_val%100/10, digit2); //digit 2
    display_seven_segment(sensor_val%1000/100, digit3); //digit 3
    display_seven_segment(sensor_val/1000, digit4); //digit 4

    counter = counter > 3 ? 0 : counter++;
}