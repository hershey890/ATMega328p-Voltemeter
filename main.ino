/*
 * 7-Segment Display
 * Datasheet: https://www.mouser.com/datasheet/2/143/ELF511GWA2-1166007.pdf
 * 
 * // datasheet: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 */

//I MAY HAVE TO USE A LOOKUP TABLE TO LINEARIZE THE ADC
#include <avr/interrupt.h>
#include <avr/io.h>

/*
 * DEADBAND_VARIANCE_FAR    - Reading is too far from deadband, show read light
 * DEADBAND_VARIANCE_NEAR   - Reading is close to deadband, show yellow light
 */
#define DEADBAND_VARIANCE_NEAR  10 //ask Angel about what this value should be
#define DEADBAND_VARIANCE_FAR   30 //ask Angel about what this value should be

/*
 * Default values for & masking.
 */
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
#define RLED    PC0
#define YLED    PC1
#define GLED    PC2

volatile uint8_t ISR_generated = 0; /* boolean, stores whether ADC conversion is ready */
volatile uint32_t sensor_val = 0; /* analogVal is stored as a value 0-5000 representing 0-5V */
volatile uint8_t counter = 0;
volatile uint32_t sensor_val_sum = 0;
volatile uint32_t sensor_val_raw = 0;
volatile uint32_t sensor_val_avg = 0;
volatile uint32_t sensor_val_avg_prev = 0;
volatile uint32_t sensor_val_avg_schmitt_out = 0;
volatile uint32_t sensor_val_avg_schmitt_out_prev = 0;
uint16_t output = 0;
const uint8_t sensor_arr_length = 16;
volatile uint16_t sensor_val_arr[sensor_arr_length] = {};
enum digits {digit3, digit4, digit2, digit1}; //digit 1 is unused

// TODO: manually populate table
/* This lookup table isn't written specifically for each individual board.
 * To make a cross-compatible one, design voltage dividers on each unused
 * ADC input, create a table of 7 points from ADC0-6, and then run some kind
 * of regression to fit everything else, and generate the table. Save this
 * table and the fact that the regression is done into the EEPROM. Then
 * recall the data during each setup and store it in SRAM/virtual memory.
 */
uint16_t lookup_table[501] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
    11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 27, 28, 29, 30,
    31, 32, 33, 34, 35, 36, 37, 38, 39, 40,
    41, 42, 43, 44, 45, 46, 47, 48, 49, 50,
    51, 52, 53, 54, 55, 56, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
    71, 72, 73, 74, 75, 76, 77, 78, 79, 80,
    81, 82, 83, 84, 85, 86, 87, 88, 89, 90,
    91, 92, 93, 94, 95, 96, 97, 98, 99, 100,
    101, 102, 103, 104, 105, 106, 107, 108, 109, 110,
    111, 112, 113, 114, 115, 116, 117, 118, 119, 120,
    121, 122, 123, 124, 125, 126, 127, 128, 129, 130,
    131, 132, 133, 134, 135, 136, 137, 138, 139, 140,
    141, 142, 143, 144, 145, 146, 147, 148, 149, 150,
    151, 152, 153, 154, 155, 156, 157, 158, 159, 160,
    161, 162, 163, 164, 165, 166, 167, 168, 169, 170,
    171, 172, 173, 174, 175, 176, 177, 178, 179, 180,
    181, 182, 183, 184, 185, 186, 187, 188, 189, 190,
    191, 192, 193, 194, 195, 196, 197, 198, 199, 200,
    201, 202, 203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 215, 216, 217, 218, 219, 220,
    221, 222, 223, 224, 225, 226, 227, 228, 229, 230,
    231, 232, 233, 234, 235, 236, 237, 238, 239, 240,
    241, 242, 243, 244, 245, 246, 247, 248, 249, 250,
    251, 252, 253, 254, 255, 256, 257, 258, 259, 260,
    261, 262, 263, 264, 265, 266, 267, 268, 269, 270,
    271, 272, 273, 274, 275, 276, 277, 278, 279, 280,
    281, 282, 283, 284, 285, 286, 287, 288, 289, 290,
    291, 292, 293, 294, 295, 296, 297, 298, 299, 300,
    301, 302, 303, 304, 305, 306, 307, 308, 309, 310,
    311, 312, 313, 314, 315, 316, 317, 318, 319, 320,
    321, 322, 323, 324, 325, 326, 327, 328, 329, 330,
    331, 332, 333, 334, 335, 336, 337, 338, 339, 340,
    341, 342, 343, 344, 345, 346, 347, 348, 349, 350,
    351, 352, 353, 354, 355, 356, 357, 358, 359, 360,
    361, 362, 363, 364, 365, 366, 367, 368, 369, 370,
    371, 372, 373, 374, 375, 376, 377, 378, 379, 380,
    381, 382, 383, 384, 385, 386, 387, 388, 389, 390,
    391, 392, 393, 394, 395, 396, 397, 398, 399, 400,
    401, 402, 403, 404, 405, 406, 407, 408, 409, 410,
    411, 412, 413, 414, 415, 416, 417, 418, 419, 420,
    421, 422, 423, 424, 425, 426, 427, 428, 429, 430,
    431, 432, 433, 434, 435, 436, 437, 438, 439, 440,
    441, 442, 443, 444, 445, 446, 447, 448, 449, 450,
    451, 452, 453, 454, 455, 456, 457, 458, 459, 460,
    461, 462, 463, 464, 465, 466, 467, 468, 469, 470,
    471, 472, 473, 474, 475, 476, 477, 478, 479, 480,
    481, 482, 483, 484, 485, 486, 487, 488, 489, 490,
    491, 492, 493, 494, 495, 496, 497, 498, 499, 500
};

/* 
 * Displays numbers on LED 7-Segment Display.
 * Parameters: number - 0-9 or DP. Pace: 0-3 or digit1, digit2, digit3, digit4
 */
void display_seven_segment(unsigned char number, unsigned char place) {
    PORTD &= PORTD_DEFAULT;
    PORTB &= PORTB_DEFAULT;
    PORTB |= (1 << place + 2) | (place == digit2 ? 1 << SEG_DP : 0); //display decimal if on digit2
  
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
    //Serial.begin(9600);
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
    ADCSRA  |= B00100000; //set ADATE in to enable auto triggering.
    ADCSRB  &= B11111000; //clear ADTS2..0 in ADCSRB to set trigger mode to free running
    ADCSRA  |= B00000111; //sets ADC prescaler to 128 - 16000KHz/128=125KHz
    ADCSRA  |= B00001000; //sets ADIE to enable ADC interrupt
#ifndef Arduino_h
    sei(); //enables global interrupts
#endif
    ISR_generated = 0;
    sensor_val = 0;
    ADCSRA  |= B01000000; //set ADSC to 1 to start ADC conversion

    //DIDR0   = B00111111; //not needed as ADC6 and ADC7 don't have digital input buffers

    //SAMPLE GND ON ANOTHER ADC PIN AT SETUP TO TARE THE ADC
}

/******************************************************************************/

volatile uint8_t dig1 = 0;
volatile uint8_t dig2 = 0;
volatile uint8_t dig3 = 0;

/*
 * Generated once Analog->Digital Conversion Complete
 */
ISR(ADC_vect) {
    sensor_val_raw = (ADCL) | ADCH << 8; /* ADCL must be read before ADCH */
    sensor_val = sensor_val_raw*500>>10;
    ISR_generated = 1;
    //ADCSRA |= B01000000; //reset ADSC ADC start conversion bit. not needed bc code is free running
}

/******************************************************************************/
uint8_t loop_counter = 0;
uint8_t schmitt_buffer = 5;

void loop() {
    if (ISR_generated) {
        /* Account for ADC Nonlinearities */
        sensor_val = lookup_table[sensor_val];

         /* Moving Average Filter */
        sensor_val_sum -= sensor_val_arr[loop_counter%sensor_arr_length];
        sensor_val_arr[loop_counter%sensor_arr_length] = sensor_val;
        sensor_val_sum += sensor_val;
        sensor_val_avg_prev = sensor_val_avg;
        sensor_val_avg = sensor_val_sum >> 4; //const uint8_t sensor_arr_length = 16; and >>4 divides by 16
        
        /* Software Schmitt Trigger */
        sensor_val_avg_schmitt_out_prev = sensor_val_avg_schmitt_out;
        if(sensor_val_avg > sensor_val_avg_prev + schmitt_buffer || sensor_val_avg < sensor_val_avg_prev - schmitt_buffer) {
            sensor_val_avg_schmitt_out = sensor_val_avg;
        } else {
            sensor_val_avg_schmitt_out = sensor_val_avg_schmitt_out_prev;
        }

        output = sensor_val;
        //Serial.println(output);
        
        /* Splitting up Digits*/
        dig1 = output > 500 ? 5 : output > 400 ? 4 : output > 300 ? 3 : 
               output > 200 ? 2 : output > 100 ? 1 : 0;
        dig2 = output%100/10;
        dig3 = output%10;   /* hundredths place */
        
        /* Turn on LED */
//        PORTC = output > 250 + DEADBAND_VARIANCE_FAR  || 
//                output < 250 - DEADBAND_VARIANCE_FAR  ? B00000001 : //red
//                output > 250 + DEADBAND_VARIANCE_NEAR || 
//                output < 250 - DEADBAND_VARIANCE_NEAR ? B00000010 : //yellow
//                                                        B00000100;  //green
        /* Reset */
        ISR_generated = 0;
    }   
    /* Using a switch ensures even lighting between digits */
    switch(loop_counter%3) {
        case 0:
            display_seven_segment(dig3, digit4);
            break;
        case 1:
            display_seven_segment(dig2, digit3);
            break;
        case 2:
            display_seven_segment(dig1, digit2);
            break;
    }
    loop_counter++;
}
