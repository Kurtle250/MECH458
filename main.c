/*########################################################################
# MILESTONE: MECH 458 project
# PROGRAM: MECH458_Project.c
# PROJECT: Final project
# GROUP: 10
# NAME 1: Kurt Elliott
# DESC:
# DATA Oct 28,2020
# REVISED
########################################################################*/

/* include libraries */
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include "myutils.h"
#include "lcd.h"
#include "LinkedQueue.h"

// Global Variable
//ADC
volatile unsigned int ADC_result = 0;
volatile unsigned int ADC_result_flag;
volatile unsigned int ADC_min = 0b1111111111;

//flags for sensors
volatile int rampdownFlag = 0;  // ramp down button
volatile int bucket_flag = 1;   // bucket aligned flag
volatile int EOT_flag  = 0;     // end of travel flag
volatile int pause_flag = 0;    // pause button
volatile int OR_flag = 0;       // optical sensor flag
volatile int motorFlag = 0;

//item positions
volatile unsigned int cur_item;
volatile unsigned int prev_item = 4;	// set to calibrated home black item

// item ADC threshold values
int Ai_max = 255;
int Stl_min = 400;
int Stl_max = 700;
int Wht_min = 900;
int Wht_max = 955;
int Blk_min = 956;
int Blk_max = 1023;
 
// Stepper motor variables
const uint8_t s_table[4] = {0b00110110,  // full step mode
							0b00101110, 
							0b00101101, 
							0b00110101};
									
volatile int Cur_position = 0; //set black as initial position 0

const unsigned int acc_profile50[50] = {
    20,19,18,17,16,15,14,13,12,11,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    11,12,13,14,15,16,17,18,19,20};
    
const unsigned int acc_profile100[100] = {
    20,19,18,17,16,15,14,13,12,11,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    8,8,8,8,8,8,8,8,8,8,
    11,12,13,14,15,16,17,18,19,20};

// Data storage
 link *head;				// The ptr to the head of the queue
 link *tail;				// The ptr to the tail of the queue
 link *newLink = NULL;		// A ptr to a new link aggregate data type (struct)
 link *rtnLink = NULL;		// A ptr to a return link aggregate data type

// DC motor directions
volatile uint8_t CW = 0b00001011;      // clockwise
volatile uint8_t CCW = 0b00000111;     //counter clockwise
volatile uint8_t BRK = 0b00001111;     // brake high

// Sorted count variables 
volatile  int Alcount = 0;
volatile  int Stcount = 0;
volatile  int Wtcount = 0;
volatile  int Bkcount = 0;

// Function prototypes
void mTimer(int count);
void uTimer(int count);
void PMW_init();
void XINT_init();
void ADC_init();
void stepper_CW (int s);
void stepper_CCW (int s);
void home_Bucket();
void rampTimer();
void updateLCD();
 
int main(int argc, char *argv[]){
    // Configure system clock
    CLKPR = (1<<CLKPCE);		// enable clock change
    CLKPR = (1<<CLKPS0);		// pre-scale system clock from 16MHz to 8MHz
	
    // Configure PORT pins:
    DDRA = 0xFF;    // Configure PORT A pins for Stepper motor control (bucket)
    DDRB = 0xFF;    // Configure PORT B pins for motor control (conveyor)
    DDRC = 0xFF;    // Configure PORT C for LCD output
    DDRD = 0xF0;    // Configure Port D pin to input for INT1,INT2,INT3
    DDRE = 0xC0;    // Configure PORT E pin to input for INT4,INT5
    DDRF = 0x00;    // Configure PORT F pin to input for ADC
	DDRL = 0xFF;
    
	// initialize miscellaneous functions
    PMW_init();
    ADC_init();
    InitLCD(LS_BLINK|LS_ULINE);
    updateLCD();
	
    cli();			// Disables all interrupts
    XINT_init();	// Set up the external Interrupts 1,2,3,4,5
    sei();			// Enable all interrupts

	setup(&head,&tail);

    home_Bucket();	// calibrate bucket to black
	
    PORTB = CCW;	// start Motor
	while(1){		// main polling state
				
		if(rampdownFlag == 160){			// Ramp down button
			updateLCD();
			PORTB = BRK;					// brake motor
			PORTB = 0x0C;					// disable motor power
			cli();							// disable interrupt commands
			while(1);						// lock into infinite loop
		}
		while(!isEmpty(&head)){
			if(EOT_flag){					// EOT flag on conveyor belt
				switch(head->e.itemCode){	// black = 4 steel = 2 white = 3  aluminum = 1 
					case 1:
						if(prev_item == 4){
							stepper_CW(50);
						}else if(prev_item == 2){
							stepper_CW(100);
						}else{
							stepper_CCW(50);
					}
						Alcount++;
					break;
					case 2:
						if(prev_item == 4){
							stepper_CCW(50);
						}else if(prev_item == 1){
							stepper_CW(100);
						}else{
							stepper_CW(50);
					}
						Stcount++;
					break;
					case 3:
						if(prev_item == 4){
							stepper_CW(100);
						}else if(prev_item == 2){
							stepper_CCW(50);
						}else{
							stepper_CW(50);
					}
						Wtcount++;
					break;
					case 4:
						if(prev_item == 3){
							stepper_CW(100);
						}else if(prev_item == 2){
							stepper_CW(50);
						}else{
							stepper_CCW(50);
					}
						Bkcount++;
					break;
				}//end switch
				EOT_flag = 0;
				prev_item = head->e.itemCode;
				dequeue(&head,&tail,&rtnLink);
				PORTB = CCW;
			}//end if (EOT)
		}//end while
	}//end while
	return(0);
}//end main

// bucket calibration using HALL sensor
ISR(INT1_vect){		
	bucket_flag = 0;
}
// Optical sensor to initiate ADC  
ISR(INT2_vect){		
	mTimer(5);
	if((PIND & 0x04) == 0x04){
		initLink(&newLink);
		ADC_min = 0x03FF;
		ADCSRA |= (1<<ADSC); // start ADC
	}
}
// EOT sensor
ISR(INT3_vect){		
	PORTB = BRK;
	EOT_flag = 1;
	if(prev_item == head->e.itemCode){
		EOT_flag = 0;
		prev_item = head->e.itemCode;
		if(prev_item == 4){
			Bkcount++;
		}
		if (prev_item == 3){
			Wtcount++;
		}
		if (prev_item == 2){
			Stcount++;
		}else{
			Alcount++;
		}
		dequeue(&head,&tail,&rtnLink);
		PORTB = CCW;
	}
}
// Pause protocol --> button Port E pin 4
ISR(INT4_vect){			
	if(pause_flag){		//un-paused state
		PORTB = CCW;    //start DC motor
		LCDClear();
		LCDWriteStringXY(0,0,"Resume state");
		pause_flag = 0;
		}else{			//paused state
		PORTB = BRK;    // Break DC motor
		updateLCD();
		pause_flag = 1;
	}
	while((PINE & 0x10) != 0x10); //wait for button release
	mTimer(10);
}
// Ramp Down protocol --> button Port E pin 5
ISR(INT5_vect){			
	rampTimer();
	while((PINE & 0x20) != 0x20);
	mTimer(10);
}
// 8 second timer for ramp down protocol
ISR(TIMER3_COMPA_vect){
	TIFR3 |= (1<<OCF3A); // clear interrupt flag
	rampdownFlag++;
}

// interrupt will be triggered at end of ADC
ISR(ADC_vect){
	if(ADC < ADC_min)		{					// check ADC Vs ADC_min
		ADC_min = ADC;
	}
	if((PIND & 0x04) == 0x04){					// optical sensor
		ADCSRA |= (1<<ADSC);					// start ADC
		}else{
		if(ADC_min <= Ai_max){        			// Aluminum
			cur_item = 1;
			}else if(ADC_min <= Stl_max){		// Steel 
			cur_item = 2;
			}else if(ADC_min <= (Wht_max)){		// White 
			cur_item = 3;
			}else if(ADC_min <= Blk_max){		// Black
			cur_item = 4;
		}
		newLink->e.itemCode = cur_item;
		enqueue(&head,&tail,&newLink);
	}
}

// <---------------------------------------- Miscellaneous Driver-------------------------------------->
// Configure External interrupts
void XINT_init(){
	EIMSK |= (1<<INT1)|(1<<INT2)|(1<<INT3)|(1<<INT4)|(1<<INT5); //enable INT1 INT2,INT3,INT4,INT5
	EICRA |= ((1<<ISC11)|(1<<ISC20)|(1<<ISC21)|(1<<ISC31));		// set rising edge interrupt for INT 2 and falling edge for INT 1,3
	EICRB |= ((1<<ISC41)|(1<<ISC51));							// set falling edge interrupts for INT 4,5
	
}
// initialize ADC
void ADC_init(){ 
	//Configure ADC set for in A1 ADC1
	ADCSRA |= ((1<<ADEN)|(1<<ADIE));	// enable ADC & interrupts of ADC
	ADCSRA |= (1<<ADPS2)|(1<<ADPS0);    // Set pre-scaler for ADC
	ADMUX |= ((1<<REFS0)|(1<<MUX0));	// Set ADC reference voltage REF:01 & MUX to MUX1 for ADC1
}
// calibrate bucket to home black
void home_Bucket(){ 
	while(bucket_flag){
		stepper_CW(1); // Step in clockwise direction until bucket calibrated to black.
	}
	return;
}

// <----------------------------------------Driver for the timer & PWM-------------------------------------->
// Timer 1 configured for millisecond timer delay function
void mTimer(int count){	

	int i = 0;							// initial loop counter to 0
	
	TCCR1B |= ((1<<WGM12)|(1<<CS11));	// Set the waveform generation mode bit description to clear timer
										// on compare math mode (CTC) and Set CS pre-scale to 1/8
	OCR1A = 0x03E8;						// Set output to compare register for 1000 cycles = 1ms
	
	TCNT1 = 0x0000;						// set initial value of timer counter to 0x0000;
	
	TIFR1 |= (1<<OCF1A);				// Clear timer interrupts flag and start new timer
	
	while(i < count){
		if((TIFR1 & 0x02) == 0x02){
			TIFR1 |= (1<<OCF1A);		// clear interrupt flag
			i++;						// increment loop counter
		} // end if
	} // end while
	return;
}// milliTimer

// Timer 4 configured for microsecond timer delay function
void uTimer(int count){	

	int i = 0;							// initial loop counter to 0

	TCCR4B |= ((1<<WGM12)|(1<<CS11));	// Set the waveform generation mode bit description to clear timer
										// on compare math mode (CTC) and Set CS pre-scale to 1/8
	OCR4A = 0x0064;						// Set output to compare register for 100 cycles = 0.1ms
	
	TCNT4 = 0x0000;						// set initial value of timer counter to 0x0000;
	
	TIFR4 |= (1<<OCF4A);				// Clear timer interrupts flag and start new timer
	
	while(i < count){
		if((TIFR4 & 0x02) == 0x02){
			TIFR4 |= (1<<OCF4A);		// clear interrupt flag
			i++;						// increment loop counter
		} // end if
	} // end while
	return;
}// microTimer	

// Timer 3 configured for ramp down protocol timer delay
void rampTimer(){
	TCCR3B |= ((1<<WGM32)|(1<<CS31));	// Set the waveform generation mode bit description to clear timer
										// on compare math mode (CTC) and Set CS pre-scale to 1/8
	OCR3A = 0xC350;						// Set output to compare register for 50,000 cycles = 50ms -> 160 = 8s
	
	TIMSK3 |= (1<<OCIE3A);				// Enable the output compare interrupt 
	
	TCNT3= 0x0000;						// set initial value of timer counter to 0x0000;
	
	TIFR3 |= (1<<OCF3A);				//Clear timer interrupts flag and start new timer
	
	return;
}// rampTimer

// Timer 0 configured for PWM for DC motor 488Hz
void PMW_init(){
	TCCR0A |= (1 << COM0A1)|(1 << WGM00)|(1<<WGM01); // set the timer to fast PWN mode
	
	OCR0A = 0b10000000;								 // set compare register initial value to 50%
	
	TCCR0B |= (1 << CS00)|(1<<CS01);				 // set pre-scaler value
	
return;
}//end PWMdutyCycle

// Clockwise stepper controller
void stepper_CW(int s){
	for(int i = 0; i<s; i++){
		Cur_position++;
		
		if(Cur_position > 3){
			Cur_position = 0;
		}
		
		PORTA = s_table[Cur_position];
		if(s == 50){
			mTimer(acc_profile50[i]);
		}else{
			mTimer(acc_profile100[i]);
		}
	}// endfor
	return;
}// end stepper_fwd

// Counterclockwise stepper controller
void stepper_CCW(int s){
	for(int i = 0; i < s; i++){
		Cur_position--;
		
		if(Cur_position < 0){
			Cur_position = 3;
		}
		PORTA = s_table[Cur_position];
		if(s == 50){
			mTimer(acc_profile50[i]);
		}else{
			mTimer(acc_profile100[i]);
		}
	}// endfor
	return;
}// end stepper_rwd

// Update LCD with item sorted and unsorted counts
void updateLCD(){
	LCDClear();
	LCDWriteStringXY(0,0," Al Wt St Bk Us ");
	LCDWriteIntXY(1,1,Alcount,2);
	LCDWriteIntXY(4,1,Wtcount,2);
	LCDWriteIntXY(7,1,Stcount,2);
	LCDWriteIntXY(10,1,Bkcount,2);
	LCDWriteIntXY(13,1,size(&head,&tail),2);
	return;
}