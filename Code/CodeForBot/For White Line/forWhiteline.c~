/************************************************************************************
 Written by: Vinod Desai, NEX Robotics Pvt. Ltd.
 Edited by: Sachitanand Malewar, NEX Robotics Pvt. Ltd.
 AVR Studio Version 4.17, Build 666

 Date: 26th December 2010
 
 This experiment demonstrates the application of a simple line follower robot. The 
 robot follows a white line over a black backround
 
 Concepts covered:  ADC, LCD interfacing, motion control based on sensor data

 LCD Connections:
 			  LCD	  Microcontroller Pins
 			  RS  --> PC0
			  RW  --> PC1
			  EN  --> PC2
			  DB7 --> PC7
			  DB6 --> PC6
			  DB5 --> PC5
			  DB4 --> PC4

 ADC Connection:
 			  ACD CH.	PORT	Sensor
			  0			PF0		Battery Voltage
			  1			PF1		White line sensor 3
			  2			PF2		White line sensor 2
			  3			PF3		White line sensor 1
			  4			PF4		IR Proximity analog sensor 1*****
			  5			PF5		IR Proximity analog sensor 2*****
			  6			PF6		IR Proximity analog sensor 3*****
			  7			PF7		IR Proximity analog sensor 4*****
			  8			PK0		IR Proximity analog sensor 5
			  9			PK1		Sharp IR range sensor 1
			  10		PK2		Sharp IR range sensor 2
			  11		PK3		Sharp IR range sensor 3
			  12		PK4		Sharp IR range sensor 4
			  13		PK5		Sharp IR range sensor 5
			  14		PK6		Servo Pod 1
			  15		PK7		Servo Pod 2

 ***** For using Analog IR proximity (1, 2, 3 and 4) sensors short the jumper J2. 
 	   To use JTAG via expansion slot of the microcontroller socket remove these jumpers.  
 
 Motion control Connection:
 			L-1---->PA0;		L-2---->PA1;
   			R-1---->PA2;		R-2---->PA3;
   			PL3 (OC5A) ----> PWM left; 	PL4 (OC5B) ----> PWM right; 
 
 LCD Display interpretation:
 ****************************************************************************
 *LEFT WL SENSOR	CENTER WL SENSOR	RIGHT WL SENSOR		BLANK			*
 *BLANK				BLANK				BLANK				BLANK			*
 ****************************************************************************
 
 Note: 
 
 1. Make sure that in the configuration options following settings are 
 	done for proper operation of the code

 	Microcontroller: atmega2560
 	Frequency: 14745600
 	Optimization: -O0 (For more information read section: Selecting proper optimization 
 					options below figure 2.22 in the Software Manual)

 2. Make sure that you copy the lcd.c file in your folder

 3. Distance calculation is for Sharp GP2D12 (10cm-80cm) IR Range sensor

*********************************************************************************/

/********************************************************************************

   Copyright (c) 2010, NEX Robotics Pvt. Ltd.                       -*- c -*-
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

   * Source code can be used for academic purpose. 
	 For commercial use permission form the author needs to be taken.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. 

  Software released under Creative Commence cc by-nc-sa licence.
  For legal information refer to: 
  http://creativecommons.org/licenses/by-nc-sa/3.0/legalcode

********************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include <math.h> //included to support power function
#include "lcd.c"

void back_mm(unsigned int);
void soft_left_degrees(unsigned int);
void soft_right_degrees(unsigned int);
void forward_line_mm(unsigned int);
unsigned char walk(void);
void forward_mm(unsigned int);
void stop(void);
/*******************************************************************************/

unsigned char data; //to store received data from UDR1
volatile unsigned int commd = 0;
unsigned char depth = 0;
unsigned char breakpoint = 0;
int brkpt =0;
void buzzer_pin_config (void)
{
 DDRC = DDRC | 0x08;		//Setting PORTC 3 as outpt
 PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}


//Configure PORTB 5 pin for servo motor 1 operation
void servo1_pin_config (void)
{
 DDRB  = DDRB | 0x20;  //making PORTB 5 pin output
 PORTB = PORTB | 0x20; //setting PORTB 5 pin to logic 1
}

//Configure PORTB 6 pin for servo motor 2 operation
void servo2_pin_config (void)
{
 DDRB  = DDRB | 0x40;  //making PORTB 6 pin output
 PORTB = PORTB | 0x40; //setting PORTB 6 pin to logic 1
}

//Configure PORTB 7 pin for servo motor 3 operation
void servo3_pin_config (void)
{
 DDRB  = DDRB | 0x80;  //making PORTB 7 pin output
 PORTB = PORTB | 0x80; //setting PORTB 7 pin to logic 1
}	


//Function to rotate Servo 1 by a specified angle in the multiples of 1.86 degrees
void servo_1(unsigned char degrees)  
{
 float PositionPanServo = 0;
  PositionPanServo = ((float)degrees / 1.86) + 35.0;
 OCR1AH = 0x00;
 OCR1AL = (unsigned char) PositionPanServo;
}


//Function to rotate Servo 2 by a specified angle in the multiples of 1.86 degrees
void servo_2(unsigned char degrees)
{
 float PositionTiltServo = 0;
 PositionTiltServo = ((float)degrees / 1.86) + 35.0;
 OCR1BH = 0x00;
 OCR1BL = (unsigned char) PositionTiltServo;
}

//Function to rotate Servo 3 by a specified angle in the multiples of 1.86 degrees
void servo_3(unsigned char degrees)
{
 float PositionServo = 0;
 PositionServo = ((float)degrees / 1.86) + 35.0;
 OCR1CH = 0x00;
 OCR1CL = (unsigned char) PositionServo;
}

//servo_free functions unlocks the servo motors from the any angle 
//and make them free by giving 100% duty cycle at the PWM. This function can be used to 
//reduce the power consumption of the motor if it is holding load against the gravity.

void servo_1_free (void) //makes servo 1 free rotating
{
 OCR1AH = 0x03; 
 OCR1AL = 0xFF; //Servo 1 off
}

void servo_2_free (void) //makes servo 2 free rotating
{
 OCR1BH = 0x03;
 OCR1BL = 0xFF; //Servo 2 off
}

void servo_3_free (void) //makes servo 3 free rotating
{
 OCR1CH = 0x03;
 OCR1CL = 0xFF; //Servo 3 off
} 

//Set the initial position of digging arm
void servos_reset_pos(void)
{
	servo_1(160);
	servo_2(0);
	servo_3(5);
	_delay_ms(2000);
}

void timer1_init(void)
{
 TCCR1B = 0x00; //stop
 TCNT1H = 0xFC; //Counter high value to which OCR1xH value is to be compared with
 TCNT1L = 0x01;	//Counter low value to which OCR1xH value is to be compared with
 OCR1AH = 0x03;	//Output compare Register high value for servo 1
 OCR1AL = 0xFF;	//Output Compare Register low Value For servo 1
 OCR1BH = 0x03;	//Output compare Register high value for servo 2
 OCR1BL = 0xFF;	//Output Compare Register low Value For servo 2
 OCR1CH = 0x03;	//Output compare Register high value for servo 3
 OCR1CL = 0xFF;	//Output Compare Register low Value For servo 3
 ICR1H  = 0x03;	
 ICR1L  = 0xFF;
 TCCR1A = 0xAB; /*{COM1A1=1, COM1A0=0; COM1B1=1, COM1B0=0; COM1C1=1 COM1C0=0}
 					For Overriding normal port functionality to OCRnA outputs.
				  {WGM11=1, WGM10=1} Along With WGM12 in TCCR1B for Selecting FAST PWM Mode*/
 TCCR1C = 0x00;
 TCCR1B = 0x0C; //WGM12=1; CS12=1, CS11=0, CS10=0 (Prescaler=256)
}

// Dig hole by moving digging arm by angle 'depth'
void dig(void)
{
	servos_reset_pos();
	unsigned char i = 0;
	unsigned char s1pos;
	unsigned char s2pos;
	s1pos = 33;
	s2pos = 40;
	servo_1(s1pos);
	servo_2(s2pos);
	_delay_ms(1000);
	while(s2pos<60) //Take position
	{
		s2pos++;
		servo_2(s2pos);
		_delay_ms(50);
	}
	unsigned char iter = 3; // Make 3 strokes
	while(iter--)
	{
		for (i = 0; i <=depth; i++)	//Move arm down
		{
			s1pos += 1;
			s2pos += 1;
 			servo_1(s1pos);
 			//_delay_ms(10);
 			servo_2(s2pos);
			_delay_ms(50);
			//servo_3(i);
			//_delay_ms(30);
	 	}
		if (depth >10)			// Make slight right horizontal movements to set the soil
		{
			s1pos +=5;
			servo_1(s1pos);
			_delay_ms(500);
			s1pos -=10;
			servo_1(s1pos);
			_delay_ms(500);
			s1pos +=5;
			servo_1(s1pos);
			_delay_ms(500);
			servo_3(0);
			_delay_ms(500);
			servo_3(10);
			_delay_ms(500);
			servo_3(5);
			_delay_ms(500);
		}
		for (i = 0; i <=depth; i++)	// Move arm up
	 	{
 			s1pos -= 1;
			s2pos -= 1;
 			servo_1(s1pos);
 			//_delay_ms(10);
 			servo_2(s2pos);
			_delay_ms(70);
			//servo_3(i);
			//_delay_ms(30);
	 	}
	}

	for (i = 0; i <=50; i++)	// Take arm up to position set previously
 	{
		s2pos -= 1;
 		
 		servo_2(s2pos);
		_delay_ms(30);
 	} 

	servos_reset_pos();
	servo_1_free(); 
	servo_2_free();
	servo_3_free();
}


void snap (void)
{
	servo_3(180);
	_delay_ms(4000);
}

void buzzer_on (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore | 0x08;
 PORTC = port_restore;
}

void buzzer_off (void)
{
 unsigned char port_restore = 0;
 port_restore = PINC;
 port_restore = port_restore & 0xF7;
 PORTC = port_restore;
}

//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}


SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	data = UDR0; 				//making copy of data from UDR0 in 'data' variable 

		if(data == 0x38) //ASCII value of 8 // Not used
		{
			commd = 8;
		}

		if(data == 0x32) //ASCII value of 2 //Not used
		{
			commd = 2;
		}

		if(data == 0x34) //ASCII value of 4 //Not used
		{
			commd = 4;
		}

		if(data == 0x36) //ASCII value of 6  //To turn the bot 90 degrees
		{
			commd = 6;
		}

		if(data == 0x35) //ASCII value of 5 //Not used
		{
			PORTA=0x00; //stop
		}

		if(data == 0x37) //ASCII value of 7 ?? Not used
		{
			commd = 7;
		}

		if(data == 0x39) //ASCII value of 9
		{
			buzzer_off();
		}
		
		if(data == 0x33) //ASCII value of 3. To start digging.
		{
			stop();
			dig();
			UDR0 = 0x64;
			commd = 0;
		}

		if(data == 0x31) //ASCII value of 1. To rotate the platform
		{
			snap();
			UDR0 = 0x31;
			_delay_ms(3000);
			commd = 0;
		}

		if(data == 0x61) //ASCII value of a, for depth angle 0.
		{
			depth = 0;
		}

		if(data == 0x62) //ASCII value of b, for depth angle 5
		{
			depth = 5;
		}

		if(data == 0x63) //ASCII value of c, for depth angle 10
		{
			depth = 10;
		}

		if(data == 0x64) //ASCII value of d, for depth angle 15
		{
			depth = 15;
		}

		if(data == 0x65) //ASCII value of e, for depth angle 20
		{
			depth = 20;
		}

		if(data == 0x67) //ASCII value of g, for starting execution
		{
			commd = 10;
		}

		if (data == 0x30) //ASCII value of 0, to stop the robot
		{
			commd = 0;
		}

}


/****************************************************************************/



void port_init();
void timer5_init();
void velocity(unsigned char, unsigned char);
void motors_delay();

unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;
volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder 
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
unsigned int Degrees; //to accept angle in degrees for turning


//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
 DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
 PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}


//Function to configure LCD port
void lcd_port_config (void)
{
 DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
 PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
 DDRF = 0x00; 
 PORTF = 0x00;
 DDRK = 0x00;
 PORTK = 0x00;
}

//Function to configure ports to enable robot's motion
void motion_pin_config (void) 
{
 DDRA = DDRA | 0x0F;
 PORTA = PORTA & 0xF0;
 DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
 PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config	
	buzzer_pin_config();
	servo1_pin_config(); //Configure PORTB 5 pin for servo motor 1 operation
 	servo2_pin_config(); //Configure PORTB 6 pin for servo motor 2 operation 
 	servo3_pin_config(); //Configure PORTB 7 pin for servo motor 3 operation  	
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/
	
	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
unsigned char ADC_Conversion(unsigned char Ch) 
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;  			
	ADMUX= 0x20| Ch;	   		
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{
	
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortARestore = 0;

 Direction &= 0x0F; 		// removing upper nibbel for the protection
 PortARestore = PORTA; 		// reading the PORTA original status
 PortARestore &= 0xF0; 		// making lower direction nibbel to 0
 PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
 PORTA = PortARestore; 		// executing the command
}

void forward (void) 
{
  motion_set (0x06);
}


void back (void) //both wheels backward
{
  motion_set(0x09);
}

void stop (void) //Stop bot
{
  motion_set (0x00);
}


void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
 EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
 sei();   // Enables the global interrupt 
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
 cli(); //Clears the global interrupt
 EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
 EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
 sei();   // Enables the global interrupt 
}


//ISR for right position encoder
ISR(INT5_vect)  
{
 ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
 ShaftCountLeft++;  //increment left shaft position count
}


void soft_left (void) //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
 motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}


void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}


void angle_rotate(unsigned int Degrees)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
 ShaftCountRight = 0; 
 ShaftCountLeft = 0; 

 while (1)
 {
  if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
  break;
 }
 stop(); //Stop robot
}


void linear_distance_mm(unsigned int DistanceInMM) // move linear distance bu DistanceInMM //Not used
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
		unsigned char res = walk();
		if((ShaftCountRight > ReqdShaftCountInt) && (res == 1))
		{
 			break;
		}
 } 
 stop(); //Stop robot
 buzzer_on();
 _delay_ms(300);
 buzzer_off();
}

void linear_woline_mm(unsigned int DistanceInMM)
{
 float ReqdShaftCount = 0;
 unsigned long int ReqdShaftCountInt = 0;

 ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
 ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;
  
 ShaftCountRight = 0;
 while(1)
 {
 	lcd_print(2,1,ShaftCountRight , 3);
  if(ShaftCountRight > ReqdShaftCountInt)
  {
  	break;
  }
 } 
 stop(); //Stop robot
  buzzer_on();
 _delay_ms(2000);
 buzzer_off();
}

void forward_line_mm(unsigned int DistanceInMM)
{
 //forward();
 linear_distance_mm(DistanceInMM);
}


void forward_mm(unsigned int DistanceInMM)
{
 forward();
 linear_woline_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
 back();
 linear_woline_mm(DistanceInMM);
}




void soft_left_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left(); //Turn soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right();  //Turn soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_left_2(); //Turn reverse soft left
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
 // 176 pulses for 360 degrees rotation 2.045 degrees per count
 soft_right_2();  //Turn reverse soft right
 Degrees=Degrees*2;
 angle_rotate(Degrees);
}

void init_devices (void)
{
 	cli(); //Clears the global interrupts
	port_init();
	adc_init();
	timer5_init();
	timer1_init();
	uart0_init(); //Initailize UART1 for serial communiaction
	left_position_encoder_interrupt_init();
 	right_position_encoder_interrupt_init();
	sei();   //Enables the global interrupts
}



unsigned char walk(void)
{

		Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor

		flag=0;

		print_sensor(1,1,3);	//Prints value of White Line Sensor1
		print_sensor(1,5,2);	//Prints Value of White Line Sensor2
		print_sensor(1,9,1);	//Prints Value of White Line Sensor3
		
		if(Left_white_line>0x28 && Right_white_line>0x28)
		{
			while(Left_white_line>0x28 && Right_white_line>0x28)
			{
				Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
				Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
				Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
				forward();
				velocity(230,220);
			}
			
			UDR0 = 0x70;
		}

		if(Center_white_line>0x28)
		{
			flag=1;
			forward();
			velocity(230,220);
		}

		if((Left_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(255,190);
		}

		if((Right_white_line<0x28) && (flag==0))
		{
			flag=1;
			forward();
			velocity(200,235);
		}

		if(Center_white_line<0x28 && Left_white_line<0x28 && Right_white_line<0x28)
		{
			forward();
			stop();
		}
		return 0; 
}
//Main Function
int main()
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	servos_reset_pos();
	while(commd != 10); // Initialise walking
	
	
	while(1)	// Loop to continuously wait for commands
	{
		if (commd == 10)	// Command for walking
			walk();
		
		if (commd == 6)		//command for turning right by 90 degrees
		{
			
			soft_right_degrees(90);
			UDR0 = 0x74;	//send confirmation
			commd = 10;
		}
		if (commd == 0)
			stop(); //Halt
	}
}
