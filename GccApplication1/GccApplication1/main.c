#define F_CPU 16000000UL // Define CPU Frequency
#include <avr/io.h>			/* Include AVR std. library file */
#include <util/delay.h>			/* Include Delay header file */	
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#define degree_sysmbol 0xdf
#include <avr/interrupt.h>
#include<time.h>

#define LCD_port PORTB			/* Define LCD data port B*/
#define RS PB0				/* Define Register Select pin */
#define EN PB1 				/* Define Enable signal pin */

//-----------------LDR-------------------

void LDR(){	
	
	if((PINA & 0b00000100)==0b00000100){
		DDRD|=0x10;
		PORTD|=0x10;	
		}
	else if((PINA & 0b00000100)==0b00000000){
		PORTD=0x00;
	}
		
	}

//--------------------------------Servo Motor-----------------

void Servo_Wait()
{
	_delay_loop_2(0);
	_delay_loop_2(0);
	_delay_loop_2(0);
	
}

void Servo(){
	DDRD = 0x20; //Makes RC0 output pin
	PORTD = 0x00;
	PORTD = 0x20;
	_delay_us(1000);
	PORTD = 0x00;
	}

//-------------------------Temperature Sensonr---------------

void ADC_Init(){
	DDRA = 0x00;	        /* Make ADC port as input */
	ADCSRA = 0x87;          /* Enable ADC, with freq/128  */
	ADMUX = 0x40;           /* Vref: Avcc, ADC channel: 0 */
}

int ADC_Read(char channel)
{
	ADMUX = 0x40 | (channel & 0x07);   /* set input channel to read */
	ADCSRA |= (1<<ADSC);               /* Start ADC conversion */
	while (!(ADCSRA & (1<<ADIF)));     /* Wait until end of conversion by polling ADC interrupt flag */
	ADCSRA |= (1<<ADIF);               /* Clear interrupt flag */
	_delay_ms(1);                      /* Wait a little bit */
	return ADCW;                       /* Return ADC word */
}

//-----------------LCD------------------

void lcdCommand( unsigned char cmd )
{
	LCD_port = (LCD_port & 0x0F) | (cmd & 0xF0); //send upper nibble (1111 0000)
	//this is doing to using LCD in 4 bit mode
	
	LCD_port &= ~ (1<<RS);	
	
	
	LCD_port |= (1<<EN);		
	_delay_us(1);
	LCD_port &= ~(1<<EN);     

	_delay_us(200);

	LCD_port = (LCD_port & 0x0F) | (cmd << 4);  //send low nibble
	LCD_port |= (1<<EN); 
	_delay_us(1);
	LCD_port &= ~ (1<<EN); 
	
	_delay_ms(2);
}
void lcddata( unsigned char data )
{
	LCD_port = (LCD_port & 0x0F) | (data & 0xF0);  // sending upper nibble 
	LCD_port |= (1<<RS); 
	LCD_port|= (1<<EN);  
	_delay_us(1);
	LCD_port &= ~ (1<<EN); 

	_delay_us(200);

	LCD_port = (LCD_port & 0x0F) | (data << 4); 
	LCD_port |= (1<<EN); 
	_delay_us(1);
	LCD_port &= ~ (1<<EN); 
	
	_delay_ms(2);
}
void lcdInit(void)			
{
	 DDRB = 0xFF;			
	_delay_ms(20);			
	
	lcdCommand(0x02);		// send for 4 bit initialization of LCD 
	lcdCommand(0x28);        // 2 line, 5*7 matrix in 4-bit mode
	lcdCommand(0x0E);        // Display on cursor on(we can send 0x0c or 0x0E)
	lcdCommand(0x06);        // Increment cursor (shift cursor to right)
	lcdCommand(0x01);        //Clear display screen
	_delay_ms(2);
	
}
void lcd_Print (char *str)		
{
	int i;
	for(i=0;str[i]!=0;i++)		
	{
		lcddata (str[i]);
	}
}
void LCD_Clear()
{
	lcdCommand (0x01);		
	_delay_ms(2);
	lcdCommand (0x80);		
}

//----------------------------Keypad------------------

unsigned char keypad()
{
	PORTC = 0b11111110; 
 	if ((PINC & (1<<PINC4)) == 0) 
	{
		_delay_ms(20);
		return '7';
	}
 	else if((PINC & (1<<PINC5)) == 0)
	{
		_delay_ms(20);
		return '8';
	}
	else if((PINC & (1<<PINC6)) == 0)
	{
		_delay_ms(20);
		return '9';
	}
	PORTC = 0b11111101; 
	if ((PINC & (1<<PINC4)) == 0) 
	{
		_delay_ms(20); 
		return '4';
	}
	else if((PINC & (1<<PINC5)) == 0)
	{
		_delay_ms(20);
		return '5';
	}
	else if((PINC & (1<<PINC6)) == 0)
	{
		_delay_ms(20);
		return '6';
	}	
	PORTC = 0b11111011; 
	if ((PINC & (1<<PINC4)) == 0) 
	{
		_delay_ms(20); 
		return '1';
	}
	else if ((PINC & (1<<PINC5)) == 0) 
	{
		_delay_ms(20); 
		return '2';
	}
	else if((PINC & (1<<PINC6)) == 0)
	{
		_delay_ms(20);
		return '3';
	}	
	PORTC = 0b11110111;
	if ((PINC & (1<<PINC5)) == 0)
	{
		_delay_ms(20); 
		return '0';
	}
	
}


//------------------------------ STEPPER MOTORS-----------------

	//PORTD = 0x0C; - 0 degree       a
	//PORTD = 0x06; - 90 degrees     b
	//PORTD = 0x03; - 180 degrees    c
	//PORTD = 0x09  - 270            d


//------------------------------  Ball Holder STEPPER MOTORS-----------------

void ball_holder_stepper(char a){

	
	DDRD |= 0x0F;
	int period;
	//DDRD = 0x0F;		/* Make PORTD lower pins as output */
	period = 100;		/* Set period in between two steps */
	
	if(a=='a'){
		PORTD = 0x06;
		_delay_ms(period);
	}
	else if(a=='b'){
		PORTD = 0x03;
		_delay_ms(period);	
	}
	else if(a=='c'){
		PORTD = 0x09;
		_delay_ms(period);
	}
	else{
		PORTD = 0x0C;
		_delay_ms(period);
	}

}

// -------------Angle making stepper motor-------------

int A(){
	PORTA=0x20;
	_delay_ms(100);
}
int B(){
	
	PORTA=0x40;
	_delay_ms(100);
}
int C(){
	
	PORTA=0x30;
	_delay_ms(100);
}
int D(){
	
	PORTA=0xc0;
	_delay_ms(100);
}
int E(){
	
	PORTA=0x10;
	_delay_ms(100);
}
int F(){
	
	PORTA=0x08;
	_delay_ms(100);
}
int G(){
	
	PORTA=0x50;
	_delay_ms(100);
}

int stepper_motor(int k){
	
	switch(k){
		case 1:
		A();
		break;
		case 2:
		B();
		break;
		case 3:
		C();
		break;
		case 4:
		D();
		break;
		case 5:
		E();
		break;
		case 6:
		F();
		break;
		case 7:
		G();
		break;
		}
		return 0;
}


//------------------------DC Motor 1/2-------------------

void InitPWM1()
{
	TCCR2=(1<<WGM20)|(1<<WGM21)|(2<<COM20)|(2<<CS20);
	DDRD|=(1<<PD7);	
}

void InitPWM2()
{
	TCCR0=(1<<WGM20)|(1<<WGM21)|(2<<COM20)|(2<<CS20);
	DDRB|=(1<<PB3);	
}

void SetPWMOutput1(uint8_t duty)
{
	OCR2=duty;
}

void SetPWMOutput2(uint8_t duty)
{
	OCR0=duty;
}



//---------------Temperature----------------

int temperature(){
	
	char Temperature[10];
	float celsius;

	lcdInit();                 // initialize 16x2 LCD
	ADC_Init();                 // initialize ADC

	LCD_Clear();
	//lcd_Print("Temperature:");
	celsius = (ADC_Read(0)*4.88);
	celsius = (celsius/10.00);
	int ret=(int)celsius;
	sprintf(Temperature,"%d%cC", (int)celsius, degree_sysmbol);//convert integer value to ASCII string
	//lcd_Print(Temperature);//send string data for printing
	//_delay_ms(1000);
	memset(Temperature,0,10);
	
	if(ret>25){
		return 0;
		}
	else{
		return 1;
	}
}


int main(void) 
{
	srand(time(NULL));
	//-------------------------------enter minimum speed------------------------		
		
		unsigned char x;
		DDRC = 0x0F ;
		PORTC =0xF0;//11110000 Make all 4 columns 1 and rows 0
		
		lcdInit();
		lcd_Print("Enter Minimun Speed:");
		lcdCommand(0xC0);
		
		int w=1;
		int temp=0;
		int speed1=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		
		lcddata(x);
		temp=(int)x-48;
		speed1=speed1*10+temp;
		
		w=1;
		temp=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		lcddata(x);
		temp=(int)x-48;
		speed1=speed1*10+temp;
		
		
		w=1;
		temp=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		lcddata(x);
		temp=(int)x-48;
		speed1=speed1*10+temp;
		
		//------------------------Enter Maximum Speed---------------------------------------
		
		lcdInit();
		lcd_Print("Enter Maximum Speed:");
		lcdCommand(0xC0);
		
		w=1;
		temp=0;
		int speed2=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		lcddata(x);
		temp=(int)x-48;
		speed2=speed2*10+temp;
		
		w=1;
		temp=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		lcddata(x);
		temp=(int)x-48;
		speed2=speed2*10+temp;
		
		w=1;
		temp=0;
		while(w>0){
			x = keypad();
			if(x=='0'||x=='1'||x=='2'||x=='3'||x=='4'||x=='5'||x=='6'||x=='7'||x=='8'||x=='9'){
				w-=1;
			}
		}
		lcddata(x);
		temp=(int)x-48;
		speed2=speed2*10+temp;
		
		
		
		 //----------get ready notification-------------
		 
		 
		_delay_ms(50);
		LCD_Clear();
		lcd_Print("Get ready");
		_delay_ms(50);
		LCD_Clear();
					
	

		int lower =speed1, upper = speed2;
		char pos='a';
		int k;
		for(int g=0;g<20;g++){
			
			if(temperature()==1){
				
				LDR();
			   //----------dc motor speed control-------------
		
				uint8_t num = (rand() % (upper - lower + 1)) + lower;
		
				int num1 = num;
				char snum[5];
				itoa(num1, snum, 10);
				lcd_Print(snum);
				lcd_Print(" kmh");
		
		
				//----------make the direction of the machine-------------
		
				
				_delay_ms(100);
				
				DDRA=0xF0;
				k=(rand()%7)+1;
				stepper_motor(k);
				
				Servo();
				_delay_ms(100);
				
				//----------mControl Speed----------------
				
				InitPWM1();
				InitPWM2();
		
				if(num1>80 && num1<120){
					SetPWMOutput1(120);
					SetPWMOutput2(10);
				}
				else if(num1>120 && num1<130){
					SetPWMOutput1(10);
					SetPWMOutput2(120);
				}
				else{
					SetPWMOutput1(5);
					SetPWMOutput2(5);
				}
		
				_delay_ms(100);
		
				//-------------Buzzer Alarm---------------
	
				DDRA=0x08;
				PORTA=0x08;
				_delay_ms(100);
				PORTA=0x00;
	
				//----------ball holder rotation-------------
		
				_delay_ms(50);
				DDRD |= 0xFF;
				PORTD |= 0xFF;
		
				if(pos=='a'){
					ball_holder_stepper(pos);
					pos='b';
				}
				else if(pos=='b'){
					ball_holder_stepper(pos);
					pos='c';
				}
				else if(pos=='c'){
					ball_holder_stepper(pos);
					pos='d';
				}
				else if(pos=='d'){
					ball_holder_stepper(pos);
					pos='a';
				}
				
				else{
				}
		
				LCD_Clear();
				_delay_ms(100);
			}
			
			else{
				lcdInit();
				lcd_Print("High Temperature");
				DDRA=0x02;
				PORTA=0x02;
				_delay_ms(10000000);
					
			}
		}

	return 0;	

}