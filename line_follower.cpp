// -O0
// 7372800Hz
#define F_CPU 7372800
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "lcd.c"



unsigned char ADC_Conversion(unsigned char);
unsigned char ADC_Value;
double l = 0;
double c = 0;
double r = 0;
unsigned char PortBRestore = 0;


double error=0;
double lastError = 0;
double position;

double P;
double I;
double D;

float Kp = 0;
float Ki = 0;
float Kd = 0;

unsigned max_l = 200;
unsigned max_r = 200;
unsigned max_c = 200;
unsigned min_l = 0;
unsigned min_r = 0;
unsigned min_c = 0;

void motion_pin_config (void)
{
 DDRB = DDRB | 0x0F;   //set direction of the PORTB3 to PORTB0 pins as output
 PORTB = PORTB & 0xF0; // set initial value of the PORTB3 to PORTB0 pins to logic 0
 DDRD = DDRD | 0x30;   //Setting PD4 and PD5 pins as output for PWM generation
 PORTD = PORTD | 0x30; //PD4 and PD5 pins are for velocity control using PWM
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
 unsigned char PortBRestore = 0;

 Direction &= 0x0F; 			// removing upper nibbel as it is not needed
 PortBRestore = PORTB; 			// reading the PORTB's original status
 PortBRestore &= 0xF0; 			// setting lower direction nibbel to 0
 PortBRestore |= Direction; 	// adding lower nibbel for direction command and restoring the PORTB status
 PORTB = PortBRestore; 			// setting the command to the port
}

void velocity (unsigned char left_motor, unsigned char right_motor) { 
	OCR1AL = left_motor;
	 OCR1BL = right_motor; 
	 	OCR1AH = 0x00;
	 	OCR1BH = 0x00;
	}

void forward (void)         //both wheels forward
{
  motion_set(0x06);
}

void back (void)            //both wheels backward
{
  motion_set(0x09);
}

void left (void)            //Left wheel backward, Right wheel forward
{
  motion_set(0x05);
}

void right (void)           //Left wheel forward, Right wheel backward
{   
  motion_set(0x0A);
}

void soft_left (void)       //Left wheel stationary, Right wheel forward
{
 motion_set(0x04);
}

void soft_right (void)      //Left wheel forward, Right wheel is stationary
{ 
 motion_set(0x02);
}

void soft_left_2 (void)     //Left wheel backward, right wheel stationary
{
 motion_set(0x01);
}

void soft_right_2 (void)    //Left wheel stationary, Right wheel backward
{
 motion_set(0x08);
}

void hard_stop (void)       //hard stop(stop suddenly)
{
  motion_set(0x00);
}

void soft_stop (void)       //soft stop(stops slowly)
{
  motion_set(0x0F);
}

//Function to Initialize ADC
void adc_init()
{
 ADCSRA = 0x00;
 ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
 ACSR = 0x80;
 ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7;    //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80;  // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRA = 0x00;   //set PORTF direction as input
	PORTA = 0x00;  //set PORTF pins floating
}

//Function to Initialize PORTS
void port_init()
{
	lcd_port_config();
	adc_pin_config();
	motion_pin_config();
}
void timer1_init(void)
{
	TCCR1B = 0x00; //stop
	TCNT1H = 0xFF; //setup
	TCNT1L = 0x01;
	OCR1AH = 0x00;
	OCR1AL = 0xFF;
	OCR1BH = 0x00;
	OCR1BL = 0xFF;
	ICR1H  = 0x00;
	ICR1L  = 0xFF;
	TCCR1A = 0xA1;
	TCCR1B = 0x0D; //start Timer
}


void init_devices (void)
{
 cli(); //Clears the global interrupts
 port_init();
 adc_init();
 sei(); //Enables the global interrupts
 timer1_init();
}


//Function to configure LCD port




//TIMER1 initialize - prescale:64
// WGM: 5) PWM 8bit fast, TOP=0x00FF
// desired value: 450Hz
// actual value: 450.000Hz (0.0%)

//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
 unsigned char a;
 Ch = Ch & 0x07;
 ADMUX= 0x20| Ch;
 ADCSRA = ADCSRA | 0x40;	//Set start conversion bit
 while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
 a=ADCH;
 ADCSRA = ADCSRA|0x10;      //clear ADIF (ADC Interrupt Flag) by writing 1 to it
 return a;
}

//Main Function
int main(void)
{
    init_devices();
    port_init();
    lcd_set_4bit();
    lcd_init();
    
    max_l = 90;
    max_r = 70;
    max_c = 90;
    min_l = 6;
    min_r = 6;
    min_c = 6;

    const unsigned maxspeeda = 60;
    const unsigned maxspeedb = 60;
    const unsigned basespeeda = 85;
    const unsigned basespeedb = 85;
	const unsigned minspeeda = 20;
    const unsigned minspeedb = 20;
    
    Kp = 12;
    Ki = 0;
    Kd = 22;
    

    while(1)
    {
		forward();
    	l=ADC_Conversion(3);
    	c=ADC_Conversion(4);
    	r=ADC_Conversion(5);
    	//lcd_print(1, 1, l, 3);
    	//lcd_print(1, 5, c, 3);
    	//lcd_print(1, 9, r, 3);
		if(l<10 && r<10 && c<10){
			velocity(100,100);
			back();
			_delay_ms(150);
		}
        l = (l-min_l)/(max_l - min_l);
        r = (r-min_r)/(max_r - min_r);
        c = (c-min_c)/(max_c - min_c);
        ///  l, c, r lies in the range of (0, 100)
        position = (l*0 + c*10 + r*20)/(l+c+r);
        // left -> 0
        // centre -> 100
        // right -> 200 --- values of position
        error = 10 - position; //150 is the ideal position (the centre)
        P = error;
        I = I + error;
        D = error - lastError;
        lastError = error;
        int motorspeed = P*Kp + I*Ki + D*Kd; //calculate the correction
                                             //needed to be applied to the speed

        int motorspeeda = basespeeda + motorspeed;
        int motorspeedb = basespeedb - motorspeed;
		//lcd_print(2, 1, motorspeedb, 3);
		////lcd_print(2, 5, error, 3);
		//lcd_print(2, 9, motorspeeda, 3);
        if (motorspeeda > maxspeeda) {
          //motorspeeda = maxspeeda;
        }
        if (motorspeedb > maxspeedb) {
          //motorspeedb = maxspeedb;
        }
        if (motorspeeda < 0) {
			//motorspeeda = -1*motorspeeda;
          //back();
		  //velocity(motorspeeda,motorspeedb);
		  //_delay_ms(10);
		  //right();
		  //velocity(motorspeedb,-motorspeeda);
		  //_delay_ms(1);
		  //continue;
		  motorspeeda=0;
		  
        }
        if (motorspeedb < 0) {
			//motorspeedb = -1*motorspeedb;
          //back();
          //velocity(motorspeeda,motorspeedb);
		  //_delay_ms(10);
		  motorspeedb=0;
		  //left();
		  //velocity(-motorspeedb,motorspeeda);
		  //_delay_ms(1);
		  //continue;
        } 
        //velocity(motorspeeda, motorspeedb);
		//velocity(48,48);
		/*OCR1AL = (unsigned char)2;
		OCR1BL = (unsigned char)2;
		OCR1AH = (unsigned char)0;
		OCR1BH = (unsigned char)0;*/
		forward();
		velocity(motorspeedb,motorspeeda);
		
		
		
	
		_delay_ms(1);
    }


/*while(1)
{
	forward();            //both wheels forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	back();               //both wheels backward						
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	left();               //Left wheel backward, Right wheel forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	right();              //Left wheel forward, Right wheel backward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_left();          //Left wheel stationary, Right wheel forward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_right();         //Left wheel forward, Right wheel is stationary
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_left_2();        //Left wheel backward, right wheel stationary
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);

	soft_right_2();       //Left wheel stationary, Right wheel backward
	_delay_ms(1000);

	hard_stop();						
	_delay_ms(300);
}*/

}

