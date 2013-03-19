/*
 * gridbase.h
 *
 * Created: 21/12/2012 08:59:20
 *  Author: Ankit
 */ 


#ifndef GRIDBASE_H_
#define GRIDBASE_H_
#ifndef F_CPU
#define F_CPU 16000000UL // or whatever may be your frequency
#endif
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define NUM_OF_BLOCK_POS 4 //to be set 4 in main run
#define  MAX_NODE_X 5		//to be set 5
#define	 MAX_NODE_Y 6		//to be set 6
#define  TIME_TO_FIND_BLOCKPOS 3
#define	 ADC_INTRUDER_THRESH 75
#define	 ADC_KEY_THRESH 920
#define  WAIT_FOR_KEYBLOCK 30000//120000
#define  WAIT_FOR_MAIN_RUN 1000
#define  KEY_DEPOSITE_X MAX_NODE_X-2
#define  KEY_DEPOSITE_Y MAX_NODE_Y
#define NUM_OF_GOALS MAX_NODE_X*MAX_NODE_Y
//motor
#define  PWMLEFT 2
#define  PWMRIGHT 5
#define  PWMFRONT 5
#define  LEFTM2 1
#define  LEFTM1 0
#define  RIGHTM1 4
#define  RIGHTM2 3
#define  FRONTM1 6
#define  FRONTM2 7
#define  ADCPIN 0
#define	 FORWARD (1<<RIGHTM1|1<<LEFTM1|0<<RIGHTM2|0<<LEFTM2)
#define	 BACK (0<<RIGHTM1|0<<LEFTM1|1<<RIGHTM2|1<<LEFTM2)
#define	 RIGHTFORWARD (0<<RIGHTM1|1<<LEFTM1|0<<RIGHTM2|0<<LEFTM2)
#define	 LEFTFORWARD (1<<RIGHTM1|0<<LEFTM1|0<<RIGHTM2|0<<LEFTM2)
#define	 RIGHTBACK (0<<RIGHTM1|0<<LEFTM1|1<<RIGHTM2|0<<LEFTM2)
#define	 LEFTBACK (0<<RIGHTM1|0<<LEFTM1|0<<RIGHTM2|1<<LEFTM2)
#define	 LEFT0TURN (1<<RIGHTM1|0<<LEFTM1|0<<RIGHTM2|1<<LEFTM2)
#define	 RIGHT0TURN (0<<RIGHTM1|1<<LEFTM1|1<<RIGHTM2|0<<LEFTM2)
#define	 STOP (0<<RIGHTM1|0<<LEFTM1|0<<RIGHTM2|0<<LEFTM2|0<<FRONTM1|0<<FRONTM2)
#define	 OPEN (1<<FRONTM1|0<<FRONTM2|0<<ADCPIN);
#define	 CLOSE (0<<FRONTM1|1<<FRONTM2|0<<ADCPIN);


//sensor
#define  LEFTEXTREME_S_PIN 3
#define  LEFT_S_PIN 4
#define  CENTER_S_PIN 5
#define  RIGHT_S_PIN 6
#define  RIGHTEXTREME_S_PIN 7
#define  LEFTEXTREME_S  (PINB &(1<<LEFTEXTREME_S_PIN))
#define  LEFT_S  (PINB &(1<<LEFT_S_PIN))
#define  CENTER_S  (PINB &(1<<CENTER_S_PIN))
#define  RIGHT_S  (PINB &(1<<RIGHT_S_PIN))
#define  RIGHTEXTREME_S  (PINB &(1<<RIGHTEXTREME_S_PIN))
#define  BLACK(pin) pin==0
#define  WHITE(pin) pin>0

class Point
{
	public:
	int x, y;
Point(int x, int y): x(x), y(y) {}
  Point() {}
};

Point goalArr[NUM_OF_GOALS];
Point blockPosArr[NUM_OF_BLOCK_POS] = {Point(1,5),Point(1,4),Point(1,3),Point(1,2)}; // to be set 1,2 1,3 1,4 1,5 in main run
int curGoalIndex;
Point curPoint;
enum Direction{NORTH, SOUTH, EAST, WEST} curDir;
enum Command{STRAIGHT, TL, TR,UT, REACHED};
enum State{LF, NODE, DATA_DEST, END} state;
enum SkillState{DRYRUNNODES,DRYRUNBLOCKPOS,FINDINTRUDER,CARRYINTRUDER,GOTOKEY,CARRYKEY} skillState;
int pwmLeft,pwmRight,pwmFront,counter; //pwm is a value b/w 0~20. counter IS FOR PWM;
Point intruderDepositePoint(0,1);
Point keyPickPoint(0,1),cubePickPoint(0,1);
bool isBlockWithUs=0;
Point keyDepositePoint(KEY_DEPOSITE_X,KEY_DEPOSITE_Y);
char next[(MAX_NODE_X)*(MAX_NODE_Y+1)][(MAX_NODE_X)*(MAX_NODE_Y+1)];
void setPWM(int left,int right)
{
	pwmLeft=left;
	pwmRight=right;
}
int maxpts,timecounter;
void EEPROM_write(unsigned char uiAddress , unsigned char ucData)
{
	while(EECR&&(1<<EEWE));
	EEAR=uiAddress;
	EEDR=ucData;
	cli();
	EECR|=(1<<EEMWE);
	EECR|=(1<<EEWE);
	sei();
	
}
unsigned char EEPROM_read(unsigned char uiAddress)
{
	while(EECR&&(1<<EEWE));
	EEAR=uiAddress;
	cli();
	EECR|=(1<<EERE);
	sei();
	return EEDR;
}
void initialiseBotState()
{
	
	curPoint.x = -1; curPoint.y = 0;
	curDir = EAST;
	state = LF;
}
void adc_init()
{
	// AREF = AVcc
	ADMUX = (1<<REFS0);


	// ADC Enable and prescaler of 128
	// 16000000/128 = 125000
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}

// read adc value
uint16_t adc_read(uint8_t ch)
{
	// select the corresponding channel 0~7
	// ANDing with '7' will always keep the value
	// of 'ch' between 0 and 7
	ch &= 0b00000111;  // AND operation with 7
	ADMUX = (ADMUX & 0xF8)|ch;     // clears the bottom 3 bits before ORing

	// start single conversion
	// write '1' to ADSC
	ADCSRA |= (1<<ADSC);

	// wait for conversion to complete
	// ADSC becomes '0' again
	// till then, run loop continuously
	while(ADCSRA & (1<<ADSC));

	return (ADC);
}
void writeNodeColour()
{
	unsigned char address = (MAX_NODE_X*curPoint.y +curPoint.x);
	if((BLACK(LEFT_S)&&BLACK(RIGHT_S))||(BLACK(LEFT_S)&&(BLACK(CENTER_S)))||(BLACK(CENTER_S)&&BLACK(RIGHT_S)))
	{EEPROM_write(address,0);_delay_ms(500);}
	else
	EEPROM_write(address,1);
}

#endif /* GRIDBASE_H_ */