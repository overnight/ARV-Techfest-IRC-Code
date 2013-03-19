
#include "gridbase.h"



enum Command pathPlanner(Point cP, Direction cD, Point gP)
{
	if(gP.x - cP.x != 0)
	{
		if(gP.x - cP.x > 0) // Move EAST
		{
			if(cD == WEST)
			return UT;
			if(cD == EAST)
			return STRAIGHT;
			if(cD == NORTH)
			return TR;
			return TL;
		}
		// Move West
		if(cD == EAST)
		return UT;
		if(cD == WEST)
		return STRAIGHT;
		if(cD == SOUTH)
		return TR;
		return TL;
	}
	if(gP.y - cP.y != 0)
	{
		if(gP.y - cP.y > 0) // Move SOUTH
		{
			if(cD == NORTH)
			return UT;
			if(cD == SOUTH)
			return STRAIGHT;
			if(cD == EAST)
			return TR;
			return TL;
		}
		// Move NORTH
		if(cD == SOUTH)
		return UT;
		if(cD == NORTH)
		return STRAIGHT;
		if(cD == WEST)
		return TR;
		return TL;
	}
	return REACHED;
}
enum Command pathPlannerDryRun(Point cP, Direction cD)
{
	if(cP.y%2==0)
	{
		if(MAX_NODE_X -1- cP.x != 0)
		{
			if(MAX_NODE_X-1 - cP.x > 0) // Move EAST
			{
				if(cD == WEST)
				return UT;
				if(cD == EAST)
				return STRAIGHT;
				if(cD == NORTH)
				return TR;
				return TL;
			}
			// Move West
			if(cD == EAST)
			return UT;
			if(cD == WEST)
			return STRAIGHT;
			if(cD == SOUTH)
			return TR;
			return TL;
		}
	}
	else
	{
		if(0-cP.x!=0)
		{
			if(0- cP.x > 0) // Move EAST
			{
				if(cD == WEST)
				return UT;
				if(cD == EAST)
				return STRAIGHT;
				if(cD == NORTH)
				return TR;
				return TL;
			}
			// Move West
			if(cD == EAST)
			return UT;
			if(cD == WEST)
			return STRAIGHT;
			if(cD == SOUTH)
			return TR;
			return TL;
		}
	}
	if(MAX_NODE_Y -1-cP.y > 0) // Move SOUTH
	{
		if(cD == NORTH)
		return UT;
		if(cD == SOUTH)
		return STRAIGHT;
		if(cD == EAST)
		return TR;
		return TL;
	}
	return REACHED;
};

void linefollow(void)
{
	
	if(WHITE(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)){
	PORTC=FORWARD;}
	//	return;
	else if(BLACK(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)) {
		PORTC=RIGHTFORWARD;
	}
	//	return;
	else if(BLACK(LEFT_S)&&BLACK(CENTER_S)&&WHITE(RIGHT_S)) {
		PORTC=RIGHTFORWARD;
	}
	//	return;
	else if(WHITE(LEFT_S)&&WHITE(CENTER_S)&&BLACK(RIGHT_S)){
		PORTC=LEFTFORWARD;
	}
	//	return;
	else if(WHITE(LEFT_S)&&BLACK(CENTER_S)&&BLACK(RIGHT_S)){
		PORTC=LEFTFORWARD;
	}
	else if(BLACK(LEFT_S)&&BLACK(CENTER_S)&&BLACK(RIGHT_S)){
		PORTC=RIGHTFORWARD;
	}
	//	return;
}
void linefollowbackward()
{
	
	if(WHITE(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)){
	PORTC=BACK;}
	//	return;
	else if(BLACK(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)) {
		PORTC=RIGHTFORWARD;
	}
	//	return;
	else if(BLACK(LEFT_S)&&BLACK(CENTER_S)&&WHITE(RIGHT_S)) {
	while(!(WHITE(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)))
	{
			PORTC=LEFTFORWARD;
			_delay_ms(20);
	}		
	}
	//	return;
	else if(WHITE(LEFT_S)&&WHITE(CENTER_S)&&BLACK(RIGHT_S)){
		PORTC=LEFTFORWARD;
	}
	//	return;
	else if(WHITE(LEFT_S)&&BLACK(CENTER_S)&&BLACK(RIGHT_S)){
	while(!(WHITE(LEFT_S)&&WHITE(CENTER_S)&&WHITE(RIGHT_S)))
	{
		PORTC=RIGHTFORWARD;
		_delay_ms(20);
	}
	}
	else if(BLACK(LEFT_S)&&BLACK(CENTER_S)&&BLACK(RIGHT_S)){
		PORTC=BACK;
	}
	
	//	return;
}
void openFront()
{
	pwmFront=10;
	PORTC=STOP;
	PORTA=OPEN;
	_delay_ms(1000);
	PORTA=STOP;
} 
void closeFront()
{
	PORTC=STOP;
	PORTA=CLOSE;
	_delay_ms(1000);
	//PORTA=STOP;
	pwmFront=3;

}
void nodeLeftTurn()
{
	setPWM(3,12);
	if(isBlockWithUs) setPWM(6,18);
	PORTC=LEFT0TURN;
	while(1)
	{
		if(BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)){
			setPWM(15,18);
			return;
		}
		
		_delay_ms(25);
	}
	
}
void nodeRightTurn()
{
	setPWM(13,5);
	if(isBlockWithUs) setPWM(18,7);
	PORTC=RIGHT0TURN;
	while(1)
	{
		if(BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)){
			setPWM(15,18);
			return;
		}
		
		_delay_ms(25);
	}
}
void nodeStraight()
{
	setPWM(5,6);
	if(isBlockWithUs) setPWM(10,12);
	PORTC=FORWARD;
	while(1)
	{
		
		if((BLACK(LEFTEXTREME_S)||BLACK(RIGHTEXTREME_S))&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)){
		setPWM(15,18);
		return;
		}
		else if((BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S))&&BLACK(LEFT_S)&&BLACK(RIGHT_S)&&BLACK(CENTER_S))	
		{
			setPWM(15,18);
			return;
		}	
		_delay_ms(25);
	}
}
void nodeUTurn()
{
	setPWM(10,12);
	if(isBlockWithUs) setPWM(15,17);
	PORTC=RIGHT0TURN;
	while(1)
	{
		if(BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)){
			setPWM(15,18);
			return;
		}
		
		_delay_ms(20);
	}
	
}
void align(Direction gD)
{
	int turnDir;
	switch(gD)
	{
		case EAST: turnDir=4;break;
		case WEST: turnDir=2;break;
		case NORTH:turnDir=1;break;
		case SOUTH:turnDir=3;break;
	}
	switch(curDir)
	{
		case EAST: turnDir-=4;break;
		case WEST: turnDir-=2;break;
		case NORTH:turnDir-=1;break;
		case SOUTH:turnDir-=3;break;
	}
	turnDir+=4;
	turnDir%=4;
			switch(turnDir)
			{
				case 0:
					nodeStraight();
					break;
				case 2:
					nodeUTurn();
					break;
				case 3:
					nodeRightTurn();					
					break;
				case 1:
					nodeLeftTurn();					
					break;
			}
	curDir=gD;
}
void setup()
{
	DDRB =0b00000011;
	DDRC =0xFF;
	PORTC=0;
	PORTB=0b11111100;
	DDRA =0b11110000;
	PORTA=0;

	TCCR0|=(1<<WGM00|(1 << CS01)|(0 << CS00));
	TCCR2|=(1<<WGM00|(1 << CS02)|(1<<CS00));
	TIMSK|=(1<<TOIE0|(1<<TOIE2));
	pwmLeft=9;
	pwmRight=12;
	pwmFront=10;
	initialiseBotState();
	DDRD = 1<<PD2;		// Set PD2 as input (Using for interrupt INT0)
	PORTD = 1<<PD2;		// Enable PD2 pull-up resistor
	GICR = 1<<INT0;					// Enable INT0
	MCUCR = 1<<ISC01 | 1<<ISC00;	// Trigger INT0 on rising edge
	sei();
}

Point nextneighbour(unsigned char val[MAX_NODE_X][MAX_NODE_Y+1],Point toadd)
{
	int i,j;
	i=toadd.x;j=toadd.y;
	if(j<MAX_NODE_Y-1)	if(val[i][j+1]==1) return Point(i,j+1);
	if(i<MAX_NODE_X-1)	if(val[i+1][j]==1) return Point(i+1,j);	
	if(j>0) if(val[i][j-1]==1) return Point(i,j-1);
	if(i>0)	if(val[i-1][j]==1) return Point(i-1,j);
	
	
	return Point(-1,-1);
}
void  FloydWarshallWithPathReconstruction ()
{

	unsigned char address;
	char dist[(MAX_NODE_X)*(MAX_NODE_Y+1)][(MAX_NODE_X)*(MAX_NODE_Y+1)];
	unsigned char val[MAX_NODE_X][MAX_NODE_Y+1];
	int i,j,k,l,totalnodes =(MAX_NODE_X)*(MAX_NODE_Y+1) ;
	for(i=0;i<totalnodes;i++)		//initializing dist = infinity and next node = -1
	{
		for(j=0;j<totalnodes;j++)
		{
			dist[i][j] =100;
			next[i][j] = 120;
		}
	}
	for(i=0;i<MAX_NODE_X;i++)  //initializing all dist to all adjacent node to 1;
	{
		for (j=0;j<MAX_NODE_Y+1;j++)
		{
			k=i+1;
			l=j;
			if(k>=0&&l>=0&&k<MAX_NODE_X&&l<MAX_NODE_Y+1)
			{
				dist[j*MAX_NODE_X +i][l*MAX_NODE_X+k]=1;
				dist[l*MAX_NODE_X+k][j*MAX_NODE_X +i]=1;
			}
			k=i;
			l=j+1;
			if(k>=0&&l>=0&&k<MAX_NODE_X&&l<MAX_NODE_Y+1)
			{
				dist[j*MAX_NODE_X +i][l*MAX_NODE_X+k]=1;
				dist[l*MAX_NODE_X+k][j*MAX_NODE_X +i]=1;
			}
			
			
			
		}
	}
	for(i=0;i<MAX_NODE_X;i++)			//for each black node , dist to adjecent node =200;
	{
		for (j=0;j<MAX_NODE_Y+1;j++)
		{
			if(j==MAX_NODE_Y)
			{
				val[i][j]=0;
				if(i==KEY_DEPOSITE_X)val[i][j]=1;
			}
			else
			{
				address = (j*MAX_NODE_X +i);
				val[i][j]=EEPROM_read(address);
			}			
			if(val[i][j]==0)
			{
				for(k=i-1;k<=i+1;k++)
				{
					for(l=j-1;l<=j+1;l++)
					{
						if(k>=0&&l>=0&&k<MAX_NODE_X&&l<MAX_NODE_Y+1)
						{
							dist[j*MAX_NODE_X +i][l*MAX_NODE_X+k]=100;
							dist[l*MAX_NODE_X+k][j*MAX_NODE_X +i]=100;
						}
						
					}
				}
			}
		}
	}
	for(i=0;i<totalnodes;i++) dist[i][i]=0;
	//for(k=0;k<totalnodes;k++)
	//{
		//	for(i=0;i<totalnodes;i++)
		//	{
			//		printf("%d\t",dist[k][i]);
		//	}
		//	printf("\n");
	//}
	for(k=0;k<totalnodes;k++)
	{
		for(i=0;i<totalnodes;i++)
		{
			for(j=0;j<totalnodes;j++)
			{
				if((int) dist[i][k] +(int) dist[k][j] < (int)dist[i][j])
				{
					dist[i][j] = (int)dist[i][k] +(int) dist[k][j];
					next[i][j] = k;
				}
			}
		}
	}
	//point list to travel to in order to find intruder block
	maxpts=0;
	Point toadd(0,0);
	char flag =0;
	do
	{
			goalArr[maxpts]=toadd;
			maxpts++;
			val[toadd.x][toadd.y]=0;
			toadd = nextneighbour(val,toadd);
			if(toadd.x==-1)
			{
				flag=1;
				for (i=0;i<MAX_NODE_X;i++)
				{
					for (j=0;j<MAX_NODE_Y;j++)
					{
					if(val[i][j]==1) {flag=2; break;}
				}
			}
		}
		if(flag==2)
		{
			for(i=maxpts-1;i>=0;i--)
			
			if(nextneighbour(val,goalArr[i]).x!=-1)
			{
				toadd=(nextneighbour(val,goalArr[i]));
				flag=0;
				break;
			}
		}
	}while(flag==0);

}
Point nextPoint(Point cP, Point gP)
{
	//if(((int)dist[MAX_NODE_X*cP.y + cP.x][MAX_NODE_X*gP.y + gP.x])>90) return cP;
	int intermediate = (next[MAX_NODE_X*cP.y + cP.x][MAX_NODE_X*gP.y + gP.x]);
	if (intermediate==120) return gP;
	Point temp(intermediate%MAX_NODE_X,intermediate/MAX_NODE_X);
	return nextPoint(cP,temp);
}
enum Command getCom()
{
	//_delay_ms(1000);
	switch(skillState)
	{
		
		case DRYRUNNODES:
		{
			writeNodeColour();
		//	_delay_ms(1000);
			return pathPlannerDryRun(curPoint, curDir);
		}
		case DRYRUNBLOCKPOS:
		{
			return pathPlanner(curPoint, curDir, blockPosArr[curGoalIndex]);
		}
		case FINDINTRUDER:
		{
			return pathPlanner(curPoint, curDir, nextPoint(curPoint,goalArr[curGoalIndex]));
		}
		case CARRYINTRUDER:
		{
			return pathPlanner(curPoint, curDir, nextPoint(curPoint,intruderDepositePoint));
		}
		case GOTOKEY:
		{
			return pathPlanner(curPoint, curDir, nextPoint(curPoint,keyPickPoint));
		}
		case CARRYKEY:
		{
			return pathPlanner(curPoint, curDir, nextPoint(curPoint,keyDepositePoint));
		}
	}	
	while(1)
	{
		PORTB^=1<<1;
		_delay_ms(500);
	}
}
void startTime()
{
	timecounter=0;
	
}

void initialiseMainRun(int delay)
{
		skillState=FINDINTRUDER;
		initialiseBotState();
		adc_init();
		curGoalIndex=0;
		PORTB=0b00000011;
		
}
void reachedDataDest()
{
	if(skillState==DRYRUNNODES)
	{
		PORTC=STOP;
		curGoalIndex=0;			
		skillState=DRYRUNBLOCKPOS;
		state=NODE;
		//FloydWarshallWithPathReconstruction();
		//_delay_ms(2000);
			//curGoalIndex=0;
			//curPoint.x = -1; curPoint.y = 0;
			//curDir = EAST;
			//state = LF;
			//adc_init();
			//PORTB=0b00000011;
			//closeFront();
			//openFront();
			//_delay_ms(WAIT_FOR_MAIN_RUN);
			//openFront();
			//PORTB=0;

	}
	else if(skillState==DRYRUNBLOCKPOS)
	{
		align(WEST);
		PORTC=STOP;
		//nodeStraight(); //will test
		while(1)
		{
		if(WHITE(LEFTEXTREME_S)&&WHITE(RIGHTEXTREME_S)){nodeStraight(); break;}
		linefollow();
		_delay_ms(50);
		}
		_delay_ms(100);
		PORTC=STOP;
		curPoint.x--;
		setPWM(5,6);
		startTime();
		while(timecounter<180*TIME_TO_FIND_BLOCKPOS)
		{
			if(WHITE(LEFTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)&&BLACK(RIGHTEXTREME_S))
			{
				keyPickPoint.x = blockPosArr[curGoalIndex].x-1;
				keyPickPoint.y = blockPosArr[curGoalIndex].y;
				openFront();
				closeFront();
				//openFront();
				//closeFront();
				break;
			}
			if(BLACK(LEFTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)&&WHITE(RIGHTEXTREME_S))
			{
				cubePickPoint.x = blockPosArr[curGoalIndex].x-1;
				cubePickPoint.y = blockPosArr[curGoalIndex].y;
				openFront();
				closeFront();
				//openFront();
				//closeFront();
				//openFront();
				//closeFront();
				break;
			}
			linefollow();
			_delay_ms(50);
		}
		//stopTime();
		setPWM(5,6);
		while(1)
		{
		if((WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S))) break;
			linefollowbackward();
			_delay_ms(50);
		}
		PORTC=STOP;
		curGoalIndex++;
		if(curGoalIndex<NUM_OF_BLOCK_POS)
		{
			state=NODE;
			return;
		}
		FloydWarshallWithPathReconstruction();
		initialiseMainRun(0);
		state=END;
		
	
	}
	else if (skillState==FINDINTRUDER)
	{
		PORTC=STOP;
		curGoalIndex++;
		
		state=NODE;
	}
	else if (skillState==CARRYINTRUDER)
	{
		PORTC=STOP;
		align(WEST);
		while(!(BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S))){ linefollow(); _delay_ms(50);}
		PORTC=STOP;
		//_delay_ms(1000);
		startTime();
		while(timecounter<=3*180) 
		{
			linefollow(); _delay_ms(20);
		}
	//	stopTime();
		PORTC=STOP;
		_delay_ms(1000);
		openFront();
		isBlockWithUs=0;
		setPWM(5,6);
		while(1)
		{
			if((WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S))) break;
			linefollowbackward();
			_delay_ms(50);
		}
		PORTC=STOP;
		pwmLeft=9;
		pwmRight=12;
		state=NODE;		
		_delay_ms(WAIT_FOR_KEYBLOCK);
		skillState=GOTOKEY;
		
	}
	else if(skillState==GOTOKEY)
	{
		PORTC=STOP;
		align(WEST);	
		PORTC=STOP;	
		//_delay_ms(4000);
		pwmLeft=5;
		pwmRight=6;
		startTime();
		while(timecounter<180)
		{
			linefollow();
			_delay_ms(50);
		}
		PORTC=STOP;
		while(adc_read(0)>ADC_KEY_THRESH)
		{
			
			_delay_ms(50);
		}
		isBlockWithUs=1;
		skillState=CARRYKEY;
		closeFront();
		setPWM(10,12);
		while(1)
		{
			if((WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S))&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S)) break;
			linefollowbackward();
			_delay_ms(50);
		}
		state=NODE;
	}
	else if(skillState==CARRYKEY)
	{
		PORTC=STOP;
		align(EAST);
		PORTC=STOP;
		while(!(BLACK(LEFTEXTREME_S)&&BLACK(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S))) linefollow();
		//delay_ms(1000);
		startTime();
		while(timecounter<=3*120)
		{
			linefollow(); _delay_ms(20);
		}
		openFront();
		isBlockWithUs=0;
		pwmLeft=5;
		pwmRight=6;
		while(1)
		{
			if((WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S)&&WHITE(LEFT_S)&&WHITE(RIGHT_S)&&WHITE(CENTER_S))) break;
			linefollowbackward();
			_delay_ms(100);
		}
		pwmLeft=9;
		pwmRight=12;
		state=END;
	}
}
void test()
{
	closeFront();
	
	while(1)
	{
		linefollowbackward();
		_delay_ms(50);
	}
}
int main (void)
{
	//int8_t gridSensorVal;
	enum Command com;
	
	setup();
	skillState=DRYRUNNODES;
	isBlockWithUs=0;
	curGoalIndex=0;
	closeFront();
	openFront();
	//	test();
	while(1)
	{
		if(skillState==FINDINTRUDER&&state!=END)
		{
			if(adc_read(0)<ADC_INTRUDER_THRESH)
			{
				skillState=CARRYINTRUDER;
				isBlockWithUs=1;
				closeFront();
			}
			
		}
		switch(state)
		{
			case LF:
			linefollow();
			if(WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S)) //both WHITE - Node is reached.
			{
				PORTC=STOP;
				_delay_ms(100);
				if(WHITE(LEFTEXTREME_S) && WHITE(RIGHTEXTREME_S))
				{
				switch(curDir)
				{
					case NORTH: curPoint.y--; break;
					case SOUTH: curPoint.y++; break;
					case EAST: curPoint.x++; break;
					case WEST: curPoint.x--; break;
				}
				state = NODE;
				}				
			}
			
			break;
			case NODE:
			PORTC=STOP;
			com =getCom();
			switch(com)
			{
				case REACHED:
				state = DATA_DEST;
				break;
				case STRAIGHT:
				nodeStraight();
				state = LF;
				break;
				case TR:
				nodeRightTurn();
				switch(curDir)
				{
					case EAST: curDir = SOUTH; break;
					case SOUTH: curDir = WEST; break;
					case WEST: curDir = NORTH; break;
					case NORTH: curDir = EAST; break;
				}
				state = LF;
				break;
				case TL:
				nodeLeftTurn();
				switch(curDir)
				{
					case EAST: curDir = NORTH; break;
					case NORTH: curDir = WEST; break;
					case WEST: curDir = SOUTH; break;
					case SOUTH: curDir = EAST; break;
				}
				state = LF;
				break;
				case UT:
				nodeUTurn();
				switch(curDir)
				{
					case EAST: curDir = WEST; break;
					case NORTH: curDir = SOUTH; break;
					case WEST: curDir = EAST; break;
					case SOUTH: curDir = NORTH; break;
				}
				state = LF;
				break;
			}
			break;
			case DATA_DEST:
			reachedDataDest();			
			break;
			case END:
			PORTC=STOP;
			break;
		}
		_delay_ms(50);
	}
	
}
ISR(TIMER0_OVF_vect)
{
	//PWM
	cli();
	counter++;
	if(counter==pwmLeft)
	{
		PORTC &= ~(1 << PWMLEFT);
	}
	if(counter==pwmRight)
	{
		PORTC &= ~(1 << PWMRIGHT);
	}
	if(counter==pwmFront)
	{
		PORTA &= ~(1 << PWMFRONT);
		//PORTB =0b11000000;
	}
	if(counter==20)
	{
		counter=0;
		PORTC |= (1 << PWMLEFT|1<<PWMRIGHT);
		PORTA |= (1<<PWMFRONT);
	}
	sei();
}
ISR(TIMER2_OVF_vect)
{
	//PWM
	cli();
	timecounter++;
	if(timecounter>5000)
	timecounter=0;
	sei();
}
ISR(INT0_vect)
{
	PORTC=STOP;
	
	if(skillState==DRYRUNBLOCKPOS)
	{
		_delay_ms(WAIT_FOR_MAIN_RUN);
		skillState=FINDINTRUDER;
		initialiseBotState();
		FloydWarshallWithPathReconstruction();
		curGoalIndex=0;
		PORTB=0;
	}
	else if(skillState==FINDINTRUDER||skillState==CARRYINTRUDER)
	{
		_delay_ms(2000);
		skillState=FINDINTRUDER;
		initialiseBotState();
		curGoalIndex=0;
		PORTB=0;
	}
	else if(skillState==CARRYKEY||skillState==GOTOKEY)
	{
		_delay_ms(3000);
		skillState=GOTOKEY;
		initialiseBotState();
		curGoalIndex=0;
		PORTB=0;
	}
	
}