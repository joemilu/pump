//====================================================================
// File Name : Adc.c
// Function  : TQ2440 ADC Test 
// Version   : 1.0
//====================================================================

#include "def.h"
#include "option.h"
#include "2440addr.h"
#include "2440lib.h"
#include "2440slib.h" 

#define REQCNT 100 
#define ADC_FREQ 2500000
 #define LOOP 10000

volatile U32 preScaler;

//==================================================================================		
int ReadAdc(int ch)
{
	U32 rADCCON_save1 = rADCCON;
	int i;
	int max = 0;
	int temp = 0, min = 5000;
	long templ = 0;
	static int prevCh=-1;

	rADCCON = (1<<14)|(preScaler<<6)|(ch<<3);		//setup channel

	if(prevCh!=ch)
	{
		prevCh=ch;
	}

	for(i=0;i<7;i++)
	{
		rADCCON|=0x1;									//start ADC
		while(rADCCON & 0x1);							//check if Enable_start is low
		while(!(rADCCON & 0x8000));						//check if EC(End of Conversion) flag is high
		temp =	(int)rADCDAT0 & 0x3ff;
		templ += temp; 
		if(temp > max)
			max = temp;
		else if(temp < min)
			min = temp;			
	}

	templ = templ - max - min;

	rADCCON = rADCCON_save1;
	return ( templ/5 );
}

//==================================================================================


void Test_Adc(void) 
{
	int a2=0; //Initialize variables
	U32 rADCCON_save = rADCCON;
    
	Uart_Printf( "\nADC INPUT Test, press ESC key to exit !\n" ) ;

	preScaler = ADC_FREQ;
	Uart_Printf("ADC conv. freq. = %dHz\n",preScaler);
	preScaler = 50000000/ADC_FREQ -1;
    
	Uart_Printf("PCLK/ADC_FREQ - 1 = %d\n",preScaler);
    
	while( Uart_GetKey() != ESC_KEY )
	{
		a2=ReadAdc(2); 	
		Uart_Printf( "AIN2: %04d\n", a2);
		Delay( 200 ) ;
		Delay( 700 ) ;
	}
    
	rADCCON = rADCCON_save;
	Uart_Printf("\nrADCCON = 0x%x\n", rADCCON);
}
