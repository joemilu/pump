/****************************************************************
 NAME: u2440mon.c
 DESC: u2440mon entry point,menu,download
 ****************************************************************/
#define	GLOBAL_CLK		1

#include <stdlib.h>
#include <string.h>
#include "def.h"
#include "option.h"
#include "2440addr.h"
#include "2440lib.h"
#include "2440slib.h"
#include "mmu.h"
#include "profile.h"
#include "memtest.h"
#include "data.h"

extern char Image$$ER_ROM1$$RO$$Limit[];
extern char Image$$ER_ROM1$$RO$$Base[];
extern char Image$$ER_ROM1$$RW$$Limit[];
extern char Image$$ER_ROM1$$RW$$Base[];
extern char Image$$ER_ROM1$$ZI$$Limit[];
extern char Image$$ER_ROM1$$ZI$$Base[];

void Isr_Init(void);
void HaltUndef(void);
void HaltSwi(void);
void HaltPabort(void);
void HaltDabort(void);
void ClearMemory(void);


void Clk0_Enable(int clock_sel);	
void Clk1_Enable(int clock_sel);
void Clk0_Disable(void);
void Clk1_Disable(void);

extern void Lcd_TFT_Init(void);
extern void Lcd_TFT_Test( void ) ;
extern void Test_Touchpanel(void) ;
extern void Test_Adc(void) ;
extern void KeyScan_Test(void) ;
extern void RTC_Display(void) ;
extern void Test_SDI(void) ;
;

volatile U32 downloadAddress;

void (*restart)(void)=(void (*)(void))0x0;

volatile unsigned char *downPt;
volatile U32 downloadFileSize;
volatile U16 checkSum;
volatile unsigned int err=0;
volatile U32 totalDmaCount;

volatile int isUsbdSetConfiguration;

U32 tempDownloadAddress;
int menuUsed=0;

extern char Image$$ER_ROM1$$RW$$Limit[];
U32 *pMagicNum=(U32 *)Image$$ER_ROM1$$RW$$Limit;
int consoleNum;

static U32 cpu_freq;
static U32 UPLL;


struct SYSTEM_STAT{
	uint  interface;			//��������
	uint refresh;
};

struct THERAPY_CONFIG{
	float compress_ratio; 	//ѹ����
	double volume;		    //ѹ������
	uint heart_beat;		
};

struct TOUCH_STAT{
	uint xpoint,ypoint;		//��Ļλ��
	uint xdata,ydata;  		//ts����ֵ
};

//��ť״̬
struct BUTTOMS{
	uint start;		//��ʼ����
	uint stop; 		//��������
};

struct DATA_STAT{
	uint index;
};

struct AD_STAT{
	uint ad_raw;
};

struct HEAT_STAT{
	uint Max;//���ֵ
	uint Period;//����
	uint Min;//��Сֵ
	uint RelaxPeriod;//�滺�ڳ���
	uint Ave; //ƽ��ֵ
//	uint useful;//�����ǹ���Ч
	uint updown;//0->������ 1->�½���
	uint ClosePoint;
	float delt[2];	 //deltaֵ��[0]��һ�ε�ֵ��[1]��ֵ
	uint rate;	//����
};

struct PUMP_STAT{
	uint direction;
	uint step;
	uint pwm_rate;
	uint stat;
	uint pos;
};


struct	SYSTEM_STAT sys_stat;
struct	THERAPY_CONFIG theo_conf;
struct	TOUCH_STAT touch_stat;
struct	BUTTOMS buttoms;
struct	DATA_STAT rawdata;
struct	DATA_STAT filtereddata;
struct	AD_STAT ad_stat;
struct	HEAT_STAT heart_stat;
struct	PUMP_STAT pump_stat;


static void cal_cpu_bus_clk(void)
{
	U32 val;
	U8 m, p, s;
	
	val = rMPLLCON;
	m = (val>>12)&0xff;
	p = (val>>4)&0x3f;
	s = val&3;

	//(m+8)*FIN*2 ��Ҫ����32λ��!
	FCLK = ((m+8)*(FIN/100)*2)/((p+2)*(1<<s))*100;
	
	val = rCLKDIVN;
	m = (val>>1)&3;
	p = val&1;	
	val = rCAMDIVN;
	s = val>>8;
	
	switch (m) {
	case 0:
		HCLK = FCLK;
		break;
	case 1:
		HCLK = FCLK>>1;
		break;
	case 2:
		if(s&2)
			HCLK = FCLK>>3;
		else
			HCLK = FCLK>>2;
		break;
	case 3:
		if(s&1)
			HCLK = FCLK/6;
		else
			HCLK = FCLK/3;
		break;
	}
	
	if(p)
		PCLK = HCLK>>1;
	else
		PCLK = HCLK;
	
	if(s&0x10)
		cpu_freq = HCLK;
	else
		cpu_freq = FCLK;
		
	val = rUPLLCON;
	m = (val>>12)&0xff;
	p = (val>>4)&0x3f;
	s = val&3;
	UPLL = ((m+8)*FIN)/((p+2)*(1<<s));
	UCLK = (rCLKDIVN&8)?(UPLL>>1):UPLL;
}

uint read(void);
void default_values(void);
uint default_hardware(void);

void __irq Timer0_ISR(void);
void __irq Timer2_ISR(void);



void Main(void)
{
	int i;
	U8 key;
	U32 mpll_val = 0 ;
	//U32 divn_upll = 0 ;
    
	#if ADS10   
//	__rt_lib_init(); //for ADS 1.0
	#endif

	Port_Init();
	
	Isr_Init();
	
	i = 2 ;	//don't use 100M!
	switch ( i ) {
	case 0:	//200
		key = 12;
		mpll_val = (92<<12)|(4<<4)|(1);
		break;
	case 1:	//300
		key = 13;
		mpll_val = (67<<12)|(1<<4)|(1);
		break;
	case 2:	//400
		key = 14;
		mpll_val = (92<<12)|(1<<4)|(1);
		break;
	case 3:	//440!!!
		key = 14;
		mpll_val = (102<<12)|(1<<4)|(1);
		break;
	default:
		key = 14;
		mpll_val = (92<<12)|(1<<4)|(1);
		break;
	}
	
	//init FCLK=400M, so change MPLL first
	ChangeMPllValue((mpll_val>>12)&0xff, (mpll_val>>4)&0x3f, mpll_val&3);
	ChangeClockDivider(key, 12);
	cal_cpu_bus_clk();
	

	
	Beep(2000, 100);
	
	Uart_SendByte('\n');
	Uart_Printf("<***************************************>\n");
	Uart_Printf("               LVAD SYSTEM BETA!\n");
	Uart_Printf("                SJTU BME !\n");
	Uart_Printf("                Edit in MDK4.12,by:joe!\n");
//	Uart_Printf("      Build time is: %s  %s\n", __DATE__ , __TIME__  );
	Uart_Printf("<***************************************>\n");

	rMISCCR=rMISCCR&~(1<<3); // USBD is selected instead of USBH1 
	rMISCCR=rMISCCR&~(1<<13); // USB port 1 is enabled.


	rDSC0 = 0x2aa;
	rDSC1 = 0x2aaaaaaa;
	//Enable NAND, USBD, PWM TImer, UART0,1 and GPIO clock,
	//the others must be enabled in OS!!!
	rCLKCON = 0xfffff0;

	MMU_Init();	//

	pISR_SWI=(_ISR_STARTADDRESS+0xf0);	//for pSOS

	Led_Display(0x66);


	Clk0_Disable();
	Clk1_Disable();
	
	mpll_val = rMPLLCON;


	
	pISR_FIQ = (int)Timer2_ISR;
	pISR_TIMER0 = (int)Timer0_ISR;

	while(1)
	{

	}	  	

}

void Isr_Init(void)
{
	pISR_UNDEF=(unsigned)HaltUndef;
	pISR_SWI  =(unsigned)HaltSwi;
	pISR_PABORT=(unsigned)HaltPabort;
	pISR_DABORT=(unsigned)HaltDabort;
	rINTMOD=0x0;	  // All=IRQ mode
	rINTMSK=BIT_ALLMSK;	  // All interrupt is masked.
}


void HaltUndef(void)
{
	Uart_Printf("Undefined instruction exception!!!\n");
	while(1);
}

void HaltSwi(void)
{
	Uart_Printf("SWI exception!!!\n");
	while(1);
}

void HaltPabort(void)
{
	Uart_Printf("Pabort exception!!!\n");
	while(1);
}

void HaltDabort(void)
{
	Uart_Printf("Dabort exception!!!\n");
	while(1);
}


void ClearMemory(void)
{
	int memError=0;
	U32 *pt;
	
	Uart_Printf("Clear Memory (%xh-%xh):WR",_RAM_STARTADDRESS,HEAPEND);

	pt=(U32 *)_RAM_STARTADDRESS;
	while((U32)pt < HEAPEND)
	{
		*pt=(U32)0x0;
		pt++;
	}
	
	if(memError==0)Uart_Printf("\b\bO.K.\n");
}

void Clk0_Enable(int clock_sel)	
{	// 0:MPLLin, 1:UPLL, 2:FCLK, 3:HCLK, 4:PCLK, 5:DCLK0
	rMISCCR = rMISCCR&~(7<<4) | (clock_sel<<4);
	rGPHCON = rGPHCON&~(3<<18) | (2<<18);
}
void Clk1_Enable(int clock_sel)
{	// 0:MPLLout, 1:UPLL, 2:RTC, 3:HCLK, 4:PCLK, 5:DCLK1	
	rMISCCR = rMISCCR&~(7<<8) | (clock_sel<<8);
	rGPHCON = rGPHCON&~(3<<20) | (2<<20);
}
void Clk0_Disable(void)
{
	rGPHCON = rGPHCON&~(3<<18);	// GPH9 Input
}
void Clk1_Disable(void)
{
	rGPHCON = rGPHCON&~(3<<20);	// GPH10 Input
}



void __irq Timer0_ISR(void) 
{

}

void __irq Timer2_ISR(void)
{

}

uint read()
{
	static uint cnt = 0;
	if(cnt == 5000)
		cnt = 0;
	return 	data[cnt++];
}

void callback_bt_start()
{

}

uint myinit()
{
//��ʼ��
	default_values();
	if(!default_hardware())
		goto fail;
	return SUCCESS;
fail:
	return FAIL;
}

void default_values()
{
	sys_stat.interface = 0;
	sys_stat.refresh = 0;
	theo_conf.compress_ratio = DEFAULT_CR;
	theo_conf.volume = DEFAULT_VO;
	theo_conf.heart_beat = DEFAULT_HB;
	touch_stat.xpoint = 0;
	touch_stat.ypoint = 0;		//��Ļλ��
	touch_stat.xdata = 0;
	touch_stat.ydata = 0; 
	buttoms.start = B_OFF;
	buttoms.stop = B_OFF;
	rawdata.index = 0;
	ad_stat.ad_raw = 0;
	heart_stat.Max = 0;//���ֵ
	heart_stat.Period = 0;//����
	heart_stat.Min = 50000;//��Сֵ
	heart_stat.RelaxPeriod = 0;//�滺�ڳ���
	heart_stat.Ave = 0; //ƽ��ֵ
	heart_stat.updown = BOTTOM;//0->������ 1->�½���
	heart_stat.ClosePoint = 0;
	heart_stat.delt[0] = 0;	 //deltaֵ��[0]��һ�ε�ֵ��[1]��ֵ
	heart_stat.delt[1] = 0;
	heart_stat.rate = 0;	//���� 
}

uint default_hardware()
{
//io
//timer	  
//LCD
	Lcd_TFT_Init() ;		// LCD initial

//ad/touch
//uart
	consoleNum = 0;	// Uart 1 select for debug.
	Uart_Init( 0,115200 );
	Uart_Select( consoleNum );
//�ж�
	return SUCCESS;
fail:
	return FAIL;
} 
