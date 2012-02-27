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





static void cal_cpu_bus_clk(void)
{
	U32 val;
	U8 m, p, s;
	
	val = rMPLLCON;
	m = (val>>12)&0xff;
	p = (val>>4)&0x3f;
	s = val&3;

	//(m+8)*FIN*2 不要超出32位数!
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

uint myinit(void);
void callback_bt_start(void);
void load_interface(int n);

void __irq Timer0_ISR(void);
void __irq Timer2_ISR(void);
extern void __irq AdcTsAuto(void);
void starttimer0(void);
void starttimer1(void);
void starttimer2(void);

volatile struct	SYSTEM_STAT sys_stat;
volatile struct	THERAPY_CONFIG theo_conf;
volatile struct	TOUCH_STAT touch_stat;
volatile struct	BUTTOMS buttoms;
volatile struct	DATA_STAT rawdata;
volatile struct	DATA_STAT filtereddata;
volatile struct	AD_STAT ad_stat;
volatile struct	HEAT_STAT heart_stat;
volatile struct	PUMP_STAT pump_stat;

extern unsigned char TQ_LOGO_800480[];
extern unsigned char Presspic[];
extern unsigned char Setpic[];

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



	pISR_FIQ = (int)Timer0_ISR;
	pISR_TIMER2 = (int)Timer2_ISR;

 	myinit();

//	Lcd_TFT_Test();

	while(1)
	{
		if(sys_stat.refresh)
		{
//			load_interface(sys_stat.interface);
			switch(sys_stat.interface)
			{
				case 0:	{	//主界面
							if(buttoms.start)
							{
								buttoms.start = B_OFF;
								starttimer0();
							}else if(buttoms.stop)
							{}else if(buttoms.set)
							{
								buttoms.set = B_OFF; 
								sys_stat.interface = 2;
								load_interface(sys_stat.interface);	
//								Uart_Printf("< buttom set >\n");
							}else if(buttoms.pressure)
							{
								buttoms.set = B_OFF; 
								sys_stat.interface = 1;
								load_interface(sys_stat.interface);	
															
							}
						}
						break;
				case 1:	{	//压力检测
							if(buttoms.back)
							{
								buttoms.back = B_OFF;
								sys_stat.interface = 0;
								load_interface(sys_stat.interface);	
									
							}							
						}
						break;
				case 2:	{	//参数设置
							if(buttoms.back)
							{
								buttoms.back = B_OFF;
								sys_stat.interface = 0;
								load_interface(sys_stat.interface);										
							}
						}
						break;
				 default:break;
			}
			sys_stat.refresh = 0;	
		}
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
	read();

}

void __irq Timer2_ISR(void)
{
	Buzzer_Stop();
	DisableIrq(BIT_TIMER2);
	ClearPending(BIT_TIMER2);
	rTCON &= ~0x001000;// 关定时器
	if(pump_stat.step++<4)
	{	
		rTCNTB2 = pump_stat.period *(pump_stat.step);
		Buzzer_Freq_Set(pump_stat.pwm_rate * pump_stat.step);
	}	
	else if(pump_stat.step == 4)
	{
		rTCNTB2 = pump_stat.period *14;
		Buzzer_Freq_Set(pump_stat.pwm_rate * pump_stat.step);
	}
	else if(pump_stat.step <8)
	{
		rTCNTB2 = pump_stat.period *(8 - pump_stat.step);  
		Buzzer_Freq_Set(pump_stat.pwm_rate * (8 - pump_stat.step));
	}
	else if(pump_stat.step == 8);//停止并且修正；
	{
		rTCON &= ~0x001000;// 关定时器
		return;
	}
	rTCON |= 0x003000;// 更新tcnt,开时器	
	EnableIrq(BIT_TIMER2);	 			
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
//初始化
	default_values();
	if(!default_hardware())
		goto fail;
	load_interface(sys_stat.interface);
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
	touch_stat.ypoint = 0;		//屏幕位置
	touch_stat.xdata = 0;
	touch_stat.ydata = 0; 
	buttoms.start = B_OFF;
	buttoms.stop = B_OFF;
	buttoms.set = B_OFF;
	buttoms.pressure = B_OFF;
	buttoms.back = B_OFF;
	buttoms.p1000 = B_OFF;
	buttoms.m1000 = B_OFF;
	buttoms.p100 = B_OFF;
	buttoms.m100 = B_OFF;
	buttoms.p10 = B_OFF;
	buttoms.m10 = B_OFF;
	buttoms.p1 = B_OFF;
	buttoms.m1 = B_OFF;
	buttoms.p01 = B_OFF;
	buttoms.m01 = B_OFF;
	rawdata.index = 0;
	ad_stat.ad_raw = 0;
	heart_stat.Max = 0;//最大值
	heart_stat.Period = 0;//周期
	heart_stat.Min = 50000;//最小值
	heart_stat.RelaxPeriod = 0;//舒缓期长度
	heart_stat.Ave = 0; //平均值
	heart_stat.updown = BOTTOM;//0->上升期 1->下降期
	heart_stat.ClosePoint = 0;
	heart_stat.delt[0] = 0;	 //delta值，[0]上一次的值，[1]现值
	heart_stat.delt[1] = 0;
	heart_stat.rate = 0;	//心率 
}

uint default_hardware()
{
//io
//timer	  
//LCD
	Lcd_TFT_Init() ;		// LCD initial
//uart
	consoleNum = 0;	// Uart 1 select for debug.
	Uart_Init( 0,115200 );
	Uart_Select( consoleNum );
	Uart_SendByte('\n');
	Uart_Printf("<***************************************>\n");
	Uart_Printf("               LVAD SYSTEM BETA!\n");
	Uart_Printf("                SJTU BME !\n");
	Uart_Printf("                Edit in MDK4.12,by:joe!\n");
//	Uart_Printf("      Build time is: %s  %s\n", __DATE__ , __TIME__  );
	Uart_Printf("<***************************************>\n");
//中断
//ad/touch
	rADCDLY=50000;  //设定ADC开始延迟寄存器的值，使得延迟为:(1/3.6864M)*50000=13.56ms
	rADCCON=(1<<14)|(9<<6);   //rADCCON[14]=1，AD转换预分频器有效，rADCCON[13:6]=9,ADC频率=PCLK/10=5M

	Uart_Printf("\n触摸屏测试开始，请点击触摸屏!\n");

	rADCTSC=(1<<7)|(1<<6)|(0<<5)|(1<<4)|(3);  //？？？

	pISR_ADC = (int)AdcTsAuto;//中断函数注册
	
	rINTMSK=~BIT_ADC;//使能ADC中断
	rINTSUBMSK=~(BIT_SUB_TC);//使能触摸点击子中断
	return SUCCESS;
fail:
	return FAIL;
} 

void load_interface(int n)
{
	switch(n)
	{
		case 0:	Paint_Bmp(0, 0, 800, 480, TQ_LOGO_800480);
				break;
		case 1:	Paint_Bmp(0, 0, 800, 480, Presspic);
				break;
		case 2: Paint_Bmp(0, 0, 800, 480, Setpic);
				break;
		default:break;
	}
}

void starttimer0()
{
	rTCFG0 |= 0x00000001;		//prescaler = 1+1
	rTCFG1 &= ~(0xf);
	rTCFG1 |= 3;				//mux = 1/16
	rTCNTB0 = 7812;			//分频数为2*16=32  50000000/32=62500*25  7812 1/200 10416 1/150 15625 1/100	  3960 1/400
	rTCMPB0 =0;	
	ClearPending(BIT_TIMER0);
	rTCON &= ~0x00001F;	//
	rTCON |= 0x00700b; //
	rTCON &= ~0x000002;
//	Uart_Printf("\nPCLK = %ld\n",PCLK);
	EnableIrq(BIT_TIMER0);
}

void starttimer1()
{}

void starttimer2()
{
		pump_stat.stat = STAT_ON;
		pump_stat.step = 1;
		pump_stat.pwm_rate = theo_conf.volume/4;//theo_conf.volume*(1+theo_conf.compress_ratio)/4;
		pump_stat.period = heart_stat.Period*TIME/20;
		rTCFG0 &= ~(0xff<<8);
		rTCFG0 |= 124<<8;			//prescaler = 124+1
		rTCFG1 &= ~(0xf<<8);
		rTCFG1 |= 2<<8;		//mux = 1/8
		rTCNTB2 = pump_stat.period;	
		rTCMPB2 = 0;
		ClearPending(BIT_TIMER2);
		rTCON &= ~0x00F000;	//清零
		rTCON |= 0x003000; // update	and start
		rTCON &= ~0x002000;//
		EnableIrq(BIT_TIMER2);
		if(pump_stat.direction)
			rGPFDAT=M_p;
		else
			rGPFDAT=M_n;			
		Buzzer_Freq_Set( pump_stat.pwm_rate ) ;

}

/************************自动调整回零********************/
void Auto_zero(void)
{
	while(ReadAdc(AD_Channel)>(Position_zero+4))
	{
		rGPFDAT=M_n;
		Buzzer_Freq_Set(6000);
	}
	while(ReadAdc(AD_Channel)<(Position_zero-4))
	{
		rGPFDAT=M_p;
		Buzzer_Freq_Set(6000);
	}
	Buzzer_Stop();
//	Uart_Printf( "Return to zero!" );	
}
