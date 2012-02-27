/*****************************************
  NAME: Touchpanel.c
  DESC: ADC & Touch screen test
 *****************************************/
#include "def.h"
#include "2440addr.h"
#include "2440lib.h"
#include "global.h"

#define REQCNT 30
#define ADCPRS 9
#define LOOP 1

void __irq AdcTsAuto(void);

int count=0;
volatile int xdata, ydata;
/*    
void Test_Touchpanel(void)
{
   
	rADCDLY=50000;                  //Normal conversion mode delay about (1/3.6864M)*50000=13.56ms
	rADCCON=(1<<14)+(ADCPRS<<6);   //ADCPRS En, ADCPRS Value

	Uart_Printf("\nTouch Screen test\n");

	rADCTSC=0xd3;  //Wfait,XP_PU,XP_Dis,XM_Dis,YP_Dis,YM_En

	pISR_ADC = (int)AdcTsAuto;
	rINTMSK=~BIT_ADC;       //ADC Touch Screen Mask bit clear
	rINTSUBMSK=~(BIT_SUB_TC);

	Uart_Printf("\nPress any key to quit!\n");
	Uart_Printf("\nStylus Down, please...... \n");
	Uart_Getch();

	rINTSUBMSK|=BIT_SUB_TC;
	rINTMSK|=BIT_ADC;
	Uart_Printf("Touch Screen Test is Finished!!!\n");

}


void __irq AdcTsAuto(void)
{
	U32 saveAdcdly;

	if(rADCDAT0&0x8000)
	{
		//Uart_Printf("\nStylus Up!!\n");
		rADCTSC&=0xff;	// Set stylus down interrupt bit
	}
	//else 
		//Uart_Printf("\nStylus Down!!\n");

	rADCTSC=(1<<3)|(1<<2);         //Pull-up disable, Seq. X,Y postion measure.
	saveAdcdly=rADCDLY;
	rADCDLY=40000;                 //Normal conversion mode delay about (1/50M)*40000=0.8ms

	rADCCON|=0x1;                   //start ADC

	while(rADCCON & 0x1);		//check if Enable_start is low
	while(!(rADCCON & 0x8000));        //check if EC(End of Conversion) flag is high, This line is necessary~!!
		
	while(!(rSRCPND & (BIT_ADC)));  //check if ADC is finished with interrupt bit

	xdata=(rADCDAT0&0x3ff);
 	ydata=(rADCDAT1&0x3ff);

	//check Stylus Up Interrupt.
	rSUBSRCPND|=BIT_SUB_TC;
	ClearPending(BIT_ADC);
	rINTSUBMSK=~(BIT_SUB_TC);
	rINTMSK=~(BIT_ADC);
			 
	rADCTSC =0xd3;    //Waiting for interrupt
	rADCTSC=rADCTSC|(1<<8); // Detect stylus up interrupt signal.

	while(1)		//to check Pen-up state
	{
		if(rSUBSRCPND & (BIT_SUB_TC))	//check if ADC is finished with interrupt bit
		{
			//Uart_Printf("Stylus Up Interrupt~!\n");
			break;	//if Stylus is up(1) state
		}
	}	

	Uart_Printf("count=%03d  XP=%04d, YP=%04d\n", count++, xdata, ydata);    //X-position Conversion data            

	rADCDLY=saveAdcdly; 
	rADCTSC=rADCTSC&~(1<<8); // Detect stylus Down interrupt signal.
	rSUBSRCPND|=BIT_SUB_TC;
	rINTSUBMSK=~(BIT_SUB_TC);	// Unmask sub interrupt (TC)     
	ClearPending(BIT_ADC);
}
*/
void Test_Touchpanel(void)//触摸屏测试
{
   
	rADCDLY=50000;  //设定ADC开始延迟寄存器的值，使得延迟为:(1/3.6864M)*50000=13.56ms
	rADCCON=(1<<14)|(9<<6);   //rADCCON[14]=1，AD转换预分频器有效，rADCCON[13:6]=9,ADC频率=PCLK/10=5M

	Uart_Printf("\n触摸屏测试开始，请点击触摸屏!\n");

	rADCTSC=(1<<7)|(1<<6)|(0<<5)|(1<<4)|(3);  //？？？

	pISR_ADC = (int)AdcTsAuto;//中断函数注册
	
	rINTMSK=~BIT_ADC;//使能ADC中断
	rINTSUBMSK=~(BIT_SUB_TC);//使能触摸点击子中断
	Uart_Getch();//等待键盘输入，有输入退出等待，测试结束
	
	Uart_SendString("\n触摸屏测试完毕!!!\n");
	
	rINTSUBMSK|=BIT_SUB_TC;//禁止触摸点击子中断
	rINTMSK|=BIT_ADC;//禁止ADC中断


}


void __irq AdcTsAuto(void)
{
	U32 saveAdcdly;
	int Xpoint,Ypoint;
	if(rADCDAT0&0x8000)//判断光标状态ADCDAT0[15]，0：按下，1提起
	{//若光标提起
		Uart_SendString("\n光标已经提起!!\n");//打印光标提起信息
		rADCTSC&=0xff;	//rADCTSC[8]=0,开始检测按下中断信号
	}
	else //若光标按下
		Uart_SendString("\n光标已经按下!!\n");

	rADCTSC=(1<<3)|(1<<2);//rADCTSC[3]=0,XP上拉无效、rADCTSC[2]=1自动连续测量X，Y坐标
	
	saveAdcdly=rADCDLY;//保存ADC开始延迟寄存器的值
	rADCDLY=40000;//重新设定ADC开始延迟寄存器的值，使得延迟为:(1/50M)*40000=0.8ms

	rADCCON|=0x1;//开始ADC转换,之后rADCCON[0]会自动被清零

	while(rADCCON & 0x1);//检测rADCCON[0]是否被清零，若是则说明已经开始转换，否则等待
	while(!(rADCCON & 0x8000));//检测rADCCON[15]判断是否AD转换结束，否则继续等待直到转换结束
		
	while(!(rSRCPND & (BIT_ADC)));//检测ADC转换结束中断是否发生，如果没有发生则继续等待，如果发生则输出ADC量化值

	ydata=(rADCDAT0&0x3ff);
 	xdata=(rADCDAT1&0x3ff);
 	Xpoint=0.89*xdata-55;           //x、y坐标由AD值转换为对应象素值
 	Ypoint=0.6*ydata-68;

	rSUBSRCPND|=BIT_SUB_TC;
	//手动修改子中断源登记寄存器rSUBSRCPND[9]=1，表示触摸屏按键光标中断源已经申请中断且在等待中断服务，使用软件方法使触摸屏按下中断发生
	
	ClearPending(BIT_ADC);//将rSRCPND[31]=1，rINTPND[32]=1，使ADC中断服务子程序执行
	
	rINTSUBMSK=~(BIT_SUB_TC);//禁止触摸屏按键提起、按下中断
	rINTMSK=~(BIT_ADC);//禁止ADC转换中断
			 
	rADCTSC =0xd3;    //rADCTSC[1:0]=1 1等待中断发生，rADCTSC[7:4]=1 1 0 1
	rADCTSC=rADCTSC|(1<<8); // 检测光标抬起中断信号.

	while(1)		//死循环检测光标抬起中断
	{
		if(rSUBSRCPND & (BIT_SUB_TC))	//检测光标抬起中断
		{
			Uart_SendString("\n光标抬起中断发生!\n");
			break;
		}
	}	

	Uart_Printf("count=%03d  XP=%04d, YP=%04d\n", count++, Xpoint, Ypoint);    //X-position Conversion data   


	switch(sys_stat.interface)
	{
		case 0:	{
					if(Xpoint>=50&&Xpoint<=200&&Ypoint>=265&&Ypoint<=415)
					{
						 sys_stat.refresh = 1;
						 buttoms.start = B_ON;
					}else if(Xpoint>=416&&Xpoint<=566&&Ypoint>=265&&Ypoint<=415)
					{
						 sys_stat.refresh = 1;
						 buttoms.stop = B_ON;						 
					}else if(Xpoint>=233&&Xpoint<=383&&Ypoint>=265&&Ypoint<=415)
					{
						 sys_stat.refresh = 1;
						 buttoms.set = B_ON;						
					}else if(Xpoint>=600&&Xpoint<=750&&Ypoint>=265&&Ypoint<=415)
					{
						 sys_stat.refresh = 1;
						 buttoms.pressure = B_ON;
						 Uart_Printf("< buttom pressure >\n");						 						
					}
				}
				break;
		case 1:	{
					if(Xpoint>=695&&Xpoint<=790&&Ypoint>=370&&Ypoint<=440)		//按下返回
					{
						 sys_stat.refresh = 1;
						 buttoms.back = B_ON;
//						 Uart_Printf("< buttom back >\n");						
					}										
				}
				break;
		case 2:	{
					if(Xpoint>=100&&Xpoint<=170&&Ypoint>=280&&Ypoint<=350)        //压缩量+1000
				  	{
						 sys_stat.refresh = 1;
						 buttoms.p1000 = B_ON;														 
					}else	
					if(Xpoint>=180&&Xpoint<=250&&Ypoint>=280&&Ypoint<=350)        //压缩量-1000
					{
						 sys_stat.refresh = 1;
						 buttoms.m1000 = B_ON;					
					}else	
					if(Xpoint>=100&&Xpoint<=170&&Ypoint>=370&&Ypoint<=440)        //压缩量+100
					{
						 sys_stat.refresh = 1;
						 buttoms.p100 = B_ON;					
					}else				
					if(Xpoint>=180&&Xpoint<=250&&Ypoint>=370&&Ypoint<=440)        //压缩量-100
					{
						 sys_stat.refresh = 1;
						 buttoms.m100 = B_ON;					
					}else			
					if(Xpoint>=310&&Xpoint<=380&&Ypoint>=280&&Ypoint<=350)        //心率+10
					{
						 sys_stat.refresh = 1;
						 buttoms.p10 = B_ON;					
					}else
					if(Xpoint>=390&&Xpoint<=460&&Ypoint>=280&&Ypoint<=350)        //心率-10
					{
						 sys_stat.refresh = 1;
						 buttoms.m10 = B_ON;					
					}else
					if(Xpoint>=310&&Xpoint<=380&&Ypoint>=370&&Ypoint<=440)        //心率+1
					{
						 sys_stat.refresh = 1;
						 buttoms.p1 = B_ON;					
					}else
					if(Xpoint>=390&&Xpoint<=460&&Ypoint>=370&&Ypoint<=440)        //心率-1
					{
						 sys_stat.refresh = 1;
						 buttoms.m1 = B_ON;					
					}else
					if(Xpoint>=560&&Xpoint<=630&&Ypoint>=280&&Ypoint<=350)        //压缩比+0.1
					{
						 sys_stat.refresh = 1;
						 buttoms.p01 = B_ON;					
					}else			
					if(Xpoint>=560&&Xpoint<=630&&Ypoint>=370&&Ypoint<=440)        //压缩比-0.1
					{
						 sys_stat.refresh = 1;
						 buttoms.m01 = B_ON;					
					}else
					if(Xpoint>=670&&Xpoint<=770&&Ypoint>=310&&Ypoint<=405)      //确定设定
					{
						 sys_stat.refresh = 1;
						 buttoms.back = B_ON;
//						 Uart_Printf("< buttom back >\n");					
					}					
				}
				break;
		 default:break;
	}
         

	rADCDLY=saveAdcdly; 
	rADCTSC=rADCTSC&~(1<<8); //开始检测光标按下中断信号
	rSUBSRCPND|=BIT_SUB_TC;
	rINTSUBMSK=~(BIT_SUB_TC);	//屏蔽触摸屏光标按下、抬起子中断     
	ClearPending(BIT_ADC);//
}
