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
void Test_Touchpanel(void)//����������
{
   
	rADCDLY=50000;  //�趨ADC��ʼ�ӳټĴ�����ֵ��ʹ���ӳ�Ϊ:(1/3.6864M)*50000=13.56ms
	rADCCON=(1<<14)|(9<<6);   //rADCCON[14]=1��ADת��Ԥ��Ƶ����Ч��rADCCON[13:6]=9,ADCƵ��=PCLK/10=5M

	Uart_Printf("\n���������Կ�ʼ������������!\n");

	rADCTSC=(1<<7)|(1<<6)|(0<<5)|(1<<4)|(3);  //������

	pISR_ADC = (int)AdcTsAuto;//�жϺ���ע��
	
	rINTMSK=~BIT_ADC;//ʹ��ADC�ж�
	rINTSUBMSK=~(BIT_SUB_TC);//ʹ�ܴ���������ж�
	Uart_Getch();//�ȴ��������룬�������˳��ȴ������Խ���
	
	Uart_SendString("\n�������������!!!\n");
	
	rINTSUBMSK|=BIT_SUB_TC;//��ֹ����������ж�
	rINTMSK|=BIT_ADC;//��ֹADC�ж�


}


void __irq AdcTsAuto(void)
{
	U32 saveAdcdly;
	int Xpoint,Ypoint;
	if(rADCDAT0&0x8000)//�жϹ��״̬ADCDAT0[15]��0�����£�1����
	{//���������
		Uart_SendString("\n����Ѿ�����!!\n");//��ӡ���������Ϣ
		rADCTSC&=0xff;	//rADCTSC[8]=0,��ʼ��ⰴ���ж��ź�
	}
	else //����갴��
		Uart_SendString("\n����Ѿ�����!!\n");

	rADCTSC=(1<<3)|(1<<2);//rADCTSC[3]=0,XP������Ч��rADCTSC[2]=1�Զ���������X��Y����
	
	saveAdcdly=rADCDLY;//����ADC��ʼ�ӳټĴ�����ֵ
	rADCDLY=40000;//�����趨ADC��ʼ�ӳټĴ�����ֵ��ʹ���ӳ�Ϊ:(1/50M)*40000=0.8ms

	rADCCON|=0x1;//��ʼADCת��,֮��rADCCON[0]���Զ�������

	while(rADCCON & 0x1);//���rADCCON[0]�Ƿ����㣬������˵���Ѿ���ʼת��������ȴ�
	while(!(rADCCON & 0x8000));//���rADCCON[15]�ж��Ƿ�ADת����������������ȴ�ֱ��ת������
		
	while(!(rSRCPND & (BIT_ADC)));//���ADCת�������ж��Ƿ��������û�з���������ȴ���������������ADC����ֵ

	ydata=(rADCDAT0&0x3ff);
 	xdata=(rADCDAT1&0x3ff);
 	Xpoint=0.89*xdata-55;           //x��y������ADֵת��Ϊ��Ӧ����ֵ
 	Ypoint=0.6*ydata-68;

	rSUBSRCPND|=BIT_SUB_TC;
	//�ֶ��޸����ж�Դ�ǼǼĴ���rSUBSRCPND[9]=1����ʾ��������������ж�Դ�Ѿ������ж����ڵȴ��жϷ���ʹ���������ʹ�����������жϷ���
	
	ClearPending(BIT_ADC);//��rSRCPND[31]=1��rINTPND[32]=1��ʹADC�жϷ����ӳ���ִ��
	
	rINTSUBMSK=~(BIT_SUB_TC);//��ֹ�������������𡢰����ж�
	rINTMSK=~(BIT_ADC);//��ֹADCת���ж�
			 
	rADCTSC =0xd3;    //rADCTSC[1:0]=1 1�ȴ��жϷ�����rADCTSC[7:4]=1 1 0 1
	rADCTSC=rADCTSC|(1<<8); // �����̧���ж��ź�.

	while(1)		//��ѭ�������̧���ж�
	{
		if(rSUBSRCPND & (BIT_SUB_TC))	//�����̧���ж�
		{
			Uart_SendString("\n���̧���жϷ���!\n");
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
					if(Xpoint>=695&&Xpoint<=790&&Ypoint>=370&&Ypoint<=440)		//���·���
					{
						 sys_stat.refresh = 1;
						 buttoms.back = B_ON;
//						 Uart_Printf("< buttom back >\n");						
					}										
				}
				break;
		case 2:	{
					if(Xpoint>=100&&Xpoint<=170&&Ypoint>=280&&Ypoint<=350)        //ѹ����+1000
				  	{
						 sys_stat.refresh = 1;
						 buttoms.p1000 = B_ON;														 
					}else	
					if(Xpoint>=180&&Xpoint<=250&&Ypoint>=280&&Ypoint<=350)        //ѹ����-1000
					{
						 sys_stat.refresh = 1;
						 buttoms.m1000 = B_ON;					
					}else	
					if(Xpoint>=100&&Xpoint<=170&&Ypoint>=370&&Ypoint<=440)        //ѹ����+100
					{
						 sys_stat.refresh = 1;
						 buttoms.p100 = B_ON;					
					}else				
					if(Xpoint>=180&&Xpoint<=250&&Ypoint>=370&&Ypoint<=440)        //ѹ����-100
					{
						 sys_stat.refresh = 1;
						 buttoms.m100 = B_ON;					
					}else			
					if(Xpoint>=310&&Xpoint<=380&&Ypoint>=280&&Ypoint<=350)        //����+10
					{
						 sys_stat.refresh = 1;
						 buttoms.p10 = B_ON;					
					}else
					if(Xpoint>=390&&Xpoint<=460&&Ypoint>=280&&Ypoint<=350)        //����-10
					{
						 sys_stat.refresh = 1;
						 buttoms.m10 = B_ON;					
					}else
					if(Xpoint>=310&&Xpoint<=380&&Ypoint>=370&&Ypoint<=440)        //����+1
					{
						 sys_stat.refresh = 1;
						 buttoms.p1 = B_ON;					
					}else
					if(Xpoint>=390&&Xpoint<=460&&Ypoint>=370&&Ypoint<=440)        //����-1
					{
						 sys_stat.refresh = 1;
						 buttoms.m1 = B_ON;					
					}else
					if(Xpoint>=560&&Xpoint<=630&&Ypoint>=280&&Ypoint<=350)        //ѹ����+0.1
					{
						 sys_stat.refresh = 1;
						 buttoms.p01 = B_ON;					
					}else			
					if(Xpoint>=560&&Xpoint<=630&&Ypoint>=370&&Ypoint<=440)        //ѹ����-0.1
					{
						 sys_stat.refresh = 1;
						 buttoms.m01 = B_ON;					
					}else
					if(Xpoint>=670&&Xpoint<=770&&Ypoint>=310&&Ypoint<=405)      //ȷ���趨
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
	rADCTSC=rADCTSC&~(1<<8); //��ʼ����갴���ж��ź�
	rSUBSRCPND|=BIT_SUB_TC;
	rINTSUBMSK=~(BIT_SUB_TC);	//���δ�������갴�¡�̧�����ж�     
	ClearPending(BIT_ADC);//
}
