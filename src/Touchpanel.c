/*****************************************
  NAME: Touchpanel.c
  DESC: ADC & Touch screen test
 *****************************************/
#include "def.h"
#include "2440addr.h"
#include "2440lib.h"


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

	xdata=(rADCDAT0&0x3ff);//ת����ɣ����ת��������ֵ
 	ydata=(rADCDAT1&0x3ff);

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

	Uart_Printf("count=%03d  XP=%04d, YP=%04d\n", count++, xdata, ydata);    //X-position Conversion data            

	rADCDLY=saveAdcdly; 
	rADCTSC=rADCTSC&~(1<<8); //��ʼ����갴���ж��ź�
	rSUBSRCPND|=BIT_SUB_TC;
	rINTSUBMSK=~(BIT_SUB_TC);	//���δ�������갴�¡�̧�����ж�     
	ClearPending(BIT_ADC);//
}
