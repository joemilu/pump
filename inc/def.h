#ifndef __DEF_H__
#define __DEF_H__

#define U32 unsigned int
#define U16 unsigned short
#define S32 int
#define S16 short int
#define U8  unsigned char
#define	S8  char

typedef unsigned char BOOL;
typedef unsigned char UCHAR;
typedef UCHAR *PUCHAR;
typedef unsigned long DWORD;
typedef DWORD *PDWORD;
typedef unsigned char   BYTE;
typedef unsigned short  WORD;

typedef unsigned long ULONG;
typedef ULONG *PULONG;
typedef unsigned short USHORT;
typedef USHORT *PUSHORT;

typedef BYTE *LPBYTE;

typedef void *PVOID;



#define	BYTE	char
#define	WORD 	short
#define	DWORD	int
#define	UINT	U32
#define	LPSTR	U8 *		

#define TRUE 	1   
#define FALSE 	0
#define OK		1
#define FAIL	0

#define	SIZE_1K		0x00000400
#define	SIZE_2K		0x00000800
#define	SIZE_4K		0x00001000
#define	SIZE_8K		0x00002000
#define	SIZE_16K	0x00004000
#define	SIZE_32K	0x00008000
#define	SIZE_64K	0x00010000
#define	SIZE_128K	0x00020000
#define	SIZE_256K	0x00040000
#define	SIZE_512K	0x00080000
#define	SIZE_1M		0x00100000
#define	SIZE_2M		0x00200000
#define	SIZE_4M		0x00400000
#define	SIZE_8M		0x00800000
#define	SIZE_16M	0x01000000
#define	SIZE_32M	0x02000000
#define	SIZE_64M	0x04000000
#define	SIZE_128M	0x08000000
#define	SIZE_256M	0x10000000
#define	SIZE_512M	0x20000000
#define	SIZE_1G		0x40000000
#define	SIZE_2G		0x80000000

#define	ENTER_KEY	0x0d
#define	BACK_KEY	0x08
#define BEEP_KEY	0x07
#define UP_KEY		0x41
#define DOWN_KEY	0x42
#define RIGHT_KEY	0x43
#define LEFT_KEY	0x44
#define HOME_KEY	0x48
#define END_KEY		0x4b
#define	ESC_KEY		0x1b

//////////////////////////////////////////////////////////////////////////
#define uint unsigned int
#define uchar unsigned char
#define SUCCESS 1
#define FAIL 0

#define DEFAULT_CR 1
#define DEFAULT_VO 5500
#define DEFAULT_HB 60
#define B_ON 1
#define B_OFF 0

#define YULIANG 5.0f
#define YULIANG_1 3.0f
#define HCLOSED 1
#define BOTTOM 0
#define TOP 2
#define Position_zero 260			//�˶���ԭʼ���
#define M_p 0x08				//����ת
#define M_n 0x10
#define AD_Channel 1

#define SAMPLE_RATE	200	//������ 200HZ
#define INI_SAMPLE_PERIOD SAMPLE_RATE*20	//��ʼ������ʱ��
#define BUFFER_SZ	200	//��������С
#define FILTER_SZ_1	3//�˲�����
#define FILTER_SZ	5//�˲�����	

#define STAT_START 1
#define STAT_STOP 3
#define STAT_ON 2
#define TIME 1 //ʱ�任�㵥λ ����ʱ�䡪��>��ʱ��ʱ��


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
	uint set;
	uint pressure;
	uint back;
	uint p1000;
	uint m1000;
	uint p100;
	uint m100;
	uint p10;
	uint m10;
	uint p1;
	uint m1;
	uint p01;
	uint m01;
};

struct DATA_STAT{
	uint index;
	uint useful;//�����ǹ���Ч
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
	uint period;//��ʱ��ʱ��
};		

#endif /*__DEF_H__*/