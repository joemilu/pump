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
#define Position_zero 260			//运动的原始零点
#define M_p 0x08				//正反转
#define M_n 0x10
#define AD_Channel 1

#define SAMPLE_RATE	200	//采样率 200HZ
#define INI_SAMPLE_PERIOD SAMPLE_RATE*20	//初始化采用时间
#define BUFFER_SZ	200	//缓存区大小
#define FILTER_SZ_1	3//滤波长度
#define FILTER_SZ	5//滤波长度	

#define STAT_START 1
#define STAT_STOP 3
#define STAT_ON 2
#define TIME 1 //时间换算单位 采用时间——>定时器时间


struct SYSTEM_STAT{
	uint  interface;			//所处界面
	uint refresh;
};

struct THERAPY_CONFIG{
	float compress_ratio; 	//压缩比
	double volume;		    //压缩总量
	uint heart_beat;		
};

struct TOUCH_STAT{
	uint xpoint,ypoint;		//屏幕位置
	uint xdata,ydata;  		//ts采样值
};

//按钮状态
struct BUTTOMS{
	uint start;		//开始工作
	uint stop; 		//结束工作
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
	uint useful;//数据是够有效
};

struct AD_STAT{
	uint ad_raw;
};

struct HEAT_STAT{
	uint Max;//最大值
	uint Period;//周期
	uint Min;//最小值
	uint RelaxPeriod;//舒缓期长度
	uint Ave; //平均值
//	uint useful;//数据是够有效
	uint updown;//0->上升期 1->下降期
	uint ClosePoint;
	float delt[2];	 //delta值，[0]上一次的值，[1]现值
	uint rate;	//心率
};

struct PUMP_STAT{
	uint direction;
	uint step;
	uint pwm_rate;
	uint stat;
	uint pos;
	uint period;//定时器时间
};		

#endif /*__DEF_H__*/