extern 	SYSTEM_STAT sys_stat;
extern	THERAPY_CONFIG theo_conf;
extern	TOUCH_STAT touch_stat;
extern	BUTTOMS buttoms;
extern	DATA_STAT rawdata;
extern	AD_STAT ad_stat;
extern	HEAT_STAT heart_stat;
extern	PUMP_STAT pump_stat;

/*
struct SYSTEM_STAT{
	uint  interface;			//��������  0.������ 1.��������
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
*/
