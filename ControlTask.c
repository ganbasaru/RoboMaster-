#include "main.h"

#define M2006_SPEED 1800

static rampfun_str GM_YAW_R_ramp, GM_YAW_L_ramp, GM_PIT_R_ramp, GM_PIT_L_ramp;	//Yaw轴云台的斜波结构体

static int PC_R_depth, PC_L_depth;	//x,y,深度信息

int Point_mid[2] = {64, 64};

/*工作模数更改*/
static Work_State WorkState = PREPARE_STATE;	//上电后停止工作状态
static Cartridge_State Now_R_Cartridge_State = Cartridge_Close;	//上电后关闭
static Cartridge_State Now_L_Cartridge_State = Cartridge_Close;	//上电后关闭

extern void Set_WorkState(Work_State workstate)	//在遥控中改变工作状态
{
    WorkState = workstate;
}

/*修改拨弹状态，在ShootTask文件中调用*/
extern void Set_R_Cartridge_State(Cartridge_State Cartridge_State)	//在射击循环函数中改变拨弹状态
{
    Now_R_Cartridge_State = Cartridge_State;
}
extern void Set_L_Cartridge_State(Cartridge_State Cartridge_State)	//在射击循环函数中改变拨弹状态
{
    Now_L_Cartridge_State = Cartridge_State;
}

/*获取拨弹状态，判断是否开启拨弹轮*/
extern Cartridge_State Get_R_CartridgeState(void)	
{
	return Now_R_Cartridge_State;
}
extern Cartridge_State Get_L_CartridgeState(void)	
{
	return Now_L_Cartridge_State;
}

extern Work_State Get_WorkState()				//获取工作状态
{
	return WorkState;
}


void CMControlLoop(void)
{
	int16_t SET_M2006_R_SPEED = M2006_SPEED;	//左右拨弹速度设置
	int16_t SET_M2006_L_SPEED = M2006_SPEED;
	static uint8_t R_flag = 0, L_flag = 0;				//左右卡弹标志位
	static double R_KA_TIME = 0 , L_KA_TIME = 0;		//卡弹时间记录
	const Encoder *CM1SPEED = GetM3508Encoder(0);
	const Encoder *CM2SPEED = GetM3508Encoder(1);
	const Encoder *GM_PITRecd = GetGM6020Encoder(0);//获取云台电机角度
	const Encoder *GM_PITLecd = GetGM6020Encoder(1);
	const Encoder *GM_YawLecd = GetGM6020Encoder(2);
	const Encoder *GM_YawRecd = GetGM6020Encoder(3);
	const Encoder *M2006_R_SPEED = GetM2006Encoder(0);
	const Encoder *M2006_L_SPEED = GetM2006Encoder(1);
	int16_t YawRecd, YawLecd, PitRecd, PitLecd;
	u8 power = 5;	//扭矩
	u8 power_2006 = 6;
	
	YawRecd = Liehu(RCYawR_result, GM_YawRecd->ecd);
	YawLecd = Liehu(RCYawL_result, GM_YawLecd->ecd);
	PitRecd = Liehu(RCPitR_result, GM_PITRecd->ecd);
	PitLecd = Liehu(RCPitL_result, GM_PITLecd->ecd);
	
	//底盘旋转量计算
	if(WorkState == PREPARE_STATE) 		//初始化状态
	{
		GET_PC();
		/*PITCH轴*/
		ramp_init(&GM_PIT_R_ramp, 1, 8192, 0);	//云台归位的斜波结构体初始化
		ramp_init(&GM_PIT_L_ramp, 1, 8192, 0);	//云台归位的斜波结构体初始化
		GM_PIT_R_ramp.out = RCPitR_result;
		GM_PIT_L_ramp.out = RCPitL_result;
		/*YAW轴*/
		ramp_init(&GM_YAW_R_ramp, 1, 8192, R_YAW_INIT);	//云台归位的斜波结构体初始化
		ramp_init(&GM_YAW_L_ramp, 1, 8192, L_YAW_INIT);	//云台归位的斜波结构体初始化
		GM_YAW_R_ramp.out = RCYawR_result;
		GM_YAW_L_ramp.out = RCYawL_result;
		
		/***********************卡弹判断***********************/
		if(Now_R_Cartridge_State && (M2006_R_SPEED->speed_rpm == 0))	
		{
			flow_led_on(0);
			R_KA_TIME ++;
			if (R_KA_TIME == 15)	R_flag = 1;//0.1S
		}
		if (Now_R_Cartridge_State && R_flag)	
		{
			flow_led_on(0);
			SET_M2006_R_SPEED = -2000;
			R_KA_TIME -= 0.5;
			if(R_KA_TIME == 0) R_flag = 0;
		}
		if(Now_L_Cartridge_State && (M2006_L_SPEED->speed_rpm == 0))
		{
			flow_led_on(1);
			L_KA_TIME ++;
			if (L_KA_TIME == 15)	L_flag = 1;//0.1S
		}
		if (Now_L_Cartridge_State && L_flag)	
		{
			flow_led_on(1);
			SET_M2006_L_SPEED = -2000;
			L_KA_TIME -= 0.5;
			if(L_KA_TIME == 0) L_flag = 0;
		}
		/***********************卡弹判断***********************/
		
		CanSend_Chassis((RUN_result - CM1SPEED->speed_rpm)*power, (RUN_result - CM2SPEED->speed_rpm)*power, 
						(Now_R_Cartridge_State * SET_M2006_R_SPEED - M2006_R_SPEED->speed_rpm) * power_2006,
						(Now_L_Cartridge_State * SET_M2006_L_SPEED - M2006_L_SPEED->speed_rpm) * power_2006);
		/*云台部分*/
//		if (!Now_Cartridge_State)	CanSend_Gimbal_Plucking(Yawecd * power, 0, 0);	//停止拨弹前要先这样操作一下，不知道原因
		CanSend_Gimbal_Plucking(PitRecd * power, PitLecd * power, YawLecd * power, YawRecd * power);		
	}

										//MiniPC控制模式
	else if(WorkState == PC_Control)
	{
		GET_PC();
		
		/***********************卡弹判断***********************/
		if(Now_R_Cartridge_State && (M2006_R_SPEED->speed_rpm == 0))	
		{
			flow_led_on(0);
			R_KA_TIME ++;
			if (R_KA_TIME == 15)	R_flag = 1;//0.1S
		}
		if (Now_R_Cartridge_State && R_flag)	
		{
			flow_led_on(0);
			SET_M2006_R_SPEED = -2000;
			R_KA_TIME -= 0.5;
			if(R_KA_TIME == 0) R_flag = 0;
		}
		if(Now_L_Cartridge_State && (M2006_L_SPEED->speed_rpm == 0))
		{
			flow_led_on(1);
			L_KA_TIME ++;
			if (L_KA_TIME == 15)	L_flag = 1;//0.1S
		}
		if (Now_L_Cartridge_State && L_flag)	
		{
			flow_led_on(1);
			SET_M2006_L_SPEED = -2000;
			L_KA_TIME -= 0.5;
			if(L_KA_TIME == 0) L_flag = 0;
		}
		/***********************卡弹判断***********************/
		
		/*PITCH轴*/
		ramp_init(&GM_PIT_R_ramp, 1, 8192, 0);	//云台归位的斜波结构体初始化
		ramp_init(&GM_PIT_L_ramp, 1, 8192, 0);	//云台归位的斜波结构体初始化
		GM_PIT_R_ramp.out = RCPitR_result;
		GM_PIT_L_ramp.out = RCPitL_result;
		/*YAW轴*/
		ramp_init(&GM_YAW_R_ramp, 1, 8192, R_YAW_INIT);	//云台归位的斜波结构体初始化
		ramp_init(&GM_YAW_L_ramp, 1, 8192, L_YAW_INIT);	//云台归位的斜波结构体初始化
		GM_YAW_R_ramp.out = RCYawR_result;
		GM_YAW_L_ramp.out = RCYawL_result;
		
		CanSend_Chassis((RUN_result - CM1SPEED->speed_rpm)*power, (RUN_result - CM2SPEED->speed_rpm)*power, 
						(Now_R_Cartridge_State * SET_M2006_R_SPEED - M2006_R_SPEED->speed_rpm) * power_2006,
						(Now_L_Cartridge_State * SET_M2006_L_SPEED - M2006_L_SPEED->speed_rpm) * power_2006);
		
		CanSend_Gimbal_Plucking(PitRecd * power, PitLecd * power, YawLecd * power, YawRecd * power);		
	}
	
	else								//停止工作状态
	{
		/*斜坡函数*/
		if (GM_PIT_R_ramp.out > 4096)	 ramp_fun(&GM_PIT_R_ramp, 50);
		else	ramp_fun(&GM_PIT_R_ramp, -50);
		if (GM_PIT_L_ramp.out > 4096)	 ramp_fun(&GM_PIT_L_ramp, 50);
		else	ramp_fun(&GM_PIT_L_ramp, -50);
		if (GM_YAW_R_ramp.out > 4096)	 ramp_fun(&GM_YAW_R_ramp, 50);
		else	ramp_fun(&GM_YAW_R_ramp, -50);
		if (GM_YAW_L_ramp.out > 4096)	 ramp_fun(&GM_YAW_L_ramp, 50);
		else	ramp_fun(&GM_YAW_L_ramp, -50);
		RCPitR_result = GM_PIT_R_ramp.out;
		RCPitL_result = GM_PIT_L_ramp.out;
		RCYawL_result = GM_YAW_L_ramp.out;
		RCYawR_result = GM_YAW_R_ramp.out;
		
		/*底盘停止*/
		CanSend_Chassis((0 - CM1SPEED->speed_rpm)*power, (0 - CM2SPEED->speed_rpm)*power, 
						(0 - M2006_R_SPEED->speed_rpm) * power_2006,
						(0 - M2006_L_SPEED->speed_rpm) * power_2006);
		/*云台归位*/		
//		if (!Now_Cartridge_State)	CanSend_Gimbal_Plucking(Yawecd * power, 0, 0);	//停止拨弹前要先这样操作一下，不知道原因
		CanSend_Gimbal_Plucking(Liehu(GM_PIT_R_ramp.out, GM_PITRecd->ecd) * power, Liehu(GM_PIT_L_ramp.out, GM_PITLecd->ecd) * power,
								Liehu(GM_YAW_L_ramp.out, GM_YawLecd->ecd) * power, Liehu(GM_YAW_R_ramp.out, GM_YawRecd->ecd) * power);
	}
}

/*处理成劣弧*/
int16_t Liehu(int16_t  RC_result, int16_t GM_Yawecd)
{
	int16_t Yawecd;
	/*************************************保证以一个劣弧旋转**************************************/
	if((RC_result - GM_Yawecd) > 8192 / 2)	Yawecd = RC_result - GM_Yawecd - 8192;
	else if ((RC_result - GM_Yawecd) < -8192 / 2)	Yawecd = RC_result - GM_Yawecd + 8192;
	else Yawecd = RC_result - GM_Yawecd;
	
	return Yawecd;
}

void GET_PC(void)
{
	int flag = 1;	//数据正负标志位
	u16 len, shu, num;
	int PC_L_Y = 0, PC_L_P = 0, PC_R_Y = 0, PC_R_P = 0;
	
	if (USART_RX_STA_L & 0x8000)	//usart_6 左侧云台的串口控制数据
	{
		shu = 0;
		len = USART_RX_STA_L & 0x3fff;
		
		for(num = 0; num < len; num ++)
		{
			if (USART_RX_BUF_L[num] == '-')
			{
				flag = -1;
			}
			else if (USART_RX_BUF_L[num] == '+');
			
			else	shu = shu*10 + (USART_RX_BUF_L[num] - '0');
			
			if (num == 3)	//x坐标接收完毕
			{
				PC_L_Y = flag * shu;
				shu = 0;
				flag = 1;
			}
			if (num == 7)	//y坐标接收完毕
			{
				PC_L_P = flag * shu;
				shu = 0;
				flag = 1;
			}
			if (num == 11)	//深度坐标接收完毕
			{
				PC_L_depth = shu;
				shu = 0;
			}
		}
		USART_RX_STA_L = 0;
	}
	if (USART_RX_STA_R & 0x8000)	//usart_6 左侧云台的串口控制数据
	{
		shu = 0;
		len = USART_RX_STA_R & 0x3fff;
		
		for(num = 0; num < len; num ++)
		{
			if (USART_RX_BUF_R[num] == '-')
			{
				flag = -1;
			}
			else if (USART_RX_BUF_R[num] == '+');
			else	shu = shu*10 + (USART_RX_BUF_R[num] - '0');
			
			if (num == 3)	//x坐标接收完毕
			{
				PC_R_Y = flag * shu;
				shu = 0;
				flag = 1;
			}
			if (num == 7)	//y坐标接收完毕
			{
				PC_R_P = flag * shu;
				shu = 0;
				flag = 1;
			}
			if (num == 11)	//深度坐标接收完毕
			{
				PC_R_depth = shu;
				shu = 0;
			}
		}
		USART_RX_STA_R = 0;
	}
}

