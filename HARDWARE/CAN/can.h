#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	



//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
void CAN1_Configuration(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4);	
void Lift_Motor_Set_Current(signed short int i5);
typedef struct
{
	uint32_t DesDEVICE_ID   	:8;     //源设备ID
	uint32_t Property		      :8;		  //指令属性值定义
	uint32_t SrcDEVICE_ID   	:8;     //目标设备ID	
	uint32_t Priority				  :4;			//优先级
	uint32_t Permit           :1;
}EXT_ID_Typedef;								  //定义29位扩展ID
typedef union
{
	uint32_t	all;
	uint32_t	StdID		:11;		//标准ID
	EXT_ID_Typedef	  ExtID;		//扩展ID
}ID;

typedef union
{
	int8_t			chars[8];			  //8个char
	int16_t			shorts[4];			//4个short
	int32_t			ints[2];			  //2个int
	int64_t			longs[1];			  //1个Long
	uint8_t			uchars[8];			//8个无符号char
	uint16_t		ushorts[4];			//4个无符号short
	uint32_t		uints[2];			  //2个无符号int
	uint64_t		ulongs[1];			//1个无符号Long
	float       floats[2];
}CAN_Data;								//定义CAN的帧内的数据类型

typedef struct
{
	ID 			id;						//ID
	char		isRemote;			//是否是远程帧
	char 		length;				//数据长度
	CAN_Data	data;				//数据
}Frame;
#endif

















