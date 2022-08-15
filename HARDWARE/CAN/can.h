#ifndef __CAN_H
#define __CAN_H	 
#include "sys.h"	



//CAN1����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
void CAN1_Configuration(void);
void CAN1_TX_IRQHandler(void);
void CAN1_RX0_IRQHandler(void);
void Motor_Set_Current(signed short int i1, signed short int i2, signed short int i3, signed short int i4);	
void Lift_Motor_Set_Current(signed short int i5);
typedef struct
{
	uint32_t DesDEVICE_ID   	:8;     //Դ�豸ID
	uint32_t Property		      :8;		  //ָ������ֵ����
	uint32_t SrcDEVICE_ID   	:8;     //Ŀ���豸ID	
	uint32_t Priority				  :4;			//���ȼ�
	uint32_t Permit           :1;
}EXT_ID_Typedef;								  //����29λ��չID
typedef union
{
	uint32_t	all;
	uint32_t	StdID		:11;		//��׼ID
	EXT_ID_Typedef	  ExtID;		//��չID
}ID;

typedef union
{
	int8_t			chars[8];			  //8��char
	int16_t			shorts[4];			//4��short
	int32_t			ints[2];			  //2��int
	int64_t			longs[1];			  //1��Long
	uint8_t			uchars[8];			//8���޷���char
	uint16_t		ushorts[4];			//4���޷���short
	uint32_t		uints[2];			  //2���޷���int
	uint64_t		ulongs[1];			//1���޷���Long
	float       floats[2];
}CAN_Data;								//����CAN��֡�ڵ���������

typedef struct
{
	ID 			id;						//ID
	char		isRemote;			//�Ƿ���Զ��֡
	char 		length;				//���ݳ���
	CAN_Data	data;				//����
}Frame;
#endif

















