#include "main.h"


/*********************
 velocity �����˵��ٶ� 
 speed    �����rpm 
 RPM      ���ת��rpm
 *********************/


M3508_REAL_INFO M3508_MOTOR_REAL_INFO;
PID M3508_PID;


// M3508��ʼ��
void m3508_m3508_motor_init(void)
{
	// PID��ʼ��
	PID_parameter_init(&M3508_PID, 10.0, 1.0, 0.0, 15000.0, 15000.0);
}


// ���õ��ͨ��CAN���������ݸ���m3508��״̬��Ϣ
void m3508_update_m3508_info(CanRxMsg *msg)
{
	switch(msg -> StdId)  // ����׼ID
	{
		case M3508_MOTOR_ID:
		{ 
			M3508_MOTOR_REAL_INFO.ANGLE = (msg -> Data[0] << 8) | msg -> Data[1];    // ת�ӻ�е�Ƕ�
			M3508_MOTOR_REAL_INFO.RPM = (msg -> Data[2] << 8) | msg -> Data[3];      // ʵ��ת��ת��
			M3508_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
		}; break;
		
		default: break;
	}
}


// ͨ��CAN2���͵���m3508�ĵ���
// ����Ƶ�ʣ�1kHz
// ת�ӽǶȷ�Χֵ��0-8191����Ӧ0��360�ȣ�
// ת��ת�ٵ�λΪRPM
// ����¶ȵ�λΪ���϶�
void m3508_send_m3508_currents(int16_t currents, u16 motor_id)
{
	CanTxMsg tx_message;

	// ���ÿ��ƶ�
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	
	// �����ٲöκ����ݶ�
	switch(motor_id)
	{
		case 0x201:
		{		
			tx_message.StdId = 0x200;  // IDΪ 1 2 3 4 �ĵ��
			tx_message.Data[0] = (uint8_t)(currents >> 8);
			tx_message.Data[1] = (uint8_t)currents;
		}
		break;
		
		case 0x202:
		{		
			tx_message.StdId = 0x200;  // IDΪ 1 2 3 4 �ĵ��
			tx_message.Data[2] = (uint8_t)(currents >> 8);
			tx_message.Data[3] = (uint8_t)currents;
		}
		break;
		
		case 0x203:
		{		
			tx_message.StdId = 0x200;  // IDΪ 1 2 3 4 �ĵ��
			tx_message.Data[4] = (uint8_t)(currents >> 8);
			tx_message.Data[5] = (uint8_t)currents;
		}
		break;
		
		case 0x204:
		{		
			tx_message.StdId = 0x200;  // IDΪ 1 2 3 4 �ĵ��
			tx_message.Data[6] = (uint8_t)(currents >> 8);
			tx_message.Data[7] = (uint8_t)currents;
		}
		break;

		case 0x205:
		{		
			tx_message.StdId = 0x1ff;  // IDΪ 5 6 7 8 �ĵ��
			tx_message.Data[0] = (uint8_t)(currents >> 8);
			tx_message.Data[1] = (uint8_t)currents;
		}
		break;

		case 0x206:
		{		
			tx_message.StdId = 0x1ff;  // IDΪ 5 6 7 8 �ĵ��
			tx_message.Data[2] = (uint8_t)(currents >> 8);
			tx_message.Data[3] = (uint8_t)currents;
		}
		break;

		case 0x207:
		{		
			tx_message.StdId = 0x1ff;  // IDΪ 5 6 7 8 �ĵ��
			tx_message.Data[4] = (uint8_t)(currents >> 8);
			tx_message.Data[5] = (uint8_t)currents;
		}
		break;

		case 0x208:
		{		
			tx_message.StdId = 0x1ff;   // IDΪ 5 6 7 8 �ĵ��
			tx_message.Data[6] = (uint8_t)(currents >> 8);
			tx_message.Data[7] = (uint8_t)currents;
		}
		break;		
		default: break;
	}
	
	CAN_Transmit(CAN2, &tx_message);
}


/*WAITING_TEST*/
// ͨ��PID���õ���m3508��ת��ת��
// �����ٶȣ���Ϊ��Ҫ����PID��ԭ�򣬲���ֻ��һ�Σ�Ӧ�ò���ѭ������
void m3508_set_m3508_rpm(int motor_rpm, u16 motor_id)  // ǿ��ת��Ϊ����
{
	// ����ʽPID����
	PID_incremental_PID_calculation(&M3508_PID, M3508_MOTOR_REAL_INFO.RPM, motor_rpm);
	
	// ���;�PID���ں�ĵ���
	m3508_send_m3508_currents(M3508_PID.output, motor_id);
}



