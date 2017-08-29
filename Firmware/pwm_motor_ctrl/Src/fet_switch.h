/*

fet_switch.h - ����������� ������� ���������� �������� �������

*/

#define STATE_FORWARD 	0x01
#define STATE_BACKWARD 	0x02
#define STATE_HARDSTOP	0x03
#define STATE_STOP 			0x00

void FET_ForwardOn(void);    	// �������� ������ �������� �������
void FET_BackwardOn(void);   	// �������� �������� �������� �������
void FET_Stop(void);					// ������� (��������� ���������� � �������� ������)
void FET_HardStop(void);			// ������� ������� (����������� ����� Q3 � Q5, ������� ��������������)
void FET_ForwardPWM(unsigned char pwm);		// ��������� ��������� � ������������ ����������� ���
void FET_ForwardPWM(unsigned char pwm);	  // �������� �������� �� ���
void MagnetOn(void); 					// ��������� �������
void MagnetOff(void); 				// ���������� �������
void FET_SoftStart(void);			// ������ ����
void FET_SoftStop(void);			// ������� ������� 






