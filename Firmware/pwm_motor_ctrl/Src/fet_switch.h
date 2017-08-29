/*

fet_switch.h - Определение функций управления силовыми ключами

*/

#define STATE_FORWARD 	0x01
#define STATE_BACKWARD 	0x02
#define STATE_HARDSTOP	0x03
#define STATE_STOP 			0x00

void FET_ForwardOn(void);    	// Включает прямое вращение привода
void FET_BackwardOn(void);   	// Включает обратное вращение привода
void FET_Stop(void);					// Останов (снимается напряжение с затворов ключей)
void FET_HardStop(void);			// Жесткий останов (открываются ключи Q3 и Q5, обмотки закорачиваются)
void FET_ForwardPWM(unsigned char pwm);		// Включение двигателя с определенной скважностью ШИМ
void FET_ForwardPWM(unsigned char pwm);	  // Обратное вращение по ШИМ
void MagnetOn(void); 					// Включение магнита
void MagnetOff(void); 				// Выключение магнита
void FET_SoftStart(void);			// Мягкий пуск
void FET_SoftStop(void);			// Плавный останов 






