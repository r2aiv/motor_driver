/*

console.h - определиния ф-ций работы с консолью UART

*/

char ConsoleWrite(void *UARTBuff);
int ConsoleRead(char *UARTBuff);
int ConsoleReadInteger(void);
