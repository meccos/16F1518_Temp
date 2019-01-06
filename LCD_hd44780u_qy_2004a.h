//LCD_hd44780u_qy_2004a.h

#define _XTAL_FREQ 16000000 //for __delay_ms it need to know the cpu speed

#define wInterruptTextSize 200

void initLCD(void);

void setData(char iValue);
void writeInsChk(char iOpCode);
void writeInsNoChk(char iOpCode);
void writeTxtChk(char iOpCode);
void waitLCDBusy(void);
void SetReadDataFromLCD(void);
void SetToSendDataToLCD(void);
void lcdWriteText(char *iText);
void lcdWriteRotText(char *iRotText, char ioRotReadPtr, char iWritePtr);
void powerOffLcd();
void powerOnLcd();
void setBlinkingCursor();
void setNotBlinkingCursor();
void setCursorOff();
void setCursorOn();
void setCursorMovingRight();
void setCursorMovingLeft();
void setCursorPosition(char iPosition);
void setDisplayMovingRight();
void setDisplayMovingLeft();
void moveCursorRight();
void moveCursorLeft();
void clearDisplay();
void moveCursorToHome();
void delay2us();



char mDisplayOnOffReg;
char mCursorDisplayShiftReg;
char mWritingPosition;

#define DDRAM_Address_Line_1_Position_1 0x00
#define DDRAM_Address_Line_2_Position_1 0x40
#define DDRAM_Address_Line_3_Position_1 0x14
#define DDRAM_Address_Line_4_Position_1 0x54

#define RS PORTAbits.RA4
#define RSDirection TRISAbits.TRISA4
#define RW PORTAbits.RA7
#define RWDirection TRISAbits.TRISA7
#define E PORTAbits.RA5
#define EDirection TRISAbits.TRISA5
#define DB7 PORTAbits.RA3
#define DB7Direction TRISAbits.TRISA3
#define DB6 PORTAbits.RA2
#define DB6Direction TRISAbits.TRISA2
#define DB5 PORTAbits.RA1
#define DB5Direction TRISAbits.TRISA1
#define DB4 PORTAbits.RA0
#define DB4Direction TRISAbits.TRISA0

#define FROM_LCD 1
#define TO_LCD 0;

#define RW_WRITE 0
#define RW_READ 1
#define RS_INSTRUCTION 0
#define RS_DATA_REGISTER 1
