/*
 * File:   161518_temp_main.c
 * Author: dell
 *
 * Created on December 11, 2018, 9:10 PM
 */


#include <xc.h>
#include <pic16f1518.h>
#include "LCD_hd44780u_qy_2004a.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

// PIC16F1518 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = ON   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = OFF     // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will not cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming)


char wInterruptText[wInterruptTextSize];
char wReceptionBuffer[30];
char wReceptionBufferPosition;
uint8_t wReceptionCounter=0;
uint8_t wReceptionCounterPrev=0;

char wI2CTxBuffer[20];
char wI2CTxBufferSize;
char wI2CRxBufferSize;
char wI2CTxSendPos;
char wHexTemp[20];
uint8_t wTrial=0;


void ToggleBitRB5()
{
    if(PORTBbits.RB5 == 1)
    {
        PORTBbits.RB5 = 0;
    }
    else
    {
        PORTBbits.RB5 = 1;
    }
}
enum{CommandSent,ProcessingCommand,CommandCompleted,CommandFailed};
uint8_t wI2CCommandState = 0;
void SetToGetTemp()
{
    wTrial=0;
    if(wI2CTxBufferSize == 0)
    {
        PIE1bits.SSPIE = 1;
        SSPCON3bits.ACKTIM = 1;  //Indicates the I2C bus is in an Acknowledge sequence, set on 8TH falling edge of SCL clock
        wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
        wI2CTxBuffer[1] = 0x03; //Function code read register
        wI2CTxBuffer[2] = 0x00; //Read register 0x00
        wI2CTxBuffer[3] = 0x04; //Read 4 registers from 0x00 to 0x04
        wI2CTxBufferSize = 4;
        wI2CCommandState=CommandSent;
        SSPCON2bits.SEN = 1;
        
    }
}

void Add_Trace(char* oText, char iSizeOfoText, char* iText)
{
    if((iSizeOfoText - 1 - strlen(oText)) > strlen(iText))
    {
        strcat(oText,iText);
    }
}

void GetTemp()
{
    wTrial=0;
    if(wI2CTxBufferSize == 0)
    {
        PIE1bits.SSPIE = 1;
        memset(wReceptionBuffer,0,sizeof(wReceptionBuffer));
        wReceptionBufferPosition=0;
        wI2CTxBuffer[0] = 0xB9; // 0xB8 Address of the AM2320
        wI2CRxBufferSize = 8;
        wI2CTxBufferSize = 1;
        wI2CCommandState=CommandSent;
        SSPCON2bits.SEN = 1;
    }
}
void WakeTemp()
{
    PIE1bits.SSPIE = 1;
    wTrial=0;
    if(wI2CTxBufferSize == 0)
    {
        wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
        wI2CTxBufferSize = 1;
        SSPCON2bits.SEN = 1;
    }
}
void PrintLog(char* iText)
{
    char wInterruptTextLen = strlen(iText);
    
    if(wInterruptTextLen !=0)
    {
        lcdWriteText(iText);
        memset(iText,0,wInterruptTextLen);
    }   
}
void printEM1812(int16_t wVariable, char* oTextOut)
{
    uint8_t wTen;
    uint8_t wUnity;
    uint8_t wDecimal;
    uint8_t wIsNegative=0;
    
    if(wVariable < 0)
    {
        wIsNegative = 1;
        wVariable = -wVariable; 
    }
    
    wTen = wVariable/100;
    wVariable = wVariable % 100;
    wUnity = wVariable/10;
    wVariable = wVariable %10;
    wDecimal = wVariable;
    if(wIsNegative)
    {
        oTextOut[0] = '-';
        oTextOut[1] = '0' + wTen;
        oTextOut[2] = '0' + wUnity;
        oTextOut[3] = ',';
        oTextOut[4] = '0' + wDecimal;
        oTextOut[5] = 0;
    }
    else
    {
        oTextOut[0] = '0' + wTen;
        oTextOut[1] = '0' + wUnity;
        oTextOut[2] = ',';
        oTextOut[3] = '0' + wDecimal;
        oTextOut[4] = 0;
    }
    
}

void Debounce(uint8_t iSwitch,uint16_t* ioTimer, uint8_t* swPressed)
{
    if(iSwitch == 0) //Button pressed
    {
      (*ioTimer)++;
    }
    else
    {
      *ioTimer = 0;   
    }
    if(*ioTimer == 2000)
    {
      *swPressed = 1;
    }
    if(*ioTimer == 8000)
    {
      *ioTimer = 2001;
      *swPressed = 1;
    }
}

uint8_t wTempUpdate=0;

#define ENTERBotton PORTBbits.RB0
#define UPBotton    PORTBbits.RB1
#define DOWNBotton  PORTBbits.RB2

#define DISPTACHERSIGNAL T1CONbits.TMR1CS

uint8_t wTimer1IntCounter=0;
uint8_t wTimer0Counter=0;
uint8_t wTempState=0;
int16_t wHumidity=0;
int16_t wTemperature=0;
  
void main(void) 
{
  char wReadout[20];
  int16_t wHumidityPrev=0;
  int16_t wTemperaturePrev=0;
  memset(wInterruptText,0,sizeof(wInterruptText));
  //OSCCONbits.IRCF = 13; //4mhz
  OSCCONbits.IRCF = 0xf; // Set frequency to 16 mhz
  OSCCONbits.SCS = 0x3; // IRCF bits for the osccon register
  INTCONbits.GIE = 0; //Disable interrupt
  
  uint8_t wUpBottonPressed=0;
  uint8_t wDownBottonPressed=0;
  uint8_t wEnterBottonPressed=0;
  
  uint8_t wMenu=0;
  uint8_t wMenuPrev=1;
  
  uint16_t wIterationCounter=0;
  uint16_t wDebounceEnter=0;
  uint16_t wDebounceUp=0;
  uint16_t wDebounceDown=0;
  
  PORTA = 0x00;
  
  //Dispatcher parameter
  
  T1CONbits.TMR1CS = 0x00; // Using instruction clock Fosc devi 4
  T1CONbits.T1OSCEN = 0x0; //Not used since we are using the internal oscillator
  T1CONbits.T1CKPS = 0x3; // Setting no prescaler
  T1CONbits.nT1SYNC = 0; // Maybe not used since we are using internal oscillation
  T1CONbits.TMR1ON = 1; // Enabling the timer1
  PIE1bits.TMR1IE =1; //Enabling the timer1 interrupt
  //INTCONbits.PEIE = 1; // Is done later on

  //TempGetTimer
  OPTION_REGbits.PS = 0x2;
  OPTION_REGbits.TMR0CS = 0;
  OPTION_REGbits.PSA = 0;
  INTCONbits.TMR0IE = 0; //Enable Timer 0 Interrupt
  
  
  //Button configuration
  PORTB = 0x00;
  ANSELB = 0x00; //Setting All to digital
  TRISB = 0x0F; //Setting bit 0 1 2 3 to input
  WPUB = 0x0F; // Activation of weak pull up
  OPTION_REGbits.nWPUEN = 0; //Enable WeekPull up
  
  
  memset(wI2CTxBuffer,0,sizeof(wI2CTxBuffer));
  wI2CTxBufferSize=0;
  wI2CTxSendPos=0;

  ANSELA = 0x00; //Setting All to digital
  TRISA = 0x00; //Setting the portA as output
  
  TRISC=0x00;
  TRISC = 0xFF; //SDA and SCL need to be define as intputs
  ANSELCbits.ANSC3 = 0;
  ANSELCbits.ANSC4 = 0;
  

  //Setting the I2cBus
  
  SSPCON1bits.SSPM = 0x8; // I2C Master Mode 
  SSPADD = 0x1F; //Setting the Baud rate of the Clock  SPADD +1 *4 / Fosc
  SSPCON1bits.SSPEN = 1;    //Enable Synchronous Serial Port Enable bit
  SSPCON2bits.GCEN = 0;  //Disable General call address
  SSPCON2bits.ACKEN = 1;  //Initiate Acknowledge Sequence Enable bits
  SSPCON3bits.PCIE = 1;     //Raise interrupt on Stop Condition
  SSPCON3bits.SCIE = 1;     //Raise interrupt on Start Condition
  SSPCON3bits.SDAHT = 1;  //300 ms hold time on SDA after falling edge of SCL
  SSPCON3bits.AHEN = 0 ; // Address holding disable
  SSPCON3bits.DHEN = 0; //Data hold disable
  SSPSTATbits.CKE = 0; //Disable SMbus specific inputs
  
  INTCONbits.PEIE = 1; //Enable peripheral interrupt
  PIE1bits.SSPIE = 1;  //Enable interrupt from MSSP
  PIE2bits.BCLIE = 1; //Enable collision detection interrupt
  
  INTCONbits.GIE = 1; //Enable interrupt
  
  
  initLCD();
  clearDisplay();
  __delay_ms(100);
  powerOnLcd();
  __delay_ms(100);
  setCursorOff();
  __delay_ms(100);
  moveCursorToHome();
  __delay_ms(100);
  setNotBlinkingCursor();
  __delay_ms(100);
 
  /*
  char wChar2[2] = {0,0};
  for(char i=0; i < 80 ; i++)
  {
      wChar2[0] = i;
      Add_Trace(wInterruptText,sizeof(wInterruptText),wChar2);
  }
  PrintLog(wInterruptText);
  __delay_ms(3000);
  for(char i=80; i < 170 ; i++)
  {
      wChar2[0] = i;
      Add_Trace(wInterruptText,sizeof(wInterruptText),wChar2);
  }
  PrintLog(wInterruptText);
  __delay_ms(3000);
  
   for(char i=160; i < 240 ; i++)
  {
      wChar2[0] = i;
     lcdWriteText(wChar2);
  }
  __delay_ms(3000); */
  
  int wCounter=0;
  char wConv[4]={'+',0, 'x',0, };
  int wTemp=0;

  
  clearDisplay();
  moveCursorToHome();
  __delay_ms(30);
  while(1)
  {
    
    if((wHumidityPrev != wHumidity) || (wTemperaturePrev != wTemperature))
    {
        wHumidityPrev = wHumidity;
        wTemperaturePrev = wTemperature;
        setCursorPosition(2,0);
        printEM1812(wHumidityPrev,wReadout);
        Add_Trace(wInterruptText,sizeof(wInterruptText),"Humidity : ");
        Add_Trace(wInterruptText,sizeof(wInterruptText),wReadout);
        printEM1812(wTemperaturePrev,wReadout);
        Add_Trace(wInterruptText,sizeof(wInterruptText),"\nTemp : ");
        Add_Trace(wInterruptText,sizeof(wInterruptText),wReadout);
        PrintLog(wInterruptText);
    }
    
    if(wMenu != wMenuPrev)
    {
      wMenuPrev = wMenu;
      switch(wMenu)
      {
        case 0:
          setCursorPosition(0,0);
          lcdWriteText("Home           ");
          break;
        case 1:
          setCursorPosition(0,0);
          lcdWriteText("Set Temp:      ");
          break;
        case 2:
          setCursorPosition(0,0);
          lcdWriteText("Set Mode:      ");
          break;
        default:
          setCursorPosition(0,0);
          lcdWriteText("WTF            ");
          break;
      }
    }
     
    switch( wTempState )
    {
        case 0:
            SetToGetTemp();
            ToggleBitRB5();
            wTempState++;
            INTCONbits.TMR0IE = 0; //Enable Timer 0 Interrupt
            TMR0 = 0;
            wTimer0Counter=0;
            INTCONbits.TMR0IE = 1; //Enable Timer 0 Interrupt
            break;
        case 1:
            if(wTimer0Counter == 2)
            {
                wTempState++;
                INTCONbits.TMR0IE = 0; //Enable Timer 0 Interrupt
            }
            break;
        case 2:
            if(wI2CCommandState == CommandFailed)
            {
                wTempState=0;
            }
            else if (wI2CCommandState == CommandCompleted)
            {
                ToggleBitRB5();
                GetTemp();
                ToggleBitRB5();
                wTempState++;
                INTCONbits.TMR0IE = 0; //Enable Timer 0 Interrupt
                TMR0 = 0;
                wTimer0Counter=0;
                INTCONbits.TMR0IE = 1; //Enable Timer 0 Interrupt
            }
            break;
        case 3:
            if(wTimer0Counter == 2)
            {
                wTempState++;
                INTCONbits.TMR0IE = 0; //Enable Timer 0 Interrupt
            }
            break;
        case 4:
            if(wI2CCommandState == CommandFailed)
            {
                wTempState=0;
            }
            else if(wI2CCommandState == CommandCompleted)
            {
                wTempState++;
            }
            break;
        case 5:
            wCounter = wCounter + 2;
            if(wCounter == 4)
            {
              wCounter = 0;
            }
            setCursorPosition(3,19);
            lcdWriteText(&wConv[wCounter]);
            wTempState++;
            break;
        case 6:
            break;
        default:
            break;
    }
   /*
    if( wTempUpdate == 1)
    {
        wTempUpdate = 0;
        WakeTemp();
        __delay_us(1000);

        SetToGetTemp();
        __delay_us(1500);

        GetTemp();
        
        wCounter = wCounter + 2;

        if(wCounter == 4)
        {
          wCounter = 0;
        }
        setCursorPosition(3,19);
        lcdWriteText(&wConv[wCounter]);
    }*/
    wIterationCounter++;


   Debounce(ENTERBotton,&wDebounceEnter,&wEnterBottonPressed);
   Debounce(UPBotton,&wDebounceUp,&wUpBottonPressed);
   Debounce(DOWNBotton,&wDebounceDown,&wDownBottonPressed);


   if(wUpBottonPressed == 1)
   {
       wUpBottonPressed = 0;
       wMenu++;
   }
   if(wDownBottonPressed == 1)
   {
       wDownBottonPressed = 0;
       wMenu--;
   }
   
   if(wMenu == 255)
   {
       wMenu = 2;
   }
   if(wMenu == 3)
   {
       wMenu = 0;
   }    
  
  
  }
    return;
}

char wCounter2=0;
void __interrupt() myint(void)
{

    if(PIR1bits.SSPIF == 1)
    {
        PIR1bits.SSPIF = 0;
        if( wI2CTxBufferSize != 0)
        {
            if((wI2CTxBuffer[0] & 0x01) == 1) //Read mode
            {
                if(SSPSTATbits.P)
                {
                  PIE1bits.SSPIE = 0;
                  wI2CTxBufferSize=0;
                  wI2CTxSendPos=0;
                }
                else if(SSPSTATbits.S && wI2CTxSendPos == 0) //Start of the transmission to slave
                {
                    SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                    wI2CCommandState = ProcessingCommand;
                    wI2CTxSendPos++;
                }
                else if(SSPSTATbits.BF == 1) //Reception of one byte from slave
                {
                    wReceptionBuffer[wReceptionBufferPosition] = SSPBUF;
                    wReceptionBufferPosition++;

                    SSPSTATbits.BF = 0 ;
                    if( wReceptionBufferPosition < wI2CRxBufferSize)
                    {
                        SSPCON2bits.ACKDT = 0; //Acknowledge the reception.
                    }
                    else
                    {
                        SSPCON2bits.ACKDT = 1; // Don't acknowledge the reception.
                        wI2CCommandState = CommandCompleted;
                        wHumidity = wReceptionBuffer[2]*256 + wReceptionBuffer[3];
                        wTemperature = wReceptionBuffer[4]*256 + wReceptionBuffer[5];
                        wReceptionCounter++; // Indicate that we received new packet

                    }
                    SSPCON2bits.ACKEN = 1;
                }
                else
                {
                    if(SSPCON2bits.ACKSTAT == 0 && wI2CTxSendPos != 0) //Acknowledge received proceeding sending other bits
                    {
                        if( wReceptionBufferPosition < wI2CRxBufferSize)
                        {
                          //__delay_us(50);
                          SSPCON2bits.RCEN = 1; //Enable reception
                        }
                        else
                        {
                          SSPCON2bits.PEN = 1; //Send Completed Generating the Stop sequence.
                        }
                    }
                    else if(SSPCON2bits.ACKSTAT == 1)
                    {
                        SSPCON2bits.ACKSTAT = 0;
                        wI2CCommandState = CommandFailed;
                        if(wI2CTxSendPos != 0)
                        {
                          SSPCON2bits.PEN = 1;
                        }
                        else
                        {
                          SSPCON2bits.PEN = 1;
                        }
                    }
                    else
                    {
                        Add_Trace(wInterruptText,sizeof(wInterruptText),",N8");
                    }
                }

            }
            else //Write to mode
            {
                if(SSPSTATbits.P)
                {
                  PIE1bits.SSPIE = 0;
                  wI2CTxBufferSize=0;
                  wI2CTxSendPos=0;
                }
                else if(SSPSTATbits.S && wI2CTxSendPos == 0) //Start of the transmission to slave
                {
                    wI2CCommandState = ProcessingCommand;
                    SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                    wI2CTxSendPos++;
                }
                else
                {
                    if(SSPCON2bits.ACKSTAT == 0 && wI2CTxSendPos != 0) //Acknowledge received proceeding sending other bits
                    {
                      if(wI2CTxSendPos < wI2CTxBufferSize) //Keep sending up to the last byte
                      {
                        SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                        wI2CTxSendPos++;
                      }
                      else
                      {
                            SSPCON2bits.PEN = 1; //Send Completed Generating the Stop sequence.
                            wI2CCommandState = CommandCompleted;
                      }
                    }
                    else if(SSPCON2bits.ACKSTAT == 1)
                    {
                        SSPCON2bits.ACKSTAT = 0;
                        SSPCON2bits.PEN = 1;
                        wI2CCommandState = CommandFailed;
                    }
                    else
                    {
                        
                    }
                }
            }



            
        }
    }
    if(PIR2bits.BCLIF == 1)
    {
        PIR2bits.BCLIF = 0;
        Add_Trace(wInterruptText,sizeof(wInterruptText),",BCLIF");
    }
    if(PIR1bits.TMR1IF == 1)
    {
        wTimer1IntCounter++;
        PIR1bits.TMR1IF = 0;
        
        if(wTimer1IntCounter == 7)
        {
            TMR1H = 0x4C;
            TMR1L = 0x83;
        }
        if(wTimer1IntCounter == 8)
        {
            wTimer1IntCounter = 0;
 
            wTempUpdate = 1;
            if(wTempState == 6)
            {
               wTempState = 0;
            }
            else
            {
                  
            }
        }
    }
    if(INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        wTimer0Counter++;
    }
}
