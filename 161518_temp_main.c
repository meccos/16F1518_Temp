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

char wI2CTxBuffer[20];
char wI2CTxBufferSize;
char wI2CRxBufferSize;
char wI2CTxSendPos;
char wHexTemp[20];


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
void SetToGetTemp()
{
    if(wI2CTxBufferSize == 0)
    {
        PIE1bits.SSPIE = 1;
        SSPCON3bits.ACKTIM = 1;  //Indicates the I2C bus is in an Acknowledge sequence, set on 8TH falling edge of SCL clock
        wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
        wI2CTxBuffer[1] = 0x03; //Function code read register
        wI2CTxBuffer[2] = 0x00; //Read register 0x00
        wI2CTxBuffer[3] = 0x04; //Read 4 registers from 0x00 to 0x04
        wI2CTxBufferSize = 4;
        SSPCON2bits.SEN = 1;
    }
}

void Add_Trace(char* oText, char iSizeOfoText, char* iText)
{
    if((iSizeOfoText + 1 - strlen(oText)) > strlen(iText))
    {
        strcat(oText,iText);
    }
}

void GetTemp()
{
    if(wI2CTxBufferSize == 0)
    {
        PIE1bits.SSPIE = 1;
        memset(wReceptionBuffer,0,sizeof(wReceptionBuffer));
        wReceptionBufferPosition=0;
        wI2CTxBuffer[0] = 0xB9; // 0xB8 Address of the AM2320
        wI2CRxBufferSize = 8;
        wI2CTxBufferSize = 1;
        SSPCON2bits.SEN = 1;
    }
}
void WakeTemp()
{
    PIE1bits.SSPIE = 1;
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
        memset(iText,0,sizeof(wInterruptTextLen));
    }   
}
void main(void) 
{
  memset(wInterruptText,0,sizeof(wInterruptText));
  //OSCCONbits.IRCF = 13; //4mhz
  OSCCONbits.IRCF = 0xf; // Set frequency to 16 mhz
  OSCCONbits.SCS = 0x3; // IRCF bits for the osccon register
  INTCONbits.GIE = 0; //Disable interrupt
  PORTA = 0x00;
  
  PORTB = 0x00;
  ANSELB = 0x00; //Setting All to digital
  TRISB = 0x00; //Setting the portA as output
  
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
  while(INTCONbits.GIE == 1)
  {
    clearDisplay();
    moveCursorToHome();
    __delay_ms(1000);
    
    Add_Trace(wInterruptText,sizeof(wInterruptText),"WT:");
    WakeTemp();
    __delay_us(1000);
        
    Add_Trace(wInterruptText,sizeof(wInterruptText),",ST:");
    PrintLog(wInterruptText);
    SetToGetTemp();
    __delay_us(1500);
    
    PrintLog(wInterruptText);
    
    Add_Trace(wInterruptText,sizeof(wInterruptText),"GT:");
    GetTemp();

    __delay_ms(2000);
    PrintLog(wInterruptText);
   
    wCounter = wCounter + 2;
    if(wCounter == 4)
    {
        wCounter = 0;
    }


    lcdWriteText(&wConv[wCounter]);
    __delay_ms(10000);
    test = 0;
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
            if(SSPSTATbits.P)
            {
              Add_Trace(wInterruptText,sizeof(wInterruptText),",P");
              PIE1bits.SSPIE = 0;
              wI2CTxBufferSize=0;
              wI2CTxSendPos=0;
            }
            else if((SSPSTATbits.BF == 1) && ((wI2CTxBuffer[0] & 0x01) == 1))
            {
                sprintf(wHexTemp," %i ",(int)SSPBUF);
                Add_Trace(wInterruptText,sizeof(wInterruptText),wHexTemp);
                wReceptionBuffer[wReceptionBufferPosition] = SSPBUF;
                if(wReceptionBufferPosition == 0)
                {
                    ToggleBitRB5();
                }
                wReceptionBufferPosition++;
                
                SSPSTATbits.BF = 0 ;
                if( wReceptionBufferPosition < wI2CRxBufferSize)
                {
                    SSPCON2bits.ACKDT = 0; //Acknowledge the reception.
                }
                else
                {
                    SSPCON2bits.ACKDT = 1; //Acknowledge the reception.
                }
                SSPCON2bits.ACKEN = 1;
            }
            else if(SSPSTATbits.S && wI2CTxSendPos == 0)
            {
              Add_Trace(wInterruptText,sizeof(wInterruptText),",S");
              if(wI2CTxBufferSize !=0)
              {
                SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                wI2CTxSendPos++;
              }
              else
              {
                Add_Trace(wInterruptText,sizeof(wInterruptText),",ISE");
              }
            }
            else
            {
                if(SSPCON2bits.ACKSTAT == 0 && wI2CTxSendPos != 0) //Acknowledge received proceeding sending other bits
                {
                  Add_Trace(wInterruptText,sizeof(wInterruptText),"A");
                  
                  if(wI2CTxSendPos < wI2CTxBufferSize) //Keep sending up to the last byte
                  {
                    SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                    wI2CTxSendPos++;
                  }
                  else
                  {
                      if((wI2CTxBuffer[0] & 0x01) == 1) //If set the Read into the address
                      {
                          if( wReceptionBufferPosition < wI2CRxBufferSize)
                          {
                            __delay_us(50);
                            SSPCON2bits.RCEN = 1; //Enable reception
                            Add_Trace(wInterruptText,sizeof(wInterruptText),"R");
                            
                          }
                          else
                          {
                            SSPCON2bits.PEN = 1; //Send Completed Generating the Stop sequence.
                          }
                      }
                      else
                      {
                        SSPCON2bits.PEN = 1; //Send Completed Generating the Stop sequence.
                      }
                  }
                }
                else if(SSPCON2bits.ACKSTAT == 1)
                {
                    Add_Trace(wInterruptText,sizeof(wInterruptText),",NA");
                    SSPCON2bits.ACKSTAT = 0;
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
    }
    if(PIR2bits.BCLIF == 1)
    {
        ToggleBitRB5();
        PIR2bits.BCLIF = 0;
        Add_Trace(wInterruptText,sizeof(wInterruptText),",BCLIF");
    }
}
