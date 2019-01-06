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

char wI2CTxBuffer[20];
char wI2CTxBufferSize;
char wI2CTxSendPos;
char wHexTemp[5];


void GetTemp(int* oTempValue)
{
    if(wI2CTxBufferSize == 0)
    {
        SSPCON3bits.ACKTIM = 1;  //Indicates the I2C bus is in an Acknowledge sequence, set on 8TH falling edge of SCL clock
        wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
        wI2CTxBuffer[1] = 0x03; //Function code read register
        wI2CTxBuffer[2] = 0x00; //Read register 0x00
        wI2CTxBuffer[3] = 0x04; //Read 4 registers from 0x00 to 0x04
        wI2CTxBufferSize = 4;
        SSPCON2bits.SEN = 1;
    }
}
void WakeTemp()
{
    if(wI2CTxBuffer[0] != 0xB9) // 0xB8 Address of the AM2320)
    {
        wI2CTxBuffer[0] = 0xB9; // 0xB8 Address of the AM2320
    }
    else
    {
        wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
    }
    if(wI2CTxBufferSize == 0)
    {
        //wI2CTxBuffer[0] = 0xB8; // 0xB8 Address of the AM2320
        wI2CTxBufferSize = 1;
        SSPCON2bits.SEN = 1;
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
 
  lcdWriteText("Temp Communication test :");

  __delay_ms(1000);
  
  int wCounter=0;
  char wConv[8]={'/',0, '-',0, 0x6C,0, '|',0};
  int wTemp=0;
  while(1)
  {
    setCursorPosition(0x67);
    __delay_ms(10);
    lcdWriteText(&wConv[wCounter]);
    

    WakeTemp();
    //__delay_ms(1);
    // SSPADD = 0x1F; //Setting the Baud rate of the Clock  SPADD +1 *4 / Fosc
    //GetTemp(&wTemp);

    wCounter = wCounter + 2;
    if(wCounter == 8)
    {
        wCounter = 0;
    }

    /*char wInterruptTextLen = strlen(wInterruptText);
    if(wInterruptTextLen !=0)
    {
        setCursorPosition(DDRAM_Address_Line_4_Position_1);
        __delay_ms(10);
        lcdWriteText(wInterruptText);
        memset(wInterruptText,0,wInterruptTextLen);
        __delay_ms(2000);

    }*/
    
    __delay_ms(1000);
  }
    return;
}


void __interrupt() myint(void)
{
    if(PIR1bits.SSPIF == 1)
    {
        /*sprintf(wHexTemp,",%x",SSPSTAT);
        if((strlen(wInterruptText)+strlen(wHexTemp) + 1) < sizeof(wInterruptText))
        {
            strcat(wInterruptText,wHexTemp);
        }*/
        PIR1bits.SSPIF = 0;
        if(SSPSTATbits.P)
        {
          //strcat(wInterruptText,",IP");
          wI2CTxBufferSize=0;
          wI2CTxSendPos=0;
        }
        if(SSPSTATbits.S && wI2CTxSendPos == 0)
        {
          //strcat(wInterruptText,",IS");
          if(wI2CTxBufferSize !=0)
          {
            SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
            wI2CTxSendPos++;
          }
          else
          {
            //strcat(wInterruptText,",ISE");
          }
        }
        else
        {
            if(SSPCON2bits.ACKSTAT == 0 && wI2CTxSendPos != 0)
            {
              //strcat(wInterruptText,",IN");
              if(wI2CTxSendPos < wI2CTxBufferSize)
              {
                SSPBUF = wI2CTxBuffer[wI2CTxSendPos]; 
                wI2CTxSendPos++;
              }
              else
              {
                SSPCON2bits.PEN = 1;
              }
            }
            if(SSPCON2bits.ACKSTAT == 1)
            {
                //strcat(wInterruptText,",NA");
                SSPCON2bits.ACKSTAT = 0;
                if(wI2CTxSendPos != 0)
                {
                  SSPCON2bits.PEN = 1;
                }
            }
        }

    }
    if(PIR2bits.BCLIF == 1)
    {
        strcat(wInterruptText,",BCLIF");
    }
}
