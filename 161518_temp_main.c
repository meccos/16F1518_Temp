/*
 * File:   161518_temp_main.c
 * Author: dell
 *
 * Created on December 11, 2018, 9:10 PM
 */


#include <xc.h>
#include <pic16f1518.h>
#include "LCD_hd44780u_qy_2004a.h"
#include "EM1812.h"
#include "I2C.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "GeneralFunction.h"
#include "I2C.h"

// PIC16F1518 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
//#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config FOSC = 0x2       // Oscillator external high speed
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

char wHexTemp[20];
uint8_t wTrial=0;

unsigned char gTxBuffer[256];
uint8_t  gTxTransmitSize=0;
uint8_t  gTxReadingPosition=0;
uint8_t  gErrorCode;

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

void PrintLog(char* iText)
{
    char wInterruptTextLen = strlen(iText);
    
    if(wInterruptTextLen !=0)
    {
        lcdWriteText(iText);
        memset(iText,0,wInterruptTextLen);
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
void Send_UART_Data( unsigned char* iData, uint8_t iData_Length)
{
    if(gTxTransmitSize != 0)
    {
        return;
    }
    gTxReadingPosition = 0;
    gTxTransmitSize = iData_Length;
    memcpy(gTxBuffer , iData,iData_Length);

    SPBRGH = 0;
    SPBRGL = 1;  //14mhz cristal + 23 = 104 us which is Baudrate set to 9600  Datasheet page 249
    ANSELCbits.ANSC6 = 0; //Desabling UART TX pin analogu fonction
    TXSTAbits.TXEN = 1; //Enable UART Port
    TXSTAbits.SYNC = 0; //Setting Asynchonus operation
    RCSTAbits.SPEN = 1; //Enabling of UART Pin TX pin will be set as output
    PIE1bits.TXIE = 1; //Enabling the UART Tx interrupt
    
}


#define ENTERBotton PORTBbits.RB0
#define UPBotton    PORTBbits.RB1
#define DOWNBotton  PORTBbits.RB2

#define DISPTACHERSIGNAL T1CONbits.TMR1CS

uint8_t wTimer1IntCounter=0;

int16_t wHumidity=0;
int16_t wTemperature=0;

unsigned char gUartTXBuffer[50];
unsigned char gUartRXBuffer[50];
unsigned char gUartRXBufferIndex;

int16_t wTempSet=210;

enum eMenu{eShowTime=0,eShowTemp,eShowMode,eSetTime=128,eSetTemp=129,eSetMode=130};
enum eMode{eElectric=0,eFuel,eThermopump,eCooling};

#define USART_TX_Direction TRISCbits.TRISC6
#define USART_TX_AN ANSELCbits.ANSC6
#define USART_RX_Direction TRISCbits.TRISC7
#define USART_RX_AN ANSELCbits.ANSC7



void main(void) 
{
    
  gErrorCode =0;
    
  char wReadout[20];
  int16_t wHumidityPrev=0;
  int16_t wTemperaturePrev=0;
  memset(wInterruptText,0,sizeof(wInterruptText));
  //OSCCONbits.IRCF = 13; //4mhz
  
  OSCCONbits.IRCF = 0xf; // Set frequency to 16 mhz
  OSCCONbits.SCS = 0x0; // IRCF bits for the osccon register
  INTCONbits.GIE = 0; //Disable interrupt
  
  uint8_t wUpBottonPressed=0;
  uint8_t wDownBottonPressed=0;
  uint8_t wEnterBottonPressed=0;
  
  uint8_t wEditingMode=0;
  uint8_t wMenu=0;
  uint8_t wUpdateMenu=1;
  
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
  
  //SCREEN Configuration done int the initLCD function
 
  I2CInit();


  //USART
  
  USART_TX_AN = 0;
  USART_RX_AN = 0;
  USART_TX_Direction = 0;
  USART_RX_Direction = 1;
  gUartRXBufferIndex = 0;
  
  
  PIE1bits.RCIE =1;

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
 
  
  int wCounter=0;
  char wConv[4]={'+',0, 'x',0, };
  int wTemp=0;

  
  clearDisplay();
  moveCursorToHome();
  __delay_ms(30);
  
 
  RCSTAbits.SPEN = 1; //Enabling Serial Port
  RCSTAbits.CREN = 1; //Enable Continuous Reception USART
  RCSTAbits.ADDEN = 0; //Disables address detection, all bits are received.
  RCSTAbits.RX9 = 0 ; // Reception in 8 bits mode.
  
  __delay_ms(1000);
  while(1)
  {
    
    if( wUpdateMenu )
    {
      wUpdateMenu = 0;
      switch(wMenu ) //{eShowTime=0,eShowTemp,eShowMode,eSetTime=128,eSetTemp=129,eSetMode=130};
      {
        case eShowTime:
          setCursorPosition(0,0);
          lcdWriteText("Time           ");
          break;
        case eShowTemp:
          setCursorPosition(0,0);
          lcdWriteText("Temp Setting:  \n");
          printEM1812(wTempSet, wReadout);
          lcdWriteText(wReadout);
          break;
        case eShowMode:
          setCursorPosition(0,0);
          lcdWriteText("Mode:          ");
          break;
        case eSetTime:
          setCursorPosition(0,0);
          lcdWriteText("-Set Time-     \n");
          break;
        case eSetTemp:
          setCursorPosition(0,0);
          lcdWriteText("-Set Temp-     \n");
          printEM1812(wTempSet, wReadout);
          lcdWriteText(wReadout);
          break;
        case eSetMode:
          setCursorPosition(0,0);
          lcdWriteText("-Set Mode-     \n");

          break;
        default:
          setCursorPosition(0,0);
          lcdWriteText("WTF            ");
          break;
      }
    }
  
    if( EM1812EntryPoint(&wHumidity, &wTemperature) == 1)
    {
        
        if((wHumidityPrev != wHumidity) || (wTemperaturePrev != wTemperature))
        {
            wHumidityPrev = wHumidity;
            wTemperaturePrev = wTemperature;
            setCursorPosition(2,0);
            printEM1812(wHumidityPrev,wReadout);
            AddTrace(wInterruptText,sizeof(wInterruptText),"Humidity : ");
            AddTrace(wInterruptText,sizeof(wInterruptText),wReadout);
            printEM1812(wTemperaturePrev,wReadout);
            AddTrace(wInterruptText,sizeof(wInterruptText),"\nTemp : ");
            AddTrace(wInterruptText,sizeof(wInterruptText),wReadout);
            PrintLog(wInterruptText);
        }
    }
    wIterationCounter++;


   Debounce(ENTERBotton,&wDebounceEnter,&wEnterBottonPressed);
   Debounce(UPBotton,&wDebounceUp,&wUpBottonPressed);
   Debounce(DOWNBotton,&wDebounceDown,&wDownBottonPressed);

//{eShowTime=0,eShowTemp,eShowMode,eSetTime=128,eSetTemp=129,eSetMode=130};
   //enum eMode{eElectric=0,eFuel,eThermopump,eCooling
   if(wUpBottonPressed == 1)
   {
       wUpdateMenu=1;
       wUpBottonPressed = 0;
        switch(wMenu)
        {
            case eSetTime:

                break;
            case eSetTemp:
                wTempSet = wTempSet+1;
                break;
            case eSetMode:
                break;
            default:
                wMenu++;
                break;
        }

   }
   if(wDownBottonPressed == 1)
   {
       wUpdateMenu=1;
       wDownBottonPressed = 0;
        switch(wMenu)
        {
            case eSetTime:

                break;
            case eSetTemp:
                wTempSet = wTempSet - 1;
                break;
            case eSetMode:
                break;
            default:
                wMenu--;
                break;
        }
   }
   if(wEnterBottonPressed == 1)
   {
       wUpdateMenu=1;
       wEnterBottonPressed = 0;

       if(wEditingMode == 0)
       {
         wEditingMode = 1;
         wMenu = wMenu+128;
         
            gUartRXBufferIndex=0;
            gUartRXBuffer[0]=0;
            memcpy(gUartTXBuffer,"AT+CWMODE=3",sizeof("AT+CWMODE=2"));
            gUartTXBuffer[sizeof("AT+CWMODE=2")] = 0x0d;
            gUartTXBuffer[sizeof("AT+CWMODE=2")+1] = 0x0a;
            gUartTXBuffer[sizeof("AT+CWMODE=2")+2] = 0;
            Send_UART_Data(gUartTXBuffer,sizeof("AT+CWMODE=2")+2);
       }
       else
       {
            gUartRXBufferIndex=0;
            gUartRXBuffer[0]=0;
            memcpy(gUartTXBuffer,"AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0",sizeof("AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0"));
            //gUartTXBuffer[0] = 'A';
            //gUartTXBuffer[1] = 'T';
            gUartTXBuffer[sizeof("AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0")] = 0x0d;
            gUartTXBuffer[sizeof("AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0")+1] = 0x0a;
            gUartTXBuffer[sizeof("AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0")+2] = 0;
            Send_UART_Data(gUartTXBuffer,sizeof("AT+CWSAP=\"ESP_9945B5\",\"12345678\",11,0")+2);
         wEditingMode = 0;   
       }
   }   
   if(wMenu == 255)
   {
       wUpdateMenu=1;
       wMenu = 2;
   }
   if(wMenu == 3)
   {
       wUpdateMenu=1;
       wMenu = 0;
   }    
  
  
  }
    return;
}

char wCounter2=0;
void __interrupt() myint(void)
{
    int wI2CError;
    wI2CError = I2C_Interrupt();
    if( wI2CError != 0 )
    {
        char wText[2];
        wText[0] = '0' + wI2CError;
        wText[1] = 0;
        lcdWriteText(wText);
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
            TempUpdateRequest();
        }
    }
    if(INTCONbits.TMR0IF == 1)
    {
        INTCONbits.TMR0IF = 0;
        wTimer0Counter++;
    }
    if( PIR1bits.TXIF == 1 ) //UART Transmission
    {
      if(gTxReadingPosition < gTxTransmitSize)
      {
          TXREG = gTxBuffer[gTxReadingPosition];
          gTxReadingPosition++;
      }
      else
      {
           gTxReadingPosition = 0;
           gTxTransmitSize = 0;
           PIE1bits.TXIE =0;
      }
    }
    if( PIR1bits.RCIF == 1)
    {
        if(RCSTAbits.FERR == 1 || RCSTAbits.OERR == 1 )
        {
            gErrorCode = RCSTAbits.FERR + (RCSTAbits.OERR<<1);
        }
        else
        {
            if(gUartRXBufferIndex < sizeof(gUartRXBuffer) )
            {
              gUartRXBuffer[gUartRXBufferIndex] = RCREG;
              gUartRXBufferIndex++;
            }
            else
            {
              gErrorCode = 3;  
            }
        }
        
    }
}
