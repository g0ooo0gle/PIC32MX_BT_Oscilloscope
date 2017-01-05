/*
 * -----------------------------------------------------------------------------
 * Bluetooth オシロスコープ
 * -----------------------------------------------------------------------------
 * 
 * File:        main.c
 * 
 * Author:      TanakaLab
 * Complier:    MPLAB XC32
 *              MPLAB X IDE v3.45
 * 
 * Dependencies:PIC32 Legacy Peripheral Libraries
 *              http://ww1.microchip.com/downloads/en//softwarelibrary/pic32%20peripheral%20library/pic32%20legacy%20peripheral%20libraries.zip
 *
 * Created on 2016/12/07, 15:47
 * 
 * ＜注意点＞
 * スパゲッティコード化
 */


#include <xc.h>
#include <plib.h>
#include <stdlib.h>
#include <time.h>

//*各種クロック******************************************************************
// SYSCLK = 40 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK  = 40 MHz
#define SYSCLK  40000000L                      //システムクロック
#define SYSCLKdiv10MHz    (SYSCLK/10000000)    //ディレイ関数クロック


// *コンフィギュレーションビット設定***********************************************
// DEVCFG3:
#pragma config IOL1WAY  = OFF           // Peripheral Pin Select Configuration
// DEVCFG2:
#pragma config FPLLODIV = DIV_2         // PLL Output Divider
#pragma config UPLLEN   = OFF           // USB PLL Enabled
#pragma config UPLLIDIV = DIV_2         // USB PLL Input Divider
#pragma config FPLLMUL  = MUL_20        // PLL Multiplier
#pragma config FPLLIDIV = DIV_2         // PLL Input Divider
// DEVCFG1:
#pragma config FWDTEN   = OFF           // Watchdog Timer
#pragma config WDTPS    = PS1           // Watchdog Timer Postscale
#pragma config FCKSM    = CSDCMD        // Clock Switching & Fail Safe Clock Monitor
#pragma config FPBDIV   = DIV_1         // Peripheral Clock divisor
#pragma config OSCIOFNC = OFF           // CLKO Enable
#pragma config POSCMOD  = OFF           // Primary Oscillator
#pragma config IESO     = OFF           // Internal/External Switch-over
#pragma config FSOSCEN  = OFF           // Secondary Oscillator Enable (KLO was off)
#pragma config FNOSC    = FRCPLL        // Oscillator Selection
// DEVCFG0:
#pragma config CP       = OFF           // Code Protect
#pragma config BWP      = ON            // Boot Flash Write Protect
#pragma config PWP      = OFF           // Program Flash Write Protect
#pragma config ICESEL   = ICS_PGx3      // ICE/ICD Comm Channel Select
#pragma config JTAGEN   = OFF           // JTAG Enable
#pragma config DEBUG    = OFF           // Background Debugger Enable

// ADC コンフィギュレーション
#define AD1CON1R ADC_MODULE_OFF | ADC_FORMAT_INTG16 | ADC_CLK_MANUAL | ADC_AUTO_SAMPLING_ON |ADC_SAMP_ON
#define AD1CON2R ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_OFF | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
#define AD1CON3R ADC_SAMPLE_TIME_3 | ADC_CONV_CLK_SYSTEM | ADC_CONV_CLK_6Tcy
#define AD1PCFGR (0x0000)
#define AD1CSSLR SKIP_SCAN_ALL

void _mon_putc(char data);
void delay_us(unsigned int usec);
void delay_ms(unsigned int msec);

//各種変数
unsigned int ADCFlag=1,Counter=0;

//バッファ
unsigned int Bufresult[255];
unsigned int Buftime[255];

int main(void)
{
    unsigned int Result = 0;
    float Voltage = 0;
    
    
    //System Setting
    SYSTEMConfigPerformance(SYSCLK);        //システム最適化設定
    mJTAGPortEnable(DEBUG_JTAGPORT_OFF);    //PORTA is used I/O, JTAG port must be disabled.

    mPORTBClearBits( BIT_5 );               //RB5　Low
    mPORTBSetPinsDigitalOut( BIT_5 );       //RB5　出力に
     
    mPORTBSetPinsDigitalIn( BIT_3| BIT_13 );//RB5　入力に
    U1RXR = 0b0011;                         //PIN RPB13 = RX
    mPORTBClearBits( BIT_15 );              //RB5　Low
    mPORTBSetPinsDigitalOut( BIT_15 );      //RB5　出力に
    RPB15R = 0b0001;                        //PIN RPB15 = TX

    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, SYSCLK, 115200);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
     
    INTEnable(INT_U1RX, INT_ENABLED);//INT_SOURCE_UART_RX(UART1)
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    INTSetVectorSubPriority(INT_VECTOR_UART(UART1), INT_SUB_PRIORITY_LEVEL_0);

    CloseADC10();
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF | ADC_CH0_POS_SAMPLEA_AN5 );
    OpenADC10(AD1CON1R, AD1CON2R, AD1CON3R, AD1PCFGR, AD1CSSLR);
    EnableADC10();


    //割り込み処理許可
    INTEnableSystemMultiVectoredInt();
    
    clock_t StartTime , CountTime;
    StartTime = clock();
    while(1)
    {
        mPORTBSetBits( BIT_5 );
        
        CountTime = clock(); //時間計測
        Buftime[Counter] = CountTime - StartTime;//差分を送信
        Buftime[Counter] = CountTime;
        
        ConvertADC10();				// 変換開始
        while(BusyADC10());			// 変換完了待ち(1チャンネル分)
        Bufresult[Counter] = ReadADC10(0);			// 変換結果読み出し
/*   
        if(ADCFlag)
            //printf("\r\nA/Ddata = %4u", Result);
            //printf("\r\n%d", Counter );
            //printf("\r\n%4.3f", Result);
            //printf("%4u,", Result);
            //printf("%\r\n%d,", Counter);
            printf("\r\n%d", Result);
        else{
            Voltage = 3.3 * (float)Result / 1023.0 ;
            //printf("\r\nVoltage = %4.3f", Voltage);
            printf("%\r\n%d,", Counter);
            printf("%4.3f,", Voltage);
        }
*/         
        mPORTBClearBits( BIT_5 );
        
        //送信判定&送信処理
        sendprocessing();
        
        
        //デバッグ
       // printf("%d,", Counter);
        //printf("%4u\r\n", Result);
        //delay_ms(50);
        printf("%f",(double)(CountTime - StartTime));
        
        Counter++;
    }
}
int sendprocessing(void){
    unsigned int Sendcounter = 0;
    
    if(Counter == 255){
    //カウンタつけて送信
        while(Sendcounter <= 255){
            mPORTBClearBits( BIT_5 );               //RB5　Low
            mPORTBSetPinsDigitalOut( BIT_5 ); 
            //printf("%d,", Sendcounter);
            printf("%4u\r\n", Buftime[Sendcounter] - Buftime[0]);//開始時間の差分を送信
            printf("%4u\r\n", Bufresult[Sendcounter]);
            delay_ms(10);
            Sendcounter++;
            mPORTBClearBits( BIT_5 ); 
        }
        printf("e");
        Counter = 0;
        Sendcounter = 0;
    }
}


void _mon_putc (char c){
   while (U1STAbits.UTXBF);
   U1TXREG = c;
}

//受信割り込み
void __ISR(32, ipl2) U1RX_interrupt(void){
	char RcvData;

        IFS1bits.U1RXIF = 0;        //mU1RXClearIntFlag();  
	RcvData = getcUART1();
	switch(RcvData){
		case 'a':
			ADCFlag = 1;
			break;
		case 'v':
			ADCFlag = 0;
			break;
		default:	break;
	}
}

/**********************************
*  ディレイ関数 usec
**********************************/
void delay_us(unsigned int usec){
 unsigned int i, delay;

 delay = usec * SYSCLKdiv10MHz;
 for(i=0; i<delay; i++){}
}

/**********************************
*  ディレイ関数 msec
**********************************/
void delay_ms(unsigned int msec){
 unsigned int i;

 for(i=0; i<msec; i++)
  delay_us(1000);
}

