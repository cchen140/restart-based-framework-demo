#include <msp430.h> 
#include "msp430g2553.h"
// ********************
// initial Restart time = 1s (1000,000)
// initial sate duration = 50ms(50,000)
//                  |       |
//                  |       |
//                  |       |
//                  |       |
//                  |       |
//                  |    1.7|-->SDK
//                  |    1.6|-->SCLK
//  Restart Timer<--|2.0    |
// Security Timer<--|2.1    |
//                  |       |
//*********************

// Parameter definition
const unsigned int IniResCnt = 5000; //initial restart count 5000*200
unsigned int NextResTime = 10000;     //next restart time----------in (ms)
const unsigned int SafeDura = 50000;         //length of safe duration
unsigned int Timer0A0SecondCounter=0;

typedef enum i2cCmdState{
    I2C_CMD_STATE_WAIT_CMD = 0,
    I2C_CMD_STATE_SET_TIME_WAIT_FIRST_BYTE,
    I2C_CMD_STATE_SET_TIME_WAIT_SECOND_BYTE
} i2cCmdState;

i2cCmdState I2cCmdState = I2C_CMD_STATE_WAIT_CMD;
unsigned char LastI2cReceivedByte = 0;

// Subfunction list
void I2C_Slave_Init(void);
void StartSecureInterval(unsigned int inSecureIntervalTime);
void EndSecureInterval(void);
void StartRestartTimer(unsigned int inTime);
void EndRestartTimer(void);

#define ROT_TIMER_CMD_SET_RESTART_TIME  0xFE
#define ROT_TIMER_CMD_END_SECURE_INTERVAL  0xFD

void main(void)
{
    // Stop watchdog timer to prevent time out reset
    WDTCTL = WDTPW + WDTHOLD;
    //Timers initialization
    P2DIR |= BIT0+BIT1; //Bit0--Restart timer signal
                        //Bit1--Security timer signal
    //P2OUT &= ~(BIT0+BIT1);
    P2OUT |= (BIT0+BIT1);
    CCTL0 = CCIE;   //CCR0 interrupt enable
    CCR0 = IniResCnt;
    TACTL = TASSEL_2 + MC_1;    //SMCLK(room temperature, 3.3v ~~1.0MHz,ACLK ~~ 32.768KHz)
                                //up to CCR0

    /* Disable all timers and wait for commands from I2C. */
    EndSecureInterval();
    EndRestartTimer();

    //I2C initialization
    I2C_Slave_Init();
    //low power mode and enable global interrupt
    while(1)
    {
        __bis_SR_register(CPUOFF + GIE);
    }

}

//Function definition
void I2C_Slave_Init(void)
{
    //I2C part
    //set p1.0 output pin
    //P1DIR |= BIT0;

    //set I2C pin: p1.6(SCL) p1.7(SDA)
    P1SEL |= BIT6+BIT7;
    P1SEL2 |= BIT6+BIT7;
    //enable SW reset
    UCB0CTL1 |= UCSWRST; //UCSWRST = 0x01
    //I2C Slave, synchronous mode
    UCB0CTL0 = UCMODE_3 + UCSYNC;

    UCB0I2COA = 0x35; // set own (slave) address
    UCB0CTL1 &= ~UCSWRST; // Clear SW reset, resume operation

    UCB0I2CIE |= UCSTTIE; // Enable STT interrupt

    IE2 |= UCB0TXIE + UCB0RXIE; // Enable TX and RX interrupt
    //IE2 &= ~(UCB0TXIE + UCB0RXIE);
}

void StartSecureInterval(unsigned int inSecureIntervalTime) {
    //enable security timer
    CCR1 = inSecureIntervalTime;
    CCTL1 |= CCIE;   //CCR1 interrupt enabled
}


void EndSecureInterval(void) {
    //P2OUT &= ~BIT1;   //set p2.1 output value = 0
    P2OUT |= BIT1;
    CCTL1 &= ~CCIE;   //CCR1 interrupt disabled

    //IE2 &= ~(UCB0TXIE + UCB0RXIE); // Disable TX and RX interrupt
}

void StartRestartTimer(unsigned int inTime) {
    CCR0 = NextResTime*5; //NextResTime*1000/200
    TACTL |= TACLR; // Clear timer A's count;
    Timer0A0SecondCounter=0;
    CCTL0 |= CCIE;   //CCR0 interrupt enable
}

void EndRestartTimer(void) {
    // Disable CCR0 (restart timer) interrupt
    CCTL0 &= ~CCIE;   //CCR0 interrupt enable
}


//Interrupt Service Function definition
//I2C receive ISF
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
    unsigned char I2cReceivedByte = UCB0RXBUF;
    //receive data

    /* If commands are detected, reset the state to whatever the command is. */
    if (I2cReceivedByte == ROT_TIMER_CMD_SET_RESTART_TIME) {
        I2cCmdState = I2C_CMD_STATE_SET_TIME_WAIT_FIRST_BYTE;
        return;
    } else if (I2cReceivedByte == ROT_TIMER_CMD_END_SECURE_INTERVAL) {

        return;
    }

    switch (I2cCmdState) {
        case I2C_CMD_STATE_SET_TIME_WAIT_FIRST_BYTE:
            LastI2cReceivedByte = I2cReceivedByte;
            I2cCmdState = I2C_CMD_STATE_SET_TIME_WAIT_SECOND_BYTE;
            break;
        case I2C_CMD_STATE_SET_TIME_WAIT_SECOND_BYTE:
            NextResTime = (LastI2cReceivedByte << 8) + I2cReceivedByte;
            I2cCmdState = I2C_CMD_STATE_WAIT_CMD;
            //StartRestartTimer(NextResTime);
            if (NextResTime < 100) {
                NextResTime = 100;   //1000ms
            }
            StartRestartTimer(NextResTime);
            break;
        default:
            break;
    }

    //out of LPM0
    //__bic_SR_register_on_exit(CPUOFF);
}

//I2C initial condition ISF
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    //clear interrupt flag
    UCB0STAT &= ~(UCSTTIFG+UCSTPIFG);
}

//Restart timer(Timer A0) interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void){

    Timer0A0SecondCounter++;
    if(Timer0A0SecondCounter==200)
    {
        EndRestartTimer();

        //P2OUT |= BIT0+BIT1;
        P2OUT &= ~(BIT0+BIT1);
        __delay_cycles(10000);  // unit = us
        //P2OUT &= ~BIT0;
        P2OUT |= BIT0;

        //StartSecureInterval(SafeDura);

        //_BIS_SR(LPM0_bits+GIE);     //Enter LPM0 w/ interrupt
    }

}

//Security timer(Timer A1) interrupt service routine
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer_B (void){
    EndSecureInterval();
    StartRestartTimer(NextResTime);
}
