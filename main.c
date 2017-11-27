/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/

#include <msp430.h>
#include "driverlib.h"
#include "eusci_uartio.h"
#include "cli_parser.h"

Uartio_t *uart;

/* In my test board for this project, this pin is driven low when an FTDI serial cable is attached and the FTDI's Vcc switches an NFET gate w/ Drain=P2.2 */
#define UART_DETECT_PxDIR P2DIR
#define UART_DETECT_PxIN P2IN
#define UART_DETECT_PxREN P2REN
#define UART_DETECT_PxOUT P2OUT
#define UART_DETECT_PxIE P2IE
#define UART_DETECT_PxIES P2IES
#define UART_DETECT_PxIFG P2IFG
#define UART_DETECT_PxBIT BIT2

void handleEpicDinosaur(Uartio_t *, unsigned int, const char **);
void handleHelp(Uartio_t *, unsigned int, const char **);
void handleVolts(Uartio_t *, unsigned int, const char **);

const cli_command_t commandManifest[] = {
    {"epic dinosaur", handleEpicDinosaur},  // Test cli_parser's double-quote feature
    {"help", handleHelp},
    {"volts", handleVolts},
    {NULL, NULL} // Signifies end of list
};

volatile static int uartSuspendSignal = 0;

void main (void)
{
    WDTCTL = WDTPW | WDTHOLD;

    P1DIR = 0;
    P1REN = 0xFF;
    P1OUT = 0;
    P2DIR = 0;
    P2REN = 0xFF;
    P2OUT = 0;
    P3DIR = 0;
    P3REN = 0xFF;
    P3OUT = 0;
    RTCCTL = 0;  // Disable RTC in case it was previously active - found this necessary when testing an RTC library before this one

    P1DIR |= BIT0|BIT1;
    P1OUT &= ~(BIT0|BIT1);
    // XT1 support
    P2SEL1 &= ~(BIT0 | BIT1);
    P2SEL0 |= BIT0|BIT1;
    // eUSCI_A0 support
    P1SEL1 &= ~(BIT4 | BIT5);
    P1SEL0 |= BIT4|BIT5;
    // eUSCI_A1 support
    P2SEL1 &= ~(BIT5 | BIT6);
    P2SEL0 |= BIT5|BIT6;

    // Init UART_DETECT
    UART_DETECT_PxDIR &= ~UART_DETECT_PxBIT;
    UART_DETECT_PxREN |= UART_DETECT_PxBIT;
    UART_DETECT_PxOUT |= UART_DETECT_PxBIT;
    UART_DETECT_PxIES = UART_DETECT_PxIN & UART_DETECT_PxBIT; // Trigger on next edge
    UART_DETECT_PxIFG &= ~UART_DETECT_PxBIT;
    UART_DETECT_PxIE |= UART_DETECT_PxBIT;  // Enable interrupt

    PMM_unlockLPM5();

    FRAMCtl_configureWaitStateControl(FRAMCTL_ACCESS_TIME_CYCLES_1);

    CS_setExternalClockSource(32768);
    bool xt1Ret;
    xt1Ret = CS_turnOnXT1LFWithTimeout(CS_XT1_DRIVE_3, 65535);
    if (xt1Ret) {
        CS_initClockSignal(CS_FLLREF, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1); // DCO FLL ref = XT1 (32.768KHz physical crystal)
    } else {
        CS_initClockSignal(CS_FLLREF, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1); // DCO FLL ref = REFO (internal RC oscillator 32.768KHz reference)
    }
    bool fllRet;
    fllRet = CS_initFLLSettle(16000, 488); // 488 is 16000000 / 32768
    if (!fllRet) {
        P1OUT |= BIT0;
        LPM4;
    }
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1); // Activate MCLK = DCO w/ FLL
    __enable_interrupt();

    // Initialize eUSCI UART driver
    uart = Uartio_init(1, 115200); // eUSCI_A1 at 115Kbps (note Uartio driver assumes SMCLK=16MHz)
    if (uart == (void *)0) {
        P1OUT |= BIT0;
        LPM4;
    }
    Uartio_setWakeup(uart, true, false, false); // Wake up for each individual RX byte
    Uartio_writen(uart, "This is only a test.\r\n", 22);

    cli_parser_init();
    cli_parser_set_command_list(commandManifest);

    // Is UART_DETECT=1?  Should we suspend the UART right away?
    if (UART_DETECT_PxIN & UART_DETECT_PxBIT) {
        Uartio_suspend(uart, true);
    }

    while (1) {
        if (Uartio_isSuspended(uart)) {
            LPM3;
        } else {
            LPM0;
        }
        
        // Process P2.2 UART_DETECT edge detect interrupt
        if (uartSuspendSignal != 0) {           // Handling the UART suspend/unsuspend in main() loop is best since
            if (uartSuspendSignal > 0) {        // the cli_parser_process_input piece is handled in main; should the cli parser be
                cli_parser_reset();             // running while the UART suspend signal comes through, a forced ISR-based suspend may
                Uartio_suspend(uart, true);     // cause the cli parser to freeze if handler functions try writing to the UART.
                while (Uartio_available(uart)) {
                    Uartio_read(uart);  // Drain remaining RX buffer since we don't care about remaining contents
                }
            } else {
                Uartio_suspend(uart, false);
            }
            uartSuspendSignal = 0;
        }
        // Process incoming RX bytes using CLI parser
        if (Uartio_available(uart)) {
            cli_parser_process_input(uart);
        }
    }
}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void PORT2_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) PORT2_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(P2IV, P2IV_P2IFG7))
  {
    case P2IV_NONE: break;
    case P2IV_P2IFG0: break;
    case P2IV_P2IFG1: break;
    case P2IV_P2IFG2:
        if (UART_DETECT_PxIN & UART_DETECT_PxBIT) { // High level = not connected, low = connected
            uartSuspendSignal = 1;  // Tell main() loop to perform suspend
        } else {
            uartSuspendSignal = -1; // Tell main() loop to unsuspend UART
        }
        UART_DETECT_PxIES = (UART_DETECT_PxIES & ~UART_DETECT_PxBIT) | (UART_DETECT_PxIN & UART_DETECT_PxBIT);
        __bic_SR_register_on_exit(LPM4_bits);
        break;
    case P2IV_P2IFG3: break;
    case P2IV_P2IFG4: break;
    case P2IV_P2IFG5: break;
    case P2IV_P2IFG6: break;
    case P2IV_P2IFG7: break;
    default: break;
  }
}



void handleEpicDinosaur(Uartio_t *uart, unsigned int argc, const char **argv)
{
    Uartio_println(uart, "Totally epic TYRANNOSAURUS REX.");
}

void handleHelp(Uartio_t *uart, unsigned int argc, const char **argv)
{
    Uartio_println(uart, "Help not available yet");
}

void handleVolts(Uartio_t *uart, unsigned int argc, const char **argv)
{
    // TODO: Pull ADC info, scale, print out voltages
    Uartio_println(uart, "Code to pull ADC info/compute voltages goes here.");
}
