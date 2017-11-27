/*
 * eusci_uartio.c
 *
 *  Created on: Nov 17, 2017
 *      Author: spiri
 */

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>
#include "driverlib.h"
#include "eusci_uartio.h"


// Buffer/data structures
#if defined(__MSP430_HAS_EUSCI_A0__) && defined(UARTLIB_EUSCIA0_ENABLED)
static Uartio_t _uartio_inst_0;
#endif
#if defined(__MSP430_HAS_EUSCI_A1__) && defined(UARTLIB_EUSCIA1_ENABLED)
static Uartio_t _uartio_inst_1;
#endif


Uartio_t * Uartio_init(uint8_t eusci_instance, uint32_t baudrate)
{
    EUSCI_A_UART_initParam drvlbParams;
    drvlbParams.uartMode = EUSCI_A_UART_MODE;
    drvlbParams.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
    drvlbParams.parity = EUSCI_A_UART_NO_PARITY;
    drvlbParams.overSampling = EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
    drvlbParams.msborLsbFirst = EUSCI_A_UART_LSB_FIRST;
    drvlbParams.numberofStopBits = EUSCI_A_UART_ONE_STOP_BIT;
    uint32_t brDiv, brMod;
    brDiv = (16000000) / baudrate; // TODO: This assumes 16MHz SMCLK!
    brMod = brDiv & 0x0FFF;
    brDiv >>= 4;
    drvlbParams.clockPrescalar = brDiv;
    drvlbParams.firstModReg = brMod & 0x0F;
    drvlbParams.secondModReg = brMod >> 4;

    switch (eusci_instance) {
    #if defined(__MSP430_HAS_EUSCI_A0__) && defined(UARTLIB_EUSCIA0_ENABLED)
    case 0:
        _uartio_inst_0.eusci_instance = 0;
        _uartio_inst_0.eusci_base = EUSCI_A0_BASE;
        // Initialize buffers and peripheral
        _uartio_inst_0.ringbuf.txbuf_head = 0;
        _uartio_inst_0.ringbuf.txbuf_tail = 0;
        _uartio_inst_0.ringbuf.rxbuf_head = 0;
        _uartio_inst_0.ringbuf.rxbuf_tail = 0;

        EUSCI_A_UART_init(_uartio_inst_0.eusci_base, &drvlbParams);
        EUSCI_A_UART_enable(_uartio_inst_0.eusci_base);
        EUSCI_A_UART_enableInterrupt(_uartio_inst_0.eusci_base, EUSCI_A_UART_RECEIVE_INTERRUPT);
        return &_uartio_inst_0;
    #endif
    #if defined(__MSP430_HAS_EUSCI_A1__) && defined(UARTLIB_EUSCIA1_ENABLED)
    case 1:
        _uartio_inst_1.eusci_instance = 1;
        _uartio_inst_1.eusci_base = EUSCI_A1_BASE;
        // Initialize buffers and peripheral
        _uartio_inst_1.ringbuf.txbuf_head = 0;
        _uartio_inst_1.ringbuf.txbuf_tail = 0;
        _uartio_inst_1.ringbuf.rxbuf_head = 0;
        _uartio_inst_1.ringbuf.rxbuf_tail = 0;

        EUSCI_A_UART_init(_uartio_inst_1.eusci_base, &drvlbParams);
        EUSCI_A_UART_enable(_uartio_inst_1.eusci_base);
        EUSCI_A_UART_enableInterrupt(_uartio_inst_1.eusci_base, EUSCI_A_UART_RECEIVE_INTERRUPT);
        return &_uartio_inst_1;
    #endif
    default:
        return (void *)0;
    }
}

void Uartio_suspend(Uartio_t *uart, bool doSuspend)
{
    if (doSuspend) {
        Uartio_purge(uart);
        EUSCI_A_UART_disable(uart->eusci_base);
    } else {
        EUSCI_A_UART_enable(uart->eusci_base);
        EUSCI_A_UART_enableInterrupt(uart->eusci_base, EUSCI_A_UART_RECEIVE_INTERRUPT);
    }
}

bool Uartio_isSuspended(Uartio_t *uart)
{
    uint16_t ucactlw0 = HWREG16(uart->eusci_base + OFS_UCAxCTLW0);
    if (ucactlw0 & UCSWRST) {
        return true;
    }
    return false;
}

void Uartio_setWakeup(Uartio_t *uart, bool rx_wake, bool rx_line_wakeup, bool txdone_wake)
{
    uart->rx_wakeup = rx_wake;
    uart->rx_line_wakeup = rx_line_wakeup;
    uart->txdone_wakeup = txdone_wake;
}

unsigned int Uartio_available(Uartio_t *uart)
{
    uint16_t srState = __get_SR_register() & GIE; // make our reads atomic
    __disable_interrupt();

    unsigned int rxhead = uart->ringbuf.rxbuf_head;
    unsigned int rxtail = uart->ringbuf.rxbuf_tail;

    __bis_SR_register(srState); // restore interrupt state

    return (unsigned int)(UARTLIB_RXBUF_SIZE + rxhead - rxtail) % UARTLIB_RXBUF_SIZE;
}

bool Uartio_line_available(Uartio_t *uart)
{
    uint16_t srState = __get_SR_register() & GIE;
    __disable_interrupt();
    bool st = uart->ringbuf.rx_line_trigger;
    uart->ringbuf.rx_line_trigger = false;
    __bis_SR_register(srState);

    return st;
}

int Uartio_read(Uartio_t *uart)
{
    uint16_t srState = __get_SR_register() & GIE;
    __disable_interrupt();

    if (uart->ringbuf.rxbuf_head == uart->ringbuf.rxbuf_tail) {
        __bis_SR_register(srState);
        return -1;  // Nothing waiting to be read
    }

    uint8_t c = uart->ringbuf.rxbuf[uart->ringbuf.rxbuf_tail];
    uart->ringbuf.rxbuf_tail = (uart->ringbuf.rxbuf_tail + 1) % UARTLIB_RXBUF_SIZE;
    __bis_SR_register(srState);

    return (int)c;
}

//
void Uartio_flush(Uartio_t *uart)
{
    unsigned int txhead, txtail;
    uint16_t srState = __get_SR_register() & GIE;

    if (!srState) {
        return;  // We can't use flush if interrupts are disabled because no bytes will be written!
    }

    // Busy-wait until all TX buffer contents have been transmitted
    do {
        __delay_cycles(16);
        txhead = uart->ringbuf.txbuf_head;
        txtail = uart->ringbuf.txbuf_tail;
    } while (txhead != txtail);
}

void Uartio_purge(Uartio_t *uart)
{
    uint16_t srState = __get_SR_register() & GIE;
    __disable_interrupt();

    unsigned int txhead = uart->ringbuf.txbuf_head;
    unsigned int txtail = uart->ringbuf.txbuf_tail;

    if (txhead != txtail) {
        EUSCI_A_UART_disableInterrupt(uart->eusci_base, EUSCI_A_UART_TRANSMIT_INTERRUPT | EUSCI_A_UART_TRANSMIT_COMPLETE_INTERRUPT_FLAG);
        // This should run successfully even with GIE=0, since it's polling the eUSCI peripheral logic.
        while (HWREG16(uart->eusci_base + OFS_UCAxSTATW) & UCBUSY)
            ;
        uart->ringbuf.txbuf_tail = uart->ringbuf.txbuf_head;
    }

    __bis_SR_register(srState);
    return;
}

size_t Uartio_write(Uartio_t *uart, const uint8_t c)
{
    unsigned int i = (uart->ringbuf.txbuf_head + 1) % UARTLIB_TXBUF_SIZE;
    unsigned int srState = __get_SR_register() & GIE;

    // If output buffer is full, nothing to do here but busy-wait?  Unless GIE=0, then return 0.
    if (i == uart->ringbuf.txbuf_tail) {
        if ( (srState & GIE) == 0 ) {
            return 0;  // Buffer full, but we can't busy-wait since the TX buffer will never drain in GIE=0 mode
        }
        while (i == uart->ringbuf.txbuf_tail)
            ;
    }

    __disable_interrupt();
    uart->ringbuf.txbuf[uart->ringbuf.txbuf_head] = c;
    uart->ringbuf.txbuf_head = i;
    // Force UCTXIFG
    if (!(HWREG16(uart->eusci_base + OFS_UCAxIE) & UCTXIE)) {
        HWREG16(uart->eusci_base + OFS_UCAxIFG) |= UCTXIFG;
        HWREG16(uart->eusci_base + OFS_UCAxIE) |= (UCTXIE | UCTXCPTIE);
    }
    __bis_SR_register(srState);

    return 1;
}

// Uartio_writen stuffs the TX buffer without activating I/O until it's done; if the buffer fills up,
// it's a short write regardless of whether we're running with GIE=0 or not.
size_t Uartio_writen(Uartio_t *uart, const void *buf, const size_t n)
{
    unsigned int i = (uart->ringbuf.txbuf_head + 1) % UARTLIB_TXBUF_SIZE;
    unsigned int wrtn = 0;
    const uint8_t * cbuf = (const uint8_t *)buf;
    unsigned int srState = __get_SR_register() & GIE;

    __disable_interrupt();
    while (wrtn < n) {
        if (i == uart->ringbuf.txbuf_tail) {
            break;  // Cut the write short if we run out of TX ringbuffer
        }

        uart->ringbuf.txbuf[uart->ringbuf.txbuf_head] = cbuf[wrtn];
        uart->ringbuf.txbuf_head = i;
        wrtn++;

        i = (uart->ringbuf.txbuf_head + 1) % UARTLIB_TXBUF_SIZE;
    }

    // Force UCTXIFG
    if (wrtn > 0 && !(HWREG16(uart->eusci_base + OFS_UCAxIE) & UCTXIE)) {
        HWREG16(uart->eusci_base + OFS_UCAxIFG) |= UCTXIFG;
        HWREG16(uart->eusci_base + OFS_UCAxIE) |= (UCTXIE | UCTXCPTIE);
    }
    __bis_SR_register(srState);

    return wrtn;
}

size_t Uartio_print(Uartio_t * uart, const char *text)
{
    if (text != NULL) {
       size_t len = strlen(text);
       return Uartio_writen(uart, text, len);
    }
    return 0;
}

size_t Uartio_println(Uartio_t * uart, const char *text)
{
    size_t len = 0, ttl = 0;
    const char * newln = "\r\n";

    if (text != NULL) {
        len = strlen(text);
    }
    ttl = Uartio_writen(uart, text, len);
    ttl += Uartio_writen(uart, newln, 2);

    return ttl;
}

/* Printf implementation */
static const unsigned long dv[] = {
//  4294967296      // 32 bit unsigned max
    1000000000,     // +0
     100000000,     // +1
      10000000,     // +2
       1000000,     // +3
        100000,     // +4
//       65535      // 16 bit unsigned max
         10000,     // +5
          1000,     // +6
           100,     // +7
            10,     // +8
             1,     // +9
};

static void xtoa(Uartio_t * uart, unsigned long x, const unsigned long *dp)
{
    char c;
    unsigned long d;
    if(x) {
        while(x < *dp) ++dp;
        do {
            d = *dp++;
            c = '0';
            while(x >= d) ++c, x -= d;
            Uartio_write(uart, c);
        } while(!(d & 1));
    } else {
        Uartio_write(uart, '0');
    }
}

static void puth(Uartio_t * uart, unsigned n)
{
    static const char hex[16] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };
    Uartio_write(uart, hex[n & 15]);
}

void Uartio_printf(Uartio_t * uart, char *format, ...)
{
    char c;
    int i;
    long n;

    if (format == NULL) {
        return;
    }
    va_list a;
    va_start(a, format);
    while( (c = *format++) ) {
        if(c == '%') {
            switch(c = *format++) {
                case 's':                       // String
                    Uartio_print(uart, va_arg(a, char*));
                    break;
                case 'c':                       // Char
                    Uartio_write(uart, va_arg(a, int)); // Char gets promoted to Int in args, so it's an int we're looking for (GCC warning)
                    break;
                case 'i':                       // 16 bit Integer
                case 'd':                       // 16 bit Integer
                case 'u':                       // 16 bit Unsigned
                    i = va_arg(a, int);
                    if( (c == 'i' || c == 'd') && i < 0 ) i = -i, Uartio_write(uart, '-');
                    xtoa(uart, (unsigned)i, dv + 5);
                    break;
                case 'l':                       // 32 bit Long
                case 'n':                       // 32 bit uNsigned loNg
                    n = va_arg(a, long);
                    if(c == 'l' &&  n < 0) n = -n, Uartio_write(uart, '-');
                    xtoa(uart, (unsigned long)n, dv);
                    break;
                case 'x':                       // 16 bit heXadecimal
                    i = va_arg(a, int);
                    puth(uart, i >> 12);
                    puth(uart, i >> 8);
                    puth(uart, i >> 4);
                    puth(uart, i);
                    break;
                case 0: return;
                default: goto bad_fmt;
            }
        } else
bad_fmt:    Uartio_write(uart, c);
    }
    va_end(a);
}


// eUSCI_A0 interrupt vector table
#if defined(__MSP430_HAS_EUSCI_A0__) && defined(UARTLIB_EUSCIA0_ENABLED)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  uint8_t c;
  unsigned int i;

  switch(__even_in_range(UCA0IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      __no_operation();
      c = UCA0RXBUF;
      i = (unsigned int)(_uartio_inst_0.ringbuf.rxbuf_head + 1) % UARTLIB_RXBUF_SIZE;
      if (i != _uartio_inst_0.ringbuf.rxbuf_tail) {
          _uartio_inst_0.ringbuf.rxbuf[_uartio_inst_0.ringbuf.rxbuf_head] = c;
          _uartio_inst_0.ringbuf.rxbuf_head = i;
      }
      if (_uartio_inst_0.rx_wakeup) {
          __bic_SR_register_on_exit(LPM4_bits);
      }
      if (c == '\n') {
          _uartio_inst_0.ringbuf.rx_line_trigger = true;
          if (_uartio_inst_0.rx_line_wakeup) {
              __bic_SR_register_on_exit(LPM4_bits);
          }
      }
      __no_operation();
      break;
    case USCI_UART_UCTXIFG:
      // TX buffer empty?  Disable interrupts if so.
      if (_uartio_inst_0.ringbuf.txbuf_head == _uartio_inst_0.ringbuf.txbuf_tail) {
          HWREG16(EUSCI_A0_BASE + OFS_UCAxIE) &= ~(UCTXIE | UCTXCPTIE);
          if (_uartio_inst_0.txdone_wakeup) {
              __bic_SR_register_on_exit(LPM4_bits);
          }
          __no_operation();
          return;
      }
      // Else, Transmit next byte.
      UCA0TXBUF = _uartio_inst_0.ringbuf.txbuf[_uartio_inst_0.ringbuf.txbuf_tail];
      _uartio_inst_0.ringbuf.txbuf_tail = (_uartio_inst_0.ringbuf.txbuf_tail + 1) % UARTLIB_TXBUF_SIZE;
      __no_operation();
      break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}
#endif

// eUSCI_A1 interrupt vector table
#if defined(__MSP430_HAS_EUSCI_A1__) && defined(UARTLIB_EUSCIA1_ENABLED)
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A1_VECTOR))) USCI_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  uint8_t c;
  unsigned int i;

  switch(__even_in_range(UCA1IV,USCI_UART_UCTXCPTIFG))
  {
    case USCI_NONE: break;
    case USCI_UART_UCRXIFG:
      __no_operation();
      c = UCA1RXBUF;
      i = (unsigned int)(_uartio_inst_1.ringbuf.rxbuf_head + 1) % UARTLIB_RXBUF_SIZE;
      if (i != _uartio_inst_1.ringbuf.rxbuf_tail) {
          _uartio_inst_1.ringbuf.rxbuf[_uartio_inst_1.ringbuf.rxbuf_head] = c;
          _uartio_inst_1.ringbuf.rxbuf_head = i;
      }
      if (_uartio_inst_1.rx_wakeup) {
          __bic_SR_register_on_exit(LPM4_bits);
      }
      if (c == '\n') {
          _uartio_inst_1.ringbuf.rx_line_trigger = true;
          if (_uartio_inst_1.rx_line_wakeup) {
              __bic_SR_register_on_exit(LPM4_bits);
          }
      }
      __no_operation();
      break;
    case USCI_UART_UCTXIFG:
      // TX buffer empty?  Disable interrupts if so.
      if (_uartio_inst_1.ringbuf.txbuf_head == _uartio_inst_1.ringbuf.txbuf_tail) {
          HWREG16(EUSCI_A1_BASE + OFS_UCAxIE) &= ~(UCTXIE | UCTXCPTIE);
          if (_uartio_inst_1.txdone_wakeup) {
              __bic_SR_register_on_exit(LPM4_bits);
          }
          __no_operation();
          return;
      }
      UCA1TXBUF = _uartio_inst_1.ringbuf.txbuf[_uartio_inst_1.ringbuf.txbuf_tail];
      _uartio_inst_1.ringbuf.txbuf_tail = (_uartio_inst_1.ringbuf.txbuf_tail + 1) % UARTLIB_TXBUF_SIZE;
      __no_operation();
      break;
    case USCI_UART_UCSTTIFG: break;
    case USCI_UART_UCTXCPTIFG: break;
    default: break;
  }
}
#endif
