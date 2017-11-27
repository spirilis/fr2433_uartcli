/*
 * eusci_uartio.h
 *
 *  Created on: Nov 17, 2017
 *      Author: spiri
 */

#ifndef EUSCI_UARTIO_H_
#define EUSCI_UARTIO_H_

#include <stdint.h>
#include <stdlib.h>

/* I/O buffer parameters - can be changed per-project */

#define UARTLIB_TXBUF_SIZE 64
#define UARTLIB_RXBUF_SIZE 64

// Comment these out to disable
#define UARTLIB_EUSCIA0_ENABLED 1
#define UARTLIB_EUSCIA1_ENABLED 1




// Internal ring-buffer for TX & RX
typedef struct {
    uint8_t txbuf[UARTLIB_TXBUF_SIZE];
    uint8_t rxbuf[UARTLIB_RXBUF_SIZE];
    volatile unsigned int txbuf_head, txbuf_tail;
    volatile unsigned int rxbuf_head, rxbuf_tail;
    volatile bool rx_line_trigger;
} Uartio_RingBuffer_t;

// Handle to the driver state for client API commands
typedef struct {
    uint8_t eusci_instance;
    uint16_t eusci_base;
    Uartio_RingBuffer_t ringbuf;
    volatile bool rx_wakeup;
    volatile bool rx_line_wakeup;
    volatile bool txdone_wakeup;
} Uartio_t;



/* Function prototypes */
Uartio_t * Uartio_init(uint8_t, uint32_t); // Takes instance# (eUSCI_A#) and baudrate
void Uartio_suspend(Uartio_t *, bool);
bool Uartio_isSuspended(Uartio_t *);
void Uartio_setWakeup(Uartio_t *, bool, bool, bool);
unsigned int Uartio_available(Uartio_t *);
bool Uartio_line_available(Uartio_t *); // Detect if available input includes a newline character
int Uartio_read(Uartio_t *);
void Uartio_flush(Uartio_t *);
void Uartio_purge(Uartio_t *);
size_t Uartio_write(Uartio_t *, const uint8_t);
size_t Uartio_writen(Uartio_t *, const void *, const size_t);
size_t Uartio_print(Uartio_t *, const char *);
size_t Uartio_println(Uartio_t *, const char *);
void Uartio_printf(Uartio_t * uart, char *format, ...);


#endif /* EUSCI_UARTIO_H_ */
