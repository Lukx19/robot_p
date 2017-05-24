/*
 * uart.h
 *
 * Created: 19-May-17 10:23:28 AM
 *  Author: petrg
 */ 
#ifndef uart_h_
#define uart_h_

#ifndef F_CPU
# warning "F_CPU macro is not defined, setting up default 1MHz"
#define F_CPU 1000000LU
#endif

#ifndef USART_BAUDRATE 
#define USART_BAUDRATE 9600
# warning "USART_BAUDRATE macro is not defined, setting up default 9600 BAUD"
#endif

#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/io.h>

void usart_init(void)
{
	// Set baud rate
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Set frame format to 8 data bits, no parity, 1 stop bit
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	//enable recv interrupt, transmission and reception
	UCSR0B |= (1 << RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
}

void usart_send_byte(uint8_t u8Data)
{
	//wait while previous byte is completed
	while(!(UCSR0A&(1<<UDRE0))){};
	// Transmit data
	UDR0 = u8Data;
}

uint8_t usart_receive_byte()
{
	// Wait for byte to be received
	while(!(UCSR0A&(1<<RXC0))){};
	// Return received data
	return UDR0;
}

//#define RING_BUFFER_MAX 128
//
//typedef struct {
	//size_t s_elem;
	//size_t n_elem;
	//void *buffer;
//} rb_attr_t;
//
//typedef unsigned int rbd_t;
//
//struct ring_buffer
//{
	//size_t s_elem;
	//size_t n_elem;
	//uint8_t *buf;
	//volatile size_t head;
	//volatile size_t tail;
//};
//
//static struct ring_buffer _rb[RING_BUFFER_MAX];
//
//int ring_buffer_init(rbd_t *rbd, rb_attr_t *attr)
//{
	//static int idx = 0;
	//int err = -1;
	//
	//if ((idx < RING_BUFFER_MAX) && (rbd != NULL) && (attr != NULL)) {
		//if ((attr->buffer != NULL) && (attr->s_elem > 0)) {
			///* Check that the size of the ring buffer is a power of 2 */
			//if (((attr->n_elem - 1) & attr->n_elem) == 0) {
				///* Initialize the ring buffer internal variables */
				//_rb[idx].head = 0;
				//_rb[idx].tail = 0;
				//_rb[idx].buf = attr->buffer;
				//_rb[idx].s_elem = attr->s_elem;
				//_rb[idx].n_elem = attr->n_elem;
				//
				//*rbd = idx++;
				//err= 0;
			//}
		//}
	//}
	//
	//return err;
//}
//
//
//static int _ring_buffer_full(struct ring_buffer *rb)
//{
	//return ((rb->head - rb->tail) == rb->n_elem) ? 1 : 0;
//}
//
//static int _ring_buffer_empty(struct ring_buffer *rb)
//{
	//return ((rb->head - rb->tail) == 0U) ? 1 : 0;
//}
//
//int ring_buffer_put(rbd_t rbd, const void *data)
//{
	//int err = 0;
	//
	//if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_full(&_rb[rbd]) == 0)) {
		//const size_t offset = (_rb[rbd].head & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
		//memcpy(&(_rb[rbd].buf[offset]), data, _rb[rbd].s_elem);
		//_rb[rbd].head++;
		//} else {
		//err = -1;
	//}
	//
	//return err;
//}
//
//const size_t offset = (_rb[rbd].head & _rb[rbd].n_elem) * _rb[rbd].s_elem;
//
//int ring_buffer_get(rbd_t rbd, void *data)
//{
	//int err = 0;
	//
	//if ((rbd < RING_BUFFER_MAX) && (_ring_buffer_empty(&_rb[rbd]) == 0)) {
		//const size_t offset = (_rb[rbd].tail & (_rb[rbd].n_elem - 1)) * _rb[rbd].s_elem;
		//memcpy(data, &(_rb[rbd].buf[offset]), _rb[rbd].s_elem);
		//_rb[rbd].tail++;
		//} else {
		//err = -1;
	//}
	//
	//return err;
//}
//
//static rbd_t _rbd;
//static char _rbmem[8];
//
//void x(void){
	//
//
	//if (i < ARRAY_SIZE(_baud_tbl)) {
		//rb_attr_t attr = {sizeof(_rbmem[0]), ARRAY_SIZE(_rbmem), _rbmem};
	//
		///* Set the baud rate */
		//UCA0BR0 = _baud_tbl[i].UCAxBR0;
		//UCA0BR1 = _baud_tbl[i].UCAxBR1;
		//UCA0MCTL = _baud_tbl[i].UCAxMCTL;
	//
		///* Initialize the ring buffer */
		//if (ring_buffer_init(&_rbd, &attr) == 0) {
			///* Enable the USCI peripheral (take it out of reset) */
			//UCA0CTL1 &= ~UCSWRST;
		//
			///* Enable rx interrupts */
			//IE2 |= UCA0RXIE;
		//
			//status = 0;
		//}
	//}
//}
//
//int uart_getchar(void)
//{
	//char c = -1;
	//
	//ring_buffer_get(_rbd, &c);
	//
	//return c;
//}
//
//__attribute__((interrupt(USCIAB0RX_VECTOR))) void rx_isr(void)
//{
	//if (IFG2 & UCA0RXIFG) {
		//const char c = UCA0RXBUF;
		//
		///* Clear the interrupt flag */
		//IFG2 &= ~UCA0RXIFG;
		//
		//ring_buffer_put(_rbd, &c);
	//}
//}


#endif