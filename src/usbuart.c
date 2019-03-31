/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2012  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/cortex.h>

#include <libopencm3/stm32/st_usbfs.h>


#include "general.h"
#include "cdcacm.h"
#include "usbuart.h"

extern uint8_t st_usbfs_force_nak[8];

#define USBUART_TIMER_FREQ_HZ 1000000U /* 1us per tick */
#define USBUART_RUN_FREQ_HZ 5000U /* 200us (or 100 characters at 2Mbps) */

// Ring buffer
#define FIFO_SIZE 128

// Flags indicating last write packet was full size (64 chars)
// so zero length packet must follow.
#ifdef ENABLE_FORTH
#define FORTH_FLAGS 0x1111
#else
#define UART1_FLAGS 0x1111
#endif // #ifdef ENABLE_FORTH

#define UART2_FLAGS 0x2222
#define UART3_FLAGS 0x4444
#define USB_IN_FULL_PACKET_FLAG(flags) (flags & 0x000f)
#define USB_WRITE_PACKET_FLAG(flags)   (flags & 0x00f0)
#define USB_IN_FLUSH_FLAG(flags)       (flags & 0x0f00)
/* #define USB_OUT_FORCE_NAK_FLAG(flags)  (flags & 0xf000) */

#define WRITING_PACKET(flags)          (usbuart_flags & USB_WRITE_PACKET_FLAG(flags))
#define WRITING_PACKET_BEGIN(flags)    (usbuart_flags |= USB_WRITE_PACKET_FLAG(flags))
#define WRITING_PACKET_DONE(flags)     (usbuart_flags &= ~USB_WRITE_PACKET_FLAG(flags))

#define FLUSH(flags)                   (usbuart_flags & USB_IN_FLUSH_FLAG(flags))
#define FLUSH_REQ(flags)               (usbuart_flags |= USB_IN_FLUSH_FLAG(flags))
#define FLUSH_DONE(flags)              (usbuart_flags &= ~USB_IN_FLUSH_FLAG(flags))

/* #define FORCED_NAK(flags)              (usbuart_flags & USB_OUT_FORCE_NAK_FLAG(flags)) */
/* #define FORCE_NAK(flags)               (usbuart_flags |= USB_OUT_FORCE_NAK_FLAG(flags)) */
/* #define UNFORCE_NAK(flags)             (usbuart_flags &= ~USB_OUT_FORCE_NAK_FLAG(flags)) */


#define HALF_BUF(buf)                  ((buf)->len >= CDCACM_PACKET_SIZE)


// cd src; arm-none-eabi-gcc -Wall -Wextra -Werror -Wno-char-subscripts -Os -std=gnu99 -g3 -MD -I. -DENABLE_TRACE -mthumb -DDISCOVERY_STLINK -I../libopencm3/include -I . -mcpu=cortex-m3 -DSTM32F1 -c usbuart.c -dM -E - < /dev/null > ../defs.cpp
// grep TX_ADDR ../defs.cpp
// USB_PMA_BASE = 0x40006000
// USB_BTABLE_REG = 0x40005c00
// #define USB_EP0_TX_ADDR USB_EP_TX_ADDR(0);


struct fifo_t {
  uint8_t len;  
  uint8_t in;
  uint8_t buf[FIFO_SIZE];
};


#define FIFO(name) \
  struct fifo_t name = { \
    .in = 0, .len = 0 \
  };

void fifo_put(struct fifo_t *fifo, uint8_t c) {
  fifo->buf[fifo->in++] = c;
  
  if(fifo->len < FIFO_SIZE) fifo->len++;  
  if(fifo->in >= FIFO_SIZE) fifo->in = 0;  
}

uint8_t fifo_get(struct fifo_t *fifo) {
  int idx = fifo->in - fifo->len;

  if(idx < 0) idx += FIFO_SIZE;
  if(fifo->len > 0) fifo->len--;  
  return fifo->buf[idx];
}		       


typedef uint16_t ep_flags_t;

struct usbuart_t {
  uint32_t usart;
  struct fifo_t *rxbuf;  
  struct fifo_t *txbuf;  
  uint8_t ep;
  ep_flags_t flags;
};


extern int usb_usart1_ctl; // set to 1 to control uart1 settings by usb
char packet_buf_out[CDCACM_PACKET_SIZE];
char packet_buf_in[CDCACM_PACKET_SIZE];
ep_flags_t usbuart_flags;


#ifdef ENABLE_FORTH
FIFO(forth_emit_buf);
FIFO(forth_key_buf);
#else
FIFO(usart1_rx_buf);
FIFO(usart1_tx_buf);
#endif

FIFO(usart2_rx_buf);
FIFO(usart2_tx_buf);
FIFO(usart3_rx_buf);
FIFO(usart3_tx_buf);


#ifdef ENABLE_FORTH

const struct usbuart_t usbuart_forth = {
  USART1, &forth_emit_buf, &forth_key_buf,
  FORTH_ENDPOINT, FORTH_FLAGS
};

#else

const struct usbuart_t usbuart_usart1 = {
  USART1, &usart1_rx_buf, &usart1_tx_buf,
  UART1_ENDPOINT, UART1_FLAGS
};

#endif  

const struct usbuart_t usbuart_usart2 = {
  USART2, &usart2_rx_buf, &usart2_tx_buf,
  UART2_ENDPOINT, UART2_FLAGS
};

const struct usbuart_t usbuart_usart3 = {
  USART3, &usart3_rx_buf, &usart3_tx_buf,
  UART3_ENDPOINT, UART3_FLAGS
};


#ifdef ENABLE_TRACE

uint8_t *trc_pos = 0;

#define TRCB(b) (*(trc_pos++) = (b))

#define TRC_FLAG_P 0x04
#define TRC_FLAG_W 0x08
#define TRC_FLAG_F 0x10

void trc_hdr(uint8_t code, uint8_t ep, uint8_t ep_flags) {
  uint8_t data = code << 5;

  data += (ep - 1) / 2;

  if(usbuart_flags & USB_IN_FULL_PACKET_FLAG(ep_flags))
    data |= TRC_FLAG_P;

  if(usbuart_flags & USB_WRITE_PACKET_FLAG(ep_flags))
    data |= TRC_FLAG_W;

  if(usbuart_flags & USB_IN_FLUSH_FLAG(ep_flags))
    data |= TRC_FLAG_F;
  
  TRCB(data);
}

#define BEGIN 0
#define END 0x04

#define TRACE_USBUART_IN_CB(x)			\
  if(trc_pos) trc_hdr(0 + x, ep, ep_flags)

#define TRACE_USBUART_IN(x)			\
  if(trc_pos) {					\
    trc_hdr(1 + x, ep, ep_flags);		\
    TRCB(buf->len);				\
  }

#define TRACE_USBUART_OUT_CB(x)			\
  if(trc_pos) {					\
    trc_hdr(2 + x, ep, 0);			\
    TRCB(buf->len);				\
  }


// hack - force set flag if transfer still in progress
#define TRACE_USBUART_OUT_CB_EMPTY(f)					\
  if(trc_pos) {								\
    trc_hdr(3, ep + (f), 0);						\
    TRCB(USB_GET_EP_RX_COUNT(ep) & 0x3ff);				\
  }


#else  // #ifdef ENABLE_TRACE

#define TRACE_USBUART_IN(x)
#define TRACE_USBUART_IN_CB(x)
#define TRACE_USBUART_OUT_CB(x)
#define TRACE_USBUART_OUT_CB_EMPTY(f)

#endif // #ifdef ENABLE_TRACE



#ifdef USB_IN_TIM4
static void tim_init(int TIM, int RCC_TIM, int NVIC_TIM_IRQ) {
	rcc_periph_clock_enable(RCC_TIM);
	timer_reset(TIM);
	timer_set_mode(TIM, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
	timer_set_prescaler(TIM,
			rcc_apb2_frequency / USBUART_TIMER_FREQ_HZ * 2 - 1);

	timer_set_period(TIM,
			USBUART_TIMER_FREQ_HZ / USBUART_RUN_FREQ_HZ - 1);

	/* Setup update interrupt in NVIC (timer counting still disabled) */
	nvic_set_priority(NVIC_TIM_IRQ, IRQ_PRI_USBUSART_TIM);
	nvic_enable_irq(NVIC_TIM_IRQ);
	timer_enable_irq(TIM, TIM_DIER_UIE);
}
#endif

static void usart_init(int USART, int RCC_USART, int NVIC_USART_IRQ) {
  rcc_periph_clock_enable(RCC_USART);

  /* Setup UART parameters. */
  usart_set_baudrate(USART, 38400);
  usart_set_databits(USART, 8);
  usart_set_stopbits(USART, USART_STOPBITS_1);
  usart_set_mode(USART, USART_MODE_TX_RX);
  usart_set_parity(USART, USART_PARITY_NONE);
  usart_set_flow_control(USART, USART_FLOWCONTROL_NONE);

  usart_enable(USART);
	
  /* Enable interrupts */
  // usart_enable_rx_interrupt(USART);
  USART_CR1(USART) |= USART_CR1_RXNEIE + USART_CR1_IDLEIE;
  nvic_set_priority(NVIC_USART_IRQ, IRQ_PRI_USBUSART);
  nvic_enable_irq(NVIC_USART_IRQ);
}


void usbuart_init(void) {
  // gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9); // handled by forth
  // TODO: maybe also set RX to pull-down?
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO10);

#ifdef USB_MAPLEMINI
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
#endif
  
#ifndef ENABLE_FORTH
  usart_init(USART1, RCC_USART1, NVIC_USART1_IRQ);
#endif
  usart_init(USART2, RCC_USART2, NVIC_USART2_IRQ);
  usart_init(USART3, RCC_USART3, NVIC_USART3_IRQ);

#ifdef USB_IN_TIM4
  tim_init(TIM4, RCC_TIM4, NVIC_TIM4_IRQ);
#endif
}


void usbuart_set_line_coding(struct usb_cdc_line_coding *coding, int USBUSART) {
#ifdef ENABLE_FORTH
  if(USBUSART == USART1 && ! usb_usart1_ctl) return; // handled by forth    
#endif
  
  uint32_t sb[] = {USART_STOPBITS_1, USART_STOPBITS_1_5, USART_STOPBITS_2};
  uint32_t pa[] = {USART_PARITY_NONE, USART_PARITY_ODD, USART_PARITY_EVEN};

  usart_set_baudrate(USBUSART, coding->dwDTERate);
  usart_set_databits(USBUSART, coding->bDataBits +
		     (coding->bParityType ? 1 : 0));

  usart_set_stopbits(USBUSART, sb[coding->bCharFormat]);
  usart_set_parity(USBUSART, pa[coding->bParityType]);
}


void interrupts(int enable) {
  if(enable) {
    nvic_enable_irq(USB_IRQ);
    nvic_enable_irq(NVIC_USART2_IRQ);
    nvic_enable_irq(NVIC_USART3_IRQ);
#if ! defined ENABLE_FORTH
    nvic_enable_irq(NVIC_USART1_IRQ);
#elif defined USB_IN_TIM4
    nvic_enable_irq(NVIC_TIM4_IRQ);
#endif
  }
  else {
    nvic_disable_irq(USB_IRQ);
    nvic_disable_irq(NVIC_USART2_IRQ);
    nvic_disable_irq(NVIC_USART3_IRQ);
#if ! defined ENABLE_FORTH
    nvic_disable_irq(NVIC_USART1_IRQ);
#elif defined USB_IN_TIM4
    nvic_disable_irq(NVIC_TIM4_IRQ);
#endif
  }  
}


/******************************************************************************
 * 
 * USB IN endpoints handling
 *
 ******************************************************************************/

/* Prepare packet from uart rx fifo and send by In endpoint */
static void usbuart_in(usbd_device *dev, int ep, struct fifo_t *buf,
		       ep_flags_t ep_flags) {
  
  /* forcibly empty fifo if no USB endpoint */
  if (cdcacm_get_config() != 1)
    buf->len = buf->in = 0;
  else {
    int packet_size = 0;

    TRACE_USBUART_IN(BEGIN);
 
    while(packet_size < CDCACM_PACKET_SIZE && buf->len) {
      packet_buf_in[packet_size++] = fifo_get(buf);

/* #ifdef ENABLE_TRACE */
/*       if(trc_pos == 0) */
/* 	usart_send_blocking(USART3, packet_buf_in[packet_size - 1]); */
/* #endif */
    }

    // TODO: check the undocumented issue: stm32 usb read from packet memory disable interrupts
    cm_disable_interrupts();
    cm_disable_faults();

    int send = usbd_ep_write_packet(dev, ep, packet_buf_in, packet_size);

    cm_enable_faults();
    cm_enable_interrupts();

    buf->len += packet_size - send;

    // zero size packet must be send just after full size packet
    if(send == CDCACM_PACKET_SIZE) {
      usbuart_flags |= USB_IN_FULL_PACKET_FLAG(ep_flags);
    }

    if(send > 0 || packet_size == 0) {
      usbuart_flags |= USB_WRITE_PACKET_FLAG(ep_flags);
    }

    TRACE_USBUART_IN(END);
  }
}


static void usbuart_in_cb(usbd_device *dev, int ep,
			  struct fifo_t *buf, ep_flags_t ep_flags) {

  TRACE_USBUART_IN_CB(BEGIN);
  interrupts(0); // USART or timer (buffer flush request) would clobber buffer if interrupts not disabled
  
  if(usbuart_flags & USB_IN_FULL_PACKET_FLAG(ep_flags)) {
    usbuart_flags &= ~USB_IN_FULL_PACKET_FLAG(ep_flags);
    usbd_ep_write_packet(dev, ep, packet_buf_in, 0);
  }
  else {
    WRITING_PACKET_DONE(ep_flags);
    
    if(buf->len > 0) {
      if(FLUSH(ep_flags) || HALF_BUF(buf)) {
	usbuart_in(dev, ep, buf, ep_flags);
      }
    }
    else {
      FLUSH_DONE(ep_flags);
    }
  }

  interrupts(1);
  TRACE_USBUART_IN_CB(END);
}


#ifdef ENABLE_FORTH

void forth_usb_in_cb(usbd_device *dev, uint8_t ep) {
  usbuart_in_cb(dev, ep, &forth_emit_buf, FORTH_FLAGS);
}

#else

void uart1_usb_in_cb(usbd_device *dev, uint8_t ep) {
  usbuart_in_cb(dev, ep, &usart1_rx_buf, UART1_FLAGS);
}

#endif


void uart2_usb_in_cb(usbd_device *dev, uint8_t ep) {
  usbuart_in_cb(dev, ep, &usart2_rx_buf, UART2_FLAGS);
}


void uart3_usb_in_cb(usbd_device *dev, uint8_t ep) {
  usbuart_in_cb(dev, ep, &usart3_rx_buf, UART3_FLAGS);
}


void usb_in_tx_char(const struct usbuart_t *uu, char c) {
  // both timer and usb irq can call usbuart_in to flush
  // rx buf so disable them to not clobber the buffer here
  interrupts(0);

  fifo_put(uu->rxbuf, c);

  // start sending of rx buffer if no transaction in progress
  // and enough data to send full size packet
  if(! WRITING_PACKET(uu->flags) && HALF_BUF(uu->rxbuf)) {
    usbuart_in(usbdev, uu->ep, uu->rxbuf, uu->flags);
  }

  interrupts(1);
}


void usb_in_buf_flush(struct fifo_t *buf, uint8_t ep, ep_flags_t flags) { 
  interrupts(0);

  if(buf->len > 0) {
    FLUSH_REQ(flags);
    
    if(! WRITING_PACKET(flags)) {
      usbuart_in(usbdev, ep, buf, flags);
    }
  }
  
  interrupts(1);
}


#ifdef ENABLE_FORTH

void usb_emit_c(char c) {
#ifdef USB_IN_TIM4
  timer_disable_counter(TIM4);
  
  usb_in_tx_char(&usbuart_forth, c);
  timer_set_counter(TIM4, 0);

  timer_enable_counter(TIM4);
  
#else // #ifdef USB_IN_TIM4
  interrupts(0);
  
  fifo_put(&forth_emit_buf, c);    
  usb_in_buf_flush(&forth_emit_buf, FORTH_ENDPOINT,
		   FORTH_FLAGS);
#endif // #ifdef USB_IN_TIM4
}

#endif // #ifdef ENABLE_FORTH


/******************************************************************************
 * 
 * USB OUT endpoints handling
 *
 ******************************************************************************/

// Clear NAK (set rx status to valid so next packet can be read)
// if NAK is forced and there is enougs space for full packet in the buffer
void usb_out_maybe_clear_nak(struct fifo_t *buf, uint8_t ep) {
  if(st_usbfs_force_nak[ep] &&
     (FIFO_SIZE - CDCACM_PACKET_SIZE >= buf->len)) {
    TRACE_USBUART_OUT_CB_EMPTY(TRC_FLAG_W);
    usbd_ep_nak_set(usbdev, ep, 0);
  }
}


void usbuart_out_cb(usbd_device *dev, uint8_t ep,
		    struct fifo_t *buf, uint32_t usart) {

  TRACE_USBUART_OUT_CB(BEGIN);
  interrupts(0); // USART Tx interrupt would clobber buffer if interrupts enabled here
  
  // read a packet if there is a place for full packet size in key buffer
  if(buf->len > FIFO_SIZE - 2 * CDCACM_PACKET_SIZE) { // next packet might not fit, set NAK
    usbd_ep_nak_set(dev, ep, 1);
  }

  // TODO: Is this needed when reading or writing to packet memory or both?
  cm_disable_interrupts();
  cm_disable_faults();

  int len = usbd_ep_read_packet(dev, ep, packet_buf_out,
				CDCACM_PACKET_SIZE);

  TRACE_USBUART_OUT_CB_EMPTY((*USB_EP_REG(ep) & USB_EP_RX_STAT) == USB_EP_RX_STAT_VALID ? TRC_FLAG_P : 0);

  cm_enable_faults();
  cm_enable_interrupts();

  
  for(int i = 0; i < len; i++) {
    fifo_put(buf, packet_buf_out[i]);
  }

  usb_out_maybe_clear_nak(buf, ep);

  // enable usart tx interrupt if something in buffer
  if((buf->len > 0)
#ifdef ENABLE_FORTH     
     && (usart != USART_FORTH)
#endif
     ) {
      usart_enable_tx_interrupt(usart);
  }    

  interrupts(1);  
  
  TRACE_USBUART_OUT_CB(END);
}


#ifdef ENABLE_FORTH

void forth_usb_out_cb(usbd_device *dev, uint8_t ep) {
  usbuart_out_cb(dev, ep, &forth_key_buf, USART_FORTH);
}

#else

void uart1_usb_out_cb(usbd_device *dev, uint8_t ep) {
  usbuart_out_cb(dev, ep, &usart1_tx_buf, USART1);
}

#endif

void uart2_usb_out_cb(usbd_device *dev, uint8_t ep) {
  usbuart_out_cb(dev, ep, &usart2_tx_buf, USART2);
}


void uart3_usb_out_cb(usbd_device *dev, uint8_t ep) {
  usbuart_out_cb(dev, ep, &usart3_tx_buf, USART3);
}


char usb_out_rx_char(struct fifo_t *buf, int ep) {
  nvic_disable_irq(USB_IRQ);

  char c = fifo_get(buf);

  usb_out_maybe_clear_nak(buf, ep);  
  nvic_enable_irq(USB_IRQ);

  return c;
}


#ifdef ENABLE_FORTH
char usb_key_c() {
  return usb_out_rx_char(&forth_key_buf, FORTH_ENDPOINT);
}
#endif


/******************************************************************************
 * 
 * USART abd Timer Interrupts
 *
 ******************************************************************************/

void usart_rxtx(const struct usbuart_t *uu) {
  uint32_t usart = uu->usart;
  uint32_t sr = USART_SR(usart);

  if(sr & USART_SR_RXNE) {
    usb_in_tx_char(uu, usart_recv(usart));
  }
  else if(sr & USART_SR_IDLE) {
    (void) usart_recv(usart); // clear IDLE flag
    usb_in_buf_flush(uu->rxbuf, uu->ep, uu->flags);
  }
  else if(sr & USART_SR_TXE) {
    char c = usb_out_rx_char(uu->txbuf, uu->ep);

    if(uu->txbuf->len == 0) {
      usart_disable_tx_interrupt(usart);
    }

    usart_send(usart, c);
  }     
}


#ifdef ENABLE_FORTH

RELOC_ISR(tim4) {
#ifdef USB_IN_TIM4
  timer_clear_flag(TIM4, TIM_SR_UIF);
  timer_disable_counter(TIM4);
  usb_in_buf_flush(&forth_emit_buf, FORTH_ENDPOINT, FORTH_FLAGS);
#endif
}

#else // #ifdef ENABLE_FORTH

RELOC_ISR(usart1) {
  usart_rxtx(&usbuart_usart1);
}

#endif // #ifdef ENABLE_FORTH


RELOC_ISR(usart2) {
  usart_rxtx(&usbuart_usart2);
}


RELOC_ISR(usart3) {
  usart_rxtx(&usbuart_usart3);
}

