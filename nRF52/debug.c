/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Bartosz Gajda - port to nRF52 DK platform (Nordic Semiconductor)  
 *******************************************************************************/


#include "nrf.h"
#include "lmic.h"
#include "debug.h"

#define LED_PIN         18
#define USART_TX_PIN    6
#define USART_RTS_PIN   5

void debug_init () {
    // configure LED pin as output
    
    NRF_GPIO->DIRSET = (1UL << LED_PIN);
    debug_led(0);

    // configure USART1 (115200/8N1, tx-only)
    NRF_GPIO->DIRSET = (1UL << USART_TX_PIN);
    //NRF_GPIO->DIRSET = (1UL << USART_RTS_PIN);

    NRF_UARTE0->CONFIG = (UART_CONFIG_HWFC_Disabled   << UART_CONFIG_HWFC_Pos) |
                         (UART_CONFIG_PARITY_Excluded << UART_CONFIG_PARITY_Pos); 
  
    NRF_UARTE0->BAUDRATE = UARTE_BAUDRATE_BAUDRATE_Baud115200 << UARTE_BAUDRATE_BAUDRATE_Pos;
    
    NRF_UARTE0->PSEL.TXD = USART_TX_PIN;
    
    NRF_UARTE0->ENABLE = UARTE_ENABLE_ENABLE_Enabled << UARTE_ENABLE_ENABLE_Pos;

    // print banner
    debug_str("\r\n============== DEBUG STARTED ==============\r\n");
}

void debug_led (u1_t val) {
    if (val == 1) {
        NRF_GPIO->OUTCLR = (1UL << LED_PIN);
    } else {
        NRF_GPIO->OUTSET = (1UL << LED_PIN);
    }
}

void debug_char (u1_t c) {
    NRF_UARTE0->TXD.MAXCNT = sizeof(c);
    NRF_UARTE0->TXD.PTR = (uint32_t)&c;
    
    //NRF_CLOCK->TASKS_HFCLKSTART = 1;
    //while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
    //NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    
    NRF_UARTE0->TASKS_STARTTX = 1;
    
    // Wait until the transfer is complete
    while (NRF_UARTE0->EVENTS_ENDTX == 0);
    NRF_UARTE0->EVENTS_ENDTX = 0;
  
    // Stop the UART TX
    NRF_UARTE0->TASKS_STOPTX = 1;
    // Wait until we receive the stopped event
    while (NRF_UARTE0->EVENTS_TXSTOPPED == 0);
}

void debug_hex (u1_t b) {
    debug_char("0123456789ABCDEF"[b>>4]);
    debug_char("0123456789ABCDEF"[b&0xF]);
}

void debug_buf (const u1_t* buf, u2_t len) {
    while(len--) {
        debug_hex(*buf++);
        debug_char(' ');
    }
    debug_char('\r');
    debug_char('\n');
}

void debug_uint (u4_t v) {
    for(s1_t n=24; n>=0; n-=8) {
        debug_hex(v>>n);
    }
}

void debug_str (const u1_t* str) {
    while(*str) {
        debug_char(*str++);
    
    }
}

void debug_val (const u1_t* label, u4_t val) {
    debug_str(label);
    debug_uint(val);
    debug_char('\r');
    debug_char('\n');
}

void debug_event (int ev) {
    static const u1_t* evnames[] = {
        [EV_SCAN_TIMEOUT]   = "SCAN_TIMEOUT",
        [EV_BEACON_FOUND]   = "BEACON_FOUND",
        [EV_BEACON_MISSED]  = "BEACON_MISSED",
        [EV_BEACON_TRACKED] = "BEACON_TRACKED",
        [EV_JOINING]        = "JOINING",
        [EV_JOINED]         = "JOINED",
        [EV_RFU1]           = "RFU1",
        [EV_JOIN_FAILED]    = "JOIN_FAILED",
        [EV_REJOIN_FAILED]  = "REJOIN_FAILED",
        [EV_TXCOMPLETE]     = "TXCOMPLETE",
        [EV_LOST_TSYNC]     = "LOST_TSYNC",
        [EV_RESET]          = "RESET",
        [EV_RXCOMPLETE]     = "RXCOMPLETE",
        [EV_LINK_DEAD]      = "LINK_DEAD",
        [EV_LINK_ALIVE]     = "LINK_ALIVE",
    };
    debug_str(evnames[ev]);
    debug_char('\r');
    debug_char('\n');
}
