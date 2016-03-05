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

#include "lmic.h"
#include "nrf.h"
#include "debug.h"


// -----------------------------------------------------------------------------
// I/O & SPI

#define SCK_PIN            25
#define MISO_PIN   				 24
#define MOSI_PIN           23
#define NSS_PIN            22 
#define RXTX_PIN             1
#define RST_PIN            3    
#define DIO0_PIN           13  //  sx1276  (line 1 irq handler)
#define DIO1_PIN           14  //  sx1276  (line 10-15 irq handler)
#define DIO2_PIN           15  //  sx1276  (line 10-15 irq handler)


// HAL state
static struct {
    int irqlevel;
    u4_t ticks;
} HAL;

// -----------------------------------------------------------------------------
// I/O

static void hal_io_init () {

    NRF_GPIOTE->CONFIG[0] =  (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
                             (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                             (DIO0_PIN                      << GPIOTE_CONFIG_PSEL_Pos);
    NRF_GPIOTE->CONFIG[1] =  (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
                             (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                             (DIO1_PIN                      << GPIOTE_CONFIG_PSEL_Pos);
    NRF_GPIOTE->CONFIG[2] =  (GPIOTE_CONFIG_MODE_Event      << GPIOTE_CONFIG_MODE_Pos) |
                             (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                             (DIO2_PIN                      << GPIOTE_CONFIG_PSEL_Pos);
    NRF_GPIOTE->INTENSET  =  (GPIOTE_INTENSET_IN0_Set       << GPIOTE_INTENSET_IN0_Pos)  |
                             (GPIOTE_INTENSET_IN1_Set       << GPIOTE_INTENSET_IN1_Pos)  |
                             (GPIOTE_INTENSET_IN2_Set       << GPIOTE_INTENSET_IN2_Pos) ;
    
    NRF_GPIO->DIRSET = (1UL << RXTX_PIN);
    
    NRF_GPIO->DIRSET = (1UL << NSS_PIN);
    NRF_GPIO->OUTSET = (1UL << NSS_PIN);
    
    
 
}

// val == 1  => tx 1, rx 0 ; val == 0 => tx 0, rx 1
void hal_pin_rxtx (u1_t val) {
    ASSERT(val == 1 || val == 0);
    if(val == 0) {
        NRF_GPIO->OUTSET = (1UL << RXTX_PIN);
    } else {
        NRF_GPIO->OUTCLR = (1UL << RXTX_PIN);
    }
}


// set radio NSS pin to given value
void hal_pin_nss (u1_t val) {
    if(val == 1) {
        NRF_GPIO->OUTSET = (1UL << NSS_PIN);
    } else {
        NRF_GPIO->OUTCLR = (1UL << NSS_PIN);
    }
}

// set radio RST pin to given value (or keep floating!)
void hal_pin_rst (u1_t val) {
    if(val == 0) { // drive pin low
        NRF_GPIO->DIRSET = (1UL << RST_PIN);
        NRF_GPIO->OUTCLR = (1UL << RST_PIN);
    } else if(val == 1) { // drive pin high
        NRF_GPIO->DIRSET = (1UL << RST_PIN);
        NRF_GPIO->OUTSET = (1UL << RST_PIN);
    } else { // keep pin floating
        NRF_GPIO->DIRCLR = (1UL << RST_PIN);
    }
}

extern void radio_irq_handler(u1_t dio);

// generic EXTI IRQ handler for all channels
void GPIOTE_IRQHandler () {
    // DIO 0
    if(NRF_GPIOTE->EVENTS_IN[0] != 0) { // pending
        NRF_GPIOTE->EVENTS_IN[0] = 0; // clear event
        // invoke radio handler (on IRQ!)
        radio_irq_handler(0);
    }
    // DIO 1
    if(NRF_GPIOTE->EVENTS_IN[1] != 0) { // pending
        NRF_GPIOTE->EVENTS_IN[1] = 0; // clear event
        // invoke radio handler (on IRQ!)
        radio_irq_handler(1);
    }
    // DIO 2
    if(NRF_GPIOTE->EVENTS_IN[2] != 0) { // pending
        NRF_GPIOTE->EVENTS_IN[2] = 0; // clear event
        // invoke radio handler (on IRQ!)
        radio_irq_handler(2);
    }
}

// -----------------------------------------------------------------------------
// SPI

//static void hal_spi_init () { //SPI pins config
//    
//    //gpio cfg: SCK
//    NRF_GPIO->PIN_CNF[SCK_PIN] = (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
//    NRF_GPIO->DIRSET = (1UL << SCK_PIN); 
//    NRF_GPIO->OUTCLR = (1UL << SCK_PIN);
//    NRF_SPIM0->PSEL.SCK = (SCK_PIN << SPIM_PSEL_SCK_PIN_Pos) | 
//                          (SPIM_PSEL_SCK_CONNECT_Connected << SPIM_PSEL_SCK_CONNECT_Pos); 
//    
//    //gpio cfg: MOSI
//    NRF_GPIO->DIRSET = (1UL << MOSI_PIN);
//    NRF_GPIO->OUTSET = (1UL << MOSI_PIN);
//    NRF_SPIM0->PSEL.MOSI = (MOSI_PIN << SPIM_PSEL_MOSI_PIN_Pos) | 
//                           (SPIM_PSEL_MOSI_CONNECT_Connected << SPIM_PSEL_MOSI_CONNECT_Pos); 
//    
//    //gpio cfg: MISO
//    NRF_GPIO->DIRCLR = (1UL << MISO_PIN);
//    NRF_GPIO->PIN_CNF[MISO_PIN] = (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
//    NRF_SPIM0->PSEL.MISO = (MISO_PIN << SPIM_PSEL_MISO_PIN_Pos) | 
//                           (SPIM_PSEL_MISO_CONNECT_Connected << SPIM_PSEL_MISO_CONNECT_Pos); 
//    
//    NRF_SPIM0->CONFIG = (SPIM_CONFIG_CPHA_Leading << SPIM_CONFIG_CPHA_Pos) | (SPIM_CONFIG_CPOL_ActiveHigh << SPIM_CONFIG_CPOL_Pos); 
//    NRF_SPIM0->FREQUENCY = (SPIM_FREQUENCY_FREQUENCY_K125 << SPIM_FREQUENCY_FREQUENCY_Pos);
//    NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Enabled << SPIM_ENABLE_ENABLE_Pos; 
//    
//}

static void hal_spi_init () { //SPI pins config
    
    //gpio cfg: SCK
    NRF_GPIO->PIN_CNF[SCK_PIN] = (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_GPIO->DIRSET = (1UL << SCK_PIN); 
    NRF_GPIO->OUTCLR = (1UL << SCK_PIN);
    NRF_SPI0->PSEL.SCK = SCK_PIN; 
    
    //gpio cfg: MOSI
    NRF_GPIO->DIRSET = (1UL << MOSI_PIN);
    NRF_GPIO->OUTCLR = (1UL << MOSI_PIN);
    NRF_SPI0->PSEL.MOSI = MOSI_PIN;  
    
    //gpio cfg: MISO
    NRF_GPIO->DIRCLR = (1UL << MISO_PIN);
    NRF_GPIO->PIN_CNF[MISO_PIN] = (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos);
    NRF_SPI0->PSEL.MISO = MISO_PIN; 
    
    //NRF_SPI0->CONFIG = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos); 
    //NRF_SPI0->FREQUENCY = (SPI_FREQUENCY_FREQUENCY_K125 << SPI_FREQUENCY_FREQUENCY_Pos);
    NRF_SPI0->ENABLE = SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos; 
    
}

// perform SPI transaction with radio
//u1_t hal_spi (u1_t out) {
//    uint32_t spi_in; 
//    uint32_t *spi_in_ptr;
//    spi_in_ptr = &spi_in;
//    
//    u1_t *spi_out_ptr;
//    spi_out_ptr = &out;
//    
//    NRF_SPIM0->TXD.PTR = (uint32_t)spi_out_ptr;
//    NRF_SPIM0->TXD.MAXCNT = 1 << SPIM_TXD_MAXCNT_MAXCNT_Pos;
//    NRF_SPIM0->RXD.PTR = (uint32_t)spi_in_ptr;
//    NRF_SPIM0->RXD.MAXCNT = 1 << SPIM_RXD_MAXCNT_MAXCNT_Pos;    
//    
//    NRF_SPIM0->TASKS_START = 1;
//    
//    while(NRF_SPIM0->EVENTS_ENDRX == 0); //SPI transaction
//    NRF_SPIM0->EVENTS_ENDRX = 0;
//    
//    //NRF_SPIM0->TASKS_STOP = 1;
//    
//    //while(NRF_SPIM0->EVENTS_STOPPED == 0); //SPI stopping
//    //NRF_SPIM0->EVENTS_STOPPED = 0;
//    
//    //NRF_SPIM0->ENABLE = SPIM_ENABLE_ENABLE_Disabled << SPIM_ENABLE_ENABLE_Pos;
//    
//    return *((u1_t *)spi_in_ptr); // return received byte
//}

u1_t hal_spi (u1_t out) {
        
    u1_t spi_in;
    
    NRF_SPI0->TXD = out;
          
    while(NRF_SPI0->EVENTS_READY == 0); //SPI transaction
    NRF_SPI0->EVENTS_READY = 0;
    
    spi_in = NRF_SPI0->RXD; 
    
    return spi_in; // return received byte
}


// -----------------------------------------------------------------------------
// TIME

static void hal_time_init () {
 
    // Start LFCLK (32kHz) crystal oscillator. If you don't have crystal on your board, choose RCOSC instead.
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    
    // set prescaler
    NRF_RTC0->PRESCALER = 639;
    
    // enable overflow event
    NRF_RTC0->EVTENSET = RTC_EVTENSET_OVRFLW_Enabled << RTC_EVTENSET_OVRFLW_Pos;
            
    //NVIC->IP[TIM9_IRQn] = 0x70; // interrupt priority
    //NVIC->ISER[TIM9_IRQn>>5] = 1<<(TIM9_IRQn&0x1F);  // set enable IRQ

    // enable overflow interrupt
    NRF_RTC0->INTENSET = RTC_INTENSET_OVRFLW_Enabled << RTC_INTENSET_OVRFLW_Pos;
    
    // Enable timer counting
    NRF_RTC0->TASKS_START = 1;  
}

u4_t hal_ticks () {
    hal_disableIRQs();
    u4_t t = HAL.ticks;
    u4_t cnt = NRF_RTC0->COUNTER;
    if(NRF_RTC0->EVENTS_OVRFLW) {
        // Overflow before we read COUNTER?
        // Include overflow in evaluation but
        // leave update of state to ISR once interrupts enabled again
        cnt = NRF_RTC0->COUNTER;
        t++;
    }
    //NRF_RTC0->EVENTS_OVRFLW = 0;
    //u4_t dummy = NRF_RTC0->EVTENCLR;
    
    hal_enableIRQs();
    return (t<<24)|cnt;
}

// return modified delta ticks from now to specified ticktime (0 for past, FFFF for far future)
static u4_t deltaticks (u4_t time) {
    u4_t t = hal_ticks();
    s4_t d = time - t;
    if( d<=0 ) return 0;    // in the past
    if( (d>>24)!=0 ) return 0xFFFF; // far ahead
    return (u4_t)d;
}

void hal_waitUntil (u4_t time) {
    while( deltaticks(time) != 0 ); // busy wait until timestamp is reached
}

// check and rewind for target time
u1_t hal_checkTimer (u4_t time) {
    u4_t dt;
   // TIM9->SR &= ~TIM_SR_CC2IF; // clear any pending interrupts
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
        
    if((dt = deltaticks(time)) < 5) { // event is now (a few ticks ahead)
        NRF_RTC0->INTENCLR = RTC_INTENCLR_COMPARE0_Enabled << RTC_INTENCLR_COMPARE0_Pos; // disable interrupt
        return 1;
    } else { // rewind timer (fully or to exact time))
        NRF_RTC0->CC[0] = NRF_RTC0->COUNTER + dt;   // set comparator
        NRF_RTC0->INTENSET = RTC_INTENSET_COMPARE0_Enabled << RTC_INTENSET_COMPARE0_Pos;  // enable IE
        return 0;
    }
}
  
void RTC0_IRQHandler () {
    if(NRF_RTC0->EVENTS_OVRFLW) { // overflow
        HAL.ticks++;
    }
    if((NRF_RTC0->EVENTS_COMPARE[0]) && (NRF_RTC0->EVTENSET)) { // expired
        // do nothing, only wake up cpu
    }
    NRF_RTC0->EVENTS_OVRFLW = 0; // clear IRQ flags ) SR - status register
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    u4_t dummy = NRF_RTC0->EVTENCLR; 
}

// -----------------------------------------------------------------------------
// IRQ

void hal_disableIRQs () {
    //__disable_irq();
    NVIC_DisableIRQ(RTC0_IRQn);
    NVIC_DisableIRQ(GPIOTE_IRQn);
    HAL.irqlevel++;
}

void hal_enableIRQs () {
    if(--HAL.irqlevel == 0) {
        //__enable_irq();
        NVIC_EnableIRQ(RTC0_IRQn);
        NVIC_EnableIRQ(GPIOTE_IRQn);
    }
}

void hal_sleep () {
    // suspend execution until IRQ, regardless of the CPSR I-bit
    __WFI();
}

// -----------------------------------------------------------------------------

void hal_init () {
    memset(&HAL, 0x00, sizeof(HAL));
    hal_disableIRQs();

    // configure radio I/O and interrupt handler
    hal_io_init();
    // configure radio SPI
    hal_spi_init();
    // configure timer and interrupt handler
    hal_time_init();

    hal_enableIRQs();
}

void hal_failed () {
    debug_str("\r\n Failure!! \r\n");
    // HALT...
    hal_disableIRQs();
    hal_sleep();
    while(1);
}


