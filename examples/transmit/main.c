/* 
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2015 Semtech

Description: MBED LoRaWAN example application

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
//#include "mbed.h"

#include "stdbool.h"
#include "lmic.h"
#include "debug.h"

/*!
 * When set to 1 the application uses the Over-the-Air activation procedure
 * When set to 0 the application uses the Personalization activation procedure
 */
#define OVER_THE_AIR_ACTIVATION                     0

#if( OVER_THE_AIR_ACTIVATION == 0 )

/*!
 * Defines the network ID when using personalization activation procedure
 */
#define LORAWAN_NET_ID                              ( u4_t )0x00000000

/*!
 * Defines the device address when using personalization activation procedure
 */
#define LORAWAN_DEV_ADDR (u4_t) 0x015D695A

#endif

/*!
 * Defines the application data transmission duty cycle
 */
#define APP_TX_DUTYCYCLE                            5000 // 5 [s] value in ms
#define APP_TX_DUTYCYCLE_RND                        1000 // 1 [s] value in ms

/*!
 * LoRaWAN Adaptative Data Rate
 */
#define LORAWAN_ADR_ON                              1

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    1

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            15

/*!
 * User application data buffer size
 */
#if ( LORAWAN_CONFIRMED_MSG_ON == 1 )
#define LORAWAN_APP_DATA_SIZE                       6

#else
#define LORAWAN_APP_DATA_SIZE                       1

#endif

//////////////////////////////////////////////////
// CONFIGURATION (FOR APPLICATION CALLBACKS BELOW)
//////////////////////////////////////////////////

// application router ID (LSBF)
static const u1_t AppEui[8] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// unique device ID (LSBF)
static const u1_t DevEui[8] =
{
    0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
};

// device-specific AES key (derived from device EUI)
static const u1_t DevKey[16] = 
{
    0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE, 0xD2, 0xA6,
    0xAB, 0xF7, 0x15, 0x88, 0x09, 0xCF, 0x4F, 0x3C
};

#if( OVER_THE_AIR_ACTIVATION == 0 )
// network session key
static u1_t NwkSKey[] = {
    // TODO: enter network, or use TTN default
    // e.g. for 2B7E151628AED2A6ABF7158809CF4F3C =>
    0xFF, 0x69, 0xA2, 0x08, 0xE4, 0x83, 0xEC, 0xB7,
    0x9C, 0xC6, 0x0F, 0x62, 0xF6, 0xBF, 0x40, 0x77
};

//BB6AD1B37AB3E2217EB0C990C13B8BB6

// application session key
static u1_t ArtSKey[] = {
    // TODO: enter application key, or use TTN default
    // e.g. for 2B7E151628AED2A6ABF7158809CF4F3C =>
    0x01, 0x8D, 0x64, 0xD5, 0x1F, 0x67, 0x42, 0xFF,
    0x5F, 0xAB, 0xF5, 0x9E, 0x2D, 0x8E, 0x6B, 0x14
};

//BD3661D17C0EB6C2AAB613A3F8DAC7AE

#endif

// LEDs and Frame jobs
osjob_t rxLedJob;
osjob_t txLedJob;
osjob_t sendFrameJob;

// LED state
static bool AppLedStateOn = false;

//////////////////////////////////////////////////
// Utility functions
//////////////////////////////////////////////////
/*!
 * \brief Computes a random number between min and max
 *
 * \param [IN] min range minimum value
 * \param [IN] max range maximum value
 * \retval random random value in range min..max
 */
//int32_t randr( int32_t min, int32_t max )
//{
//    return ( int32_t )rand( ) % ( max - min + 1 ) + min;
//}

//////////////////////////////////////////////////
// APPLICATION CALLBACKS
//////////////////////////////////////////////////

// provide application router ID (8 bytes, LSBF)
void os_getArtEui( u1_t *buf )
{
    memcpy( buf, AppEui, 8 );
}

// provide device ID (8 bytes, LSBF)
void os_getDevEui( u1_t *buf )
{
    memcpy( buf, DevEui, 8 );
}

// provide device key (16 bytes)
void os_getDevKey( u1_t *buf )
{
    memcpy( buf, DevKey, 16 );
}

//////////////////////////////////////////////////
// MAIN - INITIALIZATION AND STARTUP
//////////////////////////////////////////////////

static void onRxLed( osjob_t* j )
{
    debug_val("LED2 = ", 0 );
}

static void onTxLed( osjob_t* j )
{
    debug_val("LED1 = ", 0 );
}

static void prepareTxFrame( void )
{
    LMIC.frame[0] = AppLedStateOn;
#if ( LORAWAN_CONFIRMED_MSG_ON == 1 )
    LMIC.frame[1] = LMIC.seqnoDn >> 8;
    LMIC.frame[2] = LMIC.seqnoDn;
    LMIC.frame[3] = LMIC.rssi >> 8;
    LMIC.frame[4] = LMIC.rssi;
    LMIC.frame[5] = LMIC.snr;
#endif    
}

void processRxFrame( void )
{
    switch( LMIC.frame[LMIC.dataBeg - 1] ) // Check Rx port number
    {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( LMIC.dataLen == 1 )
            {
                AppLedStateOn = LMIC.frame[LMIC.dataBeg] & 0x01;
                debug_val( "LED3 = ", AppLedStateOn );
            }
            break;
        default:
            break;
    }
}

static void onSendFrame( osjob_t* j )
{
    prepareTxFrame( );
    LMIC_setTxData2( LORAWAN_APP_PORT, LMIC.frame, LORAWAN_APP_DATA_SIZE, LORAWAN_CONFIRMED_MSG_ON );

    // Blink Tx LED
    debug_val( "LED1 = ", 1 );
    os_setTimedCallback( &txLedJob, os_getTime( ) + ms2osticks( 25 ), onTxLed );
}

// Initialization job
static void onInit( osjob_t* j )
{
    // reset MAC state
    LMIC_reset( );
    LMIC_setAdrMode( LORAWAN_ADR_ON );
#if defined(CFG_eu868)
    LMIC_setDrTxpow( DR_SF12, 14 );
#elif defined(CFG_us915)    
    LMIC_setDrTxpow( DR_SF10, 14 );
#endif

    // start joining
#if( OVER_THE_AIR_ACTIVATION != 0 )
    LMIC_startJoining( );
#else
    LMIC_setSession( LORAWAN_NET_ID, LORAWAN_DEV_ADDR, NwkSKey, ArtSKey );
    onSendFrame( NULL );
#endif
    // init done - onEvent( ) callback will be invoked...
}

int main( void )
{
    osjob_t initjob;

    debug_init( );
    // initialize runtime env
    os_init( );
    // setup initial job
    os_setCallback( &initjob, onInit );
    // execute scheduled jobs and events
    os_runloop( );
    // (not reached)
}

//////////////////////////////////////////////////
// LMIC EVENT CALLBACK
//////////////////////////////////////////////////
void onEvent( ev_t ev )
{
    bool txOn = false;
    debug_event( ev );

    switch( ev ) 
    {
    // network joined, session established
    case EV_JOINED:
        debug_val( "Net ID = ", LMIC.netid );
        txOn = true;
        break;
    // scheduled data sent (optionally data received)
    case EV_TXCOMPLETE:
        debug_val( "Datarate = ", LMIC.datarate );
        // Check if we have a downlink on either Rx1 or Rx2 windows
        if( ( LMIC.txrxFlags & ( TXRX_DNW1 | TXRX_DNW2 ) ) != 0 )
        {
            debug_val( "LED2 = ", 1 );
            os_setTimedCallback( &rxLedJob, os_getTime( ) + ms2osticks( 25 ), onRxLed );

            if( LMIC.dataLen != 0 )
            { // data received in rx slot after tx
                debug_buf( LMIC.frame + LMIC.dataBeg, LMIC.dataLen );
                processRxFrame( );
            }
        }
        txOn = true;
        break;
    default:
        break;
    }
    if( txOn == true )
    {
        //Sends frame every APP_TX_DUTYCYCLE +/- APP_TX_DUTYCYCLE_RND random time (if not duty cycle limited)
        os_setTimedCallback( &sendFrameJob,
                             os_getTime( ) + ms2osticks( APP_TX_DUTYCYCLE ),
                             onSendFrame );
        
        ////Sends frame as soon as possible (duty cylce limitations)
        //onSendFrame( NULL );
    }
}
