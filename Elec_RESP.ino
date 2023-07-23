#include "dw3000.h"
 
#define APP_NAME "SS TWR RESP v1.0"
 
// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4; // spi select pin
 
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
  5,               /* Channel number. */
  DWT_PLEN_128,    /* Preamble length. Used in TX only. */
  DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
  9,               /* TX preamble code. Used in TX only. */
  9,               /* RX preamble code. Used in RX only. */
  1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
  DWT_BR_6M8,      /* Data rate. */
  DWT_PHRMODE_STD, /* PHY header mode. */
  DWT_PHRRATE_STD, /* PHY header rate. */
  (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
  DWT_STS_MODE_OFF, /* STS disabled */
  DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */
  DWT_PDOA_M0      /* PDOA mode off */
};
 
/* Default antenna delay values for 64 MHz PRF. See NOTE 2 below. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
 
/* Frames used in the ranging process. See NOTE 3 below. */
static uint8_t rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0};
static uint8_t tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'V', 'E', 'W', 'A', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* Frame sequence number, incremented after each transmission. */
static uint8_t frame_seq_nb = 0;
 
/* Buffer to store received messages.
   Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12//Must be less than FRAME_LEN_MAX_EX
static uint8_t rx_buffer[RX_BUF_LEN];
 
/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32_t status_reg = 0;
 
/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
#ifdef RPI_BUILD
#define POLL_RX_TO_RESP_TX_DLY_UUS 550
#endif //RPI_BUILD
#ifdef STM32F429xx
#define POLL_RX_TO_RESP_TX_DLY_UUS 450
#endif //STM32F429xx
#ifdef NRF52840_XXAA
#define POLL_RX_TO_RESP_TX_DLY_UUS 650
#endif //NRF52840_XXAA
 
#define POLL_RX_TO_RESP_TX_DLY_UUS 450
 
/* Timestamps of frames transmission/reception. */
static uint64_t poll_rx_ts;
static uint64_t resp_tx_ts;
 
/* Values for the PG_DELAY and TX_POWER registers reflect the bandwidth and power of the spectrum at the current
   temperature. These values can be calibrated prior to taking reference measurements. See NOTE 5 below. */
extern dwt_txconfig_t txconfig_options;
 
void setup() {
  UART_init();
  test_run_info((unsigned char *)APP_NAME);
 
  /* Configure SPI rate, DW3000 supports up to 38 MHz */
  /* Reset DW IC */
  spiBegin(PIN_IRQ, PIN_RST);
  spiSelect(PIN_SS);
 
  delay(2); // Time needed for DW3000 to start up (transition from INIT_RC to IDLE_RC, or could wait for SPIRDY event)
 
  while (!dwt_checkidlerc()) // Need to make sure DW IC is in IDLE_RC before proceeding
  {
    UART_puts("IDLE FAILED\r\n");
    while (1) ;
  }
 
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
  {
    UART_puts("INIT FAILED\r\n");
    while (1) ;
  }
 
  // Enabling LEDs here for debug so that for each TX the D1 LED will flash on DW3000 red eval-shield boards.
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
 
  /* Configure DW IC. See NOTE 6 below. */
  if (dwt_configure(&config)) // if the dwt_configure returns DWT_ERROR either the PLL or RX calibration has failed the host should reset the device
  {
    UART_puts("CONFIG FAILED\r\n");
    while (1) ;
  }
 
  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);
 
  /* Apply default antenna delay value. See NOTE 2 below. */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);
 
  /* Next can enable TX/RX states output on GPIOs 5 and 6 to help debug, and also TX/RX LEDs
     Note, in real low power applications the LEDs should not be used. */
  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
}
 
void loop() {
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
 
  /* Poll for reception of a frame or error/timeout. See NOTE 6 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_ERR)))
  { };
 
  if (status_reg & SYS_STATUS_RXFCG_BIT_MASK)
  {
    uint32_t frame_len;
 
    /* Clear good RX frame event in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
 
    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
    if (frame_len <= sizeof(rx_buffer))
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
 
      /* Check that the frame is a poll sent by "SS TWR initiator" example.
         As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
      rx_buffer[ALL_MSG_SN_IDX] = 0;
      if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
      {
        uint32_t resp_tx_time;
        int ret;
 
        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();
 
        /* Compute response message transmission time. See NOTE 7 below. */
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);
 
        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
 
        /* Write all timestamps in the final message. See NOTE 8 below. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
 
        /* Write and send the response message. See NOTE 9 below. */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        ret = dwt_starttx(DWT_START_TX_DELAYED);
 
        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 10 below. */
        if (ret == DWT_SUCCESS)
        {
          /* Poll DW IC until TX frame sent event set. See NOTE 6 below. */
          while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK))
          { };
 
          /* Clear TXFRS event. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK);
 
          /* Increment frame sequence number after transmission of the poll message (modulo 256). */
          frame_seq_nb++;
        }
      }
    }
  }
  else
  {
    /* Clear RX error events in the DW IC status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
  }
}