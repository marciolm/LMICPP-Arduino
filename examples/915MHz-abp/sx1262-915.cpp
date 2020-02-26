#include <Arduino.h>
#include <SPI.h>

#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#define DEVICE_SIMPLE
#include "lorakeys.h"

void do_send();

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr OsDeltaTime TX_INTERVAL = OsDeltaTime::from_sec(65);

constexpr unsigned int BAUDRATE = 115200;
// Pin mapping
/*
constexpr lmic_pinmap lmic_pins = {
    .nss = 5,
    .prepare_antenna_tx = nullptr,
    .rst = 4,
    .dio = {22, 21},  //busy   dio1
};
*/
constexpr lmic_pinmap lmic_pins = {
    .nss = 16,
    .prepare_antenna_tx = nullptr,
    .rst = 2,
    .dio = {/* busy */ 14, /* DIO1 */ 15},
};
OsScheduler OSS;
// Radio class for SX1262
RadioSx1262 radio{lmic_pins, ImageCalibrationBand::band_902_928 };
LmicUs915 LMIC{radio, OSS};


OsJob sendjob{OSS};
void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because if will be mobile.
    LMIC.setLinkCheckMode(false);
    LMIC.selectSubBand(1);
    LMIC.setDrTx(0);
    break;
  case EventType::JOIN_FAILED:
    PRINT_DEBUG(2, F("EV_JOIN_FAILED"));
    break;
  case EventType::TXCOMPLETE:
    PRINT_DEBUG(1, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
    }
    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      int messagesize=LMIC.getDataLen();
      PRINT_DEBUG(1, F("Received %d bytes of payload"),messagesize);
      auto data = LMIC.getData();
      for (int chars=0;chars<messagesize;chars++){
        PRINT_DEBUG(1, F("Data:%x"), data[chars]);
      }
    }
    // we have transmit
    // Schedule next transmission
    sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);

    break;
  case EventType::RESET:
    PRINT_DEBUG(2, F("EV_RESET"));
    break;
  case EventType::LINK_DEAD:
    PRINT_DEBUG(2, F("EV_LINK_DEAD"));
    break;
  case EventType::LINK_ALIVE:
    PRINT_DEBUG(2, F("EV_LINK_ALIVE"));
    break;
  default:
    PRINT_DEBUG(2, F("Unknown event"));
    break;
  }
}

void do_send() {
  // Check if there is not a current TX/RX job running
  if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
    PRINT_DEBUG(1, F("OpState::TXRXPEND, not sending"));
    // should not happen so reschedule anyway
    sendjob.setTimedCallback(os_getTime() + TX_INTERVAL, do_send);
  } else {
    // Some analog value
    // val = analogRead(A1) >> 4;
    uint8_t val[8];
     val[0] = 0x03;
     val[1] = 0x67;
     val[2] = 0x01;
     val[3] = 0x10;
     val[4] = 0x05;
     val[5] = 0x67;
     val[6] = 0x00;
     val[7] = 0xFF;

    // Prepare upstream data transmission at the next possible time.
    LMIC.setTxData2(2, (uint8_t*)val, sizeof(val), false);
    PRINT_DEBUG(1, F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }

  SPI.begin(12,4,13,10);
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  // RadioSx1262 set_DIO2_as_rf_switch_ctrl();
  //LMIC.setDrTx(5); // DR5 is BW=125 SF=7
  LMIC.setDrTx(0); //set power to max

  LMIC.reset();
  /*
  for (int channel=0; channel<8; ++channel) {
     LMIC.disableChannel(channel);
   }
    for (int channel=15; channel<72; ++channel) {
     LMIC.disableChannel(channel);
  }
  */

  LMIC.selectSubBand(1);
  LMIC.setDrTx(0); //set power to max
  LMIC.setRx2Parameter(923300000, 8);
  LMIC.setEventCallBack(onEvent);
 // SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
// ABP Set session information
  // Change to your device info
  const uint32_t TTN_NET_ID = 0x000013;
  const uint32_t DEV_ADRESS = 0x00000000; //inser the dev address
  const uint8_t NET_KEY[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  const uint8_t APP_KEY[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  AesKey appkey;
  std::copy(APP_KEY, APP_KEY + 16, appkey.begin());
  AesKey netkey;
  std::copy(NET_KEY, NET_KEY + 16, netkey.begin());
  LMIC.setSession(TTN_NET_ID, DEV_ADRESS, netkey, appkey);
  // set clock error to allow good connection.

  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);

  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 3 / 100);
  //LMIC.setAntennaPowerAdjustment(-14);
 
  // Start job (sending automatically starts OTAA too)
  sendjob.setCallbackRunnable(do_send);
}

void loop() {
  OsDeltaTime to_wait = OSS.runloopOnce();
  if (to_wait > OsDeltaTime(0)) {
    // if we have nothing to do just wait a little.
    delay(to_wait.to_ms() / 2);
  }
}
