/*******************************************************************************
 * Copyright (c) 2014-2015 IBM Corporation.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 * Contributors:
 *    IBM Zurich Research Lab - initial API, implementation and documentation
 *    Nicolas Graziano - cpp style.
 *******************************************************************************/

//! \file
#include "../hal/print_debug.h"

#include "bufferpack.h"
#include "lmic.eu868.h"
#include "lmic_table.h"
#include <algorithm>

// Default frequency plan for EU 868MHz ISM band
// Bands:
//  g1 :   1%  14dBm
//  g2 : 0.1%  14dBm
//  g3 :  10%  27dBm
//                 freq             band     datarates
enum {
  EU868_F1 = 868100000, // g1   SF7-12
  EU868_F2 = 868300000, // g1   SF7-12 FSK SF7/250
  EU868_F3 = 868500000, // g1   SF7-12
  EU868_F4 = 868850000, // g2   SF7-12
  EU868_F5 = 869050000, // g2   SF7-12
  EU868_F6 = 869525000, // g3   SF7-12
};

namespace {
constexpr uint32_t EU868_FREQ_MIN = 863000000;
constexpr uint32_t EU868_FREQ_MAX = 870000000;
constexpr uint32_t FREQ_DNW2 = EU868_F6;
constexpr LmicEu868::Dr DR_DNW2 = LmicEu868::Dr::SF12;

constexpr OsDeltaTime DNW2_SAFETY_ZONE = OsDeltaTime::from_ms(3000);

constexpr uint8_t rps_DR0 =
    rps_t{SF12, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR1 =
    rps_t{SF11, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR2 =
    rps_t{SF10, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR3 =
    rps_t{SF9, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR4 =
    rps_t{SF8, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR5 =
    rps_t{SF7, BandWidth::BW125, CodingRate::CR_4_5, false}.rawValue();
constexpr uint8_t rps_DR6 =
    rps_t{SF7, BandWidth::BW250, CodingRate::CR_4_5, false}.rawValue();

CONST_TABLE(uint8_t, _DR2RPS_CRC)
[] = {ILLEGAL_RPS, rps_DR0, rps_DR1, rps_DR2,    rps_DR3,
      rps_DR4,     rps_DR5, rps_DR6, ILLEGAL_RPS};

constexpr int8_t MaxEIRP = 16;

// Table below defines the size of one symbol as
//   symtime = 256us * 2^T(sf,bw)
// 256us is called one symunit.
//                 SF:
//      BW:      |__7___8___9__10__11__12
//      125kHz   |  2   3   4   5   6   7
//      250kHz   |  1   2   3   4   5   6
//      500kHz   |  0   1   2   3   4   5
//
// Times for half symbol per DR
// Per DR table to minimize rounding errors
CONST_TABLE(int32_t, DR2HSYM)
[] = {
    OsDeltaTime::from_us_round(128 << 7).tick(), // DR_SF12
    OsDeltaTime::from_us_round(128 << 6).tick(), // DR_SF11
    OsDeltaTime::from_us_round(128 << 5).tick(), // DR_SF10
    OsDeltaTime::from_us_round(128 << 4).tick(), // DR_SF9
    OsDeltaTime::from_us_round(128 << 3).tick(), // DR_SF8
    OsDeltaTime::from_us_round(128 << 2).tick(), // DR_SF7
    OsDeltaTime::from_us_round(128 << 1).tick(), // DR_SF7B
    OsDeltaTime::from_us_round(80).tick() // FSK -- not used (time for 1/2 byte)
};

} // namespace

uint8_t LmicEu868::getRawRps(dr_t const dr) const {
  return TABLE_GET_U1(_DR2RPS_CRC, dr + 1);
}

int8_t LmicEu868::pow2dBm(uint8_t const powerIndex) const {
  if (powerIndex >= 8) {
    return InvalidPower;
  }

  return MaxEIRP - 2 * powerIndex;
}

OsDeltaTime LmicEu868::getDwn2SafetyZone() const { return DNW2_SAFETY_ZONE; }

OsDeltaTime LmicEu868::dr2hsym(dr_t const dr) const {
  return OsDeltaTime(TABLE_GET_S4(DR2HSYM, dr));
}

bool LmicEu868::validRx1DrOffset(uint8_t const drOffset) const {
  return drOffset < 6;
}

void LmicEu868::initDefaultChannels() {
  PRINT_DEBUG(2, F("Init Default Channel"));

  channels.disableAll();
  channels.init();
  setupChannel(0, EU868_F1, 0);
  setupChannel(1, EU868_F2, 0);
  setupChannel(2, EU868_F3, 0);
}

bool LmicEu868::setupChannel(uint8_t const chidx, uint32_t const newfreq,
                             uint16_t const drmap) {
  if (chidx >= MAX_CHANNELS)
    return false;

  channels.configure(chidx, newfreq,
                     drmap == 0 ? dr_range_map(Dr::SF12, Dr::SF7) : drmap);
  return true;
}

void LmicEu868::disableChannel(uint8_t const channel) {
  channels.disable(channel);
}

uint32_t LmicEu868::convFreq(const uint8_t *ptr) const {
  uint32_t newfreq = rlsbf3(ptr) * 100;
  if (newfreq < EU868_FREQ_MIN || newfreq > EU868_FREQ_MAX)
    newfreq = 0;
  return newfreq;
}

void LmicEu868::handleCFList(const uint8_t *ptr) {

  for (uint8_t chidx = 3; chidx < 8; chidx++, ptr += 3) {
    uint32_t newfreq = convFreq(ptr);
    if (newfreq != 0) {
      setupChannel(chidx, newfreq, 0);

      PRINT_DEBUG(2, F("Setup channel, idx=%d, freq=%" PRIu32 ""), chidx,
                  newfreq);
    }
  }
}

bool LmicEu868::validMapChannels(uint8_t const chMaskCntl,
                                 uint16_t const chMask) {
  // Bad page
  if (chMaskCntl != 0 && chMaskCntl != 6)
    return false;

  //  disable all channel
  if (chMaskCntl == 0 && chMask == 0)
    return false;

  return true;
}

void LmicEu868::mapChannels(uint8_t const chMaskCntl, uint16_t const chMask) {
  // LoRaWAN™ 1.0.2 Regional Parameters §2.1.5
  // ChMaskCntl=6 => All channels ON
  if (chMaskCntl == 6) {
    channels.enableAll();
    return;
  }

  for (uint8_t chnl = 0; chnl < MAX_CHANNELS; chnl++) {
    if ((chMask & (1 << chnl)) != 0) {
      channels.enable(chnl);
    } else {
      channels.disable(chnl);
    }
  }
}

uint32_t LmicEu868::getTxFrequency() const {
  return channels.getFrequency(txChnl);
}

int8_t LmicEu868::getTxPower() const {
  // limit power to value ask in adr (at init MaxEIRP)
  return adrTxPow;
};

void LmicEu868::updateTxTimes(OsDeltaTime const airtime) {
  channels.updateAvailabitility(txChnl, os_getTime(), airtime);

  PRINT_DEBUG(
      2, F("Updating info for TX channel %d, airtime will be %" PRIu32 "."),
      txChnl, airtime);
}

OsTime LmicEu868::nextTx(OsTime const now) {

  bool channelFound = false;
  OsTime nextTransmitTime;
  // next channel or other (random)
  uint8_t nextChannel = txChnl + 1 + (rand.uint8() % 2);

  for (uint8_t channelIndex = 0; channelIndex < MAX_CHANNELS; channelIndex++) {
    if (nextChannel >= MAX_CHANNELS) {
      nextChannel = 0;
    }

    if (channels.is_enable_at_dr(nextChannel, datarate)) {
      auto availability = channels.getAvailability(nextChannel);

      PRINT_DEBUG(2, F("Considering channel %d"), nextChannel);

      if (!channelFound || availability < nextTransmitTime) {
        txChnl = nextChannel;
        nextTransmitTime = availability;
        channelFound = true;
      }
      if (availability < now) {
        // no need to search better
        txChnl = nextChannel;
        return availability;
      }
    }
    nextChannel++;
  }

  if (channelFound) {
    return nextTransmitTime;
  }

  // Fail to find a channel continue on current one.
  // UGLY FAILBACK
  PRINT_DEBUG(1, F("Error Fail to find a channel."));
  return now;
}

uint32_t LmicEu868::getRx1Frequency() const {
  // RX1 frequency is same as TX frequency
  return getTxFrequency();
}

dr_t LmicEu868::getRx1Dr() const { return lowerDR(datarate, rx1DrOffset); }

FrequencyAndRate LmicEu868::getRx1Parameter() const {
  return {getRx1Frequency(), getRx1Dr()};
}

void LmicEu868::initJoinLoop() {
  txChnl = rand.uint8() % 3;
  adrTxPow = MaxEIRP;
  setDrJoin(static_cast<dr_t>(Dr::SF7));
  txend = channels.getAvailability(0) + OsDeltaTime::rnd_delay(rand, 8);
  PRINT_DEBUG(1, F("Init Join loop : avail=%" PRIu32 " txend=%" PRIu32 ""),
              channels.getAvailability(0).tick(), txend.tick());
}

bool LmicEu868::nextJoinState() {
  bool failed = false;

  // Try the tree default channels with same DR
  // If both fail try next lower datarate
  if (++txChnl == 3)
    txChnl = 0;
  if ((++txCnt & 1) == 0) {
    // Lower DR every 2nd try (having tried 868.x and 864.x with the same DR)
    if (datarate == static_cast<dr_t>(Dr::SF12)) {
      // we have tried all DR - signal EV_JOIN_FAILED
      failed = true;
      // and retry from highest datarate.
      datarate = static_cast<dr_t>(Dr::SF7);
    }
    else
      datarate = decDR(datarate);
  }

  // Set minimal next join time
  auto time = os_getTime();
  auto availability = channels.getAvailability(txChnl);
  if (time < availability)
    time = availability;

  txend = time;

  if (failed)
    PRINT_DEBUG(2, F("Join failed"));
  else
    PRINT_DEBUG(2, F("Scheduling next join at %" PRIu32 ""), txend);

  // 1 - triggers EV_JOIN_FAILED event
  return !failed;
}

FrequencyAndRate LmicEu868::defaultRX2Parameter() const {
  return {FREQ_DNW2, static_cast<dr_t>(DR_DNW2)};
}

#if defined(ENABLE_SAVE_RESTORE)
void LmicEu868::saveStateWithoutTimeData(StoringAbtract &store) const {
  Lmic::saveStateWithoutTimeData(store);

  channels.saveStateWithoutTimeData(store);
  store.write(txChnl);
}

void LmicEu868::saveState(StoringAbtract &store) const {
  Lmic::saveState(store);
  channels.saveState(store);
  store.write(txChnl);
}

void LmicEu868::loadStateWithoutTimeData(RetrieveAbtract &store) {
  Lmic::loadStateWithoutTimeData(store);

  channels.loadStateWithoutTimeData(store);
  store.read(txChnl);
}

void LmicEu868::loadState(RetrieveAbtract &store) {
  Lmic::loadState(store);

  channels.loadState(store);
  store.read(txChnl);
}
#endif

LmicEu868::LmicEu868(Radio &aradio, OsScheduler &ascheduler)
    : Lmic(aradio, ascheduler) {}