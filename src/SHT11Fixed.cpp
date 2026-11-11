/**
 * @file SHT11Fixed.cpp
 * @brief Implementation of the SHT11 driver. The API is documented in SHT11Fixed.h.
 */
#include "SHT11Fixed.h"

namespace {

// Command codes (datasheet Table 4)
const uint8_t CMD_READ_STATUS = 0x07;
const uint8_t CMD_WRITE_STATUS = 0x06;
const uint8_t CMD_SOFT_RESET = 0x1E;

// Status register bits (datasheet Table 5)
const uint8_t ST_LOW_RES = 0x01;
const uint8_t ST_NO_OTP = 0x02;
const uint8_t ST_HEATER = 0x04;
const uint8_t ST_LOW_BATTERY = 0x40;  // read-only: VDD below ~2.47 V

// Sensor start-up / soft-reset time in ms
const uint16_t STARTUP_MS = 11;

// CRC-8 polynomial x^8 + x^5 + x^4 + 1
const uint8_t CRC_POLY = 0x31;

/**
 * @brief Reverses the bit order of a byte.
 *
 * The sensor sends its CRC MSB first, while the CRC register runs LSB first.
 * @param v Input byte.
 * @return Byte with bits 7..0 mirrored.
 */
uint8_t reverse8(uint8_t v) {
  uint8_t r = 0;
  for (uint8_t i = 0; i < 8; i++) {
    r = (uint8_t)((r << 1) | (v & 1));
    v >>= 1;
  }
  return r;
}

// ---- Humidity conversion constants -------------------------------------------------
// All humidity math is done in units of 1e-4 %RH.
//
//   RH_lin  = c1 + c2 * SO + c3 * SO^2                    (Table 6)
//   RH_true = (T - 25) * (t1 + t2 * SO) + RH_lin          (Table 7)
//
// c1 = -2.0468 %RH                         -> -20468
// c2 = 0.0367 (12 bit) / 0.5872 (8 bit)    ->  367 / 5872
// c3 = 1.5955e-6 (12 bit) / 4.0845e-4 (8 bit), subtracted
//      In 1e-4 units: 0.015955 (12 bit), 4.0845 (8 bit).
//      Both map to K3 = 4.0845 * 2^13 = 33460 when the 12 bit square is pre-shifted
//      by 8 (0.015955 * 256 = 4.0845), so one constant serves both resolutions:
//      term3 = ((SO^2 >> sh) * K3) >> 13, sh = 8 (12 bit) or 0 (8 bit).
//      Largest intermediate: 65504 * 33460 = 2.19e9 < 2^32.
// t1 = 0.01 -> 100 (1e-4 %RH per degC)
// t2 = 0.00008 (12 bit) / 0.00128 (8 bit) -> 0.8 / 12.8, stored x10 to stay integer
// ---- Temperature offset d1 vs. supply voltage ----------------------------------------
// Datasheet Table 8 (0.01 degC): 2.5 V -3940, 3.0 V -3960, 3.5 V -3970, 4.0 V -3980, 5.0 V -4010
// Least-squares line: d1 = -3876 - 0.026757 * mV   (slope = 438 / 2^14)
// Residual at the table points: <= 4 (0.04 degC). Valid range 2.4 .. 5.5 V.
const int32_t D1_OFFSET = -3876;
const uint32_t D1_SLOPE = 438;  // per mV, scaled by 2^14
const uint16_t SUPPLY_MIN_MV = 2400;
const uint16_t SUPPLY_MAX_MV = 5500;

const int32_t C1 = -20468;
const int32_t C2_12 = 367;
const int32_t C2_8 = 5872;
const uint32_t K3 = 33460;
const int32_t T1_X10 = 1000;   // t1 * 1e4 * 10
const int32_t T2_12_X10 = 8;   // t2 * 1e4 * 10
const int32_t T2_8_X10 = 128;

}  // namespace

// ==================================================================================
// Construction / lifecycle
// ==================================================================================

SHT11::SHT11(uint8_t dataPin, uint8_t sckPin)
    : dataPin_(dataPin),
      sckPin_(sckPin),
      powerPin_(-1),
      status_(0),
      lastCmd_(0),
      res_(Resolution::High),
      d1_(0),
      hook_(nullptr),
      crc_(false),
      pullup_(false),
      otp_(true) {
  setSupplyMillivolts(3300);  // default, until begin() / setSupplyMillivolts() sets the real value
}

SHT11::Error SHT11::begin() {
  Config cfg;
  return begin(cfg);
}

SHT11::Error SHT11::begin(const Config& cfg) {
  res_ = cfg.resolution;
  setSupplyMillivolts(cfg.supplyMv);
  crc_ = cfg.checkCrc;
  pullup_ = cfg.internalPullup;
  otp_ = cfg.otpReload;
  powerPin_ = cfg.powerPin;

  uint8_t wanted = 0;
  if (cfg.resolution == Resolution::Low) wanted |= ST_LOW_RES;
  if (!cfg.otpReload) wanted |= ST_NO_OTP;
  if (cfg.heater) wanted |= ST_HEATER;

  // Bus idle state: SCK low (driven), DATA released
  pinMode(sckPin_, OUTPUT);
  sckLow();
  dataRelease();

  // A sensor that was just powered up by us is guaranteed to be at the status defaults
  bool justPowered = false;
  if (powerPin_ >= 0) {
    pinMode((uint8_t)powerPin_, OUTPUT);
    digitalWrite((uint8_t)powerPin_, HIGH);
    waitMs(STARTUP_MS);  // no command before 11 ms after power-up
    justPowered = true;
  }

  if (cfg.connectionReset) resetConnection();

  if (cfg.statusRetained) {
    // Trust the caller: the sensor still holds 'wanted'
    status_ = wanted;
    return Error::Ok;
  }

  // The sensor state is unknown (it may have kept an old status register), so write the
  // wanted value, also when it is 0x00. Only after our own power-up the defaults are
  // guaranteed and a write is needed for a non-default Config only.
  status_ = 0;
  if (wanted != 0 || !justPowered) {
    Error e = writeStatus(wanted);
    if (e != Error::Ok) return e;
  }
  status_ = wanted;
  return Error::Ok;
}

void SHT11::end() {
  if (powerPin_ >= 0) {
    // Cut the supply and float both lines so nothing back-feeds the sensor.
    // The DATA pull-up should sit on the switched supply rail.
    pinMode(sckPin_, INPUT);
    pinMode(dataPin_, INPUT);
    digitalWrite((uint8_t)powerPin_, LOW);
  } else {
    // Sensor stays powered: keep SCK defined (low), DATA released
    sckLow();
    dataRelease();
  }
}

// ==================================================================================
// High level API
// ==================================================================================

SHT11::Error SHT11::readTemperature(int16_t& tempCentiC) {
  uint16_t raw;
  Error e = measureRaw(Measurement::Temperature, raw);
  if (e != Error::Ok) return e;
  tempCentiC = convertTemperature(raw);
  return Error::Ok;
}

SHT11::Error SHT11::readHumidity(uint16_t& rhCenti, int16_t tempCentiC) {
  uint16_t raw;
  Error e = measureRaw(Measurement::Humidity, raw);
  if (e != Error::Ok) return e;
  rhCenti = convertHumidity(raw, tempCentiC);
  return Error::Ok;
}

SHT11::Error SHT11::read(int16_t& tempCentiC, uint16_t& rhCenti) {
  Error e = readTemperature(tempCentiC);
  if (e != Error::Ok) return e;
  return readHumidity(rhCenti, tempCentiC);
}

// ==================================================================================
// Measurement
// ==================================================================================

uint16_t SHT11::maxMeasurementMs(Measurement m) const {
  // Datasheet 3.3: max 20 / 80 / 320 ms for 8 / 12 / 14 bit
  if (m == Measurement::Temperature) {
    return (res_ == Resolution::High) ? 320 : 80;  // 14 bit : 12 bit
  }
  return (res_ == Resolution::High) ? 80 : 20;     // 12 bit : 8 bit
}

SHT11::Error SHT11::start(Measurement m) {
  lastCmd_ = (uint8_t)m;
  sendStart();
  if (!sendByte(lastCmd_)) {
    resetConnection();
    return Error::NoAck;
  }
  // The sensor releases DATA after the falling edge of the ACK clock. Make sure the pull-up
  // has brought the line high, so ready() and a DATA interrupt cannot mistake the end of
  // the ACK pulse for "conversion finished". A conversion takes >= 14 ms, so DATA is
  // never legitimately low here.
  for (uint8_t i = 0; i < 50 && !dataRead(); i++) delayMicroseconds(SHT11_CLOCK_DELAY_US);
  return Error::Ok;
}

bool SHT11::ready() const {
  // The sensor pulls DATA low when the result is available
  return !dataRead();
}

SHT11::Error SHT11::fetchRaw(uint16_t& raw) {
  if (!ready()) return Error::NotReady;

  uint8_t msb = readByte(true);
  uint8_t lsb = readByte(crc_);  // without CRC: leave ACK high -> transfer ends here

  if (crc_) {
    uint8_t rx = readByte(false);  // last byte: no ACK
    // CRC is transmitted MSB first but calculated LSB first -> compare bit-reversed
    const uint8_t data[3] = {lastCmd_, msb, lsb};
    if (reverse8(rx) != crc8(data, 3)) return Error::Crc;
  }
  raw = ((uint16_t)msb << 8) | lsb;
  return Error::Ok;
}

SHT11::Error SHT11::fetchTemperature(int16_t& tempCentiC) {
  uint16_t raw;
  Error e = fetchRaw(raw);
  if (e != Error::Ok) return e;
  tempCentiC = convertTemperature(raw);
  return Error::Ok;
}

SHT11::Error SHT11::fetchHumidity(uint16_t& rhCenti, int16_t tempCentiC) {
  uint16_t raw;
  Error e = fetchRaw(raw);
  if (e != Error::Ok) return e;
  rhCenti = convertHumidity(raw, tempCentiC);
  return Error::Ok;
}

SHT11::Error SHT11::measureRaw(Measurement m, uint16_t& raw) {
  Error e = start(m);
  if (e != Error::Ok) return e;

  // The conversion can be up to 30 % faster than the maximum. Sleep through the part
  // that is guaranteed to be needed, then poll DATA in 1 ms steps.
  uint16_t maxMs = maxMeasurementMs(m);
  uint16_t first = (uint16_t)((maxMs * 7U) / 10U);
  if (!otp_) first = (first > 10) ? (uint16_t)(first - 10) : 0;  // no OTP reload: ~10 ms faster
  if (first) waitMs(first);

  uint16_t budget = (uint16_t)(maxMs + 20U);
  while (!ready()) {
    if (budget-- == 0) {
      resetConnection();
      return Error::Timeout;
    }
    waitMs(1);
  }
  return fetchRaw(raw);
}

// ==================================================================================
// Fixed-point conversion
// ==================================================================================

void SHT11::setSupplyMillivolts(uint16_t mv) {
  if (mv < SUPPLY_MIN_MV) mv = SUPPLY_MIN_MV;
  if (mv > SUPPLY_MAX_MV) mv = SUPPLY_MAX_MV;
  // mv * 438 <= 2.41e6 fits uint32; +2^13 rounds to nearest
  d1_ = (int16_t)(D1_OFFSET - (int32_t)(((uint32_t)mv * D1_SLOPE + (1UL << 13)) >> 14));
}

int16_t SHT11::convertTemperature(uint16_t rawTemp) const {
  // T = d1 + d2 * SOT. In 0.01 degC: d2 = 0.01 (14 bit) -> x1, d2 = 0.04 (12 bit) -> x4
  if (res_ == Resolution::High) {
    return (int16_t)(d1_ + (int16_t)(rawTemp & 0x3FFF));
  }
  return (int16_t)(d1_ + (int16_t)((rawTemp & 0x0FFF) << 2));
}

uint16_t SHT11::convertHumidity(uint16_t rawRh, int16_t tempCentiC) const {
  int32_t so;
  int32_t c2;
  int32_t t2x10;
  uint32_t sq;

  if (res_ == Resolution::High) {
    so = rawRh & 0x0FFF;
    c2 = C2_12;
    t2x10 = T2_12_X10;
    sq = ((uint32_t)so * (uint32_t)so) >> 8;
  } else {
    so = rawRh & 0x00FF;
    c2 = C2_8;
    t2x10 = T2_8_X10;
    sq = (uint32_t)so * (uint32_t)so;
  }

  // Linearisation, unit 1e-4 %RH
  int32_t rh = C1 + c2 * so - (int32_t)((sq * K3) >> 13);

  // Temperature compensation: (T - 25) * (t1 + t2 * SO), unit 1e-4 %RH
  // |T-25| <= 9880 (0.01 degC), factor <= 33760 -> product < 3.4e8, fits int32
  rh += ((int32_t)(tempCentiC - kRefTempCentiC) * (T1_X10 + t2x10 * so)) / 1000;

  if (rh <= 0) return 0;
  if (rh > 990000L) return 10000;  // datasheet: > 99 %RH is saturated air -> 100 %RH
  return (uint16_t)((rh + 50) / 100);
}

// ==================================================================================
// Status, reset
// ==================================================================================

SHT11::Error SHT11::writeStatus(uint8_t value) {
  sendStart();
  if (!sendByte(CMD_WRITE_STATUS) || !sendByte(value)) {
    resetConnection();
    return Error::NoAck;
  }
  return Error::Ok;
}

SHT11::Error SHT11::readStatus(uint8_t& status) {
  sendStart();
  if (!sendByte(CMD_READ_STATUS)) {
    resetConnection();
    return Error::NoAck;
  }
  uint8_t value = readByte(true);
  uint8_t rx = readByte(false);  // CRC byte, always sent by the sensor
  if (crc_) {
    // For the status register the CRC covers command + value
    const uint8_t data[2] = {CMD_READ_STATUS, value};
    if (reverse8(rx) != crc8(data, 2)) return Error::Crc;
  }
  status = value;
  return Error::Ok;
}

SHT11::Error SHT11::setHeater(bool on) {
  uint8_t wanted = on ? (uint8_t)(status_ | ST_HEATER) : (uint8_t)(status_ & (uint8_t)~ST_HEATER);
  if (wanted == status_) return Error::Ok;  // already in the requested state: no traffic
  Error e = writeStatus(wanted);
  if (e == Error::Ok) status_ = wanted;
  return e;
}

SHT11::Error SHT11::readBatteryLow(bool& low) {
  uint8_t status;
  Error e = readStatus(status);
  if (e != Error::Ok) return e;
  low = (status & ST_LOW_BATTERY) != 0;
  return Error::Ok;
}

SHT11::Error SHT11::softReset() {
  sendStart();
  if (!sendByte(CMD_SOFT_RESET)) {
    resetConnection();
    return Error::NoAck;
  }
  // The sensor is back at its defaults: keep the cached settings in sync, otherwise
  // raw values would be converted with the wrong resolution
  status_ = 0;
  res_ = Resolution::High;
  otp_ = true;
  waitMs(STARTUP_MS);
  return Error::Ok;
}

void SHT11::resetConnection() {
  // DATA high, 9 or more SCK pulses (datasheet 3.4). The next command starts with
  // a Transmission Start anyway. The status register keeps its content.
  dataRelease();
  for (uint8_t i = 0; i < 10; i++) {
    sckHigh();
    delayMicroseconds(SHT11_CLOCK_DELAY_US);
    sckLow();
    delayMicroseconds(SHT11_CLOCK_DELAY_US);
  }
}

// ==================================================================================
// Bus primitives (open-drain emulation on DATA: only ever drive low)
// ==================================================================================

void SHT11::dataLow() {
  digitalWrite(dataPin_, LOW);  // latch LOW first -> no high glitch when switching to output
  pinMode(dataPin_, OUTPUT);
}

void SHT11::dataRelease() { pinMode(dataPin_, pullup_ ? INPUT_PULLUP : INPUT); }

bool SHT11::dataRead() const { return digitalRead(dataPin_) != 0; }

void SHT11::sckHigh() { digitalWrite(sckPin_, HIGH); }

void SHT11::sckLow() { digitalWrite(sckPin_, LOW); }

// Transmission Start (Figure 12):
//   DATA high, SCK high -> DATA low -> SCK low -> SCK high -> DATA high -> SCK low
void SHT11::sendStart() {
  dataRelease();
  sckHigh();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  dataLow();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckLow();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckHigh();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  dataRelease();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckLow();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
}

bool SHT11::sendByte(uint8_t b) {
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    if (b & mask) {
      dataRelease();
    } else {
      dataLow();
    }
    delayMicroseconds(SHT11_CLOCK_DELAY_US);  // setup
    sckHigh();
    delayMicroseconds(SHT11_CLOCK_DELAY_US);
    sckLow();
  }
  // 9th clock: the sensor pulls DATA low as ACK
  dataRelease();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckHigh();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  bool ack = !dataRead();
  sckLow();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  return ack;
}

uint8_t SHT11::readByte(bool ack) {
  uint8_t v = 0;
  dataRelease();
  for (uint8_t i = 0; i < 8; i++) {
    sckHigh();
    delayMicroseconds(SHT11_CLOCK_DELAY_US);
    v = (uint8_t)((v << 1) | (dataRead() ? 1 : 0));
    sckLow();
    delayMicroseconds(SHT11_CLOCK_DELAY_US);
  }
  if (ack) {
    dataLow();
  } else {
    dataRelease();
  }
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckHigh();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  sckLow();
  delayMicroseconds(SHT11_CLOCK_DELAY_US);
  dataRelease();
  return v;
}

void SHT11::waitMs(uint16_t ms) {
  if (hook_) {
    hook_(ms);
  } else {
    delay(ms);
  }
}

// CRC-8 over the transmitted bytes (command, data). Register preset = status register
// bits 3..0, bit-reversed (same convention as the Linux sht15 driver). With the default
// status 0x00 the preset is 0.
uint8_t SHT11::crc8(const uint8_t* data, uint8_t len) const {
  uint8_t crc = reverse8(status_ & 0x0F);
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ CRC_POLY) : (uint8_t)(crc << 1);
    }
  }
  return crc;
}
