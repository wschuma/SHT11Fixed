/**
 * @file SHT11Fixed.h
 * @brief Fixed-point driver for the Sensirion SHT1x (SHT10/SHT11/SHT15) sensors.
 *
 * Target: Arduino UNO (AVR) and ESP32, battery-powered nodes with deep sleep.
 *
 * Design goals:
 *  - No floating point at runtime (all conversions use 16/32-bit integers).
 *  - Minimal sensor communication, configurable through SHT11::Config.
 *  - Optional non-blocking measurement so the MCU can sleep while the sensor works.
 *  - Optional power switching of the sensor via a GPIO.
 *
 * Units used by the whole API:
 *  - Temperature: `int16_t` in 0.01 degC   (2345 = 23.45 degC)
 *  - Humidity:    `uint16_t` in 0.01 %RH   (5012 = 50.12 %RH)
 *  - Voltage:     `uint16_t` in mV
 *
 * Datasheet: SHT1x (SHT10, SHT11, SHT15), Version 5, December 2011.
 */
#ifndef SHT11_FIXED_H
#define SHT11_FIXED_H

#include <Arduino.h>
#include <stdint.h>

/**
 * @def SHT11_CLOCK_DELAY_US
 * @brief Half-period of the software SCK in microseconds.
 *
 * 2 us gives at most ~250 kHz, well below the 1 MHz limit for VDD < 4.5 V.
 * Increase it for long wires or weak pull-ups, override with
 * `-DSHT11_CLOCK_DELAY_US=n` in the build flags.
 */
#ifndef SHT11_CLOCK_DELAY_US
#define SHT11_CLOCK_DELAY_US 2
#endif

/**
 * @brief Driver for one SHT1x sensor on two GPIO pins (DATA and SCK).
 *
 * Typical battery-node cycle:
 * @code
 * SHT11 sensor(2, 3);
 * SHT11::Config cfg;
 * cfg.supplyMv = readBatteryMv();
 * sensor.begin(cfg);
 * int16_t t; uint16_t rh;
 * sensor.read(t, rh);
 * sensor.end();
 * @endcode
 */
class SHT11 {
 public:
  /** @brief Measurement resolution (status register bit 0). */
  enum class Resolution : uint8_t {
    High = 0,  ///< 14 bit temperature / 12 bit humidity (power-up default)
    Low = 1    ///< 12 bit temperature / 8 bit humidity (faster, less energy)
  };

  /** @brief Measurement command codes (datasheet Table 4). */
  enum class Measurement : uint8_t {
    Temperature = 0x03,  ///< measure temperature
    Humidity = 0x05      ///< measure relative humidity
  };

  /** @brief Result codes of all operations that talk to the sensor. */
  enum class Error : int8_t {
    Ok = 0,        ///< success
    NoAck = -1,    ///< sensor did not acknowledge a byte (wiring? power? start-up time?)
    Timeout = -2,  ///< measurement never signalled "data ready"
    Crc = -3,      ///< CRC mismatch (only checked if Config::checkCrc is set)
    NotReady = -4  ///< fetchTemperature() / fetchHumidity() called before the measurement has finished
  };

  /**
   * @brief Options for begin().
   *
   * With the default Config, begin() writes the (default) status register once unless the
   * sensor was just powered up by the library through Config::powerPin, because then the
   * sensor is known to be at its defaults. Config::statusRetained skips the write entirely.
   */
  struct Config {
    /** Measurement resolution. Low shortens the conversion time to 80 / 20 ms (T / RH). */
    Resolution resolution = Resolution::High;

    /**
     * Sensor supply voltage in mV. Selects the temperature offset d1 (datasheet Table 8)
     * through a linear approximation, valid for 2400..5500 mV (values are clamped).
     * For a battery node measure VDD before each cycle and pass it here, or call
     * setSupplyMillivolts() later. Maximum error of the approximation: 0.04 degC.
     */
    uint16_t supplyMv = 3300;

    /**
     * `false`: do not reload the calibration data from OTP before each measurement.
     * Saves about 10 ms of active time per measurement at a small cost in accuracy.
     */
    bool otpReload = true;

    /** Enable the on-chip heater (functional check only, not for continuous use). */
    bool heater = false;

    /** Read and verify the CRC-8 byte (one extra byte per measurement). */
    bool checkCrc = false;

    /**
     * Use the MCU pull-up on DATA. The datasheet requires an external pull-up
     * (e.g. 10 kOhm); the internal one is only a fallback for short wires.
     */
    bool internalPullup = false;

    /**
     * `false` (default): the state of the sensor is unknown, so begin() writes the status
     * register to make it match this Config, also if the Config only uses defaults.
     * The only exception is a sensor that begin() has just powered up through
     * Config::powerPin: it starts at the defaults, so a write is only needed for a
     * non-default Config.
     *
     * `true`: begin() does not write the status register. Use this when the sensor stayed
     * powered since the last begin() with the same Config (e.g. only the ESP32 goes to
     * deep sleep and the sensor is permanently powered).
     */
    bool statusRetained = false;

    /**
     * Send the "connection reset" sequence (10 SCK pulses) in begin(). Only useful after
     * an MCU reset in the middle of a transfer. Failed commands trigger it automatically.
     */
    bool connectionReset = false;

    /**
     * GPIO that switches the sensor supply, -1 if the sensor is always powered.
     * begin() powers up and waits 11 ms, end() powers down and tri-states the bus.
     */
    int8_t powerPin = -1;
  };

  /**
   * @brief Callback used for every wait of 1 ms or more.
   *
   * Plug a sleep function in here (e.g. a watchdog sleep on AVR, light sleep on ESP32).
   * Without a hook the library uses `delay()`.
   * @param ms Time to wait in milliseconds.
   */
  typedef void (*DelayHook)(uint16_t ms);

  /** @brief Reference temperature (25 degC, in 0.01 degC) at which no humidity compensation is applied. */
  static const int16_t kRefTempCentiC = 2500;

  /**
   * @brief Creates a driver instance. Does not touch the pins; call begin() for that.
   * @param dataPin GPIO connected to DATA (needs a 10 kOhm pull-up to VDD).
   * @param sckPin  GPIO connected to SCK.
   */
  SHT11(uint8_t dataPin, uint8_t sckPin);

  /** @name Lifecycle
   *  @{ */

  /**
   * @brief Initializes the pins and the sensor with the default Config.
   * @return Error::Ok, or Error::NoAck if the status write was not acknowledged.
   * @see begin(const Config&)
   */
  Error begin();

  /**
   * @brief Initializes the pins and the sensor.
   *
   * Sets up the bus pins, powers the sensor through Config::powerPin (if set) and waits
   * the 11 ms start-up time, optionally resets the connection and writes the status
   * register once to make the sensor match @p cfg. The write is skipped if
   * Config::statusRetained is set, or if the sensor was just powered up by this call and
   * @p cfg only uses defaults. Call end() after each cycle when using Config::powerPin.
   * @param cfg Options, see Config.
   * @return Error::Ok, or Error::NoAck if the status write was not acknowledged.
   */
  Error begin(const Config& cfg);

  /**
   * @brief Ends a measurement cycle.
   *
   * With Config::powerPin the sensor supply is switched off and both bus lines are
   * tri-stated so nothing back-feeds the sensor. Without a power pin SCK is kept low and
   * DATA released.
   */
  void end();

  /**
   * @brief Installs a sleep function for all waits of 1 ms or more.
   * @param hook Callback, or `nullptr` to use `delay()` again.
   */
  void setDelayHook(DelayHook hook) { hook_ = hook; }

  /**
   * @brief Sets the supply voltage used for the temperature conversion.
   *
   * The temperature offset d1 of the sensor depends on VDD (datasheet Table 8). It is
   * calculated with a linear least-squares fit of that table, in 0.01 degC:
   * `d1 = -3876 - mV * 438 / 2^14` (maximum deviation from the table: 0.04 degC).
   * Call it once per wake-up after measuring the battery, or set Config::supplyMv in
   * begin(). Pure integer math, no bus traffic.
   * @param mv Supply voltage in mV (clamped to 2400..5500).
   */
  void setSupplyMillivolts(uint16_t mv);

  /** @} */

  /** @name High-level readings (blocking)
   *  @{ */

  /**
   * @brief Measures and converts the temperature.
   * @param[out] tempCentiC Temperature in 0.01 degC; unchanged on error.
   * @return Error::Ok or the error of the failed step.
   */
  Error readTemperature(int16_t& tempCentiC);

  /**
   * @brief Measures and converts the relative humidity.
   *
   * With the default @p tempCentiC (25 degC) no temperature compensation is applied, so
   * no temperature measurement is required. Pass a known temperature to keep the
   * compensation without measuring it again.
   * @param[out] rhCenti    Relative humidity in 0.01 %RH, 0..10000; unchanged on error.
   * @param[in]  tempCentiC Temperature in 0.01 degC used for the compensation.
   * @return Error::Ok or the error of the failed step.
   */
  Error readHumidity(uint16_t& rhCenti, int16_t tempCentiC = kRefTempCentiC);

  /**
   * @brief Measures the temperature, then the temperature-compensated humidity.
   * @param[out] tempCentiC Temperature in 0.01 degC.
   * @param[out] rhCenti    Relative humidity in 0.01 %RH.
   * @return Error::Ok or the error of the failed step.
   */
  Error read(int16_t& tempCentiC, uint16_t& rhCenti);

  /** @} */

  /** @name Non-blocking measurement
   *  @{ */

  /**
   * @brief Sends a measurement command and returns immediately.
   *
   * The MCU may sleep until ready() reports true, then call fetchTemperature() or
   * fetchHumidity(). The conversion takes at most 320 ms (14 bit temperature), 80 ms (12 bit
   * temperature or 12 bit humidity) or 20 ms (8 bit humidity). When start() returns, the
   * acknowledge pulse of the command is over, so DATA is high and ready() is false until
   * the conversion has finished.
   * @param m Quantity to measure.
   * @return Error::Ok, or Error::NoAck (the connection is reset automatically).
   */
  Error start(Measurement m);

  /**
   * @brief Tells whether a started measurement has finished.
   *
   * The sensor pulls DATA low when the result is available. Only meaningful between
   * start() and fetchTemperature() / fetchHumidity().
   * @return `true` if the result can be fetched.
   */
  bool ready() const;

  /**
   * @brief Fetches and converts the result of a finished temperature measurement.
   *
   * Counterpart of start(Measurement::Temperature) for the non-blocking API. Reads the
   * result and converts it with the supply voltage set through Config::supplyMv /
   * setSupplyMillivolts().
   * @param[out] tempCentiC Temperature in 0.01 degC; unchanged on error.
   * @return Error::Ok, Error::NotReady if ready() is still false, Error::Crc on a CRC mismatch.
   * @warning Call it only after start(Measurement::Temperature); the driver does not check
   *          which quantity was started.
   */
  Error fetchTemperature(int16_t& tempCentiC);

  /**
   * @brief Fetches and converts the result of a finished humidity measurement.
   *
   * Counterpart of start(Measurement::Humidity) for the non-blocking API. Reads the result
   * and converts it. With the default @p tempCentiC (25 degC) no temperature compensation
   * is applied; pass the temperature from a preceding fetchTemperature() to keep it.
   * @param[out] rhCenti    Relative humidity in 0.01 %RH, 0..10000; unchanged on error.
   * @param[in]  tempCentiC Temperature in 0.01 degC used for the compensation.
   * @return Error::Ok, Error::NotReady if ready() is still false, Error::Crc on a CRC mismatch.
   * @warning Call it only after start(Measurement::Humidity); the driver does not check
   *          which quantity was started.
   */
  Error fetchHumidity(uint16_t& rhCenti, int16_t tempCentiC = kRefTempCentiC);

  /** @} */

  /** @name Miscellaneous
   *  @{ */

  /**
   * @brief Switches the on-chip heater on or off.
   *
   * Rewrites the status register with the heater bit changed and all other settings kept.
   * Nothing is sent if the heater is already in the requested state (according to the
   * driver's cached copy of the status register). The heater raises the sensor temperature
   * by a few degrees and adds noticeable current consumption; it is meant for functional
   * checks (the humidity reading must drop while the temperature rises) and must be
   * switched off again before real measurements. A power cycle or softReset() also
   * switches it off. See the datasheet chapter on the heater for details.
   * @param on `true` to enable, `false` to disable the heater.
   * @return Error::Ok, or Error::NoAck (the connection is reset automatically and the
   *         cached state stays unchanged).
   */
  Error setHeater(bool on);

  /**
   * @brief Reads the end-of-battery flag of the sensor.
   *
   * The sensor sets the flag when its supply voltage is below about 2.47 V (accuracy about
   * +-0.05 V). The flag is updated after each measurement, so call this after a
   * measurement and, when the sensor supply is switched, before end(). It costs one status
   * read (command, status byte and CRC byte). For a finer battery check measure the supply
   * voltage yourself, see Config::supplyMv.
   * @param[out] low `true` if VDD is below the threshold; unchanged on error.
   * @return Error::Ok, Error::NoAck or Error::Crc.
   * @see readStatus()
   */
  Error readBatteryLow(bool& low);

  /**
   * @brief Soft reset: the sensor returns to its power-up defaults.
   *
   * Clears the status register (14/12 bit, OTP reload, heater off) and waits 11 ms. The
   * driver's cached settings are reset accordingly; call begin() again to re-apply a
   * non-default Config.
   * @return Error::Ok or Error::NoAck.
   */
  Error softReset();

  /** @} */

 private:
  /** @name Measurement helpers
   *  @{ */

  /**
   * @brief Reads the result of a finished measurement.
   *
   * Reads the two data bytes and, if Config::checkCrc is set, the CRC byte. The sensor
   * returns to sleep afterwards.
   * @param[out] raw Raw sensor value (right-justified, 14/12 bit or 12/8 bit).
   * @return Error::Ok, Error::NotReady if ready() is still false, Error::Crc on a CRC mismatch.
   */
  Error fetchRaw(uint16_t& raw);

  /**
   * @brief Blocking measurement returning the raw sensor value.
   *
   * Calls start(), waits (through the delay hook, if installed) for the conversion
   * and calls fetchRaw(). The first part of the wait is a single sleep of the guaranteed
   * minimum conversion time, the rest is polled in 1 ms steps.
   * @param[in]  m   Quantity to measure.
   * @param[out] raw Raw sensor value.
   * @return Error::Ok, Error::NoAck, Error::Timeout or Error::Crc.
   */
  Error measureRaw(Measurement m, uint16_t& raw);

  /**
   * @brief Worst-case conversion time for the current resolution.
   * @param m Quantity to measure.
   * @return Maximum conversion time in ms (20 / 80 / 320 ms for 8 / 12 / 14 bit).
   */
  uint16_t maxMeasurementMs(Measurement m) const;

  /**
   * @brief Converts a raw temperature value using the stored supply voltage.
   * @param rawTemp Raw value from fetchRaw() / measureRaw().
   * @return Temperature in 0.01 degC.
   * @see setSupplyMillivolts()
   */
  int16_t convertTemperature(uint16_t rawTemp) const;

  /**
   * @brief Converts a raw humidity value (linearization and temperature compensation).
   *
   * Values above 99 %RH are reported as 100 %RH (saturated air, datasheet 4.1); negative
   * results are clamped to 0.
   * @param rawRh      Raw value from fetchRaw() / measureRaw().
   * @param tempCentiC Temperature in 0.01 degC for the compensation, default 25 degC (none).
   * @return Relative humidity in 0.01 %RH, 0..10000.
   */
  uint16_t convertHumidity(uint16_t rawRh, int16_t tempCentiC = kRefTempCentiC) const;

  /**
   * @brief Reads the status register.
   *
   * Bit 6 is the end-of-battery flag (VDD < 2.47 V), updated after each measurement.
   * @param[out] status Status register content.
   * @return Error::Ok, Error::NoAck or Error::Crc.
   */
  Error readStatus(uint8_t& status);

  /**
   * @brief Resets the serial interface of the sensor (10 SCK pulses with DATA high).
   *
   * The status register keeps its content. Called automatically after a failed command
   * and, if Config::connectionReset is set, in begin().
   */
  void resetConnection();

  /** @} */

  /** @name Bus primitives
   *  @{ */

  /** @brief Drives DATA low (latches LOW first to avoid a high glitch). DATA is only ever driven low. */
  void dataLow();

  /** @brief Releases DATA (input, with the internal pull-up if Config::internalPullup is set). */
  void dataRelease();

  /** @brief Reads the DATA line. @return `true` if DATA is high. */
  bool dataRead() const;

  /** @brief Drives SCK high. */
  void sckHigh();

  /** @brief Drives SCK low. */
  void sckLow();

  /** @brief Sends the "Transmission Start" sequence that precedes every command (datasheet Fig. 12). */
  void sendStart();

  /**
   * @brief Sends one byte MSB first and clocks the acknowledge bit.
   * @param b Byte to send.
   * @return `true` if the sensor acknowledged (pulled DATA low).
   */
  bool sendByte(uint8_t b);

  /**
   * @brief Reads one byte MSB first and clocks the acknowledge bit.
   * @param ack `true` to acknowledge (more bytes follow), `false` to end the transfer.
   * @return The received byte.
   */
  uint8_t readByte(bool ack);

  /**
   * @brief Writes the status register.
   * @param value New status register content (bits 0..2 are writable).
   * @return Error::Ok, or Error::NoAck (the connection is reset automatically).
   */
  Error writeStatus(uint8_t value);

  /**
   * @brief Waits through the delay hook, or `delay()` if none is installed.
   * @param ms Time to wait in milliseconds.
   */
  void waitMs(uint16_t ms);

  /**
   * @brief CRC-8 over transmitted bytes (polynomial 0x31).
   *
   * The register preset is status register bits 3..0, bit-reversed; with the default
   * status 0x00 it is 0.
   * @param data Bytes to include (command, then data bytes).
   * @param len  Number of bytes.
   * @return CRC as calculated; the received CRC byte must be bit-reversed before comparing.
   */
  uint8_t crc8(const uint8_t* data, uint8_t len) const;

  /** @} */

  uint8_t dataPin_;     ///< GPIO of the DATA line
  uint8_t sckPin_;      ///< GPIO of the SCK line
  int8_t powerPin_;     ///< GPIO switching the sensor supply, -1 if none
  uint8_t status_;      ///< status register value expected inside the sensor
  uint8_t lastCmd_;     ///< command of the running measurement (for the CRC)
  Resolution res_;      ///< resolution the raw values are converted with
  int16_t d1_;          ///< temperature offset for the current supply voltage, 0.01 degC
  DelayHook hook_;      ///< optional sleep callback, `nullptr` = delay()
  bool crc_ : 1;        ///< read and check the CRC byte
  bool pullup_ : 1;     ///< use the internal pull-up on DATA
  bool otp_ : 1;        ///< OTP reload enabled (affects the wait time)
};

#endif  // SHT11_FIXED_H
