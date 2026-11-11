// LowPowerNode - minimal-energy measurement cycle for a battery node.
//
//   * sensor supply switched by a GPIO (off while sleeping)
//   * 12 bit temperature / 8 bit humidity, no OTP reload  -> shortest active time
//   * no CRC, no status read, no soft reset                -> minimal bus traffic
//   * the MCU sleeps while the sensor converts (delay hook)
//
// ESP32: wakes from deep sleep through setup() every SLEEP_SECONDS.
// UNO  : power-down sleep with the watchdog (8 s steps) between measurements.
//
// Hardware: 10k pull-up from DATA to the SWITCHED sensor supply (the power pin),
// 100 nF across VDD/GND at the sensor.
#include <SHT11Fixed.h>

#if defined(ARDUINO_ARCH_AVR)
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
const uint8_t PIN_DATA = 2, PIN_SCK = 3;
const int8_t PIN_PWR = 4;
const uint8_t SLEEP_STEPS_8S = 7;  // ~56 s
#else
#include <esp_sleep.h>
const uint8_t PIN_DATA = 21, PIN_SCK = 22;
const int8_t PIN_PWR = 23;
const uint32_t SLEEP_SECONDS = 60;
#endif

SHT11 sensor(PIN_DATA, PIN_SCK);

#if defined(ARDUINO_ARCH_AVR)
// Empty watchdog interrupt handler (wake-up only). EMPTY_INTERRUPT is used instead of the
// usual ISR macro with an empty body, because the Arduino/PlatformIO .ino preprocessor
// generates a prototype for it that expands into a broken declaration (the compile error then
// shows up at the next function).
EMPTY_INTERRUPT(WDT_vect);

// Sleep roughly 8 s in power-down mode (watchdog interrupt wakes the MCU)
static void sleep8s() {
  MCUSR &= ~(1 << WDRF);
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);  // 8 s, interrupt only
  wdt_reset();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  noInterrupts();
  sleep_enable();
  sleep_bod_disable();
  interrupts();
  sleep_cpu();
  sleep_disable();
  wdt_disable();
}
#endif

// Waiting inside the library (11 ms start-up, conversion time). On ESP32 delay() lets
// FreeRTOS idle; for lower power enable automatic light sleep or call
// esp_light_sleep_start() here. On AVR a plain delay() is used (busy wait); swap in a
// watchdog-based sleep such as the LowPower library if the ~100-300 ms matter.
static void idleWait(uint16_t ms) { delay(ms); }

// Measure the supply voltage of the node in mV. The sensor shares this rail
// (a GPIO power pin drops only a few mV at ~1 mA).
#if defined(ARDUINO_ARCH_AVR)
// Classic trick: measure the internal 1.1 V bandgap against VCC.
static uint16_t readSupplyMv() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                           // let the reference settle
  ADCSRA |= _BV(ADSC);                // first conversion after switching is unreliable
  while (bit_is_set(ADCSRA, ADSC)) {}
  ADCSRA |= _BV(ADSC);
  while (bit_is_set(ADCSRA, ADSC)) {}
  return (uint16_t)(1125300UL / ADC);  // 1.1 * 1023 * 1000
}
#else
// ESP32: battery through a 1:1 divider on an ADC1 pin (adapt pin and ratio to your board)
const uint8_t PIN_VBAT = 35;
static uint16_t readSupplyMv() { return (uint16_t)(analogReadMilliVolts(PIN_VBAT) * 2U); }
#endif

static void printCenti(int32_t v) {
  if (v < 0) {
    Serial.print('-');
    v = -v;
  }
  Serial.print(v / 100);
  Serial.print('.');
  uint8_t frac = (uint8_t)(v % 100);
  if (frac < 10) Serial.print('0');
  Serial.print(frac);
}

static void measureAndReport() {
  SHT11::Config cfg;
  cfg.resolution = SHT11::Resolution::Low;  // 12 bit T / 8 bit RH
  cfg.otpReload = false;                    // ~10 ms less active time
  cfg.supplyMv = readSupplyMv();            // battery voltage -> temperature offset d1
  cfg.powerPin = PIN_PWR;

  sensor.setDelayHook(idleWait);

  int16_t temp = 0;
  uint16_t rh = 0;
  SHT11::Error e = sensor.begin(cfg);  // power on, wait 11 ms, write status register once
  if (e == SHT11::Error::Ok) e = sensor.read(temp, rh);
  sensor.end();                        // power off, bus floating

  // Replace this by your radio transmission (LoRa, ESP-NOW, ...)
  Serial.print(F("err="));
  Serial.print((int)e);
  Serial.print(F(" T="));
  printCenti(temp);
  Serial.print(F(" RH="));
  printCenti(rh);
  Serial.println();
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
#if !defined(ARDUINO_ARCH_AVR)
  measureAndReport();
  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_SECONDS * 1000000ULL);
  esp_deep_sleep_start();  // restarts in setup() on wake-up
#endif
}

void loop() {
#if defined(ARDUINO_ARCH_AVR)
  measureAndReport();
  for (uint8_t i = 0; i < SLEEP_STEPS_8S; i++) sleep8s();
#endif
}
