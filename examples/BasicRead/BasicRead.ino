// BasicRead - read the SHT11 once per 2 s and print fixed-point values (no float).
//
// Wiring: DATA -> pin 2 (+ 10k pull-up to VDD), SCK -> pin 3, 100 nF across VDD/GND.
#include <SHT11Fixed.h>

const uint8_t PIN_DATA = 2;
const uint8_t PIN_SCK = 3;

SHT11 sensor(PIN_DATA, PIN_SCK);

// Print a value given in 1/100 units as "12.34" without using floating point
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

void setup() {
  Serial.begin(115200);

  SHT11::Config cfg;
  // Supply voltage in mV selects the temperature offset. With a fixed supply just set it
  // once; on a battery node measure VDD and call sensor.setSupplyMillivolts(mv) per cycle.
#if defined(ARDUINO_ARCH_AVR)
  cfg.supplyMv = 5000;  // UNO runs the sensor at 5 V
#else
  cfg.supplyMv = 3300;  // ESP32
#endif

  SHT11::Error e = sensor.begin(cfg);
  if (e != SHT11::Error::Ok) {
    Serial.print(F("begin() failed: "));
    Serial.println((int)e);
  }
}

void loop() {
  int16_t temp;   // 0.01 degC
  uint16_t rh;    // 0.01 %RH

  SHT11::Error e = sensor.read(temp, rh);
  if (e == SHT11::Error::Ok) {
    Serial.print(F("T = "));
    printCenti(temp);
    Serial.print(F(" C   RH = "));
    printCenti(rh);
    Serial.println(F(" %"));
  } else {
    Serial.print(F("read() error "));
    Serial.println((int)e);
  }

  // End-of-battery flag of the sensor (VDD < ~2.47 V), set after a measurement
  bool batteryLow;
  if (sensor.readBatteryLow(batteryLow) == SHT11::Error::Ok && batteryLow) {
    Serial.println(F("Warning: sensor supply below 2.47 V"));
  }

  delay(2000);  // >= 1 s between measurements keeps self-heating below 0.1 degC
}
