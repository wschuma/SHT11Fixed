# SHT11Fixed

Fixed-point driver for the Sensirion SHT10/SHT11/SHT15 for battery-powered nodes
(Arduino UNO, ESP32). No `float`/`double` at runtime.

## Units

| Value       | Type       | Unit      | Example            |
|-------------|------------|-----------|--------------------|
| Temperature | `int16_t`  | 0.01 °C   | `2345` = 23.45 °C  |
| Humidity    | `uint16_t` | 0.01 %RH  | `5012` = 50.12 %RH |

Verified against the datasheet formulas: max. deviation 0.01 %RH /
0.01 °C over the full raw range for both resolutions; datasheet example SORH = 1073 gives 35.50 %RH.

## Wiring

- SCK -> any GPIO (push-pull), DATA -> any GPIO with **10 kΩ pull-up to VDD**, 100 nF across VDD/GND.
- VDD 2.4–5.5 V. Set `Config::supplyMv` (or call `setSupplyMillivolts()`), it selects the temperature offset d1, see below.
- Optional: switch VDD with a GPIO (`Config::powerPin`, sensor draws ~1 mA max while measuring).
  Put the DATA pull-up on the switched rail.

## Usage

```cpp
#include <SHT11Fixed.h>
SHT11 sensor(2, 3);                // DATA, SCK

void setup() {
  SHT11::Config cfg;               // defaults: 14/12 bit, OTP reload, no CRC
  cfg.supplyMv = 5000;
  sensor.begin(cfg);
}
void loop() {
  int16_t t; uint16_t rh;
  if (sensor.read(t, rh) == SHT11::Error::Ok) { /* t = 0.01 degC, rh = 0.01 %RH */ }
}
```

## Supply voltage compensation

The temperature offset d1 of the SHT1x depends on VDD (datasheet Table 8, -39.4 °C at 2.5 V to
-40.1 °C at 5 V). The library approximates it with a line, in 0.01 °C:

    d1 = -3876 - mV * 438 / 2^14        (clamped to 2400..5500 mV)

| VDD       | 2.5 V | 3.0 V | 3.5 V | 4.0 V | 5.0 V |
|-----------|-------|-------|-------|-------|-------|
| datasheet | -3940 | -3960 | -3970 | -3980 | -4010 |
| linear fit| -3943 | -3956 | -3969 | -3982 | -4009 |

Max. deviation 0.04 °C (about 0.1 % of d1; a fixed d1 would be off by up to 0.4 °C over the
full range). That is below the sensor's own ±0.4 °C accuracy. Below 2.5 V and above 5.0 V the
line is an extrapolation of the datasheet table. Humidity has no significant voltage dependency.

Typical battery node cycle:

```cpp
cfg.supplyMv = readBatteryMv();   // before begin()
sensor.begin(cfg);
sensor.read(t, rh);
```

or call `sensor.setSupplyMillivolts(mv)` at any time before converting.

## Options that reduce communication / energy

| Option / API                         | Effect |
|--------------------------------------|--------|
| `resolution = Low`                   | 12 bit T / 8 bit RH: max 80 / 20 ms instead of 320 / 80 ms. One status write in `begin()`. |
| `otpReload = false`                  | ~10 ms shorter active time per measurement. |
| `checkCrc = false` (default)         | One byte less per measurement. |
| `statusRetained = true`              | No status register write in `begin()` (sensor stayed powered, config unchanged). Otherwise `begin()` always writes the status register, also for a default `Config`. |
| `powerPin` + default `Config`        | `begin()` sends **nothing**: the library powered the sensor up itself, so it is known to be at the defaults. |
| `readHumidity(rh)` without temp      | Skips the temperature measurement (compensation assumes 25 °C). Pass a known temperature as 2nd argument to keep compensation. |
| `powerPin`                           | Sensor completely off between measurements; `end()` floats the bus. |
| `setDelayHook(fn)`                   | All waits go through your sleep function. |
| `start()` / `ready()` / `fetchTemperature()` / `fetchHumidity()` | Non-blocking: start, sleep the MCU, fetch + convert the result. |

`begin()` -> `read()` -> `end()` per wake-up is the intended deep-sleep cycle.

## Non-blocking measurement

```cpp
sensor.start(SHT11::Measurement::Temperature);   // returns immediately
// ... sleep, then check ready() ...
int16_t t;
sensor.fetchTemperature(t);                      // read + convert

sensor.start(SHT11::Measurement::Humidity);
// ... sleep ...
uint16_t rh;
sensor.fetchHumidity(rh, t);                     // read + convert, compensated with t
```

`fetchTemperature()` / `fetchHumidity()` return `Error::NotReady` if the conversion has not
finished yet. Call each only after the matching `start()`.

Conversion times (maximum): temperature 320 ms (14 bit) / 80 ms (12 bit), humidity 80 ms (12 bit) /
20 ms (8 bit). `ready()` reads the DATA line, which the sensor pulls low when the result is
available. If you wake the MCU with a pin interrupt on DATA, arm it only after `start()` has
returned and disarm it before fetching, because the library itself toggles DATA while it talks
to the sensor.

## Public API

| Group            | Methods |
|------------------|---------|
| Lifecycle        | `begin()`, `end()`, `setSupplyMillivolts()`, `setDelayHook()` |
| Blocking read    | `read()`, `readTemperature()`, `readHumidity()` |
| Non-blocking     | `start()`, `ready()`, `fetchTemperature()`, `fetchHumidity()` |
| Sensor functions | `setHeater()`, `readBatteryLow()`, `softReset()` |

Raw values, unit conversion, status register access and the bus protocol are internal.

## Heater and end-of-battery flag

```cpp
sensor.setHeater(true);        // status register write; no traffic if already on
// ... functional check: temperature rises, humidity reading drops ...
sensor.setHeater(false);       // always switch off again before real measurements

bool low;
if (sensor.readBatteryLow(low) == SHT11::Error::Ok && low) { /* sensor VDD < ~2.47 V */ }
```

- `setHeater()` keeps all other status bits (resolution, OTP reload). `Config::heater` still
  sets the initial state in `begin()`.
- The end-of-battery flag is updated by the sensor after each measurement: call
  `readBatteryLow()` after a measurement and, with a switched supply, before `end()`.
  It costs one status read (3 bytes on the bus). For a finer battery check use your own
  supply measurement (`Config::supplyMv`).

## Final note

Vibe coded with Claude.
