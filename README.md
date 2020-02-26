This fork is intended to document some tweeks I did in the excelent library created by Ngraziano to work in US915 MHz band and Ebyte E22-900T30S and E22-900T22S modules
# LORAWAN Library for Arduino and SX1276/2 or SX1262 lora chip

* Based on LMIC librairy.
* Modified to get C++ style.
* Only class A device (no class B)
* Add some sleep of arduino board and ESP.
* Add SX1262 chip

## Limitation

:warning: This library do not compile in Arduino IDE due to dependency to STL (ArduinoSTL in case of AVR platform).
It need PlatfomIO for the dependencies to be handle correctly.

For the SX1262 it only support board with TCXO.

## AVR examples

Tested with Arduino Pro Mini and RFM95 on EU868 frequencies. Examples:

* [simple](examples/simple/) Minimal example, send one analog value.
* [balise](examples/balise/) Example, send one analog value with deepsleep using watchdog.
* [tempsensor](examples/tempsensor/) Example, send temps reads from onewire with deepsleep using watchdog.

## ESP32 examples

Tested with HelTec

* [esp32](examples/esp32/) Minimal example, send one value.
* [esp32-deepsleep](examples/esp32-deepsleep/) Example, send one analog value with deepsleep using RTC and state save in RTC RAM.

## Usage

Work with platformio with Arduino framework.

Copy balise exemple in a new directory.
Open with platformio (VSCODE with Platformio extension)
In ``src`` directory create a file named ``lorakeys.h`` wich contain the keys declared in network (for exemple <https://www.thethingsnetwork.org>)

Exemple of file:

```cpp
// Application in string format.
// For TTN issued EUIs the first bytes should be 70B3D5
constexpr char const appEui[] = "70B3D5XXXXXXXXXX";

// Device EUI in string format.
constexpr char const devEui[] = "XXXXXXXXXXXXXXXX";
// Application key in string format.
constexpr char const appKey[] = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";

```

Use define in platformio.ini `build_flags` to change activated part.

* ENABLE_SAVE_RESTORE enable save and restore functions
* LMIC_DEBUG_LEVEL set to 0,1 or 2 for different log levels (default value 1)

In ``main.cpp`` replace the content of ``do_send()`` with the data you want to send.

## Main functional change from LMIC

* Try to implement ADR a little more correctl:
  * Handle Txpower
  * ADR_ACK_LIMIT set to 64
  * ADR_ACK_DELAY set to 32
* Correct set of power for SX1276
* Various coding style fix (remove goto ...)
* Add method to save and restore state.
* Try to use specific of different platform.

## License

Most source files in this repository are made available under the Eclipse Public License v1.0.
Some of the AES code is available under MIT like license. Refer to each individual source file for more details.
