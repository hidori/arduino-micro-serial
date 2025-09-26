# Arduino: MicroSerial - Lightweight bit-bang Serial library

A minimal serial communication implementation using bit-banging technique, optimized for ATtiny and ATmega microcontrollers with cross-platform compatibility. This library provides transmit-only functionality for debug output and data logging.

## Features

- **Lightweight**: Minimal C/C++ library implementation
- **Bit-bang Serial**: No hardware UART peripheral required
- **TX Only**: Transmit-only implementation (no receive functionality)
- **Multiple instances**: Support for multiple serial outputs using different pins simultaneously
- **AVR-optimized**: Direct register manipulation for ATtiny and ATmega microcontrollers
- **Cross-platform compatible**: Automatic fallback to digitalWrite for non-AVR platforms
- **Minimal memory footprint**: Optimized for resource-constrained microcontrollers
- **Simple API**: Easy to use functions for serial communication
- **Configurable**: Support for various baud rates through macro configuration

## Code Size

The following table shows the compiled size of MicroSerial library functions for different AVR targets:

| Target | FQBN | MicroSerial Size (bytes) |
|--------|------|----------------------|
| Arduino UNO | `arduino:avr:uno` | 408 |
| Arduino Nano | `arduino:avr:nano` | 408 |
| Arduino Leonardo | `arduino:avr:leonardo` | 408 |
| Pro Micro | `SparkFun:avr:promicro` | 408 |
| ATtiny85 | `ATTinyCore:avr:attinyx5:chip=85` | 368 |
| ATtiny13 | `MicroCore:avr:13` | 364 |

*Note: Measurements are approximate. Actual size may vary depending on optimization level and used functions.*

## Supported Platforms

### AVR Microcontrollers (Optimized)

- **ATtiny13** (MicroCore) - Direct register manipulation
- **ATtiny85** (ATTinyCore) - Direct register manipulation
- **ATmega328P** (Arduino UNO/Nano) - Direct register manipulation
- **ATmega32U4** (Arduino Pro Micro, Leonardo) - Direct register manipulation
- Other AVR microcontrollers with PORTB pins - Direct register manipulation

### Other Platforms (Compatible)

- **ESP32** (ESP32-C3) - Uses digitalWrite
- **RP2040** (Raspberry Pi Pico) - Uses digitalWrite
- **Others** - Uses digitalWrite for any other Arduino-compatible platform with pinMode/digitalWrite support

**Note**: The library automatically detects AVR platforms and uses optimized register operations (DDRB, PORTB) for maximum efficiency. On non-AVR platforms, it falls back to standard Arduino functions for broader compatibility.

**Important**: Pin definitions should be handled using preprocessor directives for cross-platform compatibility. Example:

```cpp
#if defined(__AVR__)
#define TX PB0  // AVR platforms use PORTB bit numbers
#else
#define TX 2    // Non-AVR platforms use standard pin numbers
#endif
#define SERIAL_HANDLE MICRO_SERIAL_HANDLE(TX, 9600)
```

## Installation

### Arduino IDE Library Manager

1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for "MicroSerial"
4. Click Install

### Manual Installation

1. Download this repository as ZIP
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Select the downloaded ZIP file

## Usage

```cpp
#include "MicroSerial.h"

// Define TX pin based on platform
#if defined(__AVR__)
#define TX PB0  // Use PORTB bit number on AVR
#else
#define TX 2    // Use digital pin number on other platforms
#endif

#define HANDLE MICRO_SERIAL_HANDLE(TX, 9600)

void setup() {
  // Initialize serial communication
  MicroSerial_begin(HANDLE);
  delay(100);
}

void loop() {
  // Send various types of data
  MicroSerial_newline(HANDLE);
  MicroSerial_print(HANDLE, '-');
  MicroSerial_println(HANDLE, '-');
  MicroSerial_println(HANDLE, "Hi!");

  // Send integers
  MicroSerial_print(HANDLE, " int:");
  MicroSerial_printdec(HANDLE, 12345);
  MicroSerial_newline(HANDLE);
  MicroSerial_print(HANDLE, "uint:");
  MicroSerial_printdecln(HANDLE, -12345);

  // Send hexadecimal values
  MicroSerial_print(HANDLE, "byte:");
  MicroSerial_printhex(HANDLE, 0x12, 2);
  MicroSerial_newline(HANDLE);
  MicroSerial_print(HANDLE, "word:");
  MicroSerial_printhexln(HANDLE, 0x1234, 4);

  delay(3000);
}
```

### Multiple Serial Outputs

You can use multiple pins to create multiple serial outputs simultaneously:

```cpp
#include "MicroSerial.h"

// Define multiple serial outputs with different pins and baud rates
#define DEBUG_SERIAL   MICRO_SERIAL_HANDLE(PB2, 9600)   // Debug output
#define DATA_SERIAL    MICRO_SERIAL_HANDLE(PB3, 19200)  // Data logging
#define STATUS_SERIAL  MICRO_SERIAL_HANDLE(PB4, 38400)  // Status messages

void setup() {
  // Initialize all serial outputs
  MicroSerial_begin(DEBUG_SERIAL);
  MicroSerial_begin(DATA_SERIAL);
  MicroSerial_begin(STATUS_SERIAL);

  // Send initialization messages to different outputs
  MicroSerial_print(DEBUG_SERIAL, "Debug channel ready");
  MicroSerial_println(DEBUG_SERIAL);

  MicroSerial_print(DATA_SERIAL, "Data channel ready");
  MicroSerial_println(DATA_SERIAL);

  MicroSerial_print(STATUS_SERIAL, "Status channel ready");
  MicroSerial_println(STATUS_SERIAL);
}

void loop() {
  // Send different data to different outputs
  MicroSerial_print(DEBUG_SERIAL, "Debug: Loop iteration");
  MicroSerial_println(DEBUG_SERIAL);

  MicroSerial_print(DATA_SERIAL, "Sensor: ");
  MicroSerial_printhex(DATA_SERIAL, (uint8_t)123);
  MicroSerial_println(DATA_SERIAL);

  MicroSerial_print(STATUS_SERIAL, "OK");
  MicroSerial_println(STATUS_SERIAL);

  delay(1000);
}
```

## Pin Configuration

The library uses different pin numbering schemes depending on the platform:

### AVR Platforms (ATtiny, ATmega)

Uses PORTB bit numbers (0-7) directly:

```cpp
// Define serial with PORTB bit number (TX=PB2), baud rate 9600
#define SERIAL_HANDLE MICRO_SERIAL_HANDLE(PB2, 9600)  // Uses bit number 2
```

### Non-AVR Platforms (ESP32, RP2040, Others)

Uses Arduino digital pin numbers:

```cpp
// Define serial with digital pin numbers
#define SERIAL_HANDLE MICRO_SERIAL_HANDLE(21, 9600)  // GPIO21 on ESP32
// or
#define SERIAL_HANDLE MICRO_SERIAL_HANDLE(4, 9600)   // GPIO4 on RP2040
```

**Pin Mapping for Arduino UNO/Nano (ATmega328P)**:

When using `MICRO_SERIAL_HANDLE(PB2, 9600)` on AVR:

- PB2 = Digital Pin 10

Full PORTB mapping:

- PB0 = Digital Pin 8
- PB1 = Digital Pin 9
- PB2 = Digital Pin 10
- PB3 = Digital Pin 11
- PB4 = Digital Pin 12
- PB5 = Digital Pin 13

**Pin Mapping for Arduino Leonardo/Pro Micro (ATmega32U4)**:

When using `MICRO_SERIAL_HANDLE(PB2, 9600)` on AVR:

- PB2 = Digital Pin 16 (MOSI)

Full PORTB mapping:

- PB0 = Digital Pin 17 (RX LED)
- PB1 = Digital Pin 15 (SCK)
- PB2 = Digital Pin 16 (MOSI)
- PB3 = Digital Pin 14 (MISO)
- PB4 = Digital Pin 8
- PB5 = Digital Pin 9
- PB6 = Digital Pin 10
- PB7 = Digital Pin 11

## Baud Rate Configuration

The library supports various baud rates through the handle macro:

```cpp
// Common baud rates for different purposes
#define DEBUG_SERIAL    MICRO_SERIAL_HANDLE(PB2, 9600)   // Debug output
#define DATA_SERIAL     MICRO_SERIAL_HANDLE(PB3, 19200)  // Data logging
#define STATUS_SERIAL   MICRO_SERIAL_HANDLE(PB4, 38400)  // Status messages
#define FAST_SERIAL     MICRO_SERIAL_HANDLE(PB5, 115200) // High-speed data
```

## Supported Baud Rates

### Theoretical Range

The library theoretically supports baud rates from approximately **245 bps to 1,000,000 bps**, calculated as:

```text
Bit delay (μs) = 1,000,000 / baud_rate
Minimum delay: 1 μs → Maximum baud rate: 1,000,000 bps
Maximum delay: 4095 μs → Minimum baud rate: ~245 bps
```

### Practical Limitations

**Important**: Not all baud rates are usable on all microcontrollers. The maximum achievable baud rate depends on the microcontroller's processing power and clock frequency:

#### Arduino UNO/Nano (ATmega328P @ 16MHz)

- **Reliable range**: 300 bps - 9600 bps
- **May work**: 19200 bps (depending on conditions)
- **Typically fails**: 38400 bps and above

#### Arduino Leonardo/Pro Micro (ATmega32U4 @ 16MHz)

- **Reliable range**: 300 bps - 9600 bps
- **May work**: 19200 bps (depending on conditions)
- **Typically fails**: 38400 bps and above

#### ATtiny85 (@ 8MHz or lower)

- **Reliable range**: 300 bps - 4800 bps
- **May work**: 9600 bps (depending on clock speed)
- **Typically fails**: 19200 bps and above

#### ATtiny13 (@ 9.6MHz internal RC)

- **Reliable range**: 300 bps - 2400 bps
- **May work**: 4800 bps (depending on conditions)
- **Typically fails**: 9600 bps and above

### Factors Affecting Maximum Baud Rate

- **Microcontroller clock frequency**: Higher clock = higher possible baud rates
- **Instruction execution overhead**: Bit manipulation and timing functions consume cycles
- **Interrupt interference**: Timer and other interrupts can disrupt precise timing
- **Temperature and voltage**: Can affect internal RC oscillator accuracy

### Recommendation

Start with conservative baud rates (1200-4800 bps) and test higher rates if needed. For high-speed communication requirements, consider using hardware UART instead of bit-banging.

## API Reference

### Initialization

- `MicroSerial_begin(uint16_t handle)` - Initialize serial communication pin

### Output Functions

#### Character and String Output

- `MicroSerial_print(uint16_t handle, char ch)` - Send a single character
- `MicroSerial_print(uint16_t handle, const char* str)` - Send a string
- `MicroSerial_println(uint16_t handle, char ch)` - Send a character and newline
- `MicroSerial_println(uint16_t handle, const char* str)` - Send a string and newline
- `MicroSerial_newline(uint16_t handle)` - Send a newline character only

#### Numeric Output

- `MicroSerial_printdec(uint16_t handle, int data)` - Send a signed integer (decimal)
- `MicroSerial_printdecln(uint16_t handle, int data)` - Send a signed integer and newline
- `MicroSerial_printdec(uint16_t handle, unsigned int data)` - Send an unsigned integer (decimal)
- `MicroSerial_printdecln(uint16_t handle, unsigned int data)` - Send an unsigned integer and newline

#### Hexadecimal Output

- `MicroSerial_printhex(uint16_t handle, int data, int digits)` - Send a value as hexadecimal with specified digits
- `MicroSerial_printhexln(uint16_t handle, uint16_t data, int digits)` - Send a hexadecimal value and newline

### Low-level Functions

- `MicroSerial_tx_low(uint8_t tx)` - Set TX pin LOW (internal use)
- `MicroSerial_tx_high(uint8_t tx)` - Set TX pin HIGH (internal use)

### Macros

- `MICRO_SERIAL_HANDLE(tx, baudrate)` - Create serial configuration handle

## Examples

See the `examples/` directory for complete usage examples:

- **HelloSerial**: Basic "Hello World" example
- **DebugSerial**: Conditional debug output controlled by DEBUG macro definition

## Debug Support

The library includes a useful **DebugSerial.h** utility that provides conditional debugging support for development and testing:

### Debug Features

- **Conditional compilation**: Debug output is completely removed when `DEBUG` is not defined
- **Zero overhead**: No code size penalty in production builds
- **Complete API coverage**: All MicroSerial functions available as debug macros
- **Easy integration**: Simply `#include "DebugSerial.h"` and use `DEBUG_SERIAL_*()` macros

### Debug Usage

```cpp
// Enable debug output by defining DEBUG before including
#define DEBUG
#include "DebugSerial.h"

// Configure debug serial handle
#if defined(__AVR__)
#define __DEBUG_SERIAL_HANDLE__ DEBUG_SERIAL_HANDLE(PB0, 9600)
#else
#define __DEBUG_SERIAL_HANDLE__ DEBUG_SERIAL_HANDLE(2, 9600)
#endif

void setup() {
  DEBUG_SERIAL_BEGIN();
  DEBUG_SERIAL_WAIT_FOR();  // Wait for serial monitor
}

void loop() {
  DEBUG_SERIAL_PRINTLN("Debug message");
  DEBUG_SERIAL_PRINTDEC(12345);
  DEBUG_SERIAL_NEWLINE();
  DEBUG_SERIAL_PRINTHEX2(0xAB);    // 2-digit hex: AB
  DEBUG_SERIAL_PRINTHEX4(0x1234);  // 4-digit hex: 1234
  delay(1000);
}
```

### Available Debug Macros

- `DEBUG_SERIAL_HANDLE(tx, baudrate)` - Create debug serial handle
- `DEBUG_SERIAL_BEGIN()` - Initialize debug serial
- `DEBUG_SERIAL_WAIT_FOR()` - Wait for serial connection (100ms delay)
- `DEBUG_SERIAL_PRINT(x)` / `DEBUG_SERIAL_PRINTLN(x)` - Print character/string
- `DEBUG_SERIAL_PRINTDEC(x)` / `DEBUG_SERIAL_PRINTDECLN(x)` - Print decimal numbers
- `DEBUG_SERIAL_PRINTHEX(x, n)` / `DEBUG_SERIAL_PRINTHEXLN(x, n)` - Print hex with n digits
- `DEBUG_SERIAL_PRINTHEX2(x)` / `DEBUG_SERIAL_PRINTHEX2LN(x)` - Print 2-digit hex
- `DEBUG_SERIAL_PRINTHEX4(x)` / `DEBUG_SERIAL_PRINTHEX4LN(x)` - Print 4-digit hex
- `DEBUG_SERIAL_NEWLINE()` - Print newline

When `DEBUG` is not defined, all debug macros expand to empty statements, ensuring zero runtime and memory overhead in production code.

## Technical Details

### Timing

The library calculates bit timing automatically from the specified baud rate:

```cpp
// Bit delay in microseconds = 1,000,000 / baud_rate
// The macro stores the upper 8 bits of (1,000,000 / baud_rate) right-shifted by 8
```

### Serial Protocol

The library implements standard UART protocol for transmission:

1. **Start bit**: LOW for one bit period
2. **Data bits**: 8 bits, LSB first
3. **Stop bit**: HIGH for one bit period

No parity bit is used. **Note**: This library only supports data transmission (TX). Receive functionality is not implemented.

## License

MIT License - see LICENSE file for details.

## Contributing

Pull requests and issues are welcome on GitHub.
