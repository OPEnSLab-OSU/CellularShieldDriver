/* Copyright 2019 OSU OPEnS Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this
 * software and associated documentation files (the "Software"), to deal in the Software
 * without restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "Arduino.h"

#ifndef CellularShieldDriver_H_
#define CellularShieldDriver_H_

class CellularShield {
public:

    static constexpr auto LTE_SHIELD_POWER_PIN = 5;
    static constexpr auto LTE_SHIELD_BAUD = 115200;
    static constexpr auto LTE_SHIELD_COMMAND_MAX_LEN = 10;
    static constexpr auto LTE_SHIELD_POWER_PULSE_PERIOD = 3200;
    static constexpr auto LTE_SHIELD_RESET_PULSE_PERIOD = 10000;
    static constexpr auto LTE_SHIELD_POWER_TIMEOUT = 12000;
    static constexpr auto LTE_SHIELD_RESET_TIMEOUT = 10000;
    static constexpr auto LTE_SHIELD_GREETING = '@';

    enum class Protocol {
        TCP = 6,
        UDP = 17
    };

    enum class ResponseType : char {
        DATA = '+',
        OK = 'O',
        ERROR = 'E',
        TIMEOUT = 254,
        UNKNOWN = 255,
    };

    enum class Error {
        OK,
        TIMEOUT,
        INVALID_RESPONSE,
        UNEXPECTED_DATA,
        UNEXPECTED_OK,
        LTE_ERROR,
        LTE_NOT_FOUND
    };

    enum class DebugLevel {
        /** No logging output */
        NONE = 0,
        /** Only output errors that result in connection failure */
        ERROR = 1,
        /** Output errors and warnings (useful when just starting to develop) */
        WARN = 2,
        /** Output errors, warnings, and internal information (very verbose) */
        INFO = 3,
    };

    CellularShield(HardwareSerial & serial,
        const uint8_t powerDetectPin,
        const uint8_t powerPin = LTE_SHIELD_POWER_PIN,
        // MUST BE OVER 10 seconds
        const unsigned int timeout = 5000,
        const DebugLevel level = DebugLevel::NONE);

    bool begin();
    /*
    int8_t socketOpen(Protocol protocol, unsigned int localPort = 0);
    LTE_Shield_error_t socketClose(int socket);
    LTE_Shield_error_t socketConnect(int socket, const char * address, unsigned int port);
    LTE_Shield_error_t socketWrite(int socket, const char * str);
    LTE_Shield_error_t socketRead(int socket, int length, char * readDest);

    Error poll();
    */

private:

    void m_power_toggle() const;

    Error m_configure() const;

    Error m_reset() const;

    Error m_send_command(const char* const command,
        const bool at = true,
        char* response = nullptr, 
        const size_t dest_max = 0,
        const unsigned long timeout = 0,
        const uint8_t tries = 3) const;

    ResponseType m_check_response(const unsigned long start, const unsigned long timeout) const;

    ResponseType m_check_response(const unsigned long start) const { return m_check_response(start, m_timeout); }

    Error m_response_to_error(const ResponseType resp) const;

    char m_read_serial(const unsigned long start, const unsigned long timeout) const {
        while (!m_serial.available()) {
            // wait, checking timeout while we're doing so
            if (millis() - start > timeout) {
                m_error("Timed out waiting on the LTE serial");
                return 255;
            }
        }
        // read the first character recieved
        const char c = m_serial.read();
        // debug print!
        if (m_debug == DebugLevel::INFO) Serial.print(c);
        return c;
    }

    char m_read_serial(const unsigned long start) const { return m_read_serial(start, m_timeout); }

    /** @brief Prints a debugging prefix to all logs, so we can attatch them to useful information */
    void m_print_prefix() const { Serial.print("[CellularShield]"); }

    /** @brief debugging print function, only prints if m_debug is true */
    template<typename T>
    void m_print(const T& str, const DebugLevel level) const { 
        // check the current debug level and serial status
        if (static_cast<uint8_t>(level) > static_cast<uint8_t>(m_debug) || !Serial) return;
        // print the message
        m_print_prefix();
        Serial.println(str);
    }

    /** @brief Prints a info message to serial, if info messages are enabled */
    template<typename T>
    void m_info(const T& str) const { m_print(str, DebugLevel::INFO); }

    template<typename T>
    void m_warn(const T& str) const { m_print(str, DebugLevel::WARN); }

    template<typename T>
    void m_error(const T& str) const { m_print(str, DebugLevel::ERROR); }

    HardwareSerial& m_serial;
    const uint8_t m_power_detect_pin;
    const uint8_t m_power_pin;
    const unsigned int m_timeout;
    const DebugLevel m_debug;
};

#endif