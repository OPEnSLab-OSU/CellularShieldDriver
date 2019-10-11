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
    static constexpr auto LTE_SHIELD_ECHO_TIMEOUT = 1000;
    static constexpr auto LTE_SHIELD_POWER_TIMEOUT = 12000;
    static constexpr auto LTE_SHIELD_RESET_TIMEOUT = 10000;
    static constexpr auto LTE_SHIELD_REGISTER_TIMEOUT = 30000;
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
        LTE_NOT_FOUND,
        LTE_BAD_CONFIG,
        LTE_AUTO_MNO_FAILED,
        LTE_REGISTRATION_FAILED
    };

    
    enum class PDPType {
        IPV4 = 0,
        NONIP = 1,
        IPV4V6 = 2,
        IPV6 = 3,
        NONE
    };

    enum class MNOType {
        ERROR = 0,
        AUTO = 1, /** Does not work with roaming! */
        ATT = 2,
        VERIZON = 3,
        TELESTRA = 4,
        TMOBILE = 5,
        CHINETELECOM = 6,
        SPRINT = 8,
        VODAPHONE = 19,
        TELUS = 21,
        DEUTSCHETELECOM = 31,
        STANDARDEUROPE = 100
    };

    enum class RegistrationStatus : char {
        DISABLED = '0',
        HOME_NETWORK = '1',
        SEARCHING = '2',
        DENIED = '3',
        NO_SIGNAL = '4',
        ROAMING = '5',
        HOME_SMS_ONLY = '6',
        ROAMING_SMS_ONLY = '7'
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

    struct NetworkConfig {
        const char* apn;
        const MNOType mno;
        const PDPType pdp;
    };

    static const NetworkConfig CONFIG_VERIZON;
    static const NetworkConfig CONFIG_HOLOGRAM; 

    CellularShield(HardwareSerial & serial,
        const uint8_t powerDetectPin,
        const uint8_t powerPin = LTE_SHIELD_POWER_PIN,
        const NetworkConfig& netconfig = CONFIG_HOLOGRAM,
        const unsigned int timeout = 5000,
        const DebugLevel level = DebugLevel::NONE);

    bool begin();

    bool set_network_config(const NetworkConfig& config);
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
    CellularShield::Error m_wait_power_on() const;

    Error m_configure() const;
    Error m_configure_network() const;
    Error m_verify_network() const;

    Error m_reset() const;

    Error m_send_command(const char* const command,
        const bool at = true,
        char* response = nullptr, 
        const size_t dest_max = 0,
        const unsigned long timeout = 0,
        const uint8_t tries = 5) const;

    ResponseType m_check_response(const unsigned long start, const unsigned long timeout) const;
    ResponseType m_check_response(const unsigned long start) const { return m_check_response(start, m_timeout); }

    Error m_response_to_error(const ResponseType resp) const;

    char m_read_serial(const unsigned long start, const unsigned long timeout) const;
    char m_read_serial(const unsigned long start) const { return m_read_serial(start, m_timeout); }

    class SimpleStream {
    public:
        SimpleStream(const bool can_print)
            : m_can_print(can_print) {}
        template<typename T>
        SimpleStream& operator<<(const T& arg) { if (m_can_print) Serial.print(arg); return *this; }
    private:
        bool m_can_print;
    };

    /** @brief debugging print function, only prints if m_debug is true */
    SimpleStream m_print(DebugLevel level) const { 
        // check the current debug level and serial status
        return SimpleStream(static_cast<uint8_t>(level) <= static_cast<uint8_t>(m_debug) 
            && Serial) << "[CellularShield]";
    }
    /** @brief Prints a info message to serial, if info messages are enabled */
    SimpleStream m_info() const { return m_print(DebugLevel::INFO) << "[INFO]"; }
    SimpleStream m_warn() const { return m_print(DebugLevel::WARN) << "[WARN]"; }
    SimpleStream m_error() const { return m_print(DebugLevel::ERROR) << "[ERROR]"; }

    static const char* m_get_pdp_str(const PDPType pdp);
    static const char* m_get_reg_dbg_str(const RegistrationStatus reg);

    HardwareSerial& m_serial;
    NetworkConfig m_net_config;
    const uint8_t m_power_detect_pin;
    const uint8_t m_power_pin;
    const unsigned int m_timeout;
    const DebugLevel m_debug;
};

#endif