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

#include "CellularShieldDriver.h"

CellularShield::CellularShield(HardwareSerial & serial,
    const uint8_t powerDetectPin,
    const uint8_t powerPin,
    const unsigned int timeout,
    const CellularShield::DebugLevel level)
    : m_serial(serial)
    , m_power_detect_pin(powerDetectPin)
    , m_power_pin(powerPin)
    , m_timeout(timeout)
    , m_debug(level) {}

bool CellularShield::begin() {
    // setup pins before we do anything else
    pinMode(m_power_pin, INPUT);
    pinMode(m_power_detect_pin, INPUT_PULLDOWN);
    m_info("Begin initialize LTE shield!");
    // start the Serial interface
    m_serial.begin(LTE_SHIELD_BAUD);
    // check if the shield is alive
    Error err = m_send_command("E0");
    if (err == Error::OK) {
        m_info("Shield is online!");
        // test response
        char response[128];
        err = m_send_command("+COPS=?", true, response, sizeof(response));
        if (err == Error::OK) m_info(response);
    }
    // if it isn't we may need to power it on
    else if (err == Error::TIMEOUT) {
        m_info("Attempting to power on shield...");
        m_power_toggle();
        // wait for the power indicator pin to go high
        const unsigned long start = millis();
        while (digitalRead(m_power_detect_pin) != HIGH) {
            // check timeout
            if (millis() - start > LTE_SHIELD_POWER_TIMEOUT) {
                m_warn("Shield did not indicate power on! Reconfiguring...");
                err = m_configure();
                if (err != Error::OK) return false;
                break;
            }
        }
        // test shield connectivity
        delay(500);
        if (m_send_command("E0") == Error::OK) m_info("Shield successfully powered on!");
    }
    else return false;
    // TODO: check cellular connection
    return true;
}

void CellularShield::m_power_toggle() const {
    pinMode(m_power_pin, OUTPUT);
    digitalWrite(m_power_pin, LOW);
    delay(LTE_SHIELD_POWER_PULSE_PERIOD);
    pinMode(m_power_pin, INPUT); // Return to high-impedance, rely on SARA module internal pull-up
}

CellularShield::Error CellularShield::m_reset() const {
    // Send the reset command to the device for a clean slate
    Error err = m_send_command("+CFUN=15", true, nullptr, 0, LTE_SHIELD_RESET_TIMEOUT);
    if (err != Error::OK) return err;
    // we can expect this command to time out, as the data sheet says it may take up to 3min to execute
    // now we have to turn echo off so the device doesn't start jamming us
    return m_send_command("E0");
}

CellularShield::Error CellularShield::m_configure() const {
    // toggle the power and send test commands until we get something back
    uint8_t tries = 0;
    Error err = m_send_command("E0");
    while(err != Error::OK && ++tries < 4){
        m_power_toggle();
        delay(LTE_SHIELD_POWER_TIMEOUT);
        err = m_send_command("E0");
    }
    if (err != Error::OK) {
        m_error("Could not find LTE shield");
        return Error::LTE_NOT_FOUND;
    }
    // configure the modem!
    constexpr const char* const commands[] = {
        // set GPIO1 to Network Indicator 
        "+UGPIOC=16,2",
        // GPIO2 to GNSS supply enable
        "+UGPIOC=23,3",
        // and GPIO3 as a power indicator
        "+UGPIOC=24,10",
        // set SMS message format to TXT,
        "+CMGF=1",
        // set auto timezone to true
        "+CTZU=1",
        // tell the device to disconnect from the network
        // so we can configure cellular
        "+CFUN=0",
        // set the device to configure MNO based on the SIM card!
        "+UMNOPROF=1"
    };
    // run all the above commands in consecutive order
    for (uint8_t i = 0; i < sizeof(commands) / sizeof(char*); i++) {
        err = m_send_command(commands[i]);
        if (err != Error::OK) return err;
        delay(100);
    }
    // and reset the device
    err = m_reset();
    // we should now have connection?
    return err;
}

// unhandled edge cases: "\r\n" in response, response without + prefix, command without + prefix
CellularShield::Error CellularShield::m_send_command(const char* const command,
    const bool at,
    char* response, 
    const size_t dest_max,
    const unsigned long timeout,
    const uint8_t tries) const {
    
    const auto timeout_calc = timeout ? timeout : m_timeout;
    Error err;
    // check the serial bus for any URCs before transmitting
    // TODO: write poll function
    // err = poll();
    // if (err != Error::OK) return err; 

    // if we encouter a timeout error, the device may have just missed the transmission
    // in which case we should keep trying until one goes through
    for (uint8_t try_num = 0; try_num < tries; try_num++) {
        // send the command!
        m_info("Try: ");
        m_info(try_num);
        m_info("Sending command: ");
        if (m_debug == DebugLevel::INFO) {
            m_print_prefix();
            if (at) Serial.print("AT");
            Serial.println(command);
        }
        m_info("Recieved: ");

        // send the command!
        if (at) m_serial.print("AT");
        m_serial.println(command);
        m_serial.flush();
        // the datasheet recommends a 20ms delay after sending the command
        delay(20);
        const unsigned long start = millis();
        // skip first line (since it's the echo)
        {
            char c;
            do { c = m_read_serial(start, timeout_calc); } while (c != '\n' && c != 255);
            // try again!
            if (c == 255) continue;
        }
        // if we're expecting a response, wait until the serial finds something,
        // and make sure it's what we're looking for
        if (response != nullptr && dest_max > 0) {
            // check the response type!
            const ResponseType resp = m_check_response(start, timeout_calc);
            if (resp != ResponseType::DATA) {
                m_error("Got unexpected response type from data query: ");
                m_error(static_cast<uint8_t>(resp));
                return m_response_to_error(resp);
            }
            // we found a response, so parse it and return the data values
            // we skip one character in the command string because we have already looked at the '+' character
            // at the start of the command
            {
                uint8_t i = 1;
                // check that Serial recived data matches the command name
                for (; i < strnlen(command, LTE_SHIELD_COMMAND_MAX_LEN); i++) {
                    // the command name is over and we passed
                    if (command[i] == '=' || command[i] == '?') break;
                    // read Serial!
                    const char c = m_read_serial(start, timeout_calc);
                    if (c == 255) return Error::TIMEOUT;
                    // check that the command string and returned value are equivilant
                    if (command[i] != c) {
                        m_error("Command/response mismatch!");
                        if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
                            Serial.print(c);
                            while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
                        }
                        return Error::INVALID_RESPONSE;
                    }
                }
            }
            // it worked! parse the response into the response buffer
            // iterate over serial to remove ": " after command name
            if (m_read_serial(start, timeout_calc) == 255 || m_read_serial(start, timeout_calc) == 255) return Error::TIMEOUT;
            // write the rest of the serial buffer into the response, until newline or buffer max
            {
                size_t i = 0;
                for (;; i++) {
                    const char c = m_read_serial(start, timeout_calc);
                    if (c == 255) return Error::TIMEOUT;
                    if (c == '\n' || c == '\r') break;
                    // generate a warning if we hit dest_max
                    if (i >= dest_max - 1) {
                        m_warn("Response was clipped due to overflowing buffer!");
                        break;
                    }
                    response[i] = c;
                }
                response[i] = '\0';
            }
        }
        // finally, read the ERROR or OK response
        const ResponseType resp = m_check_response(start, timeout_calc);
        if (resp != ResponseType::OK) {
            m_error("Got unexpected response type from OK check:");
            m_error(static_cast<char>(resp));
            return m_response_to_error(resp);
        }
        return Error::OK;
    }
    return Error::TIMEOUT;
}

CellularShield::ResponseType CellularShield::m_check_response(const unsigned long start, const unsigned long timeout) const {
    // check for the OK or ERROR response
    do {
        const char c = m_read_serial(start, timeout);
        if (c == 255) return ResponseType::TIMEOUT;
        // discard characters that are inbetween commands
        if (c == '\n' || c == '\r' || c == ' ') continue;
        // check for a data response
        if (c == static_cast<char>(ResponseType::DATA)) return ResponseType::DATA;
        // check for "OK\r\n" response
        if (c == static_cast<char>(ResponseType::OK)) {
            // command failed successfully!
            // flush the serial until we hit the \n
            char k = 0;
            do {
                k = m_read_serial(start, timeout);
                if (k == 255) return ResponseType::TIMEOUT;
            } while(k != '\n');
            // we did it!
            return ResponseType::OK;
        }
        // invalid response!
        if (c == static_cast<char>(ResponseType::ERROR))
            m_error("LTE shield returned ERROR. Data:");
        else m_error("LTE shield returned an unexpected character. Data:");  
        // unknown character
        if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
            Serial.print(c);
            while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
        }
        if (c == 'E') return ResponseType::ERROR;
        return ResponseType::UNKNOWN;
    } while(true);
}

CellularShield::Error CellularShield::m_response_to_error(const ResponseType resp) const {
    switch (resp) {
        case ResponseType::OK: return Error::UNEXPECTED_OK;
        case ResponseType::DATA: return Error::UNEXPECTED_DATA;
        case ResponseType::ERROR: return Error::LTE_ERROR;
        case ResponseType::TIMEOUT: return Error::TIMEOUT;
        default: return Error::UNEXPECTED_DATA;
    }
}