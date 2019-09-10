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
    const uint8_t powerPin,
    const uint8_t resetPin,
    const unsigned int timeout,
    const CellularShield::DebugLevel level)
    : m_serial(serial)
    , m_power_pin(powerPin)
    , m_reset_pin(resetPin)
    , m_timeout(timeout)
    , m_debug(level) {}

bool CellularShield::begin() {
    m_info("Begin initialize LTE shield!");
    // start the Serial interface
    m_serial.begin(LTE_SHIELD_BAUD);
    // check if the shield is alive
    Error err = m_send_command("");
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
        // wait for greeting text
        const unsigned long start = millis();
        while(true) {
            const char c = m_read_serial(start, LTE_SHIELD_POWER_TIMEOUT);
            // discard whitespace and newlines
            if (c == '\n' || c == '\r' || c == ' ') continue;
            // else if it's not the character we need, reset the device
            if (c != LTE_SHIELD_GREETING) {
                m_warn("Found invalid or no greeting text, reprogramming the device!");
                m_reset();
            }
            // onwards!
            break;
        }
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

Error CellularShield::m_reset_and_configure() const {
    // toggle the power and send test commands until we get something back
    Error err = m_send_command("");
    uint8_t tries = 0;
    while(err != Error::OK && ++tries < 4) {
        m_power_toggle();
        delay(LTE_SHIELD_POWER_TIMEOUT);
        err = m_send_command("");
    }
    if (err != Error::OK) {
        m_error("Could not find LTE shield");
        return Error::LTE_NOT_FOUND;
    }
    // we did it! send the reset command to the device for a clean slate
    m_send_command("+CFUN=15");
    // we can expect this command to time out, as the data sheet says it may take up to 3min to execute
    // saftey delay
    delay(10000);
    // turn echo off for a testing command
    err = m_send_command("E0");
    if (err != Error::OK) return err;
    // configure the modem! Start with perman
    // set the startup text so it's easier to power the device on in the future,
    // set GPIO1 to Network Indicator and GPIO2 to GNSS supply enable,
    // set SMS message format to TXT, and set auto timezone to true
    err = m_send_command("+CSGT=1,\"@\";AT+UGPIOC=16,2;AT+UGPIOC=23,3;AT+CMGF=1;AT+CTZU=1");
    if (err != Error::OK) return err;
    // TODO: Configure cellular
}

CellularShield::Error CellularShield::m_send_command(const char* command,
    const bool at,
    char* response, 
    const size_t dest_max) const {
    
    Error err;
    // check the serial bus for any URCs before transmitting
    // TODO: write poll function
    // err = poll();
    // if (err != Error::OK) return err; 

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
    delay(200);
    const unsigned long start = millis();

    // if we're expecting a response, wait until the serial finds something,
    // and make sure it's what we're looking for
    if (response != nullptr && dest_max > 0) {
        while(true) {
            const char c = m_read_serial(start);
            // if we've found a plus character (the begining of a response),
            // exit the loop
            if (c == '+') break;
            // discard characters that are inbetween commands
            if (c == '\n' || c == '\r' || c == ' ') continue;
            // else we found an invalid character
            // check for a read timeout
            if (c == 255) return Error::TIMEOUT;
            // unexpected OK command termination
            if (c == 'O') {
                m_error("Recieved unexpected \"OK\" command termination");
                return Error::UNEXPECTED_OK;
            }
            // command returned error
            if (c == 'E')
                m_error("LTE shield returned ERROR. Data:");
            else m_error("LTE shield returned an unexpected character");  
            // unknown character
            if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
                Serial.print(c);
                while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
            }
            if (c == 'E') return Error::LTE_ERROR;
            return Error::UNEXPECTED_CHAR;
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
                const char c = m_read_serial(start);
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
        if (m_read_serial(start) == 255 || m_read_serial(start) == 255) return Error::TIMEOUT;
        // write the rest of the serial buffer into the response, until newline or buffer max
        {
            size_t i = 0;
            for (;; i++) {
                const char c = m_read_serial(start);
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
    // check for the OK or ERROR response
    do {
        const char c = m_read_serial(start);
        if (c == 255) return Error::TIMEOUT;
        // discard characters that are inbetween commands
        if (c == '\n' || c == '\r' || c == ' ') continue;
        // check for "OK\r\n" response
        if (c == 'O') {
            // command failed successfully!
            // flush the serial until we hit the \n
            char k = 0;
            do {
                k = m_read_serial(start);
                if (k == 255) return Error::TIMEOUT;
            } while(k != '\n');
            // we did it!
            break;
        }
        // invalid response!
        if (c == 'E')
            m_error("LTE shield returned ERROR. Data:");
        else m_error("LTE shield returned an unexpected character. Data:");  
        // unknown character
        if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
            Serial.print(c);
            while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
        }
        if (c == 'E') return Error::LTE_ERROR;
        return Error::UNEXPECTED_CHAR;
    } while(true);
    return Error::OK;
}
