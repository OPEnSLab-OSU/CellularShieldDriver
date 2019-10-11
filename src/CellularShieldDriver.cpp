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

const CellularShield::NetworkConfig CellularShield::CONFIG_VERIZON = { "vzwinternet", MNOType::VERIZON, PDPType::IPV4};
const CellularShield::NetworkConfig CellularShield::CONFIG_HOLOGRAM = { "hologram", MNOType::VERIZON, PDPType::IPV4 };

CellularShield::CellularShield(HardwareSerial & serial,
    const uint8_t powerDetectPin,
    const uint8_t powerPin,
    const NetworkConfig& netconfig,
    const unsigned int timeout,
    const CellularShield::DebugLevel level)
    : m_serial(serial)
    , m_net_config(netconfig)
    , m_power_detect_pin(powerDetectPin)
    , m_power_pin(powerPin)
    , m_timeout(timeout)
    , m_debug(level) {}

bool CellularShield::begin() {
    // setup pins before we do anything else
    pinMode(m_power_pin, INPUT);
    pinMode(m_power_detect_pin, INPUT_PULLDOWN);
    m_info() << "Begin initialize LTE shield!\n";
    // start the Serial interface
    m_serial.begin(LTE_SHIELD_BAUD);
    // if the shield is not indicating that it is alive,
    // and an echo command fails, reconfigure!
    if (digitalRead(m_power_detect_pin) != HIGH) {
        Error err = m_send_command("E0", true, nullptr, 0, 200, 3);
        if (err == Error::TIMEOUT) {
            m_info() << "Attempting to power on shield...\n";
            m_power_toggle();
            err = m_wait_power_on();
            if (err != Error::OK) return false;
            // test shield connectivity
            if (m_send_command("E0") != Error::OK) return false;
            delay(1000);
        }
        else return false;
    }
    // else we just need to restart it and clear the NVM
    else {
        m_info() << "Reseting to close all sockets...\n";
        Error err = m_reset();
        if (err != Error::OK) return false;
    }
    m_info() << "Shield is online!\n";
    // Test that the network is configured correctly
    Error err = m_verify_network();
    // configure the network
    if (err == Error::LTE_BAD_CONFIG) {
        err = m_configure_network();
        if (err != Error::OK) return false;
        // verify the network one last time
        err = m_verify_network();
        if (err != Error::OK) return false;
    }
    else return false;
    m_info() << "LTE Shield is connected and ready!\n";
    return true;
}

void CellularShield::m_power_toggle() const {
    pinMode(m_power_pin, OUTPUT);
    digitalWrite(m_power_pin, LOW);
    delay(LTE_SHIELD_POWER_PULSE_PERIOD);
    pinMode(m_power_pin, INPUT); // Return to high-impedance, rely on SARA module internal pull-up
}

CellularShield::Error CellularShield::m_wait_power_on() const {
    // wait for the power indicator pin to go high
    const unsigned long start = millis();
    while (digitalRead(m_power_detect_pin) != HIGH) {
        // check timeout
        if (millis() - start > LTE_SHIELD_POWER_TIMEOUT) {
            m_warn() << "Shield did not indicate power on! Reconfiguring...\n";
            Error err = m_configure();
            if (err != Error::OK) return err;
            break;
        }
    }
    return Error::OK;
}

CellularShield::Error CellularShield::m_reset() const {
    // Send the reset command to the device for a clean slate
    Error err = m_send_command("+CFUN=15", true, nullptr, 0, LTE_SHIELD_RESET_TIMEOUT);
    if (err != Error::OK) return err;
    // wait for the device to signal that it's on and ready for input
    delay(300);
    err = m_wait_power_on();
    if (err != Error::OK) return err;
    // wait for the device to load the SIM card and other data
    delay(300);
    // we can expect this command to time out, as the data sheet says it may take up to 3min to execute
    // now we have to turn echo off so the device doesn't start jamming us
    return m_send_command("E0", true, nullptr, 0, LTE_SHIELD_RESET_TIMEOUT);
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
        m_error() << "Could not find LTE shield\n";
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
    };
    // run all the above commands in consecutive order
    for (uint8_t i = 0; i < sizeof(commands) / sizeof(char*); i++) {
        err = m_send_command(commands[i]);
        if (err != Error::OK) return err;
        delay(100);
    }
    // and reset the device
    err = m_reset();
    return err;
}

CellularShield::Error CellularShield::m_configure_network() const {
    // this function assumes the device is on configured using m_configure
    // first we need to set the MNO profile of the device, so that we know
    // which networks to scan for
    // disable the network so we can start
    Error err = m_send_command("+CFUN=0");
    if (err != Error::OK) return err;
    // this takes awhile for some reason
    delay(1000);
    // set the MNO profile according to what was provided
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "+UMNOPROF=%d", static_cast<int>(m_net_config.mno));
        err = m_send_command(buf);
        if (err != Error::OK) return err;
    }
    // and reset the device
    err = m_reset();
    if (err != Error::OK) return err;
    // delay extra long here since changing the MNO profile can make the device unstable
    delay(1000);
    // if the MNO was auto-selected, make sure that a profile was chosen
    if (m_net_config.mno == MNOType::AUTO) {
        char res[3] = {};
        err = m_send_command("+UMNOPROF?", true, res, 2);
        if (err != Error::OK) return err;
        const int num = atoi(res);
        if (num <= 0) {
            m_error() << "SIM MNO select failed! This is probably because your SIM is not from a major carrier. Please select an MNO profile other than AUTO.\n";
            return Error::LTE_AUTO_MNO_FAILED;
        }
        else
            m_info() << "SIM autoselect found profile: " << num << '\n';
        delay(1000);
    }
    // next, set the default PDP context with the values provided, if any
    if (m_net_config.pdp != PDPType::NONE && m_net_config.apn) {
        // build the AT command
        char buf[84];
        snprintf(buf, sizeof(buf), "+CGDCONT=1,\"%s\",\"%s\"", 
            m_get_pdp_str(m_net_config.pdp), 
            m_net_config.apn);
        // configure the PDP contexts
        err = m_send_command(buf);
        if (err != Error::OK) return err;
        delay(500);
    }
    // and reset the device
    err = m_reset();
    if (err != Error::OK) return err;
    delay(1000);
    // finally, set the device to auto-register
    err = m_send_command("+COPS=0");
    return err;
}

CellularShield::Error CellularShield::m_verify_network() const {
    // check that the MNO profile is set correctly, as if it isn't
    // we might end up on the wrong networks
    {
        char res[3] = {};
        Error err = m_send_command("+UMNOPROF?", true, res, 2);
        if (err != Error::OK) return err;
        const int num = atoi(res);
        if (num == static_cast<int>(MNOType::ERROR) 
            || num != static_cast<int>(m_net_config.mno)) {
                m_warn() << "Found an incorrect MNO on the modem: " << num << '\n';
                return Error::LTE_BAD_CONFIG;
            }
    }
    // check that the network is enables and registered successfully
    {
        char res[8];
        // wait for the registration to finish
        RegistrationStatus status;
        uint8_t count = 0;
        m_info() << "Checking registration...\n";
        do {
            // network registration check
            Error err = m_send_command("+CREG?", true, res, 6);
            if (err != Error::OK) return err;
            // check the regisration code against the possible ones
            status = static_cast<RegistrationStatus>(res[2]);
            // if the device has registered, or we time out waiting for it
            if (status == RegistrationStatus::HOME_NETWORK
                || status == RegistrationStatus::ROAMING
                || ++count >= LTE_SHIELD_REGISTER_TIMEOUT / 500) break;
            else delay(500);
        } while (true);
        // check the status
        if (status == RegistrationStatus::HOME_NETWORK
            || status == RegistrationStatus::ROAMING) {
            m_info() << "LTE registered: " << m_get_reg_dbg_str(status) << '\n';
        }
        else {
            m_error() << "LTE not registered: " << m_get_reg_dbg_str(status) << '\n';
            return Error::LTE_REGISTRATION_FAILED;
        }
    }
    // we're good to go!
    return Error::OK;
}

// unhandled edge cases: "\r\n" in response, response without + prefix, command without + prefix
CellularShield::Error CellularShield::m_send_command(const char* const command,
    const bool at,
    char* response, 
    const size_t dest_max,
    const unsigned long timeout,
    const uint8_t tries) const {
    
    const auto timeout_calc = timeout ? timeout : m_timeout;
    // check the serial bus for any URCs before transmitting
    // TODO: write poll function
    // err = poll();
    // if (err != Error::OK) return err; 

    // TODO: modem can turn off after too much idle time. Handle that here?

    // if we encouter a timeout error, the device may have just missed the transmission
    // in which case we should keep trying until one goes through
    for (uint8_t try_num = 0; try_num < tries; try_num++) {
        // send the command!
        m_info() << "Try: " << try_num << ", Sending command: AT" << command << '\n';
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
            do { c = m_read_serial(start, CellularShield::LTE_SHIELD_ECHO_TIMEOUT); } while (c != '\n' && c != 255);
            // try again!
            if (c == 255) {
                m_warn() << "Device failed to echo!\n";
                // wait for a minute to let the device settle
                delay(1000);
                continue;
            }
        }
        // if we're expecting a response, wait until the serial finds something,
        // and make sure it's what we're looking for
        if (response != nullptr && dest_max > 0) {
            // check the response type!
            const ResponseType resp = m_check_response(start, timeout_calc);
            if (resp != ResponseType::DATA) {
                m_error() << "Got unexpected response type from data query: " << static_cast<uint8_t>(resp) << '\n';
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
                        m_error() << "Command/response mismatch: ";
                        if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
                            Serial.print(c);
                            while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
                            Serial.println();
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
                        m_warn() << "Response was clipped due to overflowing buffer!\n";
                        // and flush the buffer
                        char c = '\0';
                        while (m_serial.available() && c != '\n') c = m_serial.read();
                        break;
                    }
                    response[i] = c;
                }
                response[i] = '\0';

                m_info() << "Got response: " << response << '\n';
            }
        }
        // finally, read the ERROR or OK response
        const ResponseType resp = m_check_response(start, timeout_calc);
        if (resp != ResponseType::OK) {
            m_error() << "Got unexpected response type from OK check: " << static_cast<char>(resp) << '\n';
            return m_response_to_error(resp);
        }
        m_info() << "Response OK!\n";
        return Error::OK;
    }
    m_error() << "Timed out when sending command: AT" << command << '\n';
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
            m_error() << "LTE shield returned ERROR. Data:";
        else m_error() << "LTE shield returned an unexpected character. Data:";  
        // unknown character
        if (static_cast<uint8_t>(DebugLevel::ERROR) > static_cast<uint8_t>(m_debug)) {
            Serial.print(c);
            while (m_serial.available()) Serial.print(static_cast<char>(m_serial.read()));
            Serial.println();
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

 char CellularShield::m_read_serial(const unsigned long start, const unsigned long timeout) const {
        while (!m_serial.available()) {
            // wait, checking timeout while we're doing so
            if (millis() - start > timeout) {
                m_warn() << "Timed out waiting on the LTE serial\n";
                return 255;
            }
        }
        // read the first character recieved
        return m_serial.read();
    }

const char* CellularShield::m_get_pdp_str(const PDPType pdp) {
    switch(pdp) {
        case PDPType::IPV4: return "IP";
        case PDPType::IPV4V6: return "IPV4V6";
        case PDPType::IPV6: return "IPV6";
        case PDPType::NONIP: return "NOIP";
        default: return "IPV4";
    }
}

const char* CellularShield::m_get_reg_dbg_str(const RegistrationStatus reg) {
    switch (reg) {
        case RegistrationStatus::DENIED: return "DENIED";
        case RegistrationStatus::DISABLED: return "DISABLED";
        case RegistrationStatus::HOME_NETWORK: return "HOME NETWORK";
        case RegistrationStatus::HOME_SMS_ONLY: return "HOME NETWORK (SMS only)";
        case RegistrationStatus::NO_SIGNAL: return "NO_SIGNAL";
        case RegistrationStatus::ROAMING: return "ROAMING";
        case RegistrationStatus::ROAMING_SMS_ONLY: return "ROAMING (SMS only)";
        case RegistrationStatus::SEARCHING: return "SEARCHING";
        default: return "ERROR";
    }
}