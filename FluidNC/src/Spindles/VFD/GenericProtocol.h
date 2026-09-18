// Copyright (c) 2024 -	Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "VFDProtocol.h"
#include <string_view>

namespace Spindles {
    class VFDSpindle;

    namespace VFD {
        class GenericProtocol : public VFDProtocol, Configuration::Configurable {
        public:
            // model defaults to "" (config-driven via the `model:` YAML key); a subclass
            // (e.g. DeltaMS300) can pass its name directly so afterParse() picks up its
            // defaults from VFDtypes[] without needing a `model:` key in the YAML.
            GenericProtocol(const char* model = nullptr) {
                if (model) {
                    _model = model;
                }
            }

        private:
            void scale(uint32_t& n, std::string_view scale_str, uint32_t maxRPM);
            bool set_data(std::string_view token, std::basic_string_view<uint8_t>& response_view, const char* name, uint32_t& data);

        protected:
            void direction_command(SpindleState mode, ModbusCommand& data) override;
            void set_speed_command(uint32_t dev_speed, ModbusCommand& data) override;

            response_parser initialization_sequence(int index, ModbusCommand& data, VFDSpindle* vfd) override;
            response_parser get_current_speed(ModbusCommand& data) override;
            response_parser get_output_power(ModbusCommand& data) override;
            response_parser get_current_direction(ModbusCommand& data) override { return nullptr; };
            response_parser get_status_ok(ModbusCommand& data) override { return nullptr; }
            response_parser get_status(ModbusCommand& data);
            


            std::string _cw_cmd;
            std::string _ccw_cmd;
            std::string _off_cmd;
            std::string _set_rpm_cmd;
            std::string _get_min_rpm_cmd;
            std::string _get_max_rpm_cmd;
            std::string _get_rpm_cmd;
            std::string _get_power_cmd;      
            std::string _get_status_cmd;
    
            bool use_delay_settings() const override { return _get_rpm_cmd.empty(); }
            bool safety_polling() const override { return false; }

        private:
            std::string _model;  // VFD Model name
            uint32_t*   _response_data;
            uint32_t    _minRPM = 0xffffffff;
            uint32_t    _maxRPM = 0xffffffff;
            uint32_t    _output_power = 0;
            uint32_t    _drive_status = 0;

            // "Last known good" fallback values used by parser() when a reading looks
            // implausible (garbled frame that still passed the CRC check). Per-instance
            // on purpose, so that two VFDs of this type running concurrently don't share
            // state with each other.
            uint32_t _last_good_speed  = 0;
            uint32_t _last_good_power  = 0;
            uint32_t _last_good_status = 0;

            VFDSpindle* spindle;

            bool        parser(const uint8_t* response, VFDSpindle* spindle, GenericProtocol* protocol);
            void        send_vfd_command(const std::string cmd, ModbusCommand& data, uint32_t out);
            std::string _response_format;
            void        setup_speeds(VFDSpindle* vfd);

        public:
            void afterParse() override;            
            uint32_t get_output_power_value() const {
                return _output_power;
            }
            uint32_t get_drive_status_value() const {
                return _drive_status;
            }
            // Last genuinely-measured speed (see the comment on _last_good_speed above) -
            // unlike VFDSpindle::_sync_dev_speed, this is never force-reset when a new
            // speed is commanded, so it holds steady at the last real reading instead of
            // jumping to a "no data yet" sentinel during the brief window before the next
            // reading arrives.
            uint32_t get_current_speed_value() const override { return _last_good_speed; }
            void group(Configuration::HandlerBase& handler) override {
                handler.item("model", _model);
                handler.item("min_RPM", _minRPM);
                handler.item("max_RPM", _maxRPM);
                handler.item("cw_cmd", _cw_cmd);
                handler.item("ccw_cmd", _ccw_cmd);
                handler.item("off_cmd", _off_cmd);
                handler.item("set_rpm_cmd", _set_rpm_cmd);
                handler.item("get_min_rpm_cmd", _get_min_rpm_cmd);
                handler.item("get_max_rpm_cmd", _get_max_rpm_cmd);
                handler.item("get_rpm_cmd", _get_rpm_cmd);
                handler.item("get_power_cmd", _get_power_cmd);
                handler.item("get_status_cmd", _get_status_cmd);
            }
        };

    }
}
