// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cstdint>

#include "../SpindleDatatypes.h"
#include "../Machine/Macros.h"

#include "../Configuration/Configurable.h"
#include "../Configuration/GenericFactory.h"

#include "src/GCode.h"  // MaxToolNumber
#include "src/Module.h"
#include "src/ToolChangers/atc.h"

// ===============  No floats! ===========================
// ================ NO FLOATS! ==========================

namespace Spindles {
    class Spindle;
    using SpindleList = std::vector<Spindle*>;

    // This is the base class. Do not use this as your spindle
    class Spindle : public Configuration::Configurable {
    private:
        const char* _name;
        std::string _atc_info = "";

        // _zero_speed_with_disable forces speed to 0 when disabled
        bool _zero_speed_with_disable = false;

    protected:
        ATCs::ATC* _atc       = nullptr;
        uint32_t   _last_tool = 0;

    public:
        Spindle(const char* name) : _name(name) {}

        Spindle(const Spindle&)            = delete;
        Spindle(Spindle&&)                 = delete;
        Spindle& operator=(const Spindle&) = delete;
        Spindle& operator=(Spindle&&)      = delete;

        bool     _defaultedSpeeds;
        uint32_t offSpeed() { return _speeds[0].offset; }
        uint32_t maxSpeed();
        uint32_t mapSpeed(SpindleState state, SpindleSpeed speed);
        void     setupSpeeds(uint32_t max_dev_speed);
        void     shelfSpeeds(SpindleSpeed min, SpindleSpeed max);
        void     linearSpeeds(SpindleSpeed maxSpeed, float maxPercent);

        static void switchSpindle(uint32_t new_tool, SpindleList spindles, Spindle*& spindle, bool& stop_spindle, bool& new_spindle);

        void         spindleDelay(SpindleState state, SpindleSpeed speed);
        virtual void init() = 0;  // not in constructor because this also gets called when $$ settings change
        virtual void init_atc();
        std::string  atc_info() { return _atc_info; };

        // Used by Protocol.cpp to restore the state during a restart
        virtual void    setState(SpindleState state, uint32_t speed) = 0;
        SpindleState    get_state() { return _current_state; };
        void            stop() { setState(SpindleState::Disable, 0); }
        virtual void    config_message() = 0;
        virtual bool    isRateAdjusted();
        virtual bool    use_delay_settings() const { return true; }
        virtual uint8_t get_current_tool_num() { return _current_tool; }
        virtual bool    tool_change(uint32_t tool_number, bool pre_select, bool set_tool);

        virtual void setSpeedfromISR(uint32_t dev_speed) = 0;

        void spinDown() { setState(SpindleState::Disable, 0); }

        bool                  is_reversable;
        volatile SpindleState _current_state = SpindleState::Unknown;
        volatile SpindleSpeed _current_speed = 0;

        // scaler units are ms/rpm * 2^16.
        // The computation is deltaRPM * scaler >> 16
        uint32_t _spinup_ms   = 0;
        uint32_t _spindown_ms = 0;

        int _tool = -1;

        std::vector<Configuration::speedEntry> _speeds;

        bool _off_on_alarm = false;

        Macro       _m6_macro;
        std::string _atc_name = "";

        // Name is required for the configuration factory to work.
        const char* name() { return _name; }

        // Configuration handlers:
        void validate() override;
        void afterParse() override;

        void group(Configuration::HandlerBase& handler) override {
            if (use_delay_settings()) {
                handler.item("spinup_ms", _spinup_ms, 0, 60000);
                handler.item("spindown_ms", _spindown_ms, 0, 60000);
            }
            handler.item("tool_num", _tool, 0, MaxToolNumber);
            handler.item("speed_map", _speeds);
            handler.item("off_on_alarm", _off_on_alarm);
            handler.item("atc", _atc_name);
            handler.item("m6_macro", _m6_macro);
            handler.item("s0_with_disable", _zero_speed_with_disable);
        }

        // Virtual base classes require a virtual destructor.
        virtual ~Spindle() {}

    protected:
        uint8_t _current_tool = 0;
    };

    using SpindleFactory = Configuration::GenericFactory<Spindle>;
}
extern Spindles::Spindle* spindle;
