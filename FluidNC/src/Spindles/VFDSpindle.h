// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "Spindle.h"
#include "../Types.h"
#include "VFD/VFDProtocol.h"
#include "../Uart.h"

namespace Spindles {
    extern Uart _uart;

    namespace VFD {
        // VFDProtocol resides in a separate class because it doesn't need to be in IRAM. This contains all the
        // VFD specific code, which is called from a separate task.
        class VFDProtocol;
    }

    // VFD base class. Called by the stepper engine. Normally you don't want to touch this.
    class VFDSpindle : public Spindle {
    private:
        friend class Spindles::VFD::VFDProtocol;

        VFD::VFDProtocol* detail_ = nullptr;

        int32_t  _current_dev_speed   = -1;
        uint32_t _last_speed          = 0;
        Percent  _last_override_value = 100;  // no override is 100 percent

        void set_mode(SpindleState mode, bool critical);

    protected:
        // The constructor sets these
        int      _uart_num  = -1;
        Uart*    _uart      = nullptr;
        uint8_t  _modbus_id = 1;
        uint8_t  _debug     = 0;
        uint32_t _poll_ms   = 250;
        uint32_t _retries   = 5;

        // Speed-sync wait tuning (see setState()). Both are wall-clock timeouts, not a
        // count of polls, so they don't need to be recalculated if poll_ms changes.
        // sync_timeout_ms: how long setState() will wait with *zero* new speed readings
        //   arriving before concluding the VFD is unresponsive and raising
        //   ExecAlarm::SpindleControl. Needs enough margin to absorb the RS485 task being
        //   briefly busy with other commands (e.g. a rapid CW/CCW reversal queues a mode
        //   change ahead of the next speed poll) and any real comms hiccup around a
        //   direction-reversal relay, on top of normal poll_ms/retries latency.
        // sync_settle_ms: how long the reported speed is allowed to sit unchanged (e.g.
        //   parked at 0 during a VFD's direction-reversal dead time) before setState()
        //   gives up waiting and moves on *without* alarming - this is the expected,
        //   non-error case of "the VFD is legitimately not changing speed right now".
        // Both need to be longer than the spindle's own accel/decel time for a full
        // direction reversal (decel to 0 + relay dead time + accel back up) or every
        // reversal will look like a stall.
        uint32_t _sync_timeout_ms = 3000;
        uint32_t _sync_settle_ms  = 3000;

        // Idle-time (not actively mid-sync) polling cadence for output power and actual
        // speed - see VFDProtocol.cpp. Each is its own rate limit, but both still have to
        // share the single Modbus transaction that vfd_cmd_task's loop can do per
        // iteration, so neither can effectively refresh faster than poll_ms allows - lower
        // poll_ms too if you want both of these to actually keep up with a low value here.
        uint32_t _power_poll_ms = 100;
        uint32_t _speed_poll_ms = 100;

        void setSpeed(uint32_t dev_speed);

        volatile bool _syncing;

    public:
        VFDSpindle(const char* name, VFD::VFDProtocol* detail) : Spindle(name), detail_(detail) {}
        VFDSpindle(const VFDSpindle&)            = delete;
        VFDSpindle(VFDSpindle&&)                 = delete;
        VFDSpindle& operator=(const VFDSpindle&) = delete;
        VFDSpindle& operator=(VFDSpindle&&)      = delete;

        void init();
        void config_message();
        void setState(SpindleState state, SpindleSpeed speed);
        void setSpeedfromISR(uint32_t dev_speed) override;

        uint32_t output_power() const { return detail_ ? detail_->get_output_power_value() : 0;
        }
        // Deliberately NOT _sync_dev_speed: that member is setState()'s own sync-wait
        // bookkeeping and gets force-reset to UINT32_MAX (a "no reading yet, definitely
        // out of range" sentinel) every time a new speed is commanded, even for a small
        // bump like 6100->6200. get_current_speed_value() is a separate "last genuinely
        // measured reading" that only updates from real data, so a small speed change
        // shows the previous real RPM (still spinning) instead of a bogus 0/sentinel
        // flash - see GenericProtocol::_last_good_speed. Kept live outside of
        // setState()'s wait loop too, by the periodic get_current_speed() poll in
        // VFDProtocol.cpp.
        uint32_t actual_rpm() const override { return detail_ ? detail_->get_current_speed_value() : 0; }

        // volatile uint32_t _sync_dev_speed;
        uint32_t _sync_dev_speed;

        // Speed tolerance (in RPM/device units) for setState()'s "close enough to target"
        // check. 0 means "not set via YAML"; each protocol's setup_speeds()-equivalent
        // picks its own computed default in that case, but leaves an explicit `slop_rpm:`
        // YAML value alone.
        SpindleSpeed _slop = 0;

        // Optional additional tolerance as a percentage of the *target* speed (YAML
        // `slop_percent`, 0-100, default 0 = disabled). setState() uses whichever of
        // _slop or (dev_speed * _slop_percent / 100) is larger. A fixed RPM slop alone
        // can be too tight under cutting load (motor slip pulls the real RPM away from
        // target, more so under torque) or proportionally too tight/loose depending on
        // where in the speed range you're commanding - a percentage scales with it.
        uint32_t _slop_percent = 0;

        // Configuration handlers:
        void validate() override;
        void afterParse() override;
        void group(Configuration::HandlerBase& handler) override;

        virtual ~VFDSpindle() {}
    };
}
