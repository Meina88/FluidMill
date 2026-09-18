// Copyright (c) 2020 -	Bart Dring
// Copyright (c) 2020 -	Stefan de Bruijn
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is for a VFD based spindles via RS485 Modbus. The details of the 
    VFD protocol heavily depend on the VFD in question here. We have some 
    implementations, but if yours is not here, the place to start is the 
    manual. This VFD class implements the modbus functionality.

                         WARNING!!!!
    VFDs are very dangerous. They have high voltages and are very powerful
    Remove power before changing bits.

    TODO:
      - We can report spindle_state and rpm better with VFD's that support 
        either mode, register RPM or actual RPM.
      - Destructor should break down the task.

*/
#include "VFDSpindle.h"
#include "VFD/VFDProtocol.h"

#include "../Machine/MachineConfig.h"
#include "../Protocol.h"  // rtAlarm
#include "../Report.h"    // hex message
#include "../Configuration/HandlerType.h"
#include "../Platform.h"

#include <freertos/task.h>
#include <freertos/queue.h>
#include <atomic>

namespace Spindles {
    // number of commands that can be queued up.
    const int VFD_RS485_QUEUE_SIZE = 10;

    // ================== Class methods ==================================

    void VFDSpindle::init() {
        _sync_dev_speed = 0;
        _syncing        = false;

        // The following lets you have either a uart: section below the VFD section,
        // or "uart_num: N" referring to an externally defined uartN: section - but not both
        if (_uart) {
            _uart->begin();
        } else {
            _uart = config->_uarts[_uart_num];
            if (!_uart) {
                log_error("VFDSpindle: Missing uart" << _uart_num << " section");
                return;
            }
        }

        if (_uart->setHalfDuplex()) {
            log_info("VFD: RS485 UART set half duplex failed");
            return;
        }

        // These VFDs are always reversable, but most can be set via the operator panel
        // to only allow one direction.  In principle we could check that setting and
        // automatically set is_reversable.
        is_reversable = true;

        _current_state = SpindleState::Disable;

        // Initialization is complete, so now it's okay to run the queue task:
        if (!VFD::VFDProtocol::vfd_cmd_queue) {  // init can happen many times, we only want to start one task
            VFD::VFDProtocol::vfd_cmd_queue = xQueueCreate(VFD_RS485_QUEUE_SIZE, sizeof(VFD::VFDProtocol::VFDaction));
            // Depth 1: "latest wins" for speed setpoints - see VFDSpeedSetpoint in VFDProtocol.h.
            VFD::VFDProtocol::vfd_speed_setpoint_queue = xQueueCreate(1, sizeof(VFD::VFDProtocol::VFDSpeedSetpoint));
            VFD::VFDProtocol::vfd_speed_queue          = xQueueCreate(VFD_RS485_QUEUE_SIZE, sizeof(uint32_t));
            xTaskCreatePinnedToCore(VFD::VFDProtocol::vfd_cmd_task,  // task
                                    "vfd_cmdTaskHandle",             // name for task
                                    2048,                            // size of task stack
                                    this,                            // parameters
                                    1,                               // priority
                                    &VFD::VFDProtocol::vfd_cmdTaskHandle,
                                    SUPPORT_TASK_CORE  // core
            );
        }

        init_atc();
        config_message();

        set_mode(SpindleState::Disable, true);
    }

    void VFDSpindle::config_message() {
        std::string usage(" Spindle");
        usage += atc_info();
        _uart->config_message(name(), usage.c_str());
    }

    void VFDSpindle::set_mode(SpindleState mode, bool critical) {
        _last_override_value = sys.spindle_speed_ovr;  // sync these on mode changes
        if (VFD::VFDProtocol::vfd_cmd_queue) {
            VFD::VFDProtocol::VFDaction action;
            action.action   = VFD::VFDProtocol::actionSetMode;
            action.arg      = uint32_t(mode);
            action.critical = critical;
            if (xQueueSend(VFD::VFDProtocol::vfd_cmd_queue, &action, 0) != pdTRUE) {
                log_info("VFD Queue Full");
            }
        }
    }

    void VFDSpindle::setState(SpindleState state, SpindleSpeed speed) {
        log_debug(name() << ": setState:" << uint8_t(state) << " SpindleSpeed:" << speed);
        if (sys.abort) {
            return;  // Block during abort.
        }

        bool critical = (state_is(State::Cycle) || state != SpindleState::Disable);

        uint32_t dev_speed = mapSpeed(state, speed);

        if (_current_state != state) {
            // Changing state
            set_mode(state, critical);  // critical if we are in a job

            setSpeed(dev_speed);
        } else {
            // Not changing state
            if (_current_dev_speed != dev_speed) {
                // Changing speed
                setSpeed(dev_speed);
            }
        }
        if (detail_->use_delay_settings()) {
            spindleDelay(state, speed);
        } else {
            // _sync_dev_speed is fed by VFDProtocol's RS485 task, which pushes each real
            // speed reading it parses onto vfd_speed_queue (see VFDProtocol.cpp). We block
            // on that queue with a hard timeout instead of polling _sync_dev_speed in a
            // loop with a soft "unchanged" counter - the old counter-based timeout could be
            // (and was) silently defeated by anything that forced a `continue` every
            // iteration, leaving no way to ever detect a spindle that never reaches speed.

            _syncing = true;  // poll for speed

            // Whichever tolerance is wider wins: a fixed RPM floor (_slop) so low speeds
            // still get a sane minimum window, or a percentage of the target (useful
            // under cutting load, where motor slip can keep the real RPM a fair way off
            // target indefinitely - see slop_percent in VFDSpindle.h).
            uint32_t percentSlop   = uint32_t(uint64_t(dev_speed) * _slop_percent / 100);
            uint32_t effectiveSlop = percentSlop > _slop ? percentSlop : _slop;
            auto     minSpeedAllowed = dev_speed > effectiveSlop ? (dev_speed - effectiveSlop) : 0;
            auto     maxSpeedAllowed = dev_speed + effectiveSlop;

            if (_debug > 1) {
                log_debug("Syncing speed. Requested: " << int(dev_speed) << " current:" << int(_sync_dev_speed));
            }

            uint32_t   prev_sync_speed   = _sync_dev_speed;
            TickType_t last_change_ticks = xTaskGetTickCount();

            while ((_last_override_value == sys.spindle_speed_ovr) &&
                   (_sync_dev_speed < minSpeedAllowed || _sync_dev_speed > maxSpeedAllowed)) {
                if (sys.abort) {
                    _syncing = false;
                    return;
                }
                if (!xQueueReceive(VFD::VFDProtocol::vfd_speed_queue, &_sync_dev_speed, pdMS_TO_TICKS(_sync_timeout_ms))) {
                    // No speed reading arrived within sync_timeout_ms: the VFD never reported
                    // reaching the requested speed (unresponsive, alarmed, or genuinely can't
                    // get there). This is the safety net that used to be silently disabled -
                    // restore it. If this fires during routine direction reversals, increase
                    // sync_timeout_ms in the YAML - the default may be too tight for a spindle
                    // with a slow decel/reversal sequence.
                    mc_critical(ExecAlarm::SpindleControl);
                    log_error(name() << ": spindle did not reach device units " << dev_speed << ". Reported value is "
                                      << _sync_dev_speed);
                    _syncing = false;
                    return;
                }
                if (_sync_dev_speed == prev_sync_speed) {
                    if ((xTaskGetTickCount() - last_change_ticks) >= pdMS_TO_TICKS(_sync_settle_ms)) {
                        break;  // speed reported unchanged for sync_settle_ms straight; give up
                                 // waiting (not an error - e.g. the VFD legitimately parked at
                                 // 0 during a direction-reversal dead time)
                    }
                } else {
                    last_change_ticks = xTaskGetTickCount();
                    prev_sync_speed   = _sync_dev_speed;
                }
            }

            _last_override_value = sys.spindle_speed_ovr;

            if (_debug > 1) {
                log_debug("Synced speed. Requested:" << int(dev_speed) << " current:" << int(_sync_dev_speed));
            }

            _syncing = false;
            // spindleDelay() sets these when it is used
            _current_state = state;
            _current_speed = speed;
        }
    }

    void IRAM_ATTR VFDSpindle::setSpeedfromISR(uint32_t dev_speed) {
        if (_current_dev_speed == dev_speed || _last_speed == dev_speed) {
            return;
        }

        _last_speed = dev_speed;

        if (VFD::VFDProtocol::vfd_speed_setpoint_queue) {
            VFD::VFDProtocol::VFDSpeedSetpoint sp;
            sp.arg      = dev_speed;
            sp.critical = (dev_speed == 0);
            // xQueueOverwriteFromISR always succeeds on a depth-1 queue: it replaces
            // whatever setpoint hadn't been sent yet instead of queuing behind it.
            xQueueOverwriteFromISR(VFD::VFDProtocol::vfd_speed_setpoint_queue, &sp, NULL);
        }
    }

    void VFDSpindle::setSpeed(uint32_t dev_speed) {
        if (VFD::VFDProtocol::vfd_speed_setpoint_queue) {
            VFD::VFDProtocol::VFDSpeedSetpoint sp;
            sp.arg      = dev_speed;
            sp.critical = dev_speed == 0;
            // "Latest wins": a burst of speed changes (e.g. several rapid spindle-speed
            // overrides) collapses to a single Modbus write of the final target instead of
            // one write per intermediate value - xQueueOverwrite() always succeeds on a
            // depth-1 queue, replacing whatever hadn't been sent yet.
            xQueueOverwrite(VFD::VFDProtocol::vfd_speed_setpoint_queue, &sp);
        }
    }

    void VFDSpindle::validate() {
        Spindle::validate();
        Assert(_uart != nullptr || _uart_num != -1, "VFD: missing UART configuration");
    }

    void VFDSpindle::afterParse() {
        detail_->afterParse();
    }

    void VFDSpindle::group(Configuration::HandlerBase& handler) {
        if (handler.handlerType() == Configuration::HandlerType::Generator) {
            if (_uart_num == -1) {
                handler.section("uart", _uart, 1);
            } else {
                handler.item("uart_num", _uart_num);
            }
        } else {
            handler.section("uart", _uart, 1);
            handler.item("uart_num", _uart_num);
        }
        handler.item("modbus_id", _modbus_id, 0, 247);  // per https://modbus.org/docs/PI_MBUS_300.pdf
        handler.item("debug", _debug, 0, 5);
        handler.item("poll_ms", _poll_ms, 50, 20000);
        handler.item("retries", _retries);
        // See the member comments in VFDSpindle.h for what these tune.
        handler.item("sync_timeout_ms", _sync_timeout_ms, 100, 60000);
        handler.item("sync_settle_ms", _sync_settle_ms, 100, 60000);
        handler.item("slop_rpm", _slop, 0, 1000000);
        handler.item("slop_percent", _slop_percent, 0, 100);
        handler.item("power_poll_ms", _power_poll_ms, 20, 60000);
        handler.item("speed_poll_ms", _speed_poll_ms, 20, 60000);

        Spindle::group(handler);
        detail_->group(handler);
    }
}
