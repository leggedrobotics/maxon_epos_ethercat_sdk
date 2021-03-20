/*
** Copyright (2020-2021) Robotics Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** This file is part of the maxon_epos_ethercat_sdk.
** The maxon_epos_ethercat_sdk is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The maxon_epos_ethercat_sdk is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the maxon_epos_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstdint>
#include <iostream>

namespace maxon
{
struct Controlword
{
  bool switchOn_{ false };              // bit 0
  bool enableVoltage_{ false };         // bit 1
  bool quickStop_{ false };             // bit 2
  bool enableOperation_{ false };       // bit 3
  bool newSetPoint_{ false };           // bit 4 profiled position mode
  bool homingOperationStart_{ false };  // bit 4 homing mode
  bool changeSetImmediately_{ false };  // bit 5 profiled position mode
  bool relative_{ false };              // bit 6 profiled position mode
  bool faultReset_{ false };            // bit 7
  bool halt_{ false };                  // bit 8
  bool endlessMovement_{ false };       // bit 15 profiled position mode

  /*!
   * get the control word as a 16 bit unsigned integer
   * THIS DOES NOT RESPECT THE MODE SPECIFIC OPTIONS!
   * The usually used cyclic modes do not need mode specific options.
   * @return	the raw controlword
   */
  uint16_t getRawControlword();

  /*!
   * State transition 2
   * This corresponds to a "shutdown" Controlword
   */
  void setStateTransition2();

  /*!
   * State transition 3
   * This corresponds to a "switch on" Controlword
   * Initialize current sensor. Current offset calibration.
   */
  void setStateTransition3();

  /*!
   * State transition 4
   * This corresponds to a "enable operation" Controlword
   * Enable drive function (enable current controller and, if needed, position or velocity controller)
   */
  void setStateTransition4();

  /*!
   * State transition 5
   * This corresponds to a "disable operation" Controlword
   * Stop movement according to "Disable operation option code". Disable drive function.
   */
  void setStateTransition5();

  /*!
   * State transition 6
   * This corresponds to a "shutdown" Controlword
   * Disable power section
   */
  void setStateTransition6();

  /*!
   * State transition 7
   * This corresponds to a "quick stop" or "disable voltage" Controlword
   */
  void setStateTransition7();

  /*!
   * State transition 8
   * This corresponds to a "shutdown" Controlword
   * Stop movement according to "Shutdown option code". Disable drive function and power section.
   */
  void setStateTransition8();

  /*!
   * State transition 9
   * This corresponds to a "disable voltage" Controlword
   * Stop movement according to "Shutdown option code". Disable drive function and power section.
   */
  void setStateTransition9();

  /*!
   * State transition 10
   * This corresponds to a "quick stop" or "disable voltage" Controlword
   */
  void setStateTransition10();

  /*!
   * State transition 11
   * This corresponds to a "quick stop" Controlword
   * Stop movement according to "Quick stop option code"
   */
  void setStateTransition11();

  /*!
   * State transition 12
   * This corresponds to a "disable voltage" Controlword
   * Disable drive function and power section
   */
  void setStateTransition12();

  /*!
   * State transition 13
   * A fault has occurred
   * Start fault reaction
   */
  void setStateTransition13();

  /*!
   * State transition 14
   * The fault reaction is completed
   * Disable drive function and power section
   */
  void setStateTransition14();

  /*!
   * State transition 15
   * This corresponds to a "fault reset" Controlword
   * Reset fault condition if no fault is present
   */
  void setStateTransition15();

  /*!
   * State transition 16
   * This corresponds to a "enable operation" Controlword
   */
  void setStateTransition16();

  /*!
   * Sets all bools of this struct to false
   */
  void setAllFalse();

  /*!
   * goes to the init state
   * Alias for state transition 2
   */
  void setInit();

  friend std::ostream& operator<<(std::ostream& os, const Controlword& controlword);
};

}  // namespace maxon
