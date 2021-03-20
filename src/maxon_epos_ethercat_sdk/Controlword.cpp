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

#include <iomanip>

#include "maxon_epos_ethercat_sdk/Controlword.hpp"

namespace maxon
{
std::ostream& operator<<(std::ostream& os, const Controlword& controlword)
{
  using std::setfill;
  using std::setw;

  os << std::left << std::boolalpha << setw(40) << setfill('-') << "|"
     << "|\n"
     << setw(40) << setfill(' ') << "| Controlword"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| Name"
     << "| Value | Mode |"
     << "\n"
     << setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|"
     << "\n"
     << setw(25) << setfill(' ') << "| switch on:"
     << "| " << setw(6) << controlword.switchOn_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable voltage:"
     << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| quick stop:"
     << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable operation:"
     << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| new set point:"
     << "| " << setw(6) << controlword.newSetPoint_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| start homing:"
     << "| " << setw(6) << controlword.homingOperationStart_ << "|" << setw(6) << " hm"
     << "|\n"
     << setw(25) << setfill(' ') << "| change set:"
     << "| " << setw(6) << controlword.changeSetImmediately_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| relative_:"
     << "| " << setw(6) << controlword.relative_ << "|" << setw(6) << " pp "
     << "|\n"
     << setw(25) << setfill(' ') << "| fault_ reset:"
     << "| " << setw(6) << controlword.faultReset_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| halt_:"
     << "| " << setw(6) << controlword.halt_ << "|" << setw(6)
     << " all "
     //  << "|\n"
     //  << setw(25) << setfill(' ') << "| endless movement_:"
     //  << "| " << setw(6) << controlword.endlessMovement_ << "|" << setw(6) <<
     //  "  pp"
     << "|\n"
     <<

      setw(25) << setfill('-') << "|" << setw(8) << "+" << setw(7) << "+"
     << "|" << std::right << std::noboolalpha;
  return os;
}

uint16_t Controlword::getRawControlword()
{
  uint16_t rawControlword = 0;

  if (switchOn_)
  {
    rawControlword |= (1 << 0);
  }
  if (enableVoltage_)
  {
    rawControlword |= (1 << 1);
  }
  if (quickStop_)
  {
    rawControlword |= (1 << 2);
  }
  if (enableOperation_)
  {
    rawControlword |= (1 << 3);
  }
  if (faultReset_)
  {
    rawControlword |= (1 << 7);
  }
  if (halt_)
  {
    rawControlword |= (1 << 8);
  }
  // if (endlessMovement_) {
  //   rawControlword |= (1 << 15);
  // }

  return rawControlword;
}

void Controlword::setStateTransition2()
{
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
  std::cout << "State Transition 2" << std::endl;
}

void Controlword::setStateTransition3()
{
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  std::cout << "State Transition 3" << std::endl;
}

void Controlword::setStateTransition4()
{
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
  std::cout << "State Transition 4" << std::endl;
}

void Controlword::setStateTransition5()
{
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  std::cout << "State Transition 5" << std::endl;
}

void Controlword::setStateTransition6()
{
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
  std::cout << "State Transition 6" << std::endl;
}

void Controlword::setStateTransition7()
{
  setAllFalse();
  std::cout << "State Transition 7" << std::endl;
}

void Controlword::setStateTransition8()
{
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
  std::cout << "State Transition 8" << std::endl;
}

void Controlword::setStateTransition9()
{
  setAllFalse();
  std::cout << "State Transition 9" << std::endl;
}

void Controlword::setStateTransition10()
{
  setAllFalse();
  std::cout << "State Transition 10" << std::endl;
}

void Controlword::setStateTransition11()
{
  setAllFalse();
  enableVoltage_ = true;
  std::cout << "State Transition 11" << std::endl;
}

void Controlword::setStateTransition12()
{
  setAllFalse();
  std::cout << "State Transition 12" << std::endl;
}

void Controlword::setStateTransition15()
{
  setAllFalse();
  faultReset_ = true;
  std::cout << "State Transition 15" << std::endl;
}

void Controlword::setStateTransition16()
{
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
  std::cout << "State Transition 16" << std::endl;
}

void Controlword::setAllFalse()
{
  switchOn_ = false;
  enableVoltage_ = false;
  quickStop_ = false;
  enableOperation_ = false;
  newSetPoint_ = false;
  homingOperationStart_ = false;
  changeSetImmediately_ = false;
  relative_ = false;
  faultReset_ = false;
  halt_ = false;
  // endlessMovement_ = false;
}

void Controlword::setInit()
{
  setStateTransition2();
}

}  // namespace maxon
