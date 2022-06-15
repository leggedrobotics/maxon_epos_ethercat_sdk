// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#include <iomanip>

#include "maxon_epos_ethercat_sdk/Controlword.hpp"

namespace maxon {
std::ostream& operator<<(std::ostream& os, const Controlword& controlword) {
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
     << "| " << setw(6) << controlword.enableVoltage_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| quick stop:"
     << "| " << setw(6) << controlword.quickStop_ << "|" << setw(6) << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| enable operation:"
     << "| " << setw(6) << controlword.enableOperation_ << "|" << setw(6)
     << " all"
     << "|\n"
     << setw(25) << setfill(' ') << "| new set point:"
     << "| " << setw(6) << controlword.newSetPoint_ << "|" << setw(6) << " pp"
     << "|\n"
     << setw(25) << setfill(' ') << "| start homing:"
     << "| " << setw(6) << controlword.homingOperationStart_ << "|" << setw(6)
     << " hm"
     << "|\n"
     << setw(25) << setfill(' ') << "| change set:"
     << "| " << setw(6) << controlword.changeSetImmediately_ << "|" << setw(6)
     << " pp"
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

uint16_t Controlword::getRawControlword() {
  uint16_t rawControlword = 0;

  if (switchOn_) {
    rawControlword |= (1 << 0);
  }
  if (enableVoltage_) {
    rawControlword |= (1 << 1);
  }
  if (quickStop_) {
    rawControlword |= (1 << 2);
  }
  if (enableOperation_) {
    rawControlword |= (1 << 3);
  }
  if (faultReset_) {
    rawControlword |= (1 << 7);
  }
  if (halt_) {
    rawControlword |= (1 << 8);
  }
  // if (endlessMovement_) {
  //   rawControlword |= (1 << 15);
  // }

  return rawControlword;
}

void Controlword::setStateTransition2() {
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
}

void Controlword::setStateTransition3() {
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
}

void Controlword::setStateTransition4() {
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
}

void Controlword::setStateTransition5() {
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
}

void Controlword::setStateTransition6() {
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
}

void Controlword::setStateTransition7() {
  setAllFalse();
}

void Controlword::setStateTransition8() {
  setAllFalse();
  enableVoltage_ = true;
  quickStop_ = true;
}

void Controlword::setStateTransition9() {
  setAllFalse();
}

void Controlword::setStateTransition10() {
  setAllFalse();
}

void Controlword::setStateTransition11() {
  setAllFalse();
  enableVoltage_ = true;
}

void Controlword::setStateTransition12() {
  setAllFalse();
}

void Controlword::setStateTransition15() {
  setAllFalse();
  faultReset_ = true;
}

void Controlword::setStateTransition16() {
  setAllFalse();
  switchOn_ = true;
  enableVoltage_ = true;
  quickStop_ = true;
  enableOperation_ = true;
}

void Controlword::setAllFalse() {
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

void Controlword::setInit() { setStateTransition2(); }

}  // namespace maxon
