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

#include "maxon_epos_ethercat_sdk/Statusword.hpp"

namespace maxon {
std::ostream& operator<<(std::ostream& os, const Statusword& statusword) {
  using std::setfill;
  using std::setw;
  std::string driveStateString = statusword.getDriveStateString();
  int gapSize2 = driveStateString.size() + 1;
  if (gapSize2 < 6) {
    gapSize2 = 6;
  }
  os << std::left << std::boolalpha << setw(gapSize2 + 27) << setfill('-')
     << "|"
     << "|\n"
     << setw(gapSize2 + 27) << setfill(' ') << "| Statusword"
     << "|\n"
     << setw(gapSize2 + 27) << setfill('-') << "|"
     << "|\n"
     << setw(25) << setfill(' ') << "| Name of Bit" << setw(gapSize2 + 2)
     << "| Value"
     << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') <<

      setw(25) << "| Ready to switch on:"
     << "| " << setw(gapSize2) << statusword.readyToSwitchOn_ << "|\n"
     << setw(25) << "| Switched on:"
     << "| " << setw(gapSize2) << statusword.switchedOn_ << "|\n"
     << setw(25) << "| Operation enabled:"
     << "| " << setw(gapSize2) << statusword.operationEnabled_ << "|\n"
     << setw(25) << "| Fault:"
     << "| " << setw(gapSize2) << statusword.fault_ << "|\n"
     << setw(25) << "| Voltage enabled:"
     << "| " << setw(gapSize2) << statusword.voltageEnabled_ << "|\n"
     << setw(25) << "| Quick stop:"
     << "| " << setw(gapSize2) << statusword.quickStop_ << "|\n"
     << setw(25) << "| Switch on disabled:"
     << "| " << setw(gapSize2) << statusword.switchOnDisabled_ << "|\n"
     << setw(25) << "| Warning:"
     << "| " << setw(gapSize2) << statusword.warning_ << "|\n"
     << setw(25) << "| Target reached:"
     << "| " << setw(gapSize2) << statusword.targetReached_ << "|\n"
     << setw(25) << "| Internal limit active:"
     << "| " << setw(gapSize2) << statusword.internalLimitActive_ << "|\n"
     <<
      // setw(25)<<"| Following error:"<<"|
      // "<<setw(gapSize2)<<statusword.followingError_<<"| \n"<< // mode of
      // operation specific
      setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|\n"
     << setfill(' ') << setw(25) << "| Resulting Drive State:"
     << "| " << setw(gapSize2) << driveStateString << "|\n"
     << setw(25) << setfill('-') << "|" << setw(gapSize2 + 2) << "+"
     << "|" <<

      std::noboolalpha << std::right << setfill(' ');

  return os;
}

void Statusword::setFromRawStatusword(uint16_t status) {
  readyToSwitchOn_ = static_cast<bool>(status & 1 << (0));
  switchedOn_ = static_cast<bool>(status & 1 << (1));
  operationEnabled_ = static_cast<bool>(status & 1 << (2));
  fault_ = static_cast<bool>(status & 1 << (3));
  voltageEnabled_ = static_cast<bool>(status & 1 << (4));
  quickStop_ = static_cast<bool>(status & 1 << (5));
  switchOnDisabled_ = static_cast<bool>(status & 1 << (6));
  warning_ = static_cast<bool>(status & 1 << (7));
  remote_ = static_cast<bool>(status & 1 << (9));
  targetReached_ = static_cast<bool>(status & 1 << (10));
  internalLimitActive_ = static_cast<bool>(status & 1 << (11));
  followingError_ = static_cast<bool>(status & 1 << (13));
  // homingError_ = static_cast<bool>(status & 1 << (13));

  rawStatusword_ = status;
}

DriveState Statusword::getDriveState() const {
  DriveState driveState = DriveState::NA;

  // MAN-G-DS402 manual page 47
  if ((rawStatusword_ & 0b0000000001101111) == 0b0000000000000000) {
    driveState = DriveState::NotReadyToSwitchOn;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000001000000) {
    driveState = DriveState::SwitchOnDisabled;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000100001) {
    driveState = DriveState::ReadyToSwitchOn;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000100011) {
    driveState = DriveState::SwitchedOn;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000100111) {
    driveState = DriveState::OperationEnabled;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000000111) {
    driveState = DriveState::QuickStopActive;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000001111) {
    driveState = DriveState::FaultReactionActive;
  } else if ((rawStatusword_ & 0b0000000001101111) == 0b00000000000001000) {
    driveState = DriveState::Fault;
  }

  return driveState;
}
std::string Statusword::getDriveStateString() const {
  DriveState driveState = getDriveState();
  switch (driveState) {
    case DriveState::SwitchOnDisabled:
      return "switch on disabled";
      break;
    case DriveState::ReadyToSwitchOn:
      return "ready to switch on";
      break;
    case DriveState::SwitchedOn:
      return "switched on";
      break;
    case DriveState::OperationEnabled:
      return "operation enabled";
      break;
    case DriveState::QuickStopActive:
      return "quick stop active";
      break;
    case DriveState::Fault:
      return "fault_";
      break;
    case DriveState::FaultReactionActive:
      return "fault_ reaction active";
    case DriveState::NotReadyToSwitchOn:
      return "not ready to switch on";
    default:
      return "N/A";
  }
}

}  // namespace maxon
