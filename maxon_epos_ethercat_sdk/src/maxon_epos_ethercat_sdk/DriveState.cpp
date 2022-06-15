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

#include "maxon_epos_ethercat_sdk/DriveState.hpp"

std::ostream& operator<<(std::ostream& os,
                         const maxon::DriveState& driveState) {
  switch (driveState) {
    case maxon::DriveState::NotReadyToSwitchOn:
      os << "NotReadyToSwitchOn";
      break;
    case maxon::DriveState::SwitchOnDisabled:
      os << "SwitchOnDisabled";
      break;
    case maxon::DriveState::ReadyToSwitchOn:
      os << "ReadyToSwitchOn";
      break;
    case maxon::DriveState::SwitchedOn:
      os << "SwitchedOn";
      break;
    case maxon::DriveState::OperationEnabled:
      os << "OperationEnabled";
      break;
    case maxon::DriveState::QuickStopActive:
      os << "QuickStopActive";
      break;
    case maxon::DriveState::FaultReactionActive:
      os << "FaultReactionActive";
      break;
    case maxon::DriveState::Fault:
      os << "Fault";
      break;
    case maxon::DriveState::NA:
      os << "NA";
      break;
  }
  return os;
}
