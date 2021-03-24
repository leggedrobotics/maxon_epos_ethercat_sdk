// clang-format off
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
// clang-format on

#include "maxon_epos_ethercat_sdk/DriveState.hpp"

std::ostream& operator<<(std::ostream& os, const maxon::DriveState& driveState)
{
  switch (driveState)
  {
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
