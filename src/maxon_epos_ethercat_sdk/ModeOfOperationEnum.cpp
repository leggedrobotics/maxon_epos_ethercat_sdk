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

#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"
#include <unordered_map>

std::ostream& operator << (std::ostream& os, const maxon::ModeOfOperationEnum modeOfOperation) {
  std::unordered_map<maxon::ModeOfOperationEnum, std::string> mode2strMap = {
    {maxon::ModeOfOperationEnum::NA, "NA"},
    {maxon::ModeOfOperationEnum::ProfiledPositionMode, "ProfiledPositionMode"},
    {maxon::ModeOfOperationEnum::ProfiledVelocityMode, "ProfiledVelocityMode"},
    {maxon::ModeOfOperationEnum::HomingMode, "HomingMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode, "CyclicSynchronousPositionMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode, "CyclicSynchronousVelocityMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode, "CyclicSynchronousTorqueMode"},
  };
  os << mode2strMap[modeOfOperation];
  return os;
}
