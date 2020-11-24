/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
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
#include <utility>

#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"
#include "maxon_epos_ethercat_sdk/PdoTypeEnum.hpp"

namespace maxon {

class Configuration {

 public:
  ModeOfOperationEnum modeOfOperationEnum{ModeOfOperationEnum::NA};
  RxPdoTypeEnum rxPdoTypeEnum{RxPdoTypeEnum::NA};
  TxPdoTypeEnum txPdoTypeEnum{TxPdoTypeEnum::NA};
  unsigned int configRunSdoVerifyTimeout{20000};
  bool printDebugMessages{true};
  unsigned int driveStateChangeMinTimeout{20000};
  unsigned int minNumberOfSuccessfulTargetStateReadings{10};
  unsigned int driveStateChangeMaxTimeout{300000};
  bool forceAppendEqualError{true};
  bool forceAppendEqualFault{false};
  unsigned int errorStorageCapacity{100};
  unsigned int faultStorageCapacity{100};
  int32_t positionEncoderResolution{1};
  bool useRawCommands{false};
  double gearRatio{1};
  double motorConstant{1};
  double motorRatedCurrentA{0};
  double maxCurrentA{0};
  bool useMultipleModeOfOperations{false};

 public:
  // stream operator
  friend std::ostream& operator<<(std::ostream& os, const Configuration& configuration);
};

}  // namespace maxon
