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

#include "maxon_epos_ethercat_sdk/Configuration.hpp"

#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>


namespace maxon
{
std::string modeOfOperationString(ModeOfOperationEnum modeOfOperation_)
{
  switch (modeOfOperation_)
  {
    case ModeOfOperationEnum::ProfiledPositionMode:
      return "Profiled Position Mode";
    case ModeOfOperationEnum::ProfiledVelocityMode:
      return "Profiled Velocity Mode";
    case ModeOfOperationEnum::HomingMode:
      return "Homing Mode";
    case ModeOfOperationEnum::CyclicSynchronousPositionMode:
      return "Cyclic Synchronous Position Mode";
    case ModeOfOperationEnum::CyclicSynchronousVelocityMode:
      return "Cyclic Synchronous Velocity Mode";
    case ModeOfOperationEnum::CyclicSynchronousTorqueMode:
      return "Cyclic Synchronous Torque Mode";
    default:
      return "Unsupported Mode of Operation";
  }
}

std::string rxPdoString(RxPdoTypeEnum rxPdo)
{
  switch (rxPdo)
  {
    case RxPdoTypeEnum::NA:
      return "NA";
    case RxPdoTypeEnum::RxPdoStandard:
      return "Rx PDO Standard";
    case RxPdoTypeEnum::RxPdoCSP:
      return "Rx PDO CSP";
    case RxPdoTypeEnum::RxPdoCST:
      return "Rx PDO CST";
    case RxPdoTypeEnum::RxPdoCSV:
      return "Rx PDO CSV";
    case RxPdoTypeEnum::RxPdoCSTCSP:
      return "Rx PDO CST/CSP mixed mode";
    case RxPdoTypeEnum::RxPdoCSTCSPCSV:
      return "Rx PDO CST/CSP/CSV mixed mode";
    case RxPdoTypeEnum::RxPdoPVM:
      return "Rx PDO PVM";
    default:
      return "Unsupported Type";
  }
}

std::string txPdoString(TxPdoTypeEnum txPdo)
{
  switch (txPdo)
  {
    case TxPdoTypeEnum::NA:
      return "NA";
    case TxPdoTypeEnum::TxPdoCSP:
      return "Tx PDO CSP";
    case TxPdoTypeEnum::TxPdoCST:
      return "Tx PDO CST";
    case TxPdoTypeEnum::TxPdoCSV:
      return "Tx PDO CSV";
    case TxPdoTypeEnum::TxPdoCSTCSP:
      return "Tx PDO CST/CSP mixed mode";
    case TxPdoTypeEnum::TxPdoCSTCSPCSV:
      return "Rx PDO CST/CSP/CSV mixed mode";
    case TxPdoTypeEnum::TxPdoPVM:
      return "Tx PDO PVM";
    case TxPdoTypeEnum::TxPdoStandard:
      return "Tx PDO Standard";
    default:
      return "Unsupported Type";
  }
}

std::ostream& operator<<(std::ostream& os, const Configuration& configuration)
{
  std::string modeOfOperation_ = modeOfOperationString(configuration.modesOfOperation[0]);
  unsigned int tmp3 = modeOfOperation_.size();
  unsigned int len2 = tmp3;
  len2++;

  os << std::boolalpha << std::left << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "-"
     << "|\n"
     << std::setfill(' ') << std::setw(43 + len2 + 2) << "| Configuration"
     << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::setw(43) << "| 1st Mode of Operation:"
     << "| " << std::setw(len2) << modeOfOperation_ << "|\n"
     << std::setw(43) << "| Config Run SDO verify timeout:"
     << "| " << std::setw(len2) << configuration.configRunSdoVerifyTimeout << "|\n"
     << std::setw(43) << "| Print Debug Messages:"
     << "| " << std::setw(len2) << configuration.printDebugMessages << "|\n"
     << std::setw(43) << "| Drive State Change Min Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMinTimeout << "|\n"
     << std::setw(43) << "| Drive State Change Max Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMaxTimeout << "|\n"
     << std::setw(43) << "| Min Successful Target State Readings:"
     << "| " << std::setw(len2) << configuration.minNumberOfSuccessfulTargetStateReadings << "|\n"
     << std::setw(43) << "| Force Append Equal Error:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualError << "|\n"
     << std::setw(43) << "| Force Append Equal Fault:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualFault << "|\n"
     << std::setw(43) << "| Error Storage Capacity"
     << "| " << std::setw(len2) << configuration.errorStorageCapacity << "|\n"
     << std::setw(43) << "| Fault Storage Capacity"
     << "| " << std::setw(len2) << configuration.faultStorageCapacity << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::noboolalpha << std::right;
  return os;
}

std::pair<RxPdoTypeEnum, TxPdoTypeEnum> Configuration::getPdoTypeSolution() const
{
  // {ModeOfOperationEnum1, ..., ModeOfOperationEnumN} -> {RxPdoTypeEnum, TxPdoTypeEnum}
  const std::map<std::vector<ModeOfOperationEnum>, std::pair<RxPdoTypeEnum, TxPdoTypeEnum>> modes2PdoTypeMap = {
    {
      { ModeOfOperationEnum::CyclicSynchronousTorqueMode, ModeOfOperationEnum::CyclicSynchronousPositionMode },
      { RxPdoTypeEnum::RxPdoCSTCSP, TxPdoTypeEnum::TxPdoCSTCSP }
    },
    {
      { ModeOfOperationEnum::CyclicSynchronousTorqueMode, ModeOfOperationEnum::CyclicSynchronousPositionMode,
        ModeOfOperationEnum::CyclicSynchronousVelocityMode },
      { RxPdoTypeEnum::RxPdoCSTCSPCSV, TxPdoTypeEnum::TxPdoCSTCSPCSV }
    },
    {
      { ModeOfOperationEnum::CyclicSynchronousPositionMode },
      { RxPdoTypeEnum::RxPdoCSP, TxPdoTypeEnum::TxPdoCSP }
    },
    {
      { ModeOfOperationEnum::CyclicSynchronousTorqueMode },
      { RxPdoTypeEnum::RxPdoCST, TxPdoTypeEnum::TxPdoCST }
    },
    {
      { ModeOfOperationEnum::CyclicSynchronousVelocityMode },
      { RxPdoTypeEnum::RxPdoCSV, TxPdoTypeEnum::TxPdoCSV }
    },
    {
      { ModeOfOperationEnum::HomingMode },
      { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
    },
    {
      { ModeOfOperationEnum::ProfiledPositionMode },
      { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
    },
    {
      { ModeOfOperationEnum::ProfiledVelocityMode },
      { RxPdoTypeEnum::RxPdoPVM, TxPdoTypeEnum::TxPdoPVM }
    },
    {
      { ModeOfOperationEnum::NA },
      { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
    },
  };

  bool setsAreEqual;
  for (const auto& modes2PdoTypeEntry : modes2PdoTypeMap)
  {
    setsAreEqual = true;
    for (const auto& modeOfOperation : modesOfOperation)
      setsAreEqual &= std::find(modes2PdoTypeEntry.first.begin(),
                              modes2PdoTypeEntry.first.end(),
                              modeOfOperation)
        != modes2PdoTypeEntry.first.end();
    for (const auto& modeOfOperation : modes2PdoTypeEntry.first)
      setsAreEqual &= std::find(modesOfOperation.begin(),
                                modesOfOperation.end(),
                                modeOfOperation)
        != modesOfOperation.end();
    if(setsAreEqual)
      return modes2PdoTypeEntry.second;
  }
  return std::pair<RxPdoTypeEnum, TxPdoTypeEnum>{ RxPdoTypeEnum::NA, TxPdoTypeEnum::NA };
}

bool Configuration::sanityCheck(bool silent) const
{
  bool success = true;
  std::string message = "";

  auto check_and_inform = [&message, &success] (std::pair<bool, std::string> test) {
    if(test.first) {
      message += "\033[32m✓\t";
      message += test.second;
      message += "\033[m\n";
      success &= true;
    } else {
      message += "\033[31m❌\t";
      message += test.second;
      message += "\033[m\n";
      success = false;
    }
  };
  auto pdoTypePair = getPdoTypeSolution();
  const std::vector<std::pair<bool, std::string>> sanity_tests = {
    {
      (polePairs > 0),
      "pole_pairs > 0"
    },
    {
      (motorConstant > 0),
      "motor_constant > 0"
    },
    {
      (nominalCurrentA > 0),
      "nominal_current > 0"
    },
    {
      (maxCurrentA > 0),
      "max_current > 0"
    },
    {
      (torqueConstantNmA > 0),
      "torque_constant > 0"
    },
    {
      (maxProfileVelocity > 0),
      "max_profile_velocity > 0"
    },
    {
      (quickStopDecel > 0),
      "quick_stop_decel > 0"
    },
    {
      (profileDecel > 0),
      "profile_decel > 0"
    },
    {
      (profileDecel > 0),
      "profile_decel > 0"
    },
    {
      (positionEncoderResolution > 0),
      "position_encoder_resolution > 0"
    },
    {
      (gearRatio > 0),
      "gear_ratio > 0"
    },
    {
      (pdoTypePair.first != RxPdoTypeEnum::NA && pdoTypePair.second != TxPdoTypeEnum::NA),
      "modes of operation combination allowed"
    },
    {
      (driveStateChangeMinTimeout <= driveStateChangeMaxTimeout),
      "drive_state_change_min_timeout ≤ drive_state_change_max_timeout"
    },
  };

  std::for_each(sanity_tests.begin(), sanity_tests.end(), check_and_inform);

  if(!silent)
  {
    std::cout << message << std::endl;
  }

  return success;
}
}  // namespace maxon
