/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
**
** This file is part of the maxon_epos_ethercat_sdk.
** The maxon_epos_ethercat_sdk is free software: you can redistribute it and/or
*modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The maxon_epos_ethercat_sdk is distributed in the hope that it will be
*useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the maxon_epos_ethercat_sdk. If not, see
*<https://www.gnu.org/licenses/>.
*/

#pragma once

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ethercat_sdk_master/EthercatDevice.hpp>
#include <mutex>
#include <string>

#include "maxon_epos_ethercat_sdk/Command.hpp"
#include "maxon_epos_ethercat_sdk/Controlword.hpp"
#include "maxon_epos_ethercat_sdk/DriveState.hpp"
#include "maxon_epos_ethercat_sdk/Reading.hpp"

namespace maxon
{
class Maxon : public ecat_master::EthercatDevice
{
public:
  typedef std::shared_ptr<Maxon> SharedPtr;

  // create Maxon Drive from setup file
  static SharedPtr deviceFromFile(const std::string& configFile, const std::string& name, const uint32_t address);
  // constructor
  Maxon() = default;
  Maxon(const std::string& name, const uint32_t address);

  // pure virtual overwrites
public:
  bool startup() override;
  void preShutdown() override;
  void shutdown() override;
  void updateWrite() override;
  void updateRead() override;
  bool putIntoOperation()
  {
    bool success;
    bus_->setState(EC_STATE_OPERATIONAL, getAddress());
    success = bus_->waitForState(EC_STATE_OPERATIONAL, getAddress(), 1000, 0.001);
    return success;
  }
  PdoInfo getCurrentPdoInfo() const override
  {
    return pdoInfo_;
  }

public:
  void stageCommand(const Command& command);
  Reading getReading() const;
  void getReading(Reading& reading) const;

  bool loadConfigFile(const std::string& fileName);
  bool loadConfigNode(YAML::Node configNode);
  bool loadConfiguration(const Configuration& configuration);
  Configuration getConfiguration() const;

  // SDO
public:
  bool getStatuswordViaSdo(Statusword& statusword);
  bool setControlwordViaSdo(Controlword& controlword);
  bool setDriveStateViaSdo(const DriveState& driveState);

protected:
  bool stateTransitionViaSdo(const StateTransition& stateTransition);

  // PDO
public:
  bool setDriveStateViaPdo(const DriveState& driveState, const bool waitForState);
  bool lastPdoStateChangeSuccessful() const
  {
    return stateChangeSuccessful_;
  }

protected:
  void engagePdoStateMachine();
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  bool configParam(ModeOfOperationEnum modeOfOperationEnum);
  Controlword getNextStateTransitionControlword(const DriveState& requestedDriveState,
                                                const DriveState& currentDriveState);
  void autoConfigurePdoSizes();

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  // Errors
protected:
  void addErrorToReading(const ErrorType& errorType);

public:
  void printErrorCode();
  void printDiagnosis();

protected:
  Command stagedCommand_;
  Reading reading_;
  Configuration configuration_;
  Controlword controlword_;
  PdoInfo pdoInfo_;
  bool hasRead_{ false };
  bool conductStateChange_{ false };
  DriveState targetDriveState_{ DriveState::NA };
  std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;
  uint16_t numberOfSuccessfulTargetStateReadings_{ 0 };
  std::atomic<bool> stateChangeSuccessful_{ false };

  // Configurable parameters
protected:
  bool allowModeChange_{ false };
  ModeOfOperationEnum modeOfOperation_{ ModeOfOperationEnum::NA };

protected:
  mutable std::recursive_mutex stagedCommandMutex_;  // TODO required?
  mutable std::recursive_mutex readingMutex_;        // TODO required?
  mutable std::recursive_mutex mutex_;               // TODO: change name!!!!
};
}  // namespace maxon
