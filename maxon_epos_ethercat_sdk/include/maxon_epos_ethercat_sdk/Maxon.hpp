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

namespace maxon {
class Maxon : public ecat_master::EthercatDevice {
 public:
  typedef std::shared_ptr<Maxon> SharedPtr;

  // create Maxon Drive from setup file
  static SharedPtr deviceFromFile(const std::string& configFile,
                                  const std::string& name,
                                  const uint32_t address);
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
  bool putIntoOperation() {
    bool success;
    bus_->setState(EC_STATE_OPERATIONAL, getAddress());
    success =
        bus_->waitForState(EC_STATE_OPERATIONAL, getAddress(), 1000, 0.001);
    return success;
  }
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

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
  bool setDriveStateViaPdo(const DriveState& driveState,
                           const bool waitForState);
  bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

 protected:
  void engagePdoStateMachine();
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  bool configParam();
  Controlword getNextStateTransitionControlword(
      const DriveState& requestedDriveState,
      const DriveState& currentDriveState);
  void autoConfigurePdoSizes();

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  bool isAllowedModeCombination(const std::vector<ModeOfOperationEnum> modes);
  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getMixedPdoType(
      std::vector<ModeOfOperationEnum> modes);

  // Errors
 protected:
  void addErrorToReading(const ErrorType& errorType);

 public:
  void printErrorCode();
  void printDiagnosis();

 public:
  Configuration configuration_;

 protected:
  Command stagedCommand_;
  Reading reading_;
  RxPdoTypeEnum rxPdoTypeEnum_{RxPdoTypeEnum::NA};
  TxPdoTypeEnum txPdoTypeEnum_{TxPdoTypeEnum::NA};
  Controlword controlword_;
  PdoInfo pdoInfo_;
  bool hasRead_{false};
  bool conductStateChange_{false};
  DriveState targetDriveState_{DriveState::NA};
  std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;
  uint16_t numberOfSuccessfulTargetStateReadings_{0};
  std::atomic<bool> stateChangeSuccessful_{false};

  // Configurable parameters
 protected:
  bool allowModeChange_{false};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};

 protected:
  mutable std::recursive_mutex stagedCommandMutex_;  // TODO required?
  mutable std::recursive_mutex readingMutex_;        // TODO required?
  mutable std::recursive_mutex mutex_;               // TODO: change name!!!!
};
}  // namespace maxon
