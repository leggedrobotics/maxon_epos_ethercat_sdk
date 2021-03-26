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

#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <string>

#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"

namespace maxon {
class Command {
 public:
  Command() = default;
  Command(const Command& other);
  virtual ~Command() = default;

  Command& operator=(const Command& other);

  /*!
   * Set raw commands
   * This requires the "SET_USE_RAW_COMMANDS" variable of the config file to be
   * set to "true"
   */

  void setTargetPositionRaw(int32_t targetPosition);
  void setTargetVelocityRaw(int32_t targetVelocity);
  void setTargetTorqueRaw(int16_t targetTorque);
  void setPositionOffsetRaw(int32_t positionOffset);
  void setTorqueOffsetRaw(int16_t torqueOffset);
  void setVelocityOffsetRaw(int32_t velocityOffset);

  /// set factors
  void setPositionFactorRadToInteger(double factor);
  void setTorqueFactorNmToInteger(double factor);
  void setCurrentFactorAToInteger(double factor);

  /// set user units
  void setTargetPosition(double targetPosition);
  void setTargetVelocity(double targetVelocity);
  void setTargetTorque(double targetTorque);
  void setPositionOffset(double positionOffset);
  void setTorqueOffset(double velocityOffset);
  void setVelocityOffset(double velocityOffset);

  /// other
  void setDigitalOutputs(uint32_t digitalOutputs);
  void setUseRawCommands(bool useRawCommands);
  void setModeOfOperation(const ModeOfOperationEnum modeOfOperation);

  /// get (raw)
  int32_t getTargetPositionRaw() const;
  int32_t getTargetVelocityRaw() const;
  int16_t getTargetTorqueRaw() const;
  int32_t getPositionOffsetRaw() const;
  int16_t getTorqueOffsetRaw() const;
  int32_t getVelocityOffsetRaw() const;
  uint32_t getProfileAccelRaw() const;
  uint32_t getProfileDeccelRaw() const;
  int16_t getMotionProfileType() const;

  /// get (user units)
  double getTargetPosition() const;
  double getTargetVelocity() const;
  double getTargetTorque() const;
  double getTorqueOffset() const;
  double getVelocityOffset() const;

  /*!
   * Get the digital outputs.
   * Only available as integer value
   * Use the "getDigitalOutputString" method to print out the state of the
   * target state of the individual pins.
   */
  uint32_t getDigitalOutputs() const;

  /// get (other)
  std::string getDigitalOutputString() const;
  ModeOfOperationEnum getModeOfOperation() const;

  /// Convert the units
  void doUnitConversion();

  /// only works if commands in user units (A, Nm, rad/s,..) are used
  friend std::ostream& operator<<(std::ostream& os, Command& command);

  // Set command max current/torque

 private:
  double targetPositionUU_{0};
  double targetVelocityUU_{0};
  double targetTorqueUU_{0};
  double positionOffsetUU_{0};
  double torqueOffsetUU_{0};
  double velocityOffsetUU_{0};

  int32_t targetPosition_{0};
  int32_t targetVelocity_{0};
  int16_t targetTorque_{0};
  int32_t positionOffset_{0};
  int16_t torqueOffset_{0};
  int32_t velocityOffset_{0};
  uint32_t profileAccel_{0};
  uint32_t profileDeccel_{0};
  int16_t motionProfileType_{0};

  std::mutex targetTorqueCommandMutex_;

  uint32_t digitalOutputs_{0};

  double positionFactorRadToInteger_{1};
  const double velocityFactorRadPerSecToMicroRPM_{1.0 / (2 * M_PI) * 60 * 1e6};
  double torqueFactorNmToInteger_{1};
  double currentFactorAToInteger_{1};

  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};

  /*!
   * set this to true if raw commands have been used and therefore no unit
   * conversion should be done
   */
  bool useRawCommands_{false};

  bool targetTorqueCommandUsed_{false};
};

}  // namespace maxon
