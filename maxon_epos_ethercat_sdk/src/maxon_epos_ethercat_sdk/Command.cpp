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

#include "maxon_epos_ethercat_sdk/Command.hpp"

#include <iomanip>

namespace maxon {
Command::Command(const Command& other) {
  targetPositionUU_ = other.targetPositionUU_;
  targetVelocityUU_ = other.targetVelocityUU_;
  targetTorqueUU_ = other.targetTorqueUU_;
  torqueOffsetUU_ = other.torqueOffsetUU_;

  targetPosition_ = other.targetPosition_;
  targetVelocity_ = other.targetVelocity_;
  targetTorque_ = other.targetTorque_;
  torqueOffset_ = other.torqueOffset_;

  positionFactorRadToInteger_ = other.positionFactorRadToInteger_;
  torqueFactorNmToInteger_ = other.torqueFactorNmToInteger_;
  currentFactorAToInteger_ = other.currentFactorAToInteger_;

  modeOfOperation_ = other.modeOfOperation_;

  useRawCommands_ = other.useRawCommands_;
  targetTorqueCommandUsed_ = other.targetTorqueCommandUsed_;
}

Command& Command::operator=(const Command& other) {
  targetPositionUU_ = other.targetPositionUU_;
  positionOffsetUU_ = other.positionOffsetUU_;
  targetVelocityUU_ = other.targetVelocityUU_;
  velocityOffsetUU_ = other.velocityOffsetUU_;
  targetTorqueUU_ = other.targetTorqueUU_;
  torqueOffsetUU_ = other.torqueOffsetUU_;

  targetPosition_ = other.targetPosition_;
  positionOffset_ = other.positionOffset_;
  targetVelocity_ = other.targetVelocity_;
  velocityOffset_ = other.velocityOffset_;
  targetTorque_ = other.targetTorque_;
  torqueOffset_ = other.torqueOffset_;

  positionFactorRadToInteger_ = other.positionFactorRadToInteger_;
  torqueFactorNmToInteger_ = other.torqueFactorNmToInteger_;
  currentFactorAToInteger_ = other.currentFactorAToInteger_;

  modeOfOperation_ = other.modeOfOperation_;

  useRawCommands_ = other.useRawCommands_;
  targetTorqueCommandUsed_ = other.targetTorqueCommandUsed_;
  return *this;
}

std::ostream& operator<<(std::ostream& os, Command& command) {
  os << std::left << std::setw(25)
     << "Target Position:" << command.targetPositionUU_ << "\n"
     << std::setw(25) << "Position Offset:" << command.positionOffsetUU_ << "\n"
     << std::setw(25) << "Target Velocity:" << command.targetVelocityUU_ << "\n"
     << std::setw(25) << "Velocity Offset:" << command.velocityOffsetUU_ << "\n"
     << std::setw(25) << "Target Torque:" << command.targetTorqueUU_ << "\n"
     << std::setw(25) << "Torque Offset:" << command.torqueOffsetUU_ << "\n"
     << std::setw(25)
     << "Modes of operation:" << static_cast<int>(command.getModeOfOperation())
     << "\n"
     << std::right;

  return os;
}

uint32_t Command::getDigitalOutputs() const { return digitalOutputs_; }

std::string Command::getDigitalOutputString() const {
  std::string outputs;
  for (unsigned int i = 0; i < 8 * sizeof(digitalOutputs_); i++) {
    if ((digitalOutputs_ & (1 << (sizeof(digitalOutputs_) * 8 - 1 - i))) != 0) {
      outputs += "1";
    } else {
      outputs += "0";
    }
    if (((i + 1) % 8) == 0) {
      outputs += " ";
    }
  }
  outputs.erase(outputs.end() - 1);
  return outputs;
}

/*!
 * Raw set methods
 */
void Command::setTargetPositionRaw(int32_t targetPosition) {
  targetPosition_ = targetPosition;
}
void Command::setTargetVelocityRaw(int32_t targetVelocity) {
  targetVelocity_ = targetVelocity;
}
void Command::setPositionOffsetRaw(int32_t positionOffset) {
  positionOffset_ = positionOffset;
}
void Command::setTargetTorqueRaw(int16_t targetTorque) {
  targetTorque_ = targetTorque;
}
void Command::setTorqueOffsetRaw(int16_t torqueOffset) {
  torqueOffset_ = torqueOffset;
}
void Command::setVelocityOffsetRaw(int32_t velocityOffset) {
  velocityOffset_ = velocityOffset;
}

/*!
 * user unit set methods
 */
void Command::setTargetPosition(double targetPosition) {
  targetPositionUU_ = targetPosition;
}
void Command::setTargetVelocity(double targetVelocity) {
  targetVelocityUU_ = targetVelocity;
}
void Command::setTargetTorque(double targetTorque) {
  // lock for thread safety
  std::lock_guard<std::mutex> lockGuard(targetTorqueCommandMutex_);
  targetTorqueUU_ = targetTorque;
  targetTorqueCommandUsed_ = true;
}
void Command::setPositionOffset(double positionOffset) {
  positionOffsetUU_ = positionOffset;
}
void Command::setTorqueOffset(double torqueOffset) {
  torqueOffsetUU_ = torqueOffset;
}
void Command::setVelocityOffset(double velocityOffset) {
  velocityOffsetUU_ = velocityOffset;
}

/*!
 * factors set methods
 */
void Command::setPositionFactorRadToInteger(double factor) {
  positionFactorRadToInteger_ = factor;
}
void Command::setTorqueFactorNmToInteger(double factor) {
  torqueFactorNmToInteger_ = factor;
}
void Command::setCurrentFactorAToInteger(double factor) {
  currentFactorAToInteger_ = factor;
}

/*!
 * other set methods
 */
void Command::setDigitalOutputs(uint32_t digitalOutputs) {
  digitalOutputs_ = digitalOutputs;
}
void Command::setUseRawCommands(bool useRawCommands) {
  useRawCommands_ = useRawCommands;
}
void Command::setModeOfOperation(const ModeOfOperationEnum modeOfOperation) {
  modeOfOperation_ = modeOfOperation;
}

/*
 * get methods (raw units)
 */
int32_t Command::getTargetPositionRaw() const { return targetPosition_; }
int32_t Command::getTargetVelocityRaw() const { return targetVelocity_; }
int16_t Command::getTargetTorqueRaw() const { return targetTorque_; }
int32_t Command::getPositionOffsetRaw() const { return positionOffset_; }
int16_t Command::getTorqueOffsetRaw() const { return torqueOffset_; }
int32_t Command::getVelocityOffsetRaw() const { return velocityOffset_; }
uint32_t Command::getProfileAccelRaw() const { return profileAccel_; }
uint32_t Command::getProfileDeccelRaw() const { return profileDeccel_; }
int16_t Command::getMotionProfileType() const { return motionProfileType_; }

/*
 * get methods (user units)
 */
double Command::getTargetPosition() const { return targetPositionUU_; }
double Command::getTargetVelocity() const { return targetVelocityUU_; }
double Command::getTargetTorque() const { return targetTorqueUU_; }
double Command::getTorqueOffset() const { return torqueOffsetUU_; }
double Command::getVelocityOffset() const { return velocityOffsetUU_; }

void Command::doUnitConversion() {
  if (!useRawCommands_) {
    targetPosition_ =
        static_cast<int32_t>(positionFactorRadToInteger_ * targetPositionUU_);
    targetVelocity_ = static_cast<int32_t>(velocityFactorRadPerSecToMicroRPM_ *
                                           targetVelocityUU_);
    targetTorque_ =
        static_cast<int16_t>(torqueFactorNmToInteger_ * targetTorqueUU_);

    positionOffset_ =
        static_cast<int32_t>(positionFactorRadToInteger_ * positionOffsetUU_);
    torqueOffset_ =
        static_cast<int16_t>(torqueFactorNmToInteger_ * torqueOffsetUU_);
    velocityOffset_ = static_cast<int32_t>(velocityFactorRadPerSecToMicroRPM_ *
                                           velocityOffsetUU_);
  }
}

/// other get methods
ModeOfOperationEnum Command::getModeOfOperation() const {
  return modeOfOperation_;
}

}  // namespace maxon
