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

#include "maxon_epos_ethercat_sdk/Command.hpp"

#include <iomanip>

namespace maxon
{
Command::Command(const Command& other)
{
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

Command& Command::operator=(const Command& other)
{
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

std::ostream& operator<<(std::ostream& os, Command& command)
{
  os << std::left << std::setw(25) << "Target Position:" << command.targetPositionUU_ << "\n"
     << std::setw(25) << "Position Offset:" << command.positionOffsetUU_ << "\n"
     << std::setw(25) << "Target Velocity:" << command.targetVelocityUU_ << "\n"
     << std::setw(25) << "Velocity Offset:" << command.velocityOffsetUU_ << "\n"
     << std::setw(25) << "Target Torque:" << command.targetTorqueUU_ << "\n"
     << std::setw(25) << "Torque Offset:" << command.torqueOffsetUU_ << "\n"
     << std::setw(25) << "Modes of operation:" << static_cast<int>(command.getModeOfOperation()) << "\n"
     << std::right;

  return os;
}

uint32_t Command::getDigitalOutputs() const
{
  return digitalOutputs_;
}

std::string Command::getDigitalOutputString() const
{
  std::string outputs;
  for (unsigned int i = 0; i < 8 * sizeof(digitalOutputs_); i++)
  {
    if ((digitalOutputs_ & (1 << (sizeof(digitalOutputs_) * 8 - 1 - i))) != 0)
    {
      outputs += "1";
    }
    else
    {
      outputs += "0";
    }
    if (((i + 1) % 8) == 0)
    {
      outputs += " ";
    }
  }
  outputs.erase(outputs.end() - 1);
  return outputs;
}

/*!
 * Raw set methods
 */
void Command::setTargetPositionRaw(int32_t targetPosition)
{
  targetPosition_ = targetPosition;
}
void Command::setTargetVelocityRaw(int32_t targetVelocity)
{
  targetVelocity_ = targetVelocity;
}
void Command::setPositionOffsetRaw(int32_t positionOffset) {
  positionOffset_ = positionOffset;
}
void Command::setTargetTorqueRaw(int16_t targetTorque) {
  targetTorque_ = targetTorque;
}
void Command::setTorqueOffsetRaw(int16_t torqueOffset)
{
  torqueOffset_ = torqueOffset;
}
void Command::setVelocityOffsetRaw(int32_t velocityOffset) {
  velocityOffset_ = velocityOffset;
}

/*!
 * user unit set methods
 */
void Command::setTargetPosition(double targetPosition)
{
  targetPositionUU_ = targetPosition;
}
void Command::setTargetVelocity(double targetVelocity)
{
  targetVelocityUU_ = targetVelocity;
}
void Command::setTargetTorque(double targetTorque)
{
  // lock for thread safety
  std::lock_guard<std::mutex> lockGuard(targetTorqueCommandMutex_);
  targetTorqueUU_ = targetTorque;
  targetTorqueCommandUsed_ = true;
}
void Command::setPositionOffset(double positionOffset)
{
  positionOffsetUU_ = positionOffset;
}
void Command::setTorqueOffset(double torqueOffset)
{
  torqueOffsetUU_ = torqueOffset;
}
void Command::setVelocityOffset(double velocityOffset) {
  velocityOffsetUU_ = velocityOffset;
}

/*!
 * factors set methods
 */
void Command::setPositionFactorRadToInteger(double factor)
{
  positionFactorRadToInteger_ = factor;
}
void Command::setTorqueFactorNmToInteger(double factor)
{
  torqueFactorNmToInteger_ = factor;
}
void Command::setCurrentFactorAToInteger(double factor)
{
  currentFactorAToInteger_ = factor;
}

/*!
 * other set methods
 */
void Command::setDigitalOutputs(uint32_t digitalOutputs)
{
  digitalOutputs_ = digitalOutputs;
}
void Command::setUseRawCommands(bool useRawCommands)
{
  useRawCommands_ = useRawCommands;
}
void Command::setModeOfOperation(const ModeOfOperationEnum modeOfOperation)
{
  modeOfOperation_ = modeOfOperation;
}

/*
 * get methods (raw units)
 */
int32_t Command::getTargetPositionRaw() const
{
  return targetPosition_;
}
int32_t Command::getTargetVelocityRaw() const
{
  return targetVelocity_;
}
int16_t Command::getTargetTorqueRaw() const { return targetTorque_; }
int32_t Command::getPositionOffsetRaw() const
{
  return positionOffset_;
}
int16_t Command::getTorqueOffsetRaw() const
{
  return torqueOffset_;
}
int32_t Command::getVelocityOffsetRaw() const
{
  return velocityOffset_;
}
uint32_t Command::getProfileAccelRaw() const
{
  return profileAccel_;
}
uint32_t Command::getProfileDeccelRaw() const
{
  return profileDeccel_;
}
int16_t Command::getMotionProfileType() const
{
  return motionProfileType_;
}

/*
 * get methods (user units)
 */
double Command::getTargetPosition() const
{
  return targetPositionUU_;
}
double Command::getTargetVelocity() const
{
  return targetVelocityUU_;
}
double Command::getTargetTorque() const { return targetTorqueUU_; }
double Command::getTorqueOffset() const
{
  return torqueOffsetUU_;
}
double Command::getVelocityOffset() const { return velocityOffsetUU_; }

void Command::doUnitConversion()
{
  if (!useRawCommands_)
  {
    targetPosition_ = static_cast<int32_t>(positionFactorRadToInteger_ * targetPositionUU_);
    targetVelocity_ = static_cast<int32_t>(velocityFactorRadPerSecToMicroRPM_ * targetVelocityUU_);
    targetTorque_ = static_cast<int16_t>(torqueFactorNmToInteger_ * targetTorqueUU_);

    positionOffset_ = static_cast<int32_t>(positionFactorRadToInteger_ * positionOffsetUU_);
    torqueOffset_ = static_cast<int16_t>(torqueFactorNmToInteger_ * torqueOffsetUU_);
    velocityOffset_ = static_cast<int32_t>(velocityFactorRadPerSecToMicroRPM_ * velocityOffsetUU_);
  }
}

/// other get methods
ModeOfOperationEnum Command::getModeOfOperation() const
{
  return modeOfOperation_;
}

}  // namespace maxon
