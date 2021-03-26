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

#define _USE_MATH_DEFINES  // for M_PI
#include <cmath>

#include "maxon_epos_ethercat_sdk/Reading.hpp"

std::ostream& operator<<(std::ostream& os, const maxon::Reading& reading) {
  // TODO(duboisf) make table, remove statusword
  os << std::left << std::setw(30)
     << "Actual Position:" << reading.getActualPosition() << "\n"
     << std::setw(30) << "Actual Velocity:" << reading.getActualVelocity()
     << "\n"
     << std::setw(30) << "Actual Torque:" << reading.getActualTorque() << "\n"
     << std::setw(30) << "Analog input" << reading.getAnalogInput() << "\n"
     << std::setw(30) << "Actual Current:" << reading.getActualCurrent() << "\n"
     << std::setw(30) << "Digital Inputs:" << reading.getDigitalInputString()
     << "\n"
     << std::setw(30) << "Bus Voltage:" << reading.getBusVoltage() << "\n"
     << std::setw(30) << "\nStatusword:"
     << "\n"
     << reading.getStatusword() << std::right;
  return os;
}

namespace maxon {
std::string Reading::getDigitalInputString() const {
  std::string binString;
  for (unsigned int i = 0; i < 8 * sizeof(digitalInputs_); i++) {
    if ((digitalInputs_ & (1 << (8 * sizeof(digitalInputs_) - 1 - i))) != 0) {
      binString += "1";
    } else {
      binString += "0";
    }
    if ((i + 1) % 8 == 0) {
      binString += " ";
    }
  }
  binString.erase(binString.end() - 1);
  return binString;
}

DriveState Reading::getDriveState() const {
  return getStatusword().getDriveState();
}

double Reading::getAgeOfLastReadingInMicroseconds() const {
  ReadingDuration readingDuration = ReadingClock::now() - lastReadingTimePoint_;
  return readingDuration.count();
}

/*!
 * Raw get methods
 */
int32_t Reading::getActualPositionRaw() const { return actualPosition_; }
int32_t Reading::getActualVelocityRaw() const { return actualVelocity_; }
uint16_t Reading::getRawStatusword() const { return statusword_; }
int16_t Reading::getActualCurrentRaw() const { return actualCurrent_; }
uint16_t Reading::getAnalogInputRaw() const { return analogInput_; }
uint32_t Reading::getBusVoltageRaw() const { return busVoltage_; }

/*!
 * User unit get methods
 */
double Reading::getActualPosition() const {
  return static_cast<double>(actualPosition_) * positionFactorIntegerToRad_;
}
double Reading::getActualVelocity() const {
  return static_cast<double>(actualVelocity_) *
         velocityFactorMicroRPMToRadPerSec_;
}
double Reading::getActualCurrent() const {
  return static_cast<double>(actualCurrent_) * currentFactorIntegerToAmp_;
}
double Reading::getActualTorque() const {
  return static_cast<double>(actualCurrent_) * torqueFactorIntegerToNm_;
}
double Reading::getAnalogInput() const {
  return static_cast<double>(analogInput_) * 0.001;
}

/*!
 * Other readings
 */
int32_t Reading::getDigitalInputs() const { return digitalInputs_; }
Statusword Reading::getStatusword() const {
  Statusword statusword;
  statusword.setFromRawStatusword(statusword_);
  return statusword;
}
double Reading::getBusVoltage() const {
  return 0.001 * static_cast<double>(busVoltage_);
}

/*!
 * Raw set methods
 */
void Reading::setActualPosition(int32_t actualPosition) {
  actualPosition_ = actualPosition;
}
void Reading::setDigitalInputs(int32_t digitalInputs) {
  digitalInputs_ = digitalInputs;
}
void Reading::setActualVelocity(int32_t actualVelocity) {
  actualVelocity_ = actualVelocity;
}
void Reading::setDemandVelocity(int32_t demandVelocity) {
  demandVelocity_ = demandVelocity;
}
void Reading::setStatusword(uint16_t statusword) { statusword_ = statusword; }

void Reading::setAnalogInput(int16_t analogInput) {
  analogInput_ = analogInput;
}
void Reading::setActualCurrent(int16_t actualCurrent) {
  actualCurrent_ = actualCurrent;
}
void Reading::setBusVoltage(uint32_t busVoltage) { busVoltage_ = busVoltage; }
void Reading::setTimePointNow() { lastReadingTimePoint_ = ReadingClock::now(); }

void Reading::setPositionFactorIntegerToRad(double positionFactor) {
  positionFactorIntegerToRad_ = positionFactor;
}
void Reading::setCurrentFactorIntegerToAmp(double currentFactor) {
  currentFactorIntegerToAmp_ = currentFactor;
}
void Reading::setTorqueFactorIntegerToNm(double torqueFactor) {
  torqueFactorIntegerToNm_ = torqueFactor;
}

double Reading::getAgeOfLastErrorInMicroseconds() const {
  ReadingDuration errorDuration = ReadingClock::now() - lastError_.second;
  return errorDuration.count();
}

double Reading::getAgeOfLastFaultInMicroseconds() const {
  ReadingDuration faultDuration = ReadingClock::now() - lastFault_.second;
  return faultDuration.count();
}

void Reading::addError(ErrorType errorType) {
  ErrorPair errorPair;
  errorPair.first = errorType;
  errorPair.second = ReadingClock::now();
  if (lastError_.first == errorType) {
    if (forceAppendEqualError_) {
      errors_.push_front(errorPair);
    } else {
      errors_.pop_front();
      errors_.push_front(errorPair);
    }
  } else {
    errors_.push_front(errorPair);
  }
  lastError_ = errorPair;
  if (errors_.size() > errorStorageCapacity_) {
    errors_.pop_back();
  }
  hasUnreadError_ = true;
}

void Reading::addFault(uint16_t faultCode) {
  FaultPair faultPair;
  faultPair.first = faultCode;
  faultPair.second = ReadingClock::now();
  if (lastFault_.first == faultCode) {
    if (forceAppendEqualFault_) {
      faults_.push_front(faultPair);
    } else {
      faults_.pop_front();
      faults_.push_front(faultPair);
    }
  } else {
    faults_.push_front(faultPair);
  }
  lastFault_ = faultPair;
  if (faults_.size() > faultStorageCapacity_) {
    faults_.pop_back();
  }
  hasUnreadFault_ = true;
}

ErrorTimePairDeque Reading::getErrors() const {
  ReadingTimePoint now = ReadingClock::now();
  ErrorTimePairDeque errors;
  errors.resize(errors_.size());
  ReadingDuration duration;
  for (unsigned int i = 0; i < errors_.size(); i++) {
    errors[i].first = errors_[i].first;
    duration = now - errors_[i].second;
    errors[i].second = duration.count();
  }
  hasUnreadError_ = false;
  return errors;
}

FaultTimePairDeque Reading::getFaults() const {
  ReadingTimePoint now = ReadingClock::now();
  FaultTimePairDeque faults;
  faults.resize(faults_.size());
  ReadingDuration duration;
  for (unsigned int i = 0; i < faults_.size(); i++) {
    faults[i].first = faults_[i].first;
    duration = now - faults_[i].second;
    faults[i].second = duration.count();
  }
  hasUnreadFault_ = false;
  return faults;
}

ErrorType Reading::getLastError() const {
  hasUnreadError_ = false;
  return lastError_.first;
}
uint16_t Reading::getLastFault() const {
  // return 0 if no fault occured
  if (!hasUnreadFault_) {
    return 0;
  }
  hasUnreadFault_ = false;
  return lastFault_.first;
}

void Reading::configureReading(const Configuration& configuration) {
  errorStorageCapacity_ = configuration.errorStorageCapacity;
  faultStorageCapacity_ = configuration.faultStorageCapacity;
  forceAppendEqualError_ = configuration.forceAppendEqualError;
  forceAppendEqualFault_ = configuration.forceAppendEqualFault;

  double currentFactor = configuration.nominalCurrentA / 1000.0;

  currentFactorIntegerToAmp_ = currentFactor;
  positionFactorIntegerToRad_ =
      (2.0 * M_PI) /
      static_cast<double>(configuration.positionEncoderResolution);
  torqueFactorIntegerToNm_ =
      configuration.nominalCurrentA * configuration.torqueConstantNmA / 1000.0;
}

}  // namespace maxon
