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

#include <chrono>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#define _USE_MATH_DEFINES
#include <cmath>

#include "maxon_epos_ethercat_sdk/Configuration.hpp"
#include "maxon_epos_ethercat_sdk/DriveState.hpp"
#include "maxon_epos_ethercat_sdk/Error.hpp"
#include "maxon_epos_ethercat_sdk/Statusword.hpp"

namespace maxon {
/*!
 * aliases for time_points, durations and clocks
 */
using ReadingClock = std::chrono::steady_clock;
using ReadingDuration = std::chrono::duration<double, std::milli>;
using ReadingTimePoint = std::chrono::time_point<ReadingClock>;

/*!
 * An alias for a pair of ErrorType and time point
 */
using ErrorPair = std::pair<ErrorType, ReadingTimePoint>;
using FaultPair = std::pair<uint16_t, ReadingTimePoint>;
using ErrorTimePairDeque = std::deque<std::pair<ErrorType, double>>;
using FaultTimePairDeque = std::deque<std::pair<uint16_t, double>>;

class Reading {
 public:
  /*!
   * raw get methods
   */
  int32_t getActualPositionRaw() const;
  int32_t getActualVelocityRaw() const;
  uint16_t getRawStatusword() const;
  int16_t getActualCurrentRaw() const;
  uint16_t getAnalogInputRaw() const;
  uint32_t getBusVoltageRaw() const;

  /*!
   * User units get methods
   */
  double getActualPosition() const;
  double getActualVelocity() const;
  double getActualCurrent() const;
  double getActualTorque() const;
  double getAnalogInput() const;
  double getAgeOfLastReadingInMicroseconds() const;
  double getBusVoltage() const;

  /*!
   * Other get methods
   */
  int32_t getDigitalInputs() const;
  Statusword getStatusword() const;
  std::string getDigitalInputString() const;
  DriveState getDriveState() const;

  /*!
   * set methods (only raw)
   */
  void setActualPosition(int32_t actualPosition);

  void setDigitalInputs(int32_t digitalInputs);

  void setActualVelocity(int32_t actualVelocity);

  void setDemandVelocity(int32_t demandVelocity);

  void setStatusword(uint16_t statusword);

  void setAnalogInput(int16_t analogInput);

  void setActualCurrent(int16_t actualCurrent);

  void setBusVoltage(uint32_t busVoltage);

  void setTimePointNow();

  void setPositionFactorIntegerToRad(double positionFactor);

  void setCurrentFactorIntegerToAmp(double currentFactor);

  void setTorqueFactorIntegerToNm(double torqueFactor);

 protected:
  int32_t actualPosition_{0};
  int32_t digitalInputs_{0};
  int32_t actualVelocity_{0};
  int32_t demandVelocity_{0};
  uint16_t statusword_{0};
  int16_t analogInput_{0};
  int16_t actualCurrent_{0};
  uint32_t busVoltage_{0};

  double positionFactorIntegerToRad_{1};
  static constexpr double velocityFactorMicroRPMToRadPerSec_ =
      2.0 * M_PI / (60.0 * 1e6);
  double currentFactorIntegerToAmp_{1};
  double torqueFactorIntegerToNm_{1};

  ReadingTimePoint lastReadingTimePoint_;

 public:
  /*!
   * returns the age of the last added error in microseconds
   * @return	the age
   */
  double getAgeOfLastErrorInMicroseconds() const;
  /*!
   * returns the age of the last added fault in microseconds
   * @return	the age
   */
  double getAgeOfLastFaultInMicroseconds() const;

  /*!
   * get all stored errors and their age in microseconds
   *
   * @return	deque of all stored errors
   */
  ErrorTimePairDeque getErrors() const;
  /*!
   * get all stored faults and ther age in microseconds
   * @return	deque of all stored faults
   */
  FaultTimePairDeque getFaults() const;

  /*!
   * Returns the last Error that occured
   * @return	The error type of tha last error
   */
  ErrorType getLastError() const;

  /*!
   * Returns the last fault that occured
   * @return	the code of the last occuring fault
   */
  uint16_t getLastFault() const;

  /*!
   * Adds an error type to the reading
   * A time point is set automatically
   * @param errorType	The type of the error
   */
  void addError(ErrorType errorType);
  /*!
   * Adds a fault code to the reading
   * A time point is set automatically
   * @param faultCode	The Code of the fault
   */
  void addFault(uint16_t faultCode);

  /*!
   * The default constructor
   * This is used for Readings generated by the user.
   * No configuration of the capacities and appending equal faults / errors is
   * necessary.
   */
  Reading() = default;

  /*!
   * @brief	Load parameters from Configuration object
   * @param[in] configuration	The Configuration with the requested
   * configuration parameters
   */
  void configureReading(const Configuration& configuration);

  /*!
   * The configuration constructor
   * This is called for readings generated inside of the
   * maxon_epos_ethercat_sdk.
   * @param errorStorageCapacity	the number of errors that are stored
   * @param faultStorageCapacity	the number of faults that are stored
   * @param forceAppendEqualError	true if a new errer shall be appended
   * even though it is equal to the last one
   * @param forceAppendEqualFault	true if a new fault shall be appended
   * even though it is equal to the last one
   */
  Reading(unsigned int errorStorageCapacity, unsigned int faultStorageCapacity,
          bool forceAppendEqualError, bool forceAppendEqualFault);

 private:
  std::deque<ErrorPair> errors_;
  std::deque<FaultPair> faults_;

  ErrorPair lastError_;
  FaultPair lastFault_;

  mutable bool hasUnreadError_{false};
  mutable bool hasUnreadFault_{false};

  /*!
   * paramaters changeable with a Configuration object
   */
  unsigned int errorStorageCapacity_{25};
  unsigned int faultStorageCapacity_{25};
  bool forceAppendEqualError_{false};
  bool forceAppendEqualFault_{false};
};

}  // namespace maxon

// stream operator in global namespace
std::ostream& operator<<(std::ostream& os, const maxon::Reading& reading);
