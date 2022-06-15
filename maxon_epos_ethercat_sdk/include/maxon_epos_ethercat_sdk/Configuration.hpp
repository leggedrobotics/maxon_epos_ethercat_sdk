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

#include <cstdint>
#include <iostream>
#include <utility>
#include <vector>

#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"
#include "maxon_epos_ethercat_sdk/PdoTypeEnum.hpp"

namespace maxon {
class Configuration {
 public:
  std::vector<ModeOfOperationEnum> modesOfOperation = {ModeOfOperationEnum::NA};
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
  double workVoltage{48.0};
  double speedConstant{0};
  double polePairs{11};
  double nominalCurrentA{0};
  double torqueConstantNmA{0};
  double maxCurrentA{0};
  int32_t minPosition{0};
  int32_t maxPosition{0};
  uint32_t maxProfileVelocity{0};
  uint32_t quickStopDecel{10000};
  uint32_t profileDecel{10000};
  uint32_t followErrorWindow{2000};
  double currentPGainSI{1.171880};
  double currentIGainSI{3906.250};
  double positionPGainSI{1.5};
  double positionIGainSI{0.78};
  double positionDGainSI{0.016};
  double velocityPGainSI{0.02};
  double velocityIGainSI{0.5};

 public:
  // stream operator
  friend std::ostream& operator<<(std::ostream& os,
                                  const Configuration& configuration);

  /*!
   * @brief Check whether the parameters are sane.
   * Prints a list of the checks and whether they failed or passed.
   * @param[in] silent If true: Do not print. Only return the success of the
   * test.
   * @return true if the checks are successful.
   */
  bool sanityCheck(bool silent = false) const;

  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getPdoTypeSolution() const;
};

}  // namespace maxon
