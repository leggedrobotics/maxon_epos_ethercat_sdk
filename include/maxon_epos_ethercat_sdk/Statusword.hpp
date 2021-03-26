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
#include <iomanip>
#include <iostream>
#include <string>

#include "maxon_epos_ethercat_sdk/DriveState.hpp"

namespace maxon {
class Statusword {
 private:
  bool readyToSwitchOn_{false};      // bit 0
  bool switchedOn_{false};           // bit 1
  bool operationEnabled_{false};     // bit 2
  bool fault_{false};                // bit 3
  bool voltageEnabled_{false};       // bit 4
  bool quickStop_{false};            // bit 5
  bool switchOnDisabled_{false};     // bit 6
  bool warning_{false};              // bit 7
  bool remote_{false};               // bit 9
  bool targetReached_{false};        // bit 10
  bool internalLimitActive_{false};  // bit 11
  bool followingError_{false};       // bit 13, CSV & PPM mode
  // bool homingError_{false};          // bit 13, HMM mode

  // the raw statusword
  uint16_t rawStatusword_{0};

 public:
  friend std::ostream& operator<<(std::ostream& os,
                                  const Statusword& statusword);
  void setFromRawStatusword(uint16_t status);
  DriveState getDriveState() const;
  std::string getDriveStateString() const;
};

}  // namespace maxon
