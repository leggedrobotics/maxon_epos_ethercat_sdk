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

/*!
 * @file	RxPdo.hpp
 * @brief	This file contains the PDOs which are sent to the hardware
 * (RxPdo) Note: each struct MUST contain the controlWord_ variable!
 */
#pragma once

#include <cstdint>

namespace maxon {
/*!
 * Standard Rx PDO type.
 */
struct RxPdoStandard {
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CSP Rx PDO type.
 */
struct RxPdoCSP {
  int32_t targetPosition_;
  int32_t positionOffset_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CST Rx PDO type.
 */
struct RxPdoCST {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CSV Rx PDO type.
 */
struct RxPdoCSV {
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST and CSP
struct RxPdoCSTCSP {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST, CSP and CSV
struct RxPdoCSTCSPCSV {
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

struct RxPdoPVM {
  uint16_t controlWord_;
  int32_t targetVelocity_;
  uint32_t profileAccel_;
  uint32_t profileDeccel_;
  int16_t motionProfileType_;
} __attribute__((packed));

}  // namespace maxon
