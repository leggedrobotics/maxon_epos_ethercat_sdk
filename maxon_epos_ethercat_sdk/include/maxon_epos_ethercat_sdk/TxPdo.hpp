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
 * @brief	This file contains the different Tx Pdo structs. Each struct
 * must contain a statusword_, or else the state changes won't work! Each struct
 * can contain either the actual torque or the actual current but not both.
 */
#pragma once

#include <cstdint>

namespace maxon {
/*!
 * Standard Tx Pdo type
 */
struct TxPdoStandard {
  uint16_t statusword_;
} __attribute__((packed));

/*!
 * CST Tx PDO type
 * Includes padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct TxPdoCSP {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoCST {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoCSV {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

// Mixed operation mode for CST and CSP
struct TxPdoCSTCSP {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

// Mixed operation mode for CST, CSP, and CSV
struct TxPdoCSTCSPCSV {
  uint16_t statusword_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
  int32_t actualPosition_;
} __attribute__((packed));

struct TxPdoPVM {
  uint16_t statusword_;
  int32_t demandVelocity_;
} __attribute__((packed));

}  // namespace maxon
