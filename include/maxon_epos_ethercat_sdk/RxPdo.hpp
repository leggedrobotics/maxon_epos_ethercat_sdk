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

/*!
 * @file	RxPdo.hpp
 * @brief	This file contains the PDOs which are sent to the hardware
 * (RxPdo) Note: each struct MUST contain the controlWord_ variable!
 */
#pragma once

#include <cstdint>

namespace maxon
{
/*!
 * Standard Rx PDO type.
 */
struct RxPdoStandard
{
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CSP Rx PDO type.
*/
struct RxPdoCSP
{
  int32_t targetPosition_;
  int32_t positionOffset_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

/*!
 * CST Rx PDO type.
 */
struct RxPdoCST
{
  int16_t targetTorque_;
  int16_t torqueOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));


/*!
 * CSV Rx PDO type.
 */
struct RxPdoCSV
{
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST and CSP
struct RxPdoCSTCSP
{
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

// Mixed operation mode for CST, CSP and CSV
struct RxPdoCSTCSPCSV
{
  int16_t targetTorque_;
  int16_t torqueOffset_;
  int32_t targetPosition_;
  int32_t positionOffset_;
  int32_t targetVelocity_;
  int32_t velocityOffset_;
  uint16_t controlWord_;
  int8_t modeOfOperation_;
} __attribute__((packed));

struct RxPdoPVM
{
  uint16_t controlWord_;
  int32_t targetVelocity_;
  uint32_t profileAccel_;
  uint32_t profileDeccel_;
  int16_t motionProfileType_;
} __attribute__((packed));

}  // namespace maxon
