/*
** Copyright (2019-2020) Robotics Systems Lab - ETH Zurich:
** Jonas Junger, Johannes Pankert, Fabio Dubois, Lennart Nachtigall,
** Markus Staeuble
**
** This file is part of the maxon_epos_ethercat_sdk.
** The maxon_epos_ethercat_sdk is free software: you can redistribute it and/or
** modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The maxon_epos_ethercat_sdk is distributed in the hope that it will be
** useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the maxon_epos_ethercat_sdk. If not, see
** <https://www.gnu.org/licenses/>.
*/

/*!
 * @brief	This file contains the different Tx Pdo structs. Each struct
 * must contain a statusword_, or else the state changes won't work! Each struct
 * can contain either the actual torque or the actual current but not both.
 */
#pragma once

#include <cstdint>

namespace maxon
{
/*!
 * Standard Tx Pdo type
 */
struct TxPdoStandard
{
  uint16_t statusword_;
} __attribute__((packed));

/*!
 * CST Tx PDO type
 * Includes padding_ byte for firmware version 01.01.15.00 (october 2018)
 */
struct TxPdoCST
{
  int32_t actualPosition_;
  int16_t actualTorque_;
  int32_t actualVelocity_;
} __attribute__((packed));

struct TxPdoPVM
{
  uint16_t statusword_;
  int32_t demandVelocity_;
} __attribute__((packed));

}  // namespace maxon
