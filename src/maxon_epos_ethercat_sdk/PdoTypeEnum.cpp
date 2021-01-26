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

#include "maxon_epos_ethercat_sdk/PdoTypeEnum.hpp"

std::ostream& operator<<(std::ostream& os, const maxon::TxPdoTypeEnum& txPdoTypeEnum)
{
  switch (txPdoTypeEnum)
  {
    case maxon::TxPdoTypeEnum::NA:
      os << "NA";
      break;
    case maxon::TxPdoTypeEnum::TxPdoStandard:
      os << "TxPdoStandard";
      break;
    case maxon::TxPdoTypeEnum::TxPdoCST:
      os << "TxPdoCST";
      break;
    default:
      break;
  }
  return os;
}
std::ostream& operator<<(std::ostream& os, const maxon::RxPdoTypeEnum& rxPdoTypeEnum)
{
  switch (rxPdoTypeEnum)
  {
    case maxon::RxPdoTypeEnum::NA:
      os << "NA";
      break;
    case maxon::RxPdoTypeEnum::RxPdoStandard:
      os << "RxPdoStandard";
      break;
    case maxon::RxPdoTypeEnum::RxPdoCST:
      os << "RxPdoCST";
      break;
    case maxon::RxPdoTypeEnum::RxPdoPVM:
      os << "RxPdoPVM";
      break;
    default:
      break;
  }
  return os;
}
