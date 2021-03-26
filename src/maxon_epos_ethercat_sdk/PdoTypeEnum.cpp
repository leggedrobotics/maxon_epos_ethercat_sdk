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

#include "maxon_epos_ethercat_sdk/PdoTypeEnum.hpp"

std::ostream& operator<<(std::ostream& os,
                         const maxon::TxPdoTypeEnum& txPdoTypeEnum) {
  switch (txPdoTypeEnum) {
    case maxon::TxPdoTypeEnum::NA:
      os << "NA";
      break;
    case maxon::TxPdoTypeEnum::TxPdoStandard:
      os << "TxPdoStandard";
      break;
    case maxon::TxPdoTypeEnum::TxPdoCSP:
      os << "TxPdoCSP";
      break;
    case maxon::TxPdoTypeEnum::TxPdoCST:
      os << "TxPdoCST";
      break;
    case maxon::TxPdoTypeEnum::TxPdoCSV:
      os << "TxPdoCSV";
      break;
    default:
      break;
  }
  return os;
}
std::ostream& operator<<(std::ostream& os,
                         const maxon::RxPdoTypeEnum& rxPdoTypeEnum) {
  switch (rxPdoTypeEnum) {
    case maxon::RxPdoTypeEnum::NA:
      os << "NA";
      break;
    case maxon::RxPdoTypeEnum::RxPdoStandard:
      os << "RxPdoStandard";
      break;
    case maxon::RxPdoTypeEnum::RxPdoCSP:
      os << "RxPdoCSP";
      break;
    case maxon::RxPdoTypeEnum::RxPdoCST:
      os << "RxPdoCST";
      break;
    case maxon::RxPdoTypeEnum::RxPdoCSV:
      os << "RxPdoCSV";
      break;
    case maxon::RxPdoTypeEnum::RxPdoPVM:
      os << "RxPdoPVM";
      break;
    default:
      break;
  }
  return os;
}
