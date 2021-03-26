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

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon {
// Print errors
void Maxon::addErrorToReading(const ErrorType& errorType) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading_.addError(errorType);
}

/*
** Print error code
** See firmware documentation for meaning
*/
void Maxon::printErrorCode() {
  uint16_t errorcode = 0;
  bool error_read_success =
      sendSdoRead(OD_INDEX_ERROR_CODE, 0x00, false, errorcode);
  if (error_read_success) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::printErrorCode] "
                      << "Error code: " << std::hex << errorcode);
  } else {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::printErrorCode] read error code "
        "uncessuful.")
  }
}

/*
 * Print diagnosis messages
 */
void Maxon::printDiagnosis() {
  uint8_t newestIdx = 0;
  uint8_t newMsgAvailable = 0;
  std::array<uint32_t, 4> diagnosisMsg;
  sendSdoRead(OD_INDEX_DIAGNOSIS, 0x04, false, newMsgAvailable);
  if (newMsgAvailable) {
    sendSdoRead(OD_INDEX_DIAGNOSIS, 0x02, false, newestIdx);
    sendSdoRead(OD_INDEX_DIAGNOSIS, newestIdx, false, diagnosisMsg);
    MELO_INFO(
        "[maxon_epos_ethercat_sdk:Maxon::printDiagnosis] Latest diagnostic "
        "message: ");
    for (const auto& s : diagnosisMsg) {
      MELO_INFO_STREAM(std::hex << s);
    }
  } else {
    MELO_INFO(
        "[maxon_epos_ethercat_sdk:Maxon::printDiagnosis] "
        "No diagnostic message available.");
  }
}
}  // namespace maxon
