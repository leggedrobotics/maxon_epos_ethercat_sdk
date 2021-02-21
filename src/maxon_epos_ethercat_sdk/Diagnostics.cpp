#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon
{
// Print errors
void Maxon::addErrorToReading(const ErrorType& errorType)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading_.addError(errorType);
}

/* 
** Print error code
** See firmware documentation for meaning
*/
void Maxon::printErrorCode()
{
  uint16_t errorcode = 0;
  bool error_read_success = sendSdoRead(OD_INDEX_ERROR_CODE, 0x00, false, errorcode);
  if (error_read_success)
  {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::printErrorCode] "
                      << "Error code: " << std::hex << errorcode);
  }
  else
  {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::printErrorCode] read error code "
                      "uncessuful.")
  }
}

/*
 * Print diagnosis messages
 */
void Maxon::printDiagnosis() {
  uint8_t newestIdx = 0;
  bool newMsgAvailable = false;
  std::array<uint, 4> diagnosisMsg;
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
