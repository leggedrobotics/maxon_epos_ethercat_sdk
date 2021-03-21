#include "maxon_epos_ethercat_sdk/ModeOfOperationEnum.hpp"
#include <unordered_map>

std::ostream& operator << (std::ostream& os, const maxon::ModeOfOperationEnum modeOfOperation) {
  std::unordered_map<maxon::ModeOfOperationEnum, std::string> mode2strMap = {
    {maxon::ModeOfOperationEnum::NA, "NA"},
    {maxon::ModeOfOperationEnum::ProfiledPositionMode, "ProfiledPositionMode"},
    {maxon::ModeOfOperationEnum::HomingMode, "HomingMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode, "CyclicSynchronousPositionMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode, "CyclicSynchronousVelocityMode"},
    {maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode, "CyclicSynchronousTorqueMode"},
  };
  os << mode2strMap[modeOfOperation];
  return os;
}
