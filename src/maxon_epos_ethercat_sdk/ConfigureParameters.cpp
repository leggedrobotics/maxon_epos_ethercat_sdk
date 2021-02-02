#include <array>
#include <thread>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon
{
bool Maxon::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum)
{
  uint8_t subIndex;

  bool rxSuccess = true;
  switch (rxPdoTypeEnum)
  {
    case RxPdoTypeEnum::RxPdoStandard:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSP:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
        (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCST:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Troque Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSV:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Veloctity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_OFFSET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoCSTCSP:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Toruqe/Position Mixed Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 6> objects{
        (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_TARGET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_OFFSET_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::RxPdoPVM:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
        (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
        (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) | sizeof(uint32_t) * 8,
        (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
                        "RxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
    default:  // Non-implemented type
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map unimplemented "
                        "RxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum)
  {
    case TxPdoTypeEnum::TxPdoStandard:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSP:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Position Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
        (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCST:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
        (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSV:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
        (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoCSTCSP:
    {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque/Position Mixed Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
        (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
        (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
        (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
        (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::TxPdoPVM:
    {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false, OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 2> objects{
        (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
        (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects)
      {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false, objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex, configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false, static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
                        "TxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
    default:  // if any case was forgotten
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map undefined "
                        "TxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
  }

  return (txSuccess && rxSuccess);
}

bool Maxon::configParam()
{
  bool configSuccess = true;
  uint32_t maxMotorSpeed;
  uint32_t maxGearSpeed;
  uint32_t nominalCurrent;
  uint32_t maxCurrent;
  uint32_t torqueConstant;

  // Set velocity unit to micro revs per minute
  uint32_t velocity_unit;
  velocity_unit = 0xFAB44700;
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_SI_UNIT_VELOCITY, 0x00, false, velocity_unit, configuration_.configRunSdoVerifyTimeout);

  maxMotorSpeed =
      static_cast<uint32_t>(configuration_.workVoltage * configuration_.speedConstant / configuration_.polePairs);

  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MAX_MOTOR_SPEED, 0x00, false, maxMotorSpeed, configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_MAX_PROFILE_VELOCITY, 0x00, false, maxMotorSpeed,
                                  configuration_.configRunSdoVerifyTimeout);

  maxGearSpeed = static_cast<uint32_t>(maxMotorSpeed / configuration_.gearRatio);
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_GEAR_DATA, 0x03, false, maxGearSpeed, configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x01, false, configuration_.minPosition);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x02, false, configuration_.maxPosition);

  nominalCurrent = static_cast<uint32_t>(round(1000.0 * configuration_.nominalCurrentA));
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x01, false, nominalCurrent, configuration_.configRunSdoVerifyTimeout);

  maxCurrent = static_cast<uint32_t>(round(1000.0 * configuration_.maxCurrentA));
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x02, false, maxCurrent, configuration_.configRunSdoVerifyTimeout);

  torqueConstant = static_cast<uint32_t>(1000000.0 * configuration_.torqueConstantNmA);
  configSuccess &=
      sdoVerifyWrite(OD_INDEX_MOTOR_DATA, 0x05, false, torqueConstant, configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x01, false, static_cast<uint32_t>(8426858),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x02, false, static_cast<uint32_t>(10699972),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x01, false, static_cast<uint32_t>(7553428),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_POSITION_CONTROL_PARAM, 0x02, false, static_cast<uint32_t>(46226330),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_QUICKSTOP_DECELERATION, 0x00, false, configuration_.quickStopDecel,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_PROFILE_DECELERATION, 0x00, false, configuration_.profileDecel,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_FOLLOW_ERROR_WINDOW, 0x00, false, configuration_.followErrorWindow,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x01, false, static_cast<uint32_t>(119284),
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x02, false, static_cast<uint32_t>(9654194),
                                  configuration_.configRunSdoVerifyTimeout);

  if (configSuccess)
  {
    MELO_INFO("Setting configuration parameters succeeded.");
  }
  else
  {
    MELO_ERROR("Setting configuration parameters failed.");
  }

  return configSuccess;
}
}  // namespace maxon
