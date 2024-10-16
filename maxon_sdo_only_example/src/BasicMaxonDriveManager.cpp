#include "maxon_sdo_only_example/BasicMaxonDriveManager.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"
#include "maxon_epos_ethercat_sdk/Maxon.hpp"

namespace basic_maxon_drive_manager {

BasicMaxonDriveManager::BasicMaxonDriveManager(const std::string &configFilePath) {
  configurator = std::make_shared<EthercatDeviceConfigurator>(configFilePath);
  for (auto &slave: configurator->getSlaves()) {
    std::cout << " " << slave->getName() << ": " << slave->getAddress() << std::endl;
    std::shared_ptr<maxon::Maxon> maxonSlavePtr = std::dynamic_pointer_cast<maxon::Maxon>(slave);
    if (maxonSlavePtr) {
      if (maxonDriveCollection.find(maxonSlavePtr->getName()) ==
          maxonDriveCollection.end()) {
        maxonDriveCollection[maxonSlavePtr->getName()] = maxonSlavePtr;
        motorCommands[maxonSlavePtr->getName()] = MotorCommand{};
        motorReadings[maxonSlavePtr->getName()] = MotorReading{};
        receivedUpdates_[maxonSlavePtr->getName()] = false;
        MELO_INFO_STREAM("[MaxonSDOexample] maxon drive collection with: "
                         << maxonSlavePtr->getName())
      } else {
        MELO_ERROR_STREAM("[MaxonSDOexample] Duplicate maxon drive with name: "
                          << maxonSlavePtr->getName())
      }
    }
  }
}


bool BasicMaxonDriveManager::init() {

  ecatMaster = configurator->master();
  bool rtSuccess = ecatMaster->setRealtimePriority(48);
  std::cout << "Setting RT Priority: " << (rtSuccess ? "successful." : "not successful. Check user privileges.")
            << std::endl;

  if (!ecatMaster->startup()) {
    MELO_ERROR_STREAM("Could not startup master")
    return false;
  }
  initialized_ = true;

  //switch on after velocity set to zero by the thread started above (velocity cmd should default to zero)
  for(auto& maxonDrive : maxonDriveCollection){
    if(!maxonDrive.second->setDriveStateViaSdo(maxon::DriveState::OperationEnabled)){
      MELO_INFO_STREAM("[Maxons sdo example] Could not reach drive State SwitchedOn")
      return false;
    }
    MELO_INFO_STREAM("[MaxonSDOexample] "
                     << maxonDrive.first << ": Reached switched on drive state")
  }

  worker_thread = std::make_unique<std::thread>([this]() -> void {
    while (!abrt) {
      slowSDOReadAndWrite();
    }
  });

  MELO_DEBUG_STREAM("[MaxonSDOexample] Initialized");

  return true;
}

void BasicMaxonDriveManager::slowSDOReadAndWrite() {
  for (auto &maxonDrive: maxonDriveCollection) {
    if (receivedUpdates_.at(maxonDrive.first)) {
      int32_t cmdVelRaw = 0;
      int32_t cmdPosRaw = 0;
      double cmdVel = 0;
      double cmdPos = 0;
      maxon::ModeOfOperationEnum modeOfOperation_;
      {
        std::lock_guard commandLock(commandMutex);
        modeOfOperation_ = motorCommands[maxonDrive.first].modeOfOperation;
        cmdVel = motorCommands[maxonDrive.first].velocity /
                 maxonDrive.second->configuration_
                     .velocityFactorConfiguredUnitToRadPerSec;
        cmdPos =
            motorCommands[maxonDrive.first].position *
            static_cast<double>(
                maxonDrive.second->configuration_.positionEncoderResolution) /
            (2.0 * M_PI);
      }
      if (cmdVel > std::numeric_limits<int32_t>::max() ||
          cmdVel < std::numeric_limits<int32_t>::min()) {
        MELO_ERROR_STREAM("[MaxonDriveManager] Motorcommmand for Drive "
                          << maxonDrive.first << " is out off int32t range.")
        cmdVel = 0;
      }
      cmdVelRaw = static_cast<int32_t>(cmdVel);
      cmdPosRaw = static_cast<int32_t>(cmdPos);

      MELO_INFO_STREAM("[MaxonDriveManager] Motor "
                       << maxonDrive.first
                       << " received raw commands pos:  " << cmdPosRaw)

      maxon::Controlword controlword;

      switch (modeOfOperation_) {
        case maxon::ModeOfOperationEnum::NA:
          break;
        case maxon::ModeOfOperationEnum::ProfiledPositionMode:
          controlword.enableOperation_ = true;
          controlword.quickStop_ = true;
          controlword.switchOn_ = true;
          controlword.enableVoltage_ = true;
          controlword.relative_ = true;
          controlword.newSetPoint_ = false;  // we have to toggle new setpoint!
          controlword.changeSetImmediately_ = true;
          controlword.halt_ = false;
          controlword.endlessMovement_ = false;
          maxonDrive.second->sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false,
                                          controlword.getRawControlwordPPM());

          maxonDrive.second->sendSdoWrite(OD_INDEX_TARGET_POSITION, 0, false,
                                          cmdPosRaw);

          controlword.enableOperation_ = true;
          controlword.quickStop_ = true;
          controlword.switchOn_ = true;
          controlword.enableVoltage_ = true;
          controlword.relative_ = true;
          controlword.newSetPoint_ = true;
          controlword.changeSetImmediately_ = true;
          controlword.halt_ = false;
          controlword.endlessMovement_ = false;
          maxonDrive.second->sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false,
                                          controlword.getRawControlwordPPM());

          break;
        case maxon::ModeOfOperationEnum::ProfiledVelocityMode:
          maxonDrive.second->sendSdoWrite(OD_INDEX_TARGET_VELOCITY, 0, false,
                                          cmdVelRaw);

          // controlword juggling for Profiled Velocity Mode
          controlword.setStateTransition4();  // set status word to enable
                                              // operational. have to be send to
                                              // trigger the velocity change.
          maxonDrive.second->setControlwordViaSdo(controlword);
          break;
        case maxon::ModeOfOperationEnum::HomingMode:
        case maxon::ModeOfOperationEnum::CyclicSynchronousPositionMode:
        case maxon::ModeOfOperationEnum::CyclicSynchronousVelocityMode:
        case maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode:
          MELO_ERROR_STREAM(
              "[MaxonDriveManager] This mode not supported in this simple "
              "example");
          break;
      }

      // controldword juggling for Profiled Position Mode

      receivedUpdates_.at(maxonDrive.first) = false;
    }
  }

  for (auto &maxonDrive : maxonDriveCollection) {
    int32_t actualVelRaw = 0;
    int32_t actualPositionRaw = 0;
    maxonDrive.second->sendSdoRead(OD_INDEX_VELOCITY_ACTUAL, 0, false,
                                   actualVelRaw);
    maxonDrive.second->sendSdoRead(OD_INDEX_POSITION_ACTUAL, 0, false,
                                   actualPositionRaw);

    maxon::Statusword statusword;
    maxonDrive.second->getStatuswordViaSdo(statusword);
    // todo log if fault.
    // MELO_INFO_STREAM("[MaxonDriveManager] Motor: " <<maxonDrive.first << "
    // status: " << statusword);

    // abuse some of the conversion stuff meant for pdo communication
    maxon::Reading reading;
    reading.configureReading(maxonDrive.second->configuration_);
    reading.setActualPosition(actualPositionRaw);
    reading.setActualVelocity(actualVelRaw);
    {
      std::lock_guard readingLock(readingMutex);
      motorReadings[maxonDrive.first].velocity = reading.getActualVelocity();
      motorReadings[maxonDrive.first].position = reading.getActualPosition();
    }
  }
}

MotorReading BasicMaxonDriveManager::getMotorReading(const std::string& motorName){
  std::lock_guard lock(readingMutex);
  return motorReadings.at(motorName);
}

bool BasicMaxonDriveManager::setMotorCommand(const std::string& motorName, MotorCommand motorCommand){
  std::lock_guard lock(commandMutex);
  bool hasMotorName = motorCommands.find(motorName) != motorCommands.end();
  if(hasMotorName){
    if (!receivedUpdates_.at(
            motorName)) {  // make sure the last cmd was sent before setting a
                           // new one, if old one not sent, do nothing.
      motorCommands[motorName] = motorCommand;
      receivedUpdates_.at(motorName) = true;
      MELO_INFO_STREAM("[MaxonSDOExample] Motor: " << motorName
                                                   << " received command: "
                                                   << motorCommand.position);
    }
    return true;
  }

  MELO_ERROR_STREAM("[MaxonSDOexample] Could not find configured motor: "
                    << motorName << " shutdown.")
  shutdown();
  return false;
}

void BasicMaxonDriveManager::shutdown(){
  MELO_INFO_STREAM("[MaxonSDOExample] Shutdown")
  //make sure we stop.
  abrt = true;
  if (worker_thread) {
    if (worker_thread->joinable()) {
      worker_thread->join();
    }
  }
  MELO_INFO_STREAM("[MaxonSDOExample] Sdo Worker joined")

  //make sure we stop.
  if (initialized_) {
    for (auto &maxonDrive : maxonDriveCollection) {
      int32_t cmdVelRaw = 0;
      maxonDrive.second->sendSdoWrite(OD_INDEX_TARGET_VELOCITY, 0, false,
                                      cmdVelRaw);
      maxonDrive.second->sendSdoWrite(OD_INDEX_OFFSET_VELOCITY, 0, false,
                                      cmdVelRaw);
      maxon::Controlword controlword;
      controlword.setStateTransition4();  // set status word to enable
                                          // operational. have to be send to
                                          // trigger the velocity change.
      maxonDrive.second->setControlwordViaSdo(controlword);
      MELO_INFO_STREAM("[MaxonSDOexample] "
                       << maxonDrive.first
                       << ": Shutdown velocity set to 0, ramping down.")
      // give some time to ramp down.
      std::this_thread::sleep_for(std::chrono::milliseconds(5000));
      if (maxonDrive.second->setDriveStateViaSdo(
              maxon::DriveState::SwitchOnDisabled)) {
        MELO_INFO_STREAM("[MaxonSDOexample] " << maxonDrive.first
                                              << ": Switched off")
      } else {
        MELO_ERROR_STREAM(
            "[MaxonSDOexample] "
            << maxonDrive.first
            << ": Could not switch off your system. E-Stop and Debug!")
      }
    }
  }
}

}
