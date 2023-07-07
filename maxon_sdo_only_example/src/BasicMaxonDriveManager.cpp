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


  for (auto &maxonDrive: maxonDriveCollection) {
    //set all the drives back to safeOP so that only SDO communication used.
    ecatMaster->getBusPtr()->setState(EC_STATE_SAFE_OP, maxonDrive.second->getAddress());
    if (!ecatMaster->getBusPtr()->waitForState(EC_STATE_SAFE_OP, maxonDrive.second->getAddress(), 50, 0.05)) {
      MELO_ERROR_STREAM("Could not reach safeOP for slave: " << maxonDrive.second->getName())
      return false;
    }
    MELO_INFO_STREAM("[Maxon sdo example] Reached safe op state")
  }

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

  return true;
}

void BasicMaxonDriveManager::slowSDOReadAndWrite() {
  for (auto &maxonDrive: maxonDriveCollection) {
    int32_t cmdVelRaw = 0;
    double cmdVel = 0;
    {
      std::lock_guard commandLock(commandMutex);
      cmdVel = motorCommands[maxonDrive.first].velocity /maxonDrive.second->configuration_.velocityFactorConfiguredUnitToRadPerSec;
    }
    if(cmdVel > std::numeric_limits<int32_t>::max() || cmdVel < std::numeric_limits<int32_t>::min()){
      MELO_ERROR_STREAM("[MaxonDriveManager] Motorcommmand for Drive " << maxonDrive.first << " is out off int32t range.")
      cmdVel = 0;
    }
    cmdVelRaw = static_cast<int32_t>(cmdVel);
    if (receivedUpdate_) {
      maxonDrive.second->sendSdoWrite(OD_INDEX_TARGET_VELOCITY, 0, false,
                                      cmdVelRaw);
      maxon::Controlword controlword;
      controlword.setStateTransition4();  // set status word to enable
                                          // operational. have to be send to
                                          // trigger the velocity change.
      maxonDrive.second->setControlwordViaSdo(controlword);
    }
  }
  if (receivedUpdate_) {  // todo only set the cmd to the drive which receives
                          // the cmd.
    // we have updated all drives with the cmds. so unset the flag and accept
    // new commands
    receivedUpdate_ = false;
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
    // abuse some of the conversion stuff meant for pdo communicat
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
    if(!receivedUpdate_){ //make sure the last cmd was sent before setting a new one, if old one not sent, do nothing.
      motorCommands[motorName] = motorCommand;
      receivedUpdate_ = true;
    }
    return true;
  }

  MELO_ERROR_STREAM("[MaxonSDOexample] Could not find configured motor: "
                    << motorName << " shutdown.")
  shutdown();
  return false;
}

void BasicMaxonDriveManager::shutdown(){
  //make sure we stop.
  abrt = true;
  worker_thread->join();

  //make sure we stop.
  for (auto &maxonDrive: maxonDriveCollection) {
    int32_t cmdVelRaw = 0;
    maxonDrive.second->sendSdoWrite(OD_INDEX_TARGET_VELOCITY, 0, false,
                                    cmdVelRaw);
    maxonDrive.second->sendSdoWrite(OD_INDEX_OFFSET_VELOCITY, 0, false,
                                    cmdVelRaw);
    maxon::Controlword controlword;
    controlword
        .setStateTransition4();  // set status word to enable operational. have
                                 // to be send to trigger the velocity change.
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
