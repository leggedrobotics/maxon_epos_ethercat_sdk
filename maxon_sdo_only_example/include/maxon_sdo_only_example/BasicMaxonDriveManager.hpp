#pragma once

#include "ethercat_device_configurator/EthercatDeviceConfigurator.hpp"
#include <maxon_epos_ethercat_sdk/Maxon.hpp>
#include <ethercat_sdk_master/EthercatMaster.hpp>
#include "message_logger/message_logger.hpp"

#include <atomic>
#include <mutex>
#include <thread>


namespace basic_maxon_drive_manager{

struct MotorReading{
  double velocity{0}; 
  double position{0}; 
};

struct MotorCommand{
  double velocity{0};
};

class BasicMaxonDriveManager{

private:
  std::unique_ptr<std::thread> worker_thread;
  std::atomic<bool> abrt = false;

  EthercatDeviceConfigurator::SharedPtr configurator;
  ecat_master::EthercatMaster::SharedPtr ecatMaster;

  std::mutex readingMutex;
  std::map<std::string, MotorReading> motorReadings;

  std::mutex commandMutex;
  std::atomic<bool> receivedUpdate_{false};
  std::map<std::string, MotorCommand> motorCommands;

  //should be done by the sdk... but needs improved sdk.
  static constexpr double velocityFactorMicroRPMToRadPerSec_ =
          2.0 * M_PI / (60.0 * 1e6);

  void slowSDOReadAndWrite();

protected:
  std::map<std::string, std::shared_ptr<maxon::Maxon>> maxonDriveCollection;

public:
  explicit BasicMaxonDriveManager(const std::string& configFilePath);
  bool init();

  MotorReading getMotorReading(const std::string& motorName);
  bool setMotorCommand(const std::string& motorName, MotorCommand motorCommand);
  void shutdown();

};


}
