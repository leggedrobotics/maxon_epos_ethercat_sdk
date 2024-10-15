/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 **
 ** 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 **
 ** 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 **
 ** 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 **
 ** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include "maxon_sdo_only_example/BasicMaxonDriveManager.hpp"
#include "maxon_sdo_only_example/simple_motor_reading.h"
#include "maxon_sdo_only_example/trigger_position_change.h"
#include "maxon_sdo_only_example/trigger_velocity_change.h"

namespace basic_maxon_drive_manager{
class BasicMaxonDriveManagerRos : public BasicMaxonDriveManager{
public:
    BasicMaxonDriveManagerRos(const std::string& configFilePath, ros::NodeHandle& nh) :
     BasicMaxonDriveManager(configFilePath),
     nh_(nh){
        init();
        velCmdServer_ = nh_.advertiseService("velocity_change", &BasicMaxonDriveManagerRos::velCmdServerCb, this);
        velCmdServer_ = nh_.advertiseService(
            "position_change", &BasicMaxonDriveManagerRos::posCmdServerCb,
            this);
        for(const auto& maxonDrive : maxonDriveCollection){
          //insert.
            readingPublishers_[maxonDrive.first] = nh_.advertise<maxon_sdo_only_example::simple_motor_reading>(maxonDrive.first+"_pub",10);
        }
    }
    
    void runUpdate(){
        ros::Rate loop_rate(400);
        while(ros::ok()){
            ros::spinOnce();
            publishReadings();
            loop_rate.sleep();
        }
        shutdown();
    }

private: 
    ros::NodeHandle nh_;
    ros::ServiceServer velCmdServer_;
    std::map<std::string, ros::Publisher>
        readingPublishers_;  // one publisher per motor..

    bool velCmdServerCb(maxon_sdo_only_example::trigger_velocity_change::Request& req,
      maxon_sdo_only_example::trigger_velocity_change::Response& rep ){
      MotorCommand motorCommand;
      motorCommand.modeOfOperation =
          maxon::ModeOfOperationEnum::ProfiledVelocityMode;
      motorCommand.velocity = req.req_velocity;
      if(!setMotorCommand(req.motor_name, motorCommand)){
        rep.success = -1;
        return false;
      }
      rep.success = 1;
      return true;
    }

    bool posCmdServerCb(
        maxon_sdo_only_example::trigger_position_change::Request& req,
        maxon_sdo_only_example::trigger_position_change::Response& rep) {
      MotorCommand motorCommand;
      motorCommand.modeOfOperation =
          maxon::ModeOfOperationEnum::ProfiledPositionMode;
      motorCommand.position = req.req_position;
      if (!setMotorCommand(req.motor_name, motorCommand)) {
        rep.success = -1;
        return false;
      }
      rep.success = 1;
      return true;
    }

    void publishReadings(){
      for(const auto& maxonDrive : maxonDriveCollection){
        MotorReading reading = getMotorReading(maxonDrive.first);
        maxon_sdo_only_example::simple_motor_reading  msg;
        msg.position = reading.position;
        msg.velocity = reading.velocity;
        readingPublishers_.at(maxonDrive.first).publish(msg);
      }
    }
};

}


int main(int argc, char**argv)
{   

    if(argc < 2)
    {
        std::cerr << "pass path to 'setup.yaml' as command line argument" << std::endl;
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "hardware_interface");
    ros::NodeHandle nh;
    basic_maxon_drive_manager::BasicMaxonDriveManagerRos maxonDriveManagerRos(argv[1], nh);

    maxonDriveManagerRos.runUpdate();

}
