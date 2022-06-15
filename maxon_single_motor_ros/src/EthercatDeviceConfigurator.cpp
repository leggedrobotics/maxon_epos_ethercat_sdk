/*
 ** Copyright 2021 Robotic Systems Lab - ETH Zurich:
 ** Lennart Nachtigall, Jonas Junger
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

#include "hardware_interface/EthercatDeviceConfigurator.hpp"

/*Maxon*/
#ifdef _MAXON_FOUND_
#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#endif

/*yaml-cpp*/
#include "yaml-cpp/yaml.h"

/*std*/
#if __GNUC__ < 8
#include <experimental/filesystem>
#else
#include <filesystem>
#endif


static bool path_exists(std::string& path)
{
    #if __GNUC__ < 8
    return std::experimental::filesystem::exists(path);
    #else
    return std::filesystem::exists(path);
    #endif
}


EthercatDeviceConfigurator::EthercatDeviceConfigurator(std::string path, bool startup):
    m_setup_file_path(path)
{
    parseFile(m_setup_file_path);
    setup(startup);
    MELO_DEBUG("[EthercatDeviceConfigurator] Parsing and setup finished");
}

std::vector<std::shared_ptr<ecat_master::EthercatMaster> > EthercatDeviceConfigurator::getMasters()
{
    return m_masters;
}

std::vector<std::shared_ptr<ecat_master::EthercatDevice>> EthercatDeviceConfigurator::getSlaves()
{
    return m_slaves;
}

std::shared_ptr<ecat_master::EthercatDevice> EthercatDeviceConfigurator::getSlave(std::string name)
{
    for(auto & slave: m_slaves)
    {
        if(slave->getName() == name)
            return slave;
    }
    throw std::runtime_error("[EthercatDeviceConfigurator] Slave: "+name + " not found");
}

const EthercatDeviceConfigurator::EthercatSlaveEntry &EthercatDeviceConfigurator::getInfoForSlave(const std::shared_ptr<ecat_master::EthercatDevice> &slave)
{
    return m_slave_to_entry_map[slave];
}

std::shared_ptr<ecat_master::EthercatMaster> EthercatDeviceConfigurator::master()
{
    if(m_masters.size() > 1)
        throw std::runtime_error("[EthercatDeviceConfigurator] More than one master configured, use getMasters instead of master");

    if(m_masters.empty())
        throw std::out_of_range("[EthercatDeviceConfigurator] No master configured");

    return m_masters[0];
}

const std::string &EthercatDeviceConfigurator::getSetupFilePath()
{
    return m_setup_file_path;
}

void EthercatDeviceConfigurator::parseFile(std::string path)
{
    //Check if file exists
    if(!path_exists(path))
        throw std::runtime_error("[EthercatDeviceConfigurator] File not found: "+path);
    //Load into yaml
    YAML::Node node = YAML::LoadFile(path);

    //Ethercat master configuration
    if(node["ethercat_master"])
    {
        const auto ecat_master_node = node["ethercat_master"];

        if(ecat_master_node["time_step"])
        {
            m_master_configuration.timeStep = ecat_master_node["time_step"].as<double>();
        }
        else
        {
            throw std::runtime_error("[EthercatDeviceConfigurator] Node time_step missing in ethercat_master");
        }
        if(ecat_master_node["update_rate_too_low_warn_threshold"])
        {
            m_master_configuration.updateRateTooLowWarnThreshold = ecat_master_node["update_rate_too_low_warn_threshold"].as<int>();
        }
        else
        {
            throw std::runtime_error("[EthercatDeviceConfigurator] Node update_rate_too_low_warn_threshold missing in ethercat_master");
        }
    }
    else
    {
        throw std::runtime_error("[EthercatDeviceConfigurator] Node ethercat_master is missing in yaml");
    }

    //Check if node is ethercat_devices
    if(node["ethercat_devices"])
    {
        //Get all children
        const YAML::Node& nodes = node["ethercat_devices"];
        if(nodes.size() == 0)
            throw std::runtime_error("[EthercatDeviceConfigurator] No devices defined in yaml");

        //Iterate through child nodes
        for(YAML::const_iterator it = nodes.begin(); it != nodes.end();++it)
        {
            const YAML::Node& child = *it;
            EthercatSlaveEntry entry;
            //type - entry
            if(child["type"])
            {
                auto type_str = child["type"].as<std::string>();

                if(type_str == "Maxon")
                {
                    entry.type = EthercatSlaveType::Maxon;
                }
                else
                {
                    throw std::runtime_error("[EthercatDeviceConfigurator] " +type_str + " is an undefined type of ethercat device");
                }
            }
            else
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] Node: " + child.Tag() + " has no entry type");
            }

            //name - entry
            if(child["name"])
            {

                entry.name = child["name"].as<std::string>();
            }
            else
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] Node: " + child.Tag() + " has no entry name");
            }

            //configuration_file - entry
            if(child["configuration_file"])
            {
                entry.config_file_path = child["configuration_file"].as<std::string>();
            }
            else
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] Node: " + child.Tag() + " has no entry configuration_file");
            }

            //ethercat_bus_address - entry
            if(child["ethercat_address"])
            {
                entry.ethercat_address = child["ethercat_address"].as<int>();
            }
            else
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] Node: " + child.Tag() + " has no entry ethercat_bus_address");
            }

            //ethercat_bus - entry
            if(child["ethercat_bus"])
            {
                entry.ethercat_bus = child["ethercat_bus"].as<std::string>();
            }
            else
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] Node: " + child.Tag() + " has no entry ethercat_bus");
            }

            //ethercat_pdo_type - entry

            m_slave_entries.push_back(entry);
        }
    }
    else
    {
        throw std::runtime_error("[EthercatDeviceConfigurator] Node ethercat_devices missing in yaml");
    }

}

void EthercatDeviceConfigurator::setup(bool startup)
{
    for(auto & entry: m_slave_entries)
    {
        MELO_DEBUG_STREAM("[EthercatDeviceConfigurator] Creating slave: " << entry.name);

        std::shared_ptr<ecat_master::EthercatDevice> slave = nullptr;

        switch (entry.type) {
    
        case EthercatSlaveType::Maxon:
        {
#ifdef _MAXON_FOUND_
            std::string configuration_file_path = handleFilePath(entry.config_file_path,m_setup_file_path);
            slave = maxon::Maxon::deviceFromFile(configuration_file_path, entry.name, entry.ethercat_address);
#else
            throw std::runtime_error("maxon_epos_ethercat_sdk not availabe.");
#endif

        }
            break;

        default:
            throw std::runtime_error("[EthercatDeviceConfigurator] Not existing EthercatSlaveType passed");
            break;


        }
        m_slaves.push_back(slave);
        m_slave_to_entry_map.insert({slave, entry});
    }


    //Create master for each bus needed

    for(auto & slave: m_slaves)
    {
        //Find entry object for each slave because the slave base class does not provide info about the interface name
        EthercatSlaveEntry entry = m_slave_to_entry_map[slave];

        //See if we already have a master for that interface
        bool master_found = false;
        for(auto & master: m_masters)
        {
            if(master->getConfiguration().networkInterface == entry.ethercat_bus)
            {
                master_found = true;
                //Yes we attach the slave
                if(!master->attachDevice(slave))
                {
                    throw std::runtime_error("[EthercatDeviceConfigurator] could not attach slave: " + slave->getName() + " to master on interface: " + master->getConfiguration().networkInterface);
                }
                break;
            }
        }

        //No we create new master
        if(!master_found)
        {
            std::shared_ptr<ecat_master::EthercatMaster> master = std::make_shared<ecat_master::EthercatMaster>();

            m_master_configuration.networkInterface = entry.ethercat_bus;
            master->loadEthercatMasterConfiguration(m_master_configuration);

            m_masters.push_back(master);

            //And attach the slave
            if(!master->attachDevice(slave))
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] could not attach slave: " + slave->getName() + " to master on interface: " + master->getConfiguration().networkInterface);
            }
        }

    }

    if(startup)
    {
        for(auto & master: m_masters)
        {
            MELO_DEBUG("Starting master on: " + master->getConfiguration().networkInterface)
            if(!master->startup())
            {
                throw std::runtime_error("[EthercatDeviceConfigurator] could not start master on interface: " + master->getConfiguration().networkInterface);
            }
        }
    }



}

std::string EthercatDeviceConfigurator::handleFilePath(const std::string &path, const std::string &setup_file_path) const
{
    std::string result_path = "";
    if (path.front() == '/')
    {
        result_path = path;
        // Path to the configuration file is absolute, we can use it as is.
    }
    else if (path.front() == '~')
    {
        // Path to the configuration file is absolute, we need to replace '~' with the home directory.
        const char* homeDirectory = getenv("HOME");
        if (homeDirectory == nullptr)
            throw std::runtime_error("[EthercatDeviceConfigurator] Environment variable 'HOME' could not be evaluated.");
        result_path = path;
        result_path.erase(result_path.begin());
        result_path = homeDirectory + result_path;
    }
    else
    {
        // Path to the configuration file is relative, we need to append it to the path of the setup file.
        result_path = setup_file_path.substr(0, setup_file_path.find_last_of("/")+1) + path;
    }
    if(!path_exists(result_path))
        throw std::runtime_error("Path: " + result_path + " does not exist");
    return  result_path;
}
