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

#pragma once

#include <memory>
#include <string>
#include <ethercat_sdk_master/EthercatMaster.hpp>
#include <type_traits>

class EthercatDeviceConfigurator
{
public:
    //Convinience typedef for a shared pointer
    typedef std::shared_ptr<EthercatDeviceConfigurator> SharedPtr ;

    //Type ethercat slave device. If you want to wire in a new slave device type, add an entry to this enum
    enum class EthercatSlaveType
    {
        Maxon,
        NA
    };

    struct EthercatSlaveEntry
    {
        EthercatSlaveType type;
        std::string name;
        std::string config_file_path;

        uint32_t ethercat_address;
        std::string ethercat_bus;
        std::string ethercat_pdo_type;
    };
    /**
     * @brief EthercatDeviceConfigurator
     * @param path - path to the setup.yaml
     * @param startup - if true -> calls startup on all masters
     */
    EthercatDeviceConfigurator(std::string path, bool startup = false);
    /**
     * @brief getMasters
     * @return a vector of all masters
     */
    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> getMasters();
    /**
     * @brief getSlaves
     * @return a vector of all slaves
     */
    std::vector<std::shared_ptr<ecat_master::EthercatDevice>> getSlaves();
    /**
     * @brief getSlave - get a certain slave by its name
     * @param name
     * @return shared_ptr on slave
     */
    std::shared_ptr<ecat_master::EthercatDevice> getSlave(std::string name);
    /**
     * @brief getInfoForSlave
     * @param slave - shared ptr on slave
     * @return Info entry parsed from setup.yaml
     */
    const EthercatSlaveEntry& getInfoForSlave(const std::shared_ptr<ecat_master::EthercatDevice>& slave);
    /**
     * @brief master
     * @return pointer on master if only a single master is available
     * @throw std::runtime_error if more than one master is configured
     */
    std::shared_ptr<ecat_master::EthercatMaster> master();

    /**
     * @brief getSetupFilePath
     * @return path to setup file
     */
    const std::string& getSetupFilePath();

    /**
     * @brief getSlavesOfType - return all slaves of type T (vector of shared_ptr).
     * @note Warning cache the result if you need them on a regular base. Might have bad performance
     */
    template<typename T, typename dummy = std::enable_if_t<std::is_base_of_v<ecat_master::EthercatDevice, T>>>
    std::vector<std::shared_ptr<T>> getSlavesOfType()
    {

        std::vector<std::shared_ptr<T>> slaves;

        for(auto & slave: m_slaves)
        {
            auto ptr = std::dynamic_pointer_cast<T>(slave);
            if(ptr)
            {
                slaves.push_back(ptr);
            }
        }
        return slaves;
    }

private:
    //Stores the general master configuration.
    //If slaves on multiple bus interfaces are detected, the bus interface in this object will be the interface of the last configured interface
    ecat_master::EthercatMasterConfiguration m_master_configuration;
    //Vector of all configured masters
    std::vector<std::shared_ptr<ecat_master::EthercatMaster>> m_masters;
    //Vecotr of all configured slaves (For all masters)
    std::vector<std::shared_ptr<ecat_master::EthercatDevice>> m_slaves;

    //List of all parsed slave entries from the setup.yaml
    std::vector<EthercatSlaveEntry> m_slave_entries;
    //Map that helps finding the right slave entry for a certain slave
    std::map<std::shared_ptr<ecat_master::EthercatDevice>, EthercatSlaveEntry> m_slave_to_entry_map;


    /*Internal methods*/

    /**
     * @brief parseFile - parses a setup.yaml. This methods adds the found entries in the m_slave_entries list and sets the m_master_configuration (without the bus interface)
     * @param path
     */
    void parseFile(std::string path);
    /**
     * @brief setup - uses the m_slave_entries to create slaves and bus masters. Attaches the slaves to the bus master. Can startup the bus
     * @param startup - true: call startup for all busses
     */
    void setup(bool startup);
    /**
     * @brief handleFilePath - helps with parsing file paths in the setup.yaml
     * @param path
     * @param setup_file_path
     * @return
     */
    std::string handleFilePath(const std::string& path, const std::string &setup_file_path) const;

    //Path to the setup file
    std::string m_setup_file_path ="";
};
