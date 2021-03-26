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

#pragma once

#include <yaml-cpp/yaml.h>

#include <message_logger/message_logger.hpp>
#include <string>

#include "maxon_epos_ethercat_sdk/Configuration.hpp"

namespace maxon {
/*!
 * @brief	Read configuration data from a yaml file
 */
class ConfigurationParser {
 public:
  /*!
   * no default constructor
   */
  ConfigurationParser() = delete;

  /*!
   * @brief	Constructor
   * Using an absolute path may be preferable, depending on the way the final
   * program is being run.
   * @param filename	The path to the configuration file
   */
  explicit ConfigurationParser(const std::string& filename);

  /*!
   * @brief	Constructor
   * This enables the use of a yaml node instead of a yaml file for the
   * configuration. This is useful for the creation of nested config files
   * @param configNode	The yaml node containing the configuration data
   */
  explicit ConfigurationParser(YAML::Node configNode);

  /*!
   * @brief	return the configuration
   * The Configuration object is filled according to the specified configuration
   * file. Any missing parameters will be automatically filled with well tested
   * default values.
   * @return	A configured maxon::ethercat::Configuration object
   */
  Configuration getConfiguration() const;

 private:
  /*!
   * @brief	Parse the configuration data
   * @param configNode	yaml node containing the config data
   */
  void parseConfiguration(YAML::Node configNode);
  /*!
   * The Configuration object
   */
  Configuration configuration_;
};

}  // namespace maxon
