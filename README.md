# Maxon EtherCAT SDK

## Overview

This is a C++ library providing a high-level interface for controlling [Maxon](https://www.maxonmc.com/) motor drivers of the [EPOS line](https://www.maxongroup.com/maxon/view/content/epos-detailsite) over EtherCAT (using the [CANopen over EtherCAT CoE](https://www.ethercat.org/en/technology.html#1.9.1) protocol). It is modified by Linghao Zhang based on [elmo_ethercat_sdk](https://github.com/leggedrobotics/elmo_ethercat_sdk) by Jonas Junger.

The lower level EtherCAT communication is handled by the [soem_interface](https://github.com/leggedrobotics/soem_interface) library.

The `maxon_epos_ethercat_sdk` is developed on Ubuntu 20.04 LTS with [ROS Noetic](https://wiki.ros.org/noetic).

The source code is released under the BSD-3-Clause license.
A copy of the license is available in the [LICENSE](LICENSE) file.

**Authors:** Linghao Zhang, Jonas Junger, Lennart Nachtigall

**Maintainer:** Linghao Zhang, lingzhang@ethz.ch

**Contributors:** Fabio Dubois, Markus Staeuble, Martin Wermelinger

## Installation

### Dependencies

#### Catkin Packages

|        Repo         |                          url                          |   License    |               Content               |
| :-----------------: | :---------------------------------------------------: | :----------: | :---------------------------------: |
|   soem_interface    | https://github.com/leggedrobotics/soem_interface.git  |    GPLv3     | Low-level EtherCAT functionalities  |
| ethercat_sdk_master | https://github.com/leggedrobotics/ethercat_sdk_master | BSD 3-Clause | High-level EtherCAT functionalities |
|   message_logger    | https://github.com/leggedrobotics/message_logger.git  | BSD 3-Clause |         simple log streams          |

#### System Dependencies (tested on Ubuntu 20.04 LTS)

- [ROS Noetic](https://wiki.ros.org/noetic)
- catkin
- yaml-cpp

> Likely to work with [ROS Meolodic](https://wiki.ros.org/melodic) and Ubuntu 18.04 LTS

### Building from Source

To build the library from source, clone the latest version from this repository and from the dependencies into your catkin workspace and compile the package using

```bash
cd catkin_workspace/src
git clone https://github.com/leggedrobotics/soem_interface.git
git clone https://github.com/leggedrobotics/ethercat_sdk_master.git
git clone https://github.com/leggedrobotics/message_logger.git
git clone https://github.com/leggedrobotics/maxon_epos_ethercat_sdk.git
cd ../
catkin build maxon_epos_ethercat_sdk
```

## Example

See [ethercat_device_configurator](https://github.com/leggedrobotics/ethercat_device_configurator) for an minimal working example.

## Usage

See [Usage.md](doc/Usage.md).

## Contributing

See [Contributing.md](Contributing.md)
