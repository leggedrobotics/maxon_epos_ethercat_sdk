# EtherCAT Device Configurator
Configures the EtherCAT communication with the following sdks:

- anydrive_sdk
- elmo_ethercat_sdk
- rokubimini_ethercat_sdk
- elmo_ethercat_sdk 
- maxon_epos_ethercat_sdk

Any single SDK or any combination of SDKs may be used.
The build system automatically builds the EtherCAT device SDKs available in the current catkin workspace.

In the future this will be changed to a plugin architecture

## Building
Requires gcc ≥ 7.5 (default for Ubuntu ≥ 18.04).

### Dependencies
- __optional__ anydrive_sdk (master)
- __optional__ rokubimini_ethercat_sdk (master)
- __optional__ elmo_ethercat_sdk (master)
- __optional__ maxon_epos_ethercat_sdk (master)
- ethercat_sdk_master (master)
- soem_interface (release)
- message_logger (master)
- any_node (master)
- yaml-cpp (system install)
