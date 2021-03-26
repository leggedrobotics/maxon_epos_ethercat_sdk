# Maxon EPOS Ethercat SDK Usage

## Configuration

The `maxon::Maxon::configParam()` function contains all the configuration parameters required by the implemented modes of operation:

- `PVM`
- `CST`
- `CSP`
- `CSV`

You can edit all available parameters in the sample configuration file [Maxon.yaml](example_configs/Maxon.yaml). The feedback control parameters (P & I gains) for velocity, current, and position are temporarily hard-coded in [ConfigureParameters.cpp](src/maxon_epos_ethercat_sdk/ConfigureParameters.cpp). They are exported from EPOS Studio after performing `Auto Tune`.

You can also ignore the function `maxon::Maxon::configParam()` after you set up the driver correctly in EPOS Studio. The parameter values are stored in non-volatile storage on-drive.

### Configuring mode(s) of operation

The Maxon EPOS driver is capable of switching mode of operation on-fly. Desired modes can be added to the [Maxon.yaml](example_configs/Maxon.yaml) file:

```yaml
Hardware:
  mode_of_operation:
    - DesiredMode1
    - DesiredMode2
```

The driver can switch between the selected modes. Currently the following modes are supported:

| Single Mode of Operation | Multiple Modes of Operation |
| ------------------------ | --------------------------- |
| PVM                      | CST and CSP                 |
| CSP                      | CST, CSP, and CSV           |
| CST                      |
| CSV                      |

### Adding mode(s) of operation

If the desired mode of operation is not supported, they can be added with these steps:

1. Create add corresponding `Pdo` types to `RxPdoTypeEnum` an `TxPdoTypeEnum` in [PdoTypeEnum.hpp](include/maxon_epos_ethercat_sdk/PdoTypeEnum.hpp), the `operator<<` methods in [PdoTypeEnum.cpp](src/maxon_epos_ethercat_sdk/PdoTypeEnum.cpp), and `std::string modeOfOperationString(ModeOfOperationEnum modeOfOperation_)` & `std::string *xPdoString(*xPdoTypeEnum *xPdo)` methods in [Configuration.cpp](src/maxon_epos_ethercat_sdk/Configuration.cpp).
 
    For example, for the mixed CSP and CST mode, `Pdo` type `*xPdoCSTCSP` is created in [PdoTypeEnum.hpp](include/maxon_epos_ethercat_sdk/PdoTypeEnum.hpp):

    ```c++
    // different RxPdo Types
    enum class RxPdoTypeEnum : int8_t
    {
      NA = 0,
      RxPdoStandard,
      RxPdoCSP,
      RxPdoCST,
      RxPdoCSV,
      RxPdoCSTCSP,
      RxPdoPVM
    };

    // different TxPdo Types
    enum class TxPdoTypeEnum : int8_t
    {
      NA = -128,
      TxPdoStandard,
      TxPdoCSP,
      TxPdoCST,
      TxPdoCSV,
      TxPdoCSTCSP,
      TxPdoPVM
    };
    ```

2. Then add `struct`s for the `Pdo` types in [RxPdo.hpp](include/maxon_epos_ethercat_sdk/RxPdo.hpp) and [TxPdo.hpp](include/maxon_epos_ethercat_sdk/TxPdo.hpp):
  
    ```c++
    struct RxPdoCSTCSP
    {
      int16_t targetTorque_;
      int16_t torqueOffset_;
      int32_t targetPosition_;
      int32_t positionOffset_;
      uint16_t controlWord_;
      int8_t modeOfOperation_;
    }  __attribute__((packed));
    ```

    and:

    ```c++
    struct TxPdoCSTCSP
    {
      uint16_t statusword_;
      int16_t actualTorque_;
      int32_t actualVelocity_;
      int32_t actualPosition_;
    } __attribute__((packed));
    ```

    **Note**: the `RxPdo` `struct` contains all command parameters required by **all** modes, and the **control word** and **mode of operation** even not required by the firmware documentation. Similarly, the `TxPdo` `struct` contains all output data and **status word**.

3. If new combination modes of operation are desired, in [Configuration.cpp](src/maxon_epos_ethercat_sdk/Configuration.cpp), add your combination of operation modes and `Rx/TxPdo` modes in the method `std::pair<RxPdoTypeEnum, TxPdoTypeEnum> Configuration::getPdoTypeSolution() const`.

4. Then in [ConfigureParameters.cpp](src/maxon_epos_ethercat_sdk/ConfigureParameters.cpp), add your `Pdo` mappings to `bool Maxon::mapPdos()`. e.g.:

    <details>
      <summary>RxPdo</summary>

    ```c++
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
    ```
    </details>

    <details>
      <summary>TxPdo</summary>

    ```c++
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
    ```
    </details>

    This tells the driver what data to receive and send, therefore, the objects in the `Pdo`s **must have the same order** as the `struct`s.

5. Then in [ConfigureParameters.cpp](src/maxon_epos_ethercat_sdk/ConfigureParameters.cpp), add the parameters you wish to configure to `bool Maxon::configParam()`.

6. To add more variables configurable via `Maxon.yaml`, modify:

  - [Configuration.hpp](include/maxon_epos_ethercat_sdk/Configuration.hpp)
  - [ConfigurationParser.cpp](src/maxon_epos_ethercat_sdk/ConfigurationParser.cpp)
  - [ConfigureParameters.cpp](src/maxon_epos_ethercat_sdk/ConfigureParameters.cpp)
  - [Command.cpp](src/maxon_epos_ethercat_sdk/Command.cpp) and [Command.hpp](src/maxon_epos_ethercat_sdk/Command.hpp)
  - [Reading.cpp](src/maxon_epos_ethercat_sdk/Reading.cpp) and [Reading.hpp](src/maxon_epos_ethercat_sdk/Reading.hpp)

  Also refers to the firmware specification for the required parameters. **Note**: `statusword` and `controlword` are always required in the `Pdo`s, despite the omission in the documentation.

## Usage

See [ethercat_device_configurator](https://github.com/leggedrobotics/ethercat_device_configurator) for an minimal working example. In particular, to set mode of operation and/or send command during cyclic synchronous update, change the content inside `void worker()`'s `while` loop of [standalone.cpp](https://github.com/leggedrobotics/ethercat_device_configurator/blob/master/src/standalone.cpp). For example, the existing commands are:

```c++
maxon::Command command; // Create command object
command.setModeOfOperation(maxon::ModeOfOperationEnum::CyclicSynchronousTorqueMode); // Set to CST mode
auto reading = maxon_slave_ptr->getReading(); // Read from TxPdo
command.setTargetPosition(reading.getActualPosition() + 10); // Target +10 rad from current position, but there is no effect since the driver is not in CSP mode
command.setTargetTorque(-0.5); // Apply -0.5 Nm torque
maxon_slave_ptr->stageCommand(command); // Send command to the driver
```

## Comparison to `elmo_ethercat_sdk`

### Unit conversions

Both `maxon_epos_ethercat_sdk` and `elmo_ethercat_sdk` take SI units for velocity, position, and torque for the `command` and `reading` API. The raw data for communication with the driver has different units:

| Quantity                 | Maxon EPOS              | Elmo                                                                        |
| ------------------------ | ----------------------- | --------------------------------------------------------------------------- |
| Velocity                 | $\mu RPM$               | encoder increments                                                          |
| Torque (General)         | $\mu Nm$                | ‰ of motor rated current (motor constant is explicitly used for conversion) |
| Torque (Target & Offset) | ‰ of motor rated torque | ‰ of motor rated current (motor constant is explicitly used for conversion) |
| Position                 | $inc$                   | encoder increments                                                          |

### Update rate

In cyclic communication, we recommend to set the update rate for Maxon EPOS slaves to at least `500 Hz`. Elmo specifies the update rate to be a multiple of `200 Hz` and does not handle latency spikes. A real-time master might be required to run both at stable high refresh rate. No actual testing is done.

### Configuration parameters

Maxon EPOS and Elmo takes different configuration parameters. See [sample config](example_configs/Maxon.yaml) for all parameters the current implementation writes to the Maxon EPOS driver at configuration phrase.

### Others

Refer to the documentations for EPOS-specific `Object Dictionary`, `Sdo`, and `Pdo`, etc. implementations ([Reference](##Reference)).

## Reference

- EPOS4 Firmware documentation: [EPOS4-Firmware-Specification-En.pdf](https://www.maxongroup.com/medias/sys_master/root/8839867007006/EPOS4-Firmware-Specification-En.pdf) ([Archive](https://web.archive.org/web/20200918110944/https://www.maxongroup.com/medias/sys_master/root/8839867007006/EPOS4-Firmware-Specification-En.pdf))
