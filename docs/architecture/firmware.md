# Firmware architecture (one binary per board)

Detail for **`lucy_embedded_firmware`**.

**Workspace overview (when checked out under `lucy_ws/src/`):**  
[`lucy_control_panel/docs/architecture/overview.md`](../../../lucy_control_panel/docs/architecture/overview.md)  
**ROS pipeline / SHM:** [`lucy_ros_packages/docs/architecture/pipeline_shm.md`](../../../lucy_ros_packages/docs/architecture/pipeline_shm.md)  
**Index:** [README.md](README.md)

## Control path

Host `LucySystemHardware` writes **`f64` radians** into POSIX SHM (`ActuatorSharedState` / `JointTable`). The MCU never mmaps host SHM.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  HI["LucySystemHardware"]
  SHM["POSIX_SHM_f64_JointTable"]
  LinuxFW["firmwares_linux"]
  Bridge["host_to_MCU_bridge"]
  S2040["rp2040_servo2040"]
  HI --> SHM
  SHM -->|"SO101_host_USB_Feetech"| LinuxFW
  SHM -->|"not_on_MCU"| Bridge
  Bridge -->|"CDC_or_serial"| S2040
  S2040 -->|"HAS_PWM"| PWM["PwmBank"]
  S2040 -->|"HAS_BUS UART0 Feetech"| Bus["BusBank"]
  S2040 -->|"HAS_I2C_PWM"| I2C["I2cPcaBank"]
  S2040 -->|"HAS_ADC"| ADC["AdcBank"]
  linkStyle default stroke:#00FF41,stroke-width:2px
```

| Robot / profile | Path |
|-----------------|------|
| **SO-ARM101** | Prefer `firmwares/linux` mmaps SHM → **USB serial** Feetech. Alternate: Servo2040 `bus_servo_only` → UART0 Feetech behind the host↔MCU bridge. |
| **InMoov** | Servo2040 PWM / I2C-PCA / ADC profiles (`board_class`) behind the host↔MCU bridge. |

Feetech on Servo2040 is **UART half-duplex** (GPIO0 TX, GPIO1 RX, GPIO2 DIR @ 1 Mbaud) — not “Bus” as a protocol name beyond the bank type.

**ABI:** ros [`ActuatorSharedState`](https://github.com/Lucy-Robotics/lucy_ros_packages/pull/63) uses flat `double[MAX_ACTUATORS]` (`MAX_ACTUATORS=32`). Firmware `JointTable<N>` nests `CommandBlock` / `StateBlock` / `HeartbeatBlock`. Field sets match for the same `N`; agree `N` before overlaying host and MCU consumers.

## Crate layout

**UML Component** — YAML codegen into shared banks and thin board mains.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  YAML["config.yaml"] -.-> Builder["builder_build_dep"]
  Builder -.-> Cfg["GENERATED_tables"]
  subgraph support ["rp2040_support"]
    USB["USB_CDC"]
    Link["host_link_CDC"]
    PWM["PwmBank"]
    UartBus["BusBank_UART0_Feetech"]
    I2C["I2cPwmBank_PCA9685"]
    ADC["AdcBank"]
  end
  Cfg --> USB
  Cfg --> Link
  Cfg -->|"HAS_PWM"| PWM
  Cfg -->|"HAS_BUS"| UartBus
  Cfg -->|"HAS_I2C_PWM"| I2C
  Cfg -->|"HAS_ADC"| ADC
  Core["lucy_embedded_firmware_core"]
  support --> Core
  S2040["rp2040_servo2040"] --> support
  S2040 -.-> Builder
  Linux["firmwares_linux"] --> Core
  linkStyle default stroke:#00FF41,stroke-width:2px
```

Cargo reality: **`rp2040_support` depends on `core`**; **`builder` is a build-dependency of the board firmware**, not of `core`. Host Feetech lives under `firmwares/linux` and mmaps the same SHM contract.

**UML Class (support banks)**

```mermaid
classDiagram
  class rp2040_support {
    <<utility>>
    +PicoToolReset
    +build_usb_device()
  }
  class PwmBank
  class BusBank
  class I2cPwmBank
  class AdcBank
  class Rp2040UartChannel

  rp2040_support ..> PwmBank : provides
  rp2040_support ..> BusBank : provides
  rp2040_support ..> I2cPwmBank : provides
  rp2040_support ..> AdcBank : provides
  BusBank --> Rp2040UartChannel : UART0_Feetech
```

Rust names: `BusBank` = UART0 Feetech/STS; `I2cPwmBank` = PCA9685 over **I2C**. Builder **rejects** YAML that enables UART bus together with PWM on Servo1–3 (GPIO0–2), and rejects `UART` ≠ 0 on Servo2040.

## Board → package

| Physical board | Package | Enabled banks (from robot YAML `board_class`) |
|----------------|---------|------------------------------------------------|
| Pimoroni Servo2040 | `rp2040_servo2040` | `internal_servo_only` → PWM<br>`internal_servo_i2c_pwm` → PWM + I2C PCA9685<br>`bus_servo_only` → UART0 Feetech (`HAS_BUS`) |
| Host (SO-ARM101) | `firmwares/linux` | USB serial Feetech from f64 SHM |

`board_class` is a **profile**; banks initialize only when `GENERATED_HAS_*` is true. PWM + UART0 can coexist only if PWM uses pads **outside** GPIO0–2 (builder-enforced).

## Codegen device tables

| Table | Hardware identity | Bank / wire |
|-------|-------------------|-------------|
| `GENERATED_PWM_DEVICES` | `PwmGpio` (`ServoN`) | `PwmBank` (RP2040 PWM) |
| `GENERATED_BUS_DEVICES` | `UartBus` (`UART0:id`) | `BusBank` → **UART0** half-duplex Feetech |
| `GENERATED_I2C_PWM_DEVICES` | `I2cPwm` (`I2C0:PCA9685:N`) | `I2cPwmBank` → **I2C** to PCA9685 |
| `GENERATED_ADC_DEVICES` | `Adc` (`ADC0`…) | `AdcBank` |

## Angles

| Layer | Unit |
|-------|------|
| Robot YAML / HI command interfaces | **radians** |
| Host SHM (`ActuatorSharedState` / `JointTable`) | **`f64` radians** + sequence counters |
| Feetech / PWM drivers | rad ↔ STS ticks or PWM pulse at the driver edge |

## Related

- Firmware README: [`../../README.md`](../../README.md)
- Control-panel overview: [`../../../lucy_control_panel/docs/architecture/overview.md`](../../../lucy_control_panel/docs/architecture/overview.md)
- ROS pipeline / SHM: [`../../../lucy_ros_packages/docs/architecture/pipeline_shm.md`](../../../lucy_ros_packages/docs/architecture/pipeline_shm.md)
