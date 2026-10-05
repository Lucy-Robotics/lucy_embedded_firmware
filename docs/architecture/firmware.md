# Firmware architecture (one binary per board)

Detail for **`lucy_embedded_firmware`**.

**Workspace overview (when checked out under `lucy_ws/src/`):**  
[`lucy_control_panel/docs/architecture/overview.md`](../../../lucy_control_panel/docs/architecture/overview.md)  
**ROS pipeline / SHM:** [`lucy_ros_packages/docs/architecture/pipeline_shm.md`](../../../lucy_ros_packages/docs/architecture/pipeline_shm.md)  
**Index:** [README.md](README.md)

> **Branch note.** Schematics below describe the **Servo2040 unification** on `cma/fw-boards` (`rp2040_servo2040` + `rp2040_support`) and the **host Feetech** direction on WIP `mbo/feat-protocol`. This docs PR may sit on an older tree (`firmwares/rp2040`); treat package names as the intended tip, not necessarily files on this branch tip.

## Current vs target control paths

### What tips ship today (`cma/fw-boards` + ros Modbus tips)

Host `LucySystemHardware` writes **pulse `u16` dirty registers** in POSIX SHM; **`lucy_modbus_bridge`** (on ros `cma/pipeline-flash`) polls SHM and sends **Modbus RTU FC06 over USB CDC** to the RP2040. The MCU never mmaps host SHM.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  HI["LucySystemHardware"]
  SHM["POSIX_SHM_pulse_regs"]
  Bridge["lucy_modbus_bridge"]
  S2040["rp2040_servo2040"]
  HI -->|"u16 pulse + dirty"| SHM
  Bridge -->|"poll SHM"| SHM
  Bridge -->|"Modbus FC06 USB CDC"| S2040
  S2040 -->|"HAS_PWM"| PWM["PwmBank"]
  S2040 -->|"HAS_BUS UART0 Feetech"| Bus["BusBank"]
  S2040 -->|"HAS_I2C_PWM placeholder"| I2C["I2cPcaBank"]
  linkStyle default stroke:#00FF41,stroke-width:2px
```

| Robot / profile | Board path today |
|-----------------|------------------|
| **SO-ARM101** | `board_class: bus_servo_only` → `rp2040_servo2040` + `HAS_BUS` → **UART0** Feetech (GPIO0 TX, GPIO1 RX, GPIO2 DIR @ 1 Mbaud), still commanded via Modbus CDC |
| **InMoov** | Servo2040 PWM / I2C-PCA / ADC tables as YAML enables (I2C/ADC banks may still be Modbus placeholders until HW is wired) |

Feetech on the Servo2040 path is **UART half-duplex** (not “Bus” as a protocol). RX/position feedback on that UART channel is still limited on current tips (TX-oriented).

### Target (WIP — not on this PR / not end-to-end yet)

Align with m-brl: HI writes **`f64` radians** into POSIX SHM (`ActuatorSharedState` / `JointTable`). **Do not rebase** onto unfinished `mbo/feat-protocol` yet; match the contract when integrating.

| Consumer | Role | Status |
|----------|------|--------|
| `firmwares/linux` | Host process mmaps SHM → **USB serial** Feetech (SO-ARM101 alternate) | WIP on `mbo/feat-protocol` |
| Host bridge → RP2040 | Something must still carry commands to the MCU (today: Modbus; future: TBD / `mcu_bridge`) | **MCU cannot mmap SHM** |
| Servo2040 banks | Same UART0 Feetech / PWM / I2C profiles | Keep both SO101 Feetech options once host path lands |

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  HI["LucySystemHardware"]
  SHM["POSIX_SHM_f64_JointTable"]
  LinuxFW["firmwares_linux_WIP"]
  Bridge["host_to_MCU_bridge"]
  S2040["rp2040_servo2040"]
  HI --> SHM
  SHM -->|"SO101_host_USB_Feetech"| LinuxFW
  SHM -->|"not_on_MCU"| Bridge
  Bridge -->|"CDC_or_future"| S2040
  linkStyle default stroke:#00FF41,stroke-width:2px
```

**ABI caveat:** ros [#63](https://github.com/Lucy-Robotics/lucy_ros_packages/pull/63) `ActuatorSharedState` uses flat `double[MAX_ACTUATORS]` with `MAX_ACTUATORS=32`. Firmware `JointTable<N>` nests `CommandBlock` / `StateBlock` / `HeartbeatBlock`. Field sets match for the same `N`, but `firmwares/linux` currently maps `JointTable<6>` — not a drop-in overlay on N=32 without agreeing `N`.

## Crate layout (`cma/fw-boards`)

**UML Component** — YAML codegen into shared banks and thin board mains.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  YAML["config.yaml"] -.-> Builder["builder_build_dep"]
  Builder -.-> Cfg["GENERATED_tables"]
  subgraph support ["rp2040_support"]
    USB["USB_CDC"]
    Modbus["ModbusCdcState"]
    PWM["PwmBank"]
    UartBus["BusBank_UART0_Feetech"]
    I2C["I2cPwmBank_PCA9685"]
    ADC["AdcBank"]
  end
  Cfg --> USB
  Cfg --> Modbus
  Cfg -->|"HAS_PWM"| PWM
  Cfg -->|"HAS_BUS"| UartBus
  Cfg -->|"HAS_I2C_PWM"| I2C
  Cfg -->|"HAS_ADC"| ADC
  Core["lucy_embedded_firmware_core"]
  support --> Core
  S2040["rp2040_servo2040"] --> support
  S2040 -.-> Builder
  linkStyle default stroke:#00FF41,stroke-width:2px
```

Cargo reality: **`rp2040_support` depends on `core`**; **`builder` is a build-dependency of the board firmware**, not of `core`.

**UML Class (support banks)**

```mermaid
classDiagram
  class rp2040_support {
    <<utility>>
    +PicoToolReset
    +ModbusCdcState
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

Rust names: `BusBank` = UART0 Feetech/STS; `I2cPwmBank` = PCA9685 over **I2C** (PWM is on the expander). Builder **rejects** YAML that enables UART bus together with PWM on Servo1–3 (GPIO0–2), and rejects `UART` ≠ 0 on Servo2040.

## Board → package

| Physical board | Package | Enabled banks (from robot YAML `board_class`) |
|----------------|---------|------------------------------------------------|
| Pimoroni Servo2040 | `rp2040_servo2040` | `internal_servo_only` → PWM<br>`internal_servo_i2c_pwm` → PWM + I2C PCA9685<br>`bus_servo_only` → UART0 Feetech (`HAS_BUS`) |

`board_class` is a **profile**; banks initialize only when `GENERATED_HAS_*` is true. PWM + UART0 can coexist only if PWM uses pads **outside** GPIO0–2 (builder-enforced).

## Codegen device tables

| Table | Hardware identity | Bank / wire |
|-------|-------------------|-------------|
| `GENERATED_PWM_DEVICES` | `PwmGpio` (`ServoN`) | `PwmBank` (RP2040 PWM) |
| `GENERATED_BUS_DEVICES` | `UartBus` (`UART0:id`) | `BusBank` → **UART0** half-duplex Feetech |
| `GENERATED_I2C_PWM_DEVICES` | `I2cPwm` (`I2C0:PCA9685:N`) | `I2cPwmBank` → **I2C** to PCA9685 (placeholder HW on some tips) |
| `GENERATED_ADC_DEVICES` | `Adc` (`ADC0`…) | `AdcBank` (placeholder until ADC wired) |

## Angles

| Layer | Unit today | Target |
|-------|------------|--------|
| Robot YAML / HI command interfaces | radians in YAML on rad tips | radians |
| Host SHM (Modbus tips) | pulse `u16` + dirty bits | — |
| Host SHM (WIP #63 / `JointTable`) | — | **`f64` rad** arrays + seq counters |
| Feetech ticks | driver maps rad ↔ STS ticks (`0…4095` typical) | unchanged |

## Related

- Firmware README: [`../../README.md`](../../README.md)
- Control-panel overview: [`../../../lucy_control_panel/docs/architecture/overview.md`](../../../lucy_control_panel/docs/architecture/overview.md)
- ROS pipeline / SHM: [`../../../lucy_ros_packages/docs/architecture/pipeline_shm.md`](../../../lucy_ros_packages/docs/architecture/pipeline_shm.md)
