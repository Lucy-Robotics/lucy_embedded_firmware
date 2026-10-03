# Firmware architecture (one binary per board)

Detail for **`lucy_embedded_firmware`**.  
**System schematic:** [`lucy_ws/docs/architecture/overview.md`](../../../../docs/architecture/overview.md)  
**Index:** [`lucy_ws/docs/architecture/README.md`](../../../../docs/architecture/README.md)  
**Conventions:** [`lucy_ws/docs/architecture/GUIDE.md`](../../../../docs/architecture/GUIDE.md)

## Crate layout

**UML Component (firmware crates)** - YAML codegen into shared banks and thin board mains.

```mermaid
%%{init: {"theme": "base", "themeVariables": {"darkMode": true, "background": "#0d1117", "mainBkg": "#21262d", "primaryColor": "#21262d", "primaryTextColor": "#f0f6fc", "primaryBorderColor": "#00FF41", "secondaryColor": "#161b22", "secondaryTextColor": "#f0f6fc", "secondaryBorderColor": "#00FF41", "tertiaryColor": "#161b22", "tertiaryTextColor": "#f0f6fc", "tertiaryBorderColor": "#00FF41", "lineColor": "#00FF41", "textColor": "#f0f6fc", "nodeTextColor": "#f0f6fc", "edgeLabelBackground": "#161b22", "clusterBkg": "#0d1117", "clusterBorder": "#00FF41", "titleColor": "#f0f6fc"}}}%%
flowchart TB
  YAML["config.yaml"] -.-> Builder["builder"]
  Builder -.-> Cfg["GENERATED_tables"]
  subgraph support ["rp2040_support"]
    USB["USB_CDC"]
    Modbus["ModbusCdcState"]
    PWM["PwmBank"]
    Bus["BusBank"]
    I2C["I2cPwmBank"]
    ADC["AdcBank"]
  end
  Cfg --> USB
  Cfg --> Modbus
  Cfg -->|"HAS_PWM"| PWM
  Cfg -->|"HAS_BUS"| Bus
  Cfg -->|"HAS_I2C_PWM"| I2C
  Cfg -->|"HAS_ADC"| ADC
  Core["lucy_embedded_firmware_core"]
  Core --> Builder
  Core --> support
  S2040["rp2040_servo2040"] --> support
  linkStyle default stroke:#00FF41,stroke-width:2px
```

**UML Class (support banks)** - shared `rp2040_support` utilities and bank structs.

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
  BusBank --> Rp2040UartChannel : uses
```

The `rp2040_support` crate box lists utilities (types + builder function) available
to all banks. Arrows show which bank structs the support crate provides, and which
peripheral drivers a bank uses (e.g., `BusBank` uses `Rp2040UartChannel`).

## Board → package

| Physical board | Package | Enabled banks (from robot YAML `board_class`) |
|----------------|---------|------------------------------------------------|
| Pimoroni Servo2040 | `rp2040_servo2040` | `internal_servo_only` → PWM<br>`internal_servo_i2c_pwm` → PWM + I2C_PWM<br>`bus_servo_only` → BUS |

One board → one firmware package → one UF2 per robot config. Robot YAML
`board_class` determines which device tables are generated; banks initialize
only when their `GENERATED_HAS_*` const is true (non-empty table).

## Codegen device tables

| Table | Hardware identity | Bank |
|-------|-------------------|------|
| `GENERATED_PWM_DEVICES` | `PwmGpio` (`ServoN`) | `PwmBank` |
| `GENERATED_BUS_DEVICES` | `UartBus` (`UART0:id`) | `BusBank` |
| `GENERATED_I2C_PWM_DEVICES` | `I2cPwm` (`I2C0:PCA9685:N`) | `I2cPwmBank` (placeholder HW) |
| `GENERATED_ADC_DEVICES` | `Adc` (`ADC0`…) | `AdcBank` (placeholder until ADC wired) |

## Register blocks

| Driver | Registers |
|--------|-----------|
| PwmServo / I2cPwm | 2 - cmd, angle (millirad) |
| BusServo | 3 - id, angle, cmd |
| PressureSensor | 2 - cmd, value |

## Related

- Firmware README: [`../../README.md`](../../README.md)
- Workspace overview: [`lucy_ws/docs/architecture/overview.md`](../../../../docs/architecture/overview.md)
- Architecture guide: [`lucy_ws/docs/architecture/GUIDE.md`](../../../../docs/architecture/GUIDE.md)
