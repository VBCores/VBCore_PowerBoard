# STM32 Power Distribution Board - UART Configuration Interface

The board uses a UART-based serial interface to configure parameters and control system behavior.

## **Connection Details**

- **Baud Rate**: 115200.
- **Format**: Commands are ASCII strings, terminated with `\r`, `\n`, or spaces (automatically stripped).

---

## **Command Syntax**

- **Enter Config Mode**: Send `START` to enable configuration.
- **Get Parameter**: `<parameter_name>:?`
  Example: `node_id:?` → Responds with `node_id:9`
- **Set Parameter**: `<parameter_name>:<value>`
  Example: `uvlo_level:24.5` → Responds with `OK: uvlo_level:24.50`
- **System Commands**: `APPLY`, `RESET`, `STOP` (see below).

---

## **Configuration Workflow**

1. **Enter Config Mode**: Send `START` to view current settings and unlock configuration.
2. **Set Parameters**: Use `<param>:<value>` syntax to adjust settings.
3. **Save & Apply**:
   - Send `APPLY` to reboot the system and apply changes.
   - Send `STOP` to exit config mode. Changes will be applied on reboot

---

## **Parameters**

| Parameter            | Description                          | Type    | Example Values |
|-----------------------|--------------------------------------|---------|----------------|
| `node_id`             | CAN node ID                          | Integer | `1`, `42`      |
| `data_baud`           | FDCAN data channel baud rate         | Integer | Predefined enum|
| `nominal_baud`        | FDCAN nominal channel baud rate      | Integer | Predefined enum|
| `auto_disarm`         | Auto-disable output on fault         | Boolean | `0` (off), `1` (on)|
| `uvlo_level`          | Under-voltage lockout threshold (V)  | Float   | `22.5`, `24.0` |
| `uvlo_hyst`           | UVLO hysteresis voltage (V)          | Float   | `0.5`, `1.0`   |
| `charge_current`      | Nominal charging current (A)         | Float   | `2.0`, `5.5`   |
| `charged_level`       | Battery "charged" voltage level (V)  | Float   | `28.0`, `29.5` |

## FDCAN Baud Rate Configuration

| parameter          | Value Name | Speed    | Numeric Value |
|---------------------|------------|----------|---------------|
| `nominal_baud`  | `KHz62`    | 62.5 kHz | `0`           |
|                     | `KHz125`   | 125 kHz  | `1`           |
|                     | `KHz250`   | 250 kHz  | `2`           |
|                     | `KHz500`   | 500 kHz  | `3`           |
|                     | `KHz1000`  | 1 MHz    | `4`           |
|---------------------|------------|----------|---------------|
| `data_baud`     | `KHz1000`  | 1 MHz    | `0`           |
|                     | `KHz2000`  | 2 MHz    | `1`           |
|                     | `KHz4000`  | 4 MHz    | `2`           |
|                     | `KHz8000`  | 8 MHz    | `3`           |

---

## **System Commands**

| Command   | Description                                                                 |
|-----------|-----------------------------------------------------------------------------|
| `START`   | Enter config mode. Displays current parameters.                            |
| `APPLY`   | Reboot the system (via `NVIC_SystemReset`) to apply changes.               |
| `RESET`   | Reset configuration to defaults (requires `APPLY` to take effect).          |
| `STOP`    | Exit config mode without applying changes.                                  |

---

## **Response Format**

- **Success**: `OK: <param>:<value>` (for set operations).
- **Error**: `ERROR: <reason>` (e.g., `ERROR: Invalid value`).
- **Auto-Save**: Changes are saved to flash automatically after valid `SET` operations.

---

## **Example Usage**

```bash
# Enter config mode
> START
CONFIG MODE ENABLED
uvlo_level=24.0 uvlo_hyst=0.5 charged_level=28.5 charge_current=5.0
node_id:10
data_baud:1000000
...

# Set new UVLO threshold
> uvlo_level:23.5
OK: uvlo_level:23.50

# Apply changes (reboot)
> APPLY
```
