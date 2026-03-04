# Lora-PS-Node

LoRa parking-sensor node: fuses radar and magnetometer to detect car presence and reports status (car + battery) to a LoRaWAN gateway every minute. Built with **STM32CubeIDE** for **STM32L462CEU6**.

---

## Components

| Component | Role | Interface | Notes |
|-----------|------|------------|--------|
| **STM32L462CEU6** | Main MCU | — | Cortex-M4, runs sensors, fusion, RAK driver, logging |
| **Acconeer A121** | Radar presence | SPI1 | Short-range presence; score, distance, debounce |
| **MMC5983MA** | Magnetometer | I2C1 | Magnetic delta for metal (car) detection |
| **RAK3172** | LoRaWAN modem | LPUART1 | AT commands, OTAA join, uplink every 1 min |
| **SIP32431** | Battery gate | GPIO (BAT_MON) | Enables divider for ADC when measuring |
| **ADC1 (PA0)** | Battery voltage | — | Divider 10k / 49.9k → 0–100% (3.0–4.2 V) |
| **USART1** | Debug log | UART | Optional; `printf` / `Log_Printf` when logging enabled |

### Pin usage (summary)

- **PA0** – ADC1 (battery divider)
- **PA1** – BAT_MON (SIP32431 enable)
- **PA3** – A121 enable
- **PA4** – A121 interrupt
- **PA8** – MMC5983 interrupt
- **PB0** – A121 CS (SPI)
- **PB9** – Debug pin
- **PB12 / PB13** – RAK reset, boot
- **LPUART1** – RAK3172 AT
- **USART1** – Log output (e.g. debug console)

---

## Architecture

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                    STM32L462CEU6                        │
                    │                                                         │
  SPI1 ────────────►│  A121 (radar)  ──┐                                      │
                    │                  │    ┌──────────────────┐              │
  I2C1 ────────────►│  MMC5983 (mag) ──┼───►│  Car Detector    │              │
                    │                  │    │  (fusion +       │              │
                    │                  │    │   debounce)      │              │
                    │                  │    └────────┬────────┘              │
                    │                  │             │                        │
                    │  ADC1 + BAT_MON ──┼─────────────┼────────► Battery %     │
                    │                  │             │                        │
                    │                  │             ▼                        │
                    │                  │    ┌──────────────────┐   LPUART1   │
                    │                  └───│  main.c           │─────────────┼────► RAK3172 ──► LoRaWAN
                    │                       │  (every 1 min or  │             │
                    │                       │  5 s retry)       │             │
                    │                       └────────┬─────────┘             │
                    │                                │                        │
                    │  Log (optional)                │   USART1              │
                    │  printf / Log_Printf ───────────┼───────────────────────┼────► Debug console
                    └─────────────────────────────────────────────────────────┘
```

### Data flow

1. **Sensors**  
   - **Radar:** `RadarPresence_Tick` → score/distance filtering and debounce → `car_detected`.  
   - **Magnetometer:** `MagneticSensor_Tick` → baseline, delta threshold, debounce → `car_present`.

2. **Fusion**  
   `CarDetector_Tick` combines radar and mag: both ON → immediate present; one ON → confirm after `on_confirm_ms`; both OFF for `off_confirm_ms` → clear. Output: `CarDetector_GetCarPresent()`.

3. **Uplink**  
   Every 1 min (or 5 s on send failure), if joined: read car (fusion) and battery % (ADC), then `RAK_SendStatus(&rak, car, battery_pct)` → 2-byte payload on FPort 10.

4. **Log**  
   `_write` → `Log_Write`; when logging enabled, all `printf` and `Log_Printf` go to USART1.

---

## Project layout

| Path | Description |
|------|-------------|
| **Core/Src/** | main.c, HAL init (gpio, adc, i2c, spi, usart), log.c, syscalls |
| **Core/Inc/** | main.h, log.h, Cube-generated headers |
| **Core/Sensors/** | car_detector (fusion), radar_presence (A121), magnetic_sensor, mmc5983, Acconeer integration |
| **Core/RAK/** | RAK.c / RAK.h – LoRaWAN AT driver (init, join, send) |
| **Core/BatMon/** | battery_adc – gated battery voltage and percentage |
| **Core/Log/** | README for log module; sources in Core/Inc + Core/Src |
| **Drivers/** | STM32 HAL, CMSIS, Acconeer RSS / examples |

---

## LoRaWAN payload

- **FPort:** 10 (`RAK_FPORT_CAR_STATUS`).
- **Payload:** 2 bytes (hex), e.g. `01 32`:
  - **Byte 0:** Car present (0 = no, 1 = yes).
  - **Byte 1:** Battery 0–100 (%).

---

## Timing and retry

- **Send interval:** 60 s when send succeeds.
- **Retry:** 5 s when send fails (RAK_ERR_BUSY, RAK_ERR_TIMEOUT, RAK_ERR_AT_FAIL).
- **Not joined:** Next attempt in 60 s (no spam).

---

## Configuration (high level)

- **RAK (Core/RAK/RAK.h):** `RAK_JOIN_EUI`, `RAK_APP_KEY`, `RAK_BAND_EU868`, `RAK_FPORT_CAR_STATUS`.
- **Car detector (Core/Sensors/car_detector.c):** Thresholds, debounce, confirm times, radar preset (e.g. `RADAR_PRESET_SHORT_RANGE`).
- **Battery (Core/BatMon/battery_adc.h):** Divider `BAT_ADC_RTOP_OHMS` / `BAT_ADC_RBOT_OHMS`; in app, 3000–4200 mV → 0–100% (see `BAT_MV_EMPTY` / `BAT_MV_FULL` in main.c).
- **Log (Core/Inc/log.h):** `LOG_ENABLE` (0 = compile-time off); runtime: `Log_Enable()` / `Log_Disable()`.

---

## Build and run

- **IDE:** STM32CubeIDE.
- **Target:** STM32L462CEU6.
- Ensure **Core/Src/log.c** is in the build and **Core/Inc** (and any Sensor/RAK/BatMon paths) are in the include path.
- Flash and connect USART1 (e.g. 115200 8N1) to see log output when logging is enabled.

---

## Log module (debug)

- **API:** `Log_Init(&huart1)`, `Log_Enable()`, `Log_Disable()`, `Log_Printf(...)`, `Log_Write(...)`.
- **printf:** Redirected via `_write()` → `Log_Write()` so all printf goes to USART1 when logging is enabled.
- **Compile-time:** Set `LOG_ENABLE` to 0 in `Core/Inc/log.h` to remove log code.
- See **Core/Log/README.md** for a short reference.
