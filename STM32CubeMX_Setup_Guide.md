# STM32CubeMX Configuration Guide - MAX30102 Heartbeat Monitor

Setup for **MAX30102** pulse oximeter sensor (I2C interface).

---

## Step 1: Create New Project

1. Open **STM32CubeMX**
2. Click **ACCESS TO MCU SELECTOR**
3. Search for `STM32F401RCT6` → Select it → **Start Project**

---

## Step 2: System Core Configuration

### SYS
| Setting | Value |
|---------|-------|
| Debug | Serial Wire |
| Timebase Source | SysTick |

### RCC
| Setting | Value |
|---------|-------|
| HSE | Crystal/Ceramic Resonator |

---

## Step 3: Clock Configuration

Go to **Clock Configuration** tab:
- Input: **8 MHz** (HSE)
- HCLK: **84 MHz**

---

## Step 4: Peripheral Configuration

### 4.1 I2C1 - MAX30102 Sensor

**Click on PB6** → Select **I2C1_SCL**
**Click on PB7** → Select **I2C1_SDA**

Go to **Connectivity → I2C1 → Parameter Settings:**

| Parameter | Value |
|-----------|-------|
| I2C Mode | I2C |
| Speed Mode | Fast Mode |
| Clock Speed | 400000 Hz |

---

### 4.2 GPIO - MAX30102 Interrupt (Optional)

**Click on PB5** → Select **GPIO_EXTI5**

Go to **GPIO → PB5:**
| Parameter | Value |
|-----------|-------|
| GPIO Mode | External Interrupt with Falling edge |
| GPIO Pull-up/Pull-down | Pull-up |
| User Label | MAX30102_INT |

Go to **NVIC** and enable:
- ✅ EXTI line[9:5] interrupts

> The INT pin signals when new data is ready.

---

### 4.3 USART2 - Serial Output

**Click on PA2** → Select **USART2_TX**
**Click on PA3** → Select **USART2_RX**

Go to **Connectivity → USART2 → Parameter Settings:**

| Parameter | Value |
|-----------|-------|
| Mode | Asynchronous |
| Baud Rate | 115200 |
| Word Length | 8 Bits |
| Parity | None |
| Stop Bits | 1 |

---

### 4.4 GPIO - LED Alert

**Click on PA5** → Select **GPIO_Output**

| Parameter | Value |
|-----------|-------|
| User Label | LED_ALERT |
| Output Level | Low |

---

## Step 5: Project Manager

| Parameter | Value |
|-----------|-------|
| Project Name | `HeartbeatMonitor` |
| Project Location | `c:\Users\yousef\realtime` |
| Toolchain/IDE | STM32CubeIDE |

### Code Generator:
- ✅ Generate peripheral initialization as pair of .c/.h files
- ✅ Keep User Code when regenerating

---

## Step 6: Generate Code

Click **GENERATE CODE** → Open in STM32CubeIDE

---

## Hardware Wiring

Connect MAX30102 to STM32F401RCT6:

```
MAX30102        STM32F401
--------        ---------
VIN     →       3.3V
GND     →       GND
SCL     →       PB6
SDA     →       PB7
INT     →       PB5 (optional)
```

---

## Pin Summary

| Pin | Function | Purpose |
|-----|----------|---------|
| PB6 | I2C1_SCL | MAX30102 clock |
| PB7 | I2C1_SDA | MAX30102 data |
| PB5 | GPIO_EXTI | MAX30102 interrupt |
| PA2 | USART2_TX | Serial output |
| PA3 | USART2_RX | Serial input |
| PA5 | GPIO_Output | Alert LED |

---

## MAX30102 I2C Address

| Address | Value |
|---------|-------|
| Write | 0xAE |
| Read | 0xAF |
| 7-bit | 0x57 |

---

## After Generation

Let me know when ready and I'll provide:
1. MAX30102 driver code (I2C communication)
2. Heart rate calculation algorithm
3. Main application code with serial output
