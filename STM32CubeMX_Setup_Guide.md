# STM32CubeMX Configuration Guide: Heartbeat Monitor

Follow these steps precisely to configure your STM32F401RCT6 for the Heartbeat Monitor project.

## 1. MCU Selection
1.  Open **STM32CubeMX**.
2.  Click **Access to MCU Selector**.
3.  Search for `STM32F401RCT6`.
4.  Select the MCU and click **Start Project**.

## 2. Pinout & Configuration

### System Core
1.  **RCC**:
    - High Speed Clock (HSE): **Crystal/Ceramic Resonator** (Assuming you have an external crystal on your board).
    - Low Speed Clock (LSE): **Disable** (or Crystal if you need RTC, but likely not for this).
2.  **SYS**:
    - Debug: **Serial Wire**.
    - Timebase Source: **TIM1** (Important! Since FreeRTOS uses Systick, we need another timer for HAL).

### Connectivity
1.  **I2C1** (For MAX30102 and LCD):
    - I2C: **I2C**.
    - Configuration Parameter Settings:
        - I2C Speed Mode: **Fast Mode** (400 KHz) or Standard Mode (100 KHz). Start with **Standard Mode** for stability.
    - **GPIO Settings**:
        - Verify pins (usually PB6=SCL, PB7=SDA).
        - Pull-up: **Pull-up** (Internal pull-ups can help, but external are better. Set to Pull-up to be safe).

### Middleware
1.  **FREERTOS**:
    - Interface: **CMSIS_V2**.
    - **Config parameters**:
        - `TOTAL_HEAP_SIZE`: Increase to `15360` (15KB) or more to be safe.
        - `MINIMAL_STACK_SIZE`: `128` (Words).

## 3. FreeRTOS Tasks & Queues
Go to **Middleware > FREERTOS > Tasks and Queues** tab.

1.  **defaultTask** (Rename if you want, or leave as main runner).
    - We will use this or create new ones. Let's create specific tasks.
    - Double click `defaultTask` -> rename to `HeartbeatTask`.
        - Priority: `osPriorityNormal`.
        - Stack Size: `512` words (give it space for I2C and math).
2.  **Add New Task**:
    - Task Name: `DisplayTask`.
    - Priority: `osPriorityNormal` (or `osPriorityLow` if display updates are less critical).
    - Stack Size: `512` words.

## 4. Clock Configuration
1.  Go to the **Clock Configuration** tab.
2.  Input Frequency (HSE): Set this to your generic board's crystal (usually **8 MHz** or **25 MHz**).
    - *Example for 25MHz*: Input 25.
3.  Select **HSE** in the PLL Source Mux.
4.  Select **PLLCLK** in the System Clock Mux.
5.  Set **HCLK (MHz)** to **84** (Max for F401).
6.  Hit Enter. CubeMX will calculate the multipliers/dividers.

## 5. Project Manager

### Project
1.  **Project Name**: `HeartbeatMonitor`
2.  **Project Location**: `C:\Users\yousef\realtime`
3.  **Toolchain/IDE**: **Makefile** (Since we are using valid VSCode workflow, or choose **EWARM**/*MDK-ARM* if you strictly use those, but for this agent workflow, `Makefile` or `CMake` is best. If you strictly strictly use CubeIDE, choose **STM32CubeIDE**. *Recommendation: STM32CubeIDE* if you want to generate `.cproject` files, or *Makefile* for generic).
    - **Selection**: Choose **STM32CubeIDE** if you are importing into the IDE, or **Makefile** if we are building via CLI. Let's stick to **STM32CubeIDE** or **Makefile**.
    - *Agent Preference*: **Makefile** allows us to build easily.

### Code Generator
1.  **Copy all used libraries into the project folder**: Checked.
2.  **Generate peripheral initialization as a pair of '.c/.h' files**: Checked (Important for cleanliness).

## 6. Generate Code
1.  Click **GENERATE CODE**.
2.  Wait for completion.
3.  Close CubeMX.
