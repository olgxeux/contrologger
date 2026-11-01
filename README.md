# Envonitor

Environmental logging and servo control firmware for STM32F411CEU6 written in C using STM32 HAL, CubeMX and Codex.

The project measures temperature, humidity and atmospheric pressure, controls an SG90 servo using a potentiometer and displays data on an OLED. It was built to practice fault handling, automatic recovery and a non-blocking embedded architecture.

## Features

- AHT20 temperature and humidity measurement
- BMP280 temperature and pressure measurement
- SSD1306 display with four selectable modes
- Potentiometer-based SG90 control with configurable angle limits
- UART commands and logging
- Persistent settings in internal FLASH
- I²C timeout handling and automatic recovery attempts
- Validation of sensor data

## Hardware

- STM32F103C8T6
- ST-Link V2
- AHT20 + BMP280 module
- SSD1306 128x64 I2C OLED
- CP2102 USB-UART adapter
- 20 kΩ potentiometer
- SG90 positional servo

## Connections

- USART1: PA9 TX, PA10 RX, 115200 8N1
- I2C1: PB8 SCL, PB9 SDA, 100 kHz
- ADC1: PA0
- TIM3 CH1: PA6 - SG90 control signal, 50 Hz

## UART commands

Format:

```text
aht20 on
aht20 off

bmp280 on
bmp280 off

display on
display off
display mode 1
display mode 2
display mode 3
display mode 4

servo on
servo off
servo min A
servo max B
```

## Build

Requirements:

- `arm-none-eabi-gcc`
- `GNU Make`
- `openocd`

Build:

```bash
make
```

Flash:

```bash
make flash
```

Clean:

```bash
make clean
```