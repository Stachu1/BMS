# BMS Monitoring Project (AVR / ATmega)

## 🔋 Overview

This project is a simple Battery Management System (BMS) monitor implemented on an AVR microcontroller. It was developed as part of a semester project to:

- Learn ADC-based voltage and current measurements
- Interface with LCD and UART
- Handle basic battery protection conditions

The code continuously monitors battery voltage and current, calculates power, and displays the results on an LCD. It also triggers a protection routine in case of overcurrent or undervoltage.

---

## ⚙️ Features

- Real-time voltage and current measurement using ADC
- Display of voltage (V), current (A), and power (W) on an LCD
- UART interface for debugging
- Overcurrent and undervoltage protection alert
- Simple interrupt-driven ADC switching between two channels
- Time tracking using Timer1

---

## 🔌 Hardware Requirements

- ATmega microcontroller (e.g., ATmega328P)
- LCD (compatible with `LCD_init()` and related functions)
- Voltage and current sensors (connected to ADC0 and ADC7)
- IR LED for signaling
- UART for serial communication
- Voltage divider for battery voltage measurement

---

## 🚨 Protection Thresholds

- **Undervoltage:** 8.0 V  
- **Overcurrent:** 20.0 A  
- If either condition is violated, the system:
  - Activates a protection output on PD3
  - Displays an alert on the LCD

---

## 🧪 Measurement Details

- **Voltage:** Read from ADC0 (scaled by voltage divider and offset)
- **Current:** Read from ADC7 (amplified by sensor gain and offset)
- **Power:** Calculated as `voltage × current`
