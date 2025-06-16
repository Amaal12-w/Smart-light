# 💡 Smart Light Dimming using AVR

This project is a simple yet practical implementation of **automatic and manual light dimming** using **AVR ATmega328P** microcontroller.  
It allows LED brightness control based on ambient input or user commands over UART, and provides real-time mode switching using a button interrupt.

---

## 🔧 Tools & Technologies

- **AVR ATmega328P** microcontroller  
- **C Language** (AVR-GCC)  
- **PWM (Fast PWM on OC0A - PD6)**  
- **ADC** for reading analog input from a potentiometer  
- **UART** for manual control via Serial Monitor  
- **External Interrupt (INT0)** to toggle modes  
- **Proteus (for simulation)**  
- **Atmel Studio / Microchip Studio** or `avr-gcc` toolchain  

---

## ⚙️ How It Works

The system operates in **two modes**:

### 🔁 Automatic Mode
- The LED brightness adjusts in real-time based on the value of a **potentiometer**.
- The ADC reads the analog voltage and maps it to a brightness percentage (0–100%).
- PWM signal updates LED duty cycle accordingly.
- UART prints the brightness status.

### ✍️ Manual Mode
- You can enter a number (0–100) via Serial Monitor.
- That number sets the brightness manually.
- UART confirms the input and updates PWM accordingly.

### 🛎️ Mode Switching
- A push button connected to `INT0` is used to toggle between modes.
- Debounce logic is implemented using `Timer1`.
- `PORTB` LEDs indicate current mode (e.g., PB0 for Auto, PB1 for Manual).

---

## 🧪 Features

- Real-time brightness adjustment  
- Serial communication for user control  
- External interrupt with software debounce  
- Clean and modular code  
- UART echo & feedback  

---

## 📁 Project Files

You can view the full source code here: 
https://github.com/Amaal12-w/Smart-light/commit/0763ef4c068afb97aa4efa3fadadd61e9fac5884
## 📸 Simulation

![Circuit Simulation](images/proteus-circuit.png)

## 🙏 Special Thanks

Big thanks to my team and everyone who helped me in the control part 💚

