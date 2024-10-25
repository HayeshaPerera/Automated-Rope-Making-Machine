# Automated Rope Making Machine

## Introduction
The Automated Rope Making Machine is designed to automate the process of rope production, ensuring consistent quality and tension while minimizing manual labor. The system incorporates various sensors and motors to achieve precise control over the rope-making process.

## Purpose
The purpose of this project is to create an efficient, automated solution for producing ropes made from various materials (e.g., wool, plastic, coir) while monitoring and maintaining the desired tension levels.

## Components
- **Microcontroller**: Arduino Uno
- **Motor Driver**: BTS7960B for controlling the DC gear motor
- **Stepper Motor**: NEMA 17 with TB6600 driver for winding the rope
- **Servo Motor**: For aligning the rope
- **Load Cell**: HX711 for real-time tension monitoring
- **Display**: LCD (Liquid Crystal Display) for showing tension levels and system status
- **Rotary Encoder**: KY-040 for setting rope length and starting the system
- **Emergency Stop Button**: To halt the operation if needed
- **Power Supply**: To provide necessary voltage and current to the components

## Basic Instructions
1. **Setup**: Connect all components according to the schematic provided in the project documentation.
2. **Calibration**: Before operating, calibrate the load cell to ensure accurate tension readings.
3. **Setting Tension and Length**:
   - Use the rotary encoder to set the desired rope tension.
   - Rotate the encoder to set the length of the rope to be produced.
4. **Start the Machine**:
   - Short press the rotary encoder button to initialize the system.
   - Long press to toggle between the modes to set the parameters and start the machine after.
5. **Monitoring**: The LCD will display real-time tension values and current length of the rope being produced.
6. **Emergency Stop**: If the emergency stop button is pressed, the system will halt immediately.

## Conclusion
This project demonstrates the feasibility of automating rope production using readily available components, contributing to efficiency in manufacturing processes while maintaining safety and control.
