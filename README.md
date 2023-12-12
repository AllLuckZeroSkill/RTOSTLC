Traffic Control System
Overview
This project implements a traffic control system using a microcontroller-based approach, developed using the Keil µVision5 IDE. It focuses on managing traffic lights at an intersection, with real-time response to sensor inputs using a round-robin scheduling with interrupts.

Features
Round-Robin Scheduling with Interrupts: Employs a round-robin scheduling mechanism complemented by an interrupt-driven design for immediate response to external sensor inputs, enhancing the system's real-time performance.
Traffic Light Control: Manages traffic lights at intersections, toggling between different light sequences based on sensor input and predetermined timing.
Sensor Integration: Uses sensor inputs for dynamic traffic light control, adapting to real-time traffic conditions.
Modular Code Structure: The code is organized modularly, separating initialization, main control loop, sensor handling, and utility functions for clarity and maintenance.
Requirements
Tiva C Series TM4C123GH6PM microcontroller
LED lights for traffic signal representation
Traffic sensor (e.g., IR sensor)
TivaWare for C Series (for driver libraries)
Keil µVision5 IDE for development and programming
Setup and Initialization
System Clock and GPIO Initialization: Configure the system clock and initialize GPIO ports for sensor and LED connections.
Sensor and LED Configuration: Set up the sensor input and LED outputs, including the sensor interrupt.
Console Initialization: Initialize a console for debugging and monitoring outputs.
Main Functionality
The main loop controls traffic lights following a specific sequence, modifiable based on real-time conditions.
Sensor inputs trigger interrupts, allowing the system to respond dynamically to traffic changes.
Interrupt Handling
The Sensor_ISR function handles sensor interrupts, modifying the traffic light sequence in response to traffic data.
Post-interrupt, the system resumes the regular light sequence.
Usage
Compile the code in Keil µVision5 IDE and upload it to the TM4C123GH6PM microcontroller. Connect the LEDs and sensor as defined in the GPIO configurations. Power up the system to begin the traffic control simulation.
