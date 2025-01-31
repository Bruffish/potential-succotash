# RFID Reader Application

## Description
This project is a simple RFID reader application written in C++ that reads data from an RFID card via a serial port and prints the received data to the console.

## Features
- Configures serial port settings for communication with RFID reader.
- Continuously reads data from the RFID reader.
- Prints received RFID data to the console.

## Requirements
- Linux operating system (tested on Ubuntu 20.04)
- C++ compiler (g++)
- Connected RFID reader device
- Serial port access permissions

## Installation Instructions
1. Clone this repository:
   ```bash
   git clone https://github.com/Bruffish/potential-succotash.git
   cd potential-succotash
   ```

2. Compile the application:
   ```bash
   g++ main.cpp -o rfid_reader
   ```

## Usage
1. Connect your RFID reader to a serial port (e.g., `/dev/ttyUSB0`).
2. Ensure you have read/write permissions for the serial port:
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```
3. Run the application:
   ```bash
   ./rfid_reader
   ```


