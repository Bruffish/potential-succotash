#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>    // usleep
#include <fcntl.h>     // File control definitions
#include <errno.h>     // Error number definitions
#include <termios.h>   // POSIX terminal control definitions

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B9600

int configureSerialPort(int fd) {
    struct termios tty;

    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << errno << std::endl;
        return -1;
    }

    cfsetospeed(&tty, BAUD_RATE);
    cfsetispeed(&tty, BAUD_RATE);

    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // One stop bit
    tty.c_cflag |= CS8;     // 8 bits per byte
    tty.c_cflag &= ~CRTSCTS;// Disable hardware flow control
    tty.c_lflag &= ~ICANON; // Disable canonical mode
    tty.c_lflag &= ~ECHO;   // Disable echo
    tty.c_lflag &= ~ECHOE;  // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG;   // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error from tcsetattr: " << errno << std::endl;
        return -1;
    }

    return 0;
}

int main() {
    int fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        std::cerr << "Failed to open serial port: " << SERIAL_PORT << std::endl;
        return 1;
    }

    if (configureSerialPort(fd) != 0) {
        close(fd);
        return 1;
    }

    char buffer[256];
    while (true) {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if (bytesRead > 0) {
            std::string data(buffer, bytesRead);
            std::cout << "Card Data: " << data << std::endl;
            // TODO Process / parse the RFID data here...
        } else if (bytesRead < 0) {
            std::cerr << "Error reading from serial port" << std::endl;
            break;
        }
        usleep(100); // Sleep for 100 microseconds
    }

    close(fd);
    return 0;
}