#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>    // usleep
#include <fcntl.h>     // File control definitions
#include <errno.h>     // Error number definitions
#include <termios.h>   // POSIX terminal control definitions

#define SERIAL_PORT "/dev/ttyUSB0"
#define BAUD_RATE B9600

// Base64 decoding function
std::string base64_decode(const std::string &in) {
    std::string out;

    std::vector<int> T(256,-1);
    const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for(int i=0; i<64; i++) T[b64[i]]=i;

    int val=0, valb=-6;
    for(unsigned char c : in) {
        if(T[c] == -1) break;
        val = (val<<6) + T[c];
        valb += 6;
        if(valb>=0) {
            out.push_back(char((val>>valb)&0xFF));
            valb-=8;
        }
    }
    return out;
}

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

    char buffer[12]; // Buffer size optimized to handle up to 10 digits of data plus null terminator
    while (true) {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1); // Leave space for null terminator
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // Null-terminate the string
            std::string data(buffer);
            std::cout << "Card Data: " << data << std::endl;
std::string decodedData = base64_decode(data);
std::cout << "Decoded Data: " << decodedData << std::endl; // Debugging statement
int cardNumber = 0; // Declare cardNumber outside the try block
try {
    cardNumber = std::stoi(decodedData); // Convert Base64 decoded string to integer
    std::cout << "Decoded Card Number: " << cardNumber << std::endl;
} catch (const std::invalid_argument& e) {
    std::cerr << "Invalid argument error: " << e.what() << std::endl;
}
if (cardNumber != 0) { // Check if cardNumber was successfully set
    std::cout << "Decoded Card Number: " << cardNumber << std::endl;
}
        } else if (bytesRead < 0) {
            std::cerr << "Error reading from serial port, retrying..." << std::endl;
            continue;
        } else if (bytesRead == 0) {
            // No data read, check if the device is still available
            int flags = fcntl(fd, F_GETFL);
            if (flags == -1) {
                std::cerr << "Error getting file status flags: " << errno << std::endl;
                break;
            }
            if ((flags & O_RDWR) != O_RDWR) {
                std::cerr << "Serial device is not available, exiting..." << std::endl;
                break;
            }
        }
        usleep(100); // Sleep for 100 microseconds
    }

    close(fd);
    return 0;
}
