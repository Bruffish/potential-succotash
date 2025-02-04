#include <iostream>
#include <string>
#include <vector>
#include <algorithm>  // Include for std::all_of
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
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 1;  // Timeout after 1 second
    timeout.tv_usec = 0;

    int selectResult = select(fd + 1, &readfds, NULL, NULL, &timeout);
    if (selectResult == -1) {
        std::cerr << "Error in select: " << errno << std::endl;
        break;
    } else if (selectResult == 0) {
        // Timeout occurred
        continue;
    } else {
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer) - 1); // Leave space for null terminator
        if (bytesRead > 0) {
            buffer[bytesRead] = '\0'; // Null-terminate the string
            std::string data(buffer);
            std::cout << "" << data << std::endl;
// Test the base64_decode function with a known Base64 string
std::string testBase64 = "SGVsbG8gV29ybGQh"; // "Hello World!" in Base64
std::string decodedTest = base64_decode(testBase64);
std::cout << "Decoded Test Data: " << decodedTest << std::endl; // Debugging statement

std::string decodedData = base64_decode(data);
std::cout << "Received Data: " << data << std::endl; // Debugging statement
std::cout << "Decoded Data: " << decodedData << std::endl; // Debugging statement

// Check if the decoded data is a valid integer
if (!decodedData.empty() && std::all_of(decodedData.begin(), decodedData.end(), ::isdigit)) {
    try {
        int cardNumber = std::stoi(decodedData); // Convert Base64 decoded string to integer
        std::cout << "Decoded Card Number: " << cardNumber << std::endl;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Invalid argument error: " << e.what() << std::endl;
    }
} else {
    std::cerr << "Decoded data is not a valid integer: " << decodedData << std::endl;
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
    }
}

    close(fd);
    return 0;
}
