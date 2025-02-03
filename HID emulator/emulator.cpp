#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <pty.h>
#include <string>

int main() {
    int master_fd, slave_fd;
    char name[1024];

    // Open a pseudo-terminal
    if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
        perror("openpty");
        return 1;
    }

    std::cout << "Virtual serial port created: " << name << std::endl;

if (symlink(name, "/dev/ttyUSB0") == -1) {
    perror("symlink");
    close(master_fd);
    close(slave_fd);
    return 1;
}

std::cout << "Symbolic link created: /dev/ttyUSB0" << std::endl;

    // Send "8888" every 5 seconds for 5 minutes
    const int duration = 300; // 5 minutes in seconds
    const int interval = 5;   // 5 seconds interval

    for (int i = 0; i < duration; i += interval) {
        std::string message = "ODg4OA==\n";
        if (write(master_fd, message.c_str(), message.size()) != static_cast<ssize_t>(message.size())) {
            perror("write");
            break;
        }
        sleep(interval);
    }

    // Clean up
    unlink("/dev/ttyUSB0");

    // Close the pseudo-terminal
    if (close(master_fd) == -1) {
        perror("close master_fd");
    }
    if (close(slave_fd) == -1) {
        perror("close slave_fd");
    }

    return 0;
}
