#include <iostream>
#include <fstream>

int main() {
    std::ofstream shutdownFile("/home/odroid/ros2_thrust_ws/src/comms/tcp_server_shutdown");
    if (!shutdownFile) {
        std::cerr << "Failed to write shutdown flag.\n";
        return 1;
    }

    shutdownFile << "shutdown";
    shutdownFile.close();
    std::cout << "Shutdown flag written.\n";
    return 0;
}
