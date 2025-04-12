#include <iostream>
#include <fstream>
#include <cstring>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <filesystem>
#define PORT 5000  
#define BUFFER_SIZE 1024


void process_file(const std::string& filename) {

    ////////////////////////////////////////////
    // Placeholder: Process received MATLAB file
    ////////////////////////////////////////////
    std::cout << "Processing file: " << filename << std::endl;

    static int times_received = 0;
    times_received++;

    std::ifstream inFile(filename, std::ios::binary);
    if (!inFile) {
        std::cerr << "Error opening received file for reading.\n";
        return;
    }

    inFile.seekg(-1, std::ios::end);
    char lastChar;
    inFile.get(lastChar);
    inFile.close();

    std::ofstream outFile(filename, std::ios::app | std::ios::out);
    if (!outFile) {
        std::cerr << "Error opening received file for writing.\n";
        return;
    }

    
    if (lastChar != '\n') {
        outFile << "\n";  
    }

    
    outFile << "Times received: " << times_received << "\n";
    outFile.close();
    std::filesystem::copy("/home/odroid/ros2_thrust_ws/src/comms/gains_pid.json", "/home/odroid/ros2_thrust_ws/src/flightstack/params/control/pid/gains_pid.json", std::filesystem::copy_options::overwrite_existing);
    std::cout << "Processing file: " << filename << std::endl;
    char command1[255];
    char command2[255];
    sprintf(command1, "sh -c ./start_uxrce.sh");
    usleep(10000);
    sprintf(command2, "sh -c ./run_flightstack.sh");
    std::cout << "flightstack ran" << std::endl;

}

void receive_file(int socket, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    char buffer[BUFFER_SIZE];
    int bytesRead;

    if (!outFile) {
        std::cerr << "Error opening file for writing.\n";
        return;
    }

    while ((bytesRead = read(socket, buffer, BUFFER_SIZE)) > 0) {
        if (strncmp(buffer, "EOF", 3) == 0) break;
        outFile.write(buffer, bytesRead);
    }

    outFile.close();
    std::cout << "File received: " << filename << std::endl;
}

void send_file(int socket, const std::string& filename) {
    std::ifstream inFile(filename, std::ios::binary);
    char buffer[BUFFER_SIZE];

    if (!inFile) {
        std::cerr << "Error opening file for reading.\n";
        return;
    }

    while (!inFile.eof()) {
        inFile.read(buffer, BUFFER_SIZE);
        send(socket, buffer, inFile.gcount(), 0);
    }

    send(socket, "EOF", 3, 0);
    inFile.close();
    std::cout << "File sent back: " << filename << std::endl;
}

int main() {
    int server_fd, client_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    int n = 5;
    std::string filename = "gains_pid.json";

    // Create socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "Socket creation failed\n";
        return -1;
    }

    // Set up address
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    // Bind socket
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed\n";
        return -1;
    }

    // Listen
    if (listen(server_fd, 3) < 0) {
        std::cerr << "Listen failed\n";
        return -1;
    }

    std::cout << "Server listening on port " << PORT << "...\n";

    // Accept connection
    if ((client_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cerr << "Accept failed\n";
        return -1;
    }
    std::cout << "Client connected!\n";
    const char* L2filename = "L2norm.json";
    for (int i = 0; i < n; i++) {
        std::cout << "Iteration " << i + 1 << " of " << n << std::endl;

        //receive_file(client_socket, filename);
        //process_file(filename);
        send_file(client_socket, L2filename);
    }
    close(client_socket);
    close(server_fd);

    return 0;
}
