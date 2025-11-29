/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 11/26/2025

Description: TCP Debug / Logging server implementation. Accepts connections from clients and logs messages to a server.log file.
Server validates command line args and handles client connections.
*/

#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <vector>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <signal.h>

// Mutex - Thread-Safe File Writing
std::mutex log_mutex;

// Global Flag for Clean Shutdown
bool server_running = true;

/**
 * Function: validatePort()
 * Description: Validates given port number. Checks if within valid range (61000 - 65535).
 * 
 * Input: portStr - Port Number as string
 * Return: True if valid, otherwise False
 */
bool validatePort(const std::string &portStr, int &portNum)
{
    try 
    {
        // Check if string is exclusively positive and numeric
        for (size_t i = 0; i < portStr.length(); i++)
        {
            if (i == 0 && portStr[i] == '-')
            {
                return false; // '-' = Invalid Input
            }

            if (!isdigit(portStr[i]))
            {
                return false; // Non-Numeric Character = Invalid Input
            }
        }

        portNum = std::stoi(portStr);

        // Check if portNum is within valid range
        if (portNum < 61000 || portNum > 65535)
        {
            return false; // Out of Range = Invalid Input
        }

        return true;
    }
    catch (...) // Handles Any Exceptions
    {
        return false; // Exception = Invalid Input
    }
}



/**
 * Function: writeToLog()
 * Description: Thread-Safe writing to server.log file.
 * 
 * Input: message - Message string written to server.log
 * Output: Appends message to server.log
 */
void writeToLog(const std::string &message)
{
    std::lock_guard<std::mutex> lock(log_mutex); // Lock Mutex for Thread Safety
    std::ofstream logFile("server.log", std::ios::app); // Open log file in append mode

    if (logFile.is_open())
    {
        logFile << message << std::endl;
        logFile.close();
    }
    else
    {
        std::cerr << "Error: Couldn't open server.log" << std::endl;
    }
}



/**
 * Function: handleClient()
 * Description: Handles server-client communication
 * 
 * Inputs: 
 * clientSocket - Socket descriptor for connected client
 * clientAddr - Client address structure of the client
 * 
 * Output: Receives messages from client and logs them to server.log
 */
void handleClient(int clientSocket, struct sockaddr_in clientAddr)
{
    char buffer[4096];
    std::string client_ip = inet_ntoa(clientAddr.sin_addr);
    int client_port = ntohs(clientAddr.sin_port);

    // Log Client Connection
    std::string connect_message = "Client Connected From: " + client_ip + ":" + std::to_string(client_port);
    std::cout << connect_message << std::endl;
    writeToLog(connect_message);

    // Testing Timeout Removal Fix
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    setsockopt(clientSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // Debugging
    // std::cout << "DEBUG: Starting receive loop, server_running=" << server_running << std::endl;

    // Receive Messages from Client
    while (server_running)
    {
        // Debugging
        // std::cout << "DEBUG: Waiting to receive..." << std::endl;

        memset(buffer, 0, sizeof(buffer)); // Clear Buffer
        int bytes_received = recv(clientSocket, buffer, sizeof(buffer) - 1, 0);

        // Debugging
        // std::cout << "DEBUG: recv returned " << bytes_received << std::endl;

        if (bytes_received <= 0)
        {
            // Debugging
            // std::cout << "DEBUG: Breaking because bytes_received=" << bytes_received << std::endl;

            break; // Connection Disconnected or Error
        }
        
        // Remove Trailing Newline Characters
        std::string message(buffer, bytes_received);
        
        while (!message.empty() && (message.back() == '\n' || message.back() == '\r'))
        {
            message.pop_back();
        }

        // Log Message
        writeToLog(message);
        std::cout << "Received Message From " << client_ip << ":" << client_port << " - " << message << std::endl;
    }

    // Log Client Disconnect
    std::string disconnect_message = "Client Disconnected From: " + client_ip + ":" + std::to_string(client_port);
    std::cout << disconnect_message << std::endl;
    writeToLog(disconnect_message);

    close(clientSocket);
}



/**
 * Function: signalHandler()
 * Description: Handles SIGINT (Keyboard Input: CTRL + C) signal for clean server shutdown
 * 
 * Input: signum - Signal Number
 * Output: Set server_running flag to false
 */
void signalHandler(int signum)
{
    std::cout <<"\nShutting Down Server..." << std::endl;
    server_running = false;
}


/**
 * Function: main()
 * Description: Main Server Function. Configures TCP server and accepts connections
 * 
 * Input: argc - argument count, argv - argument values
 * 
 * Return: 0 on success, 1 on error
 */
int main(int argc, char *argv[])
{
    // Check Command Line Args
    if (argc != 2)
    {
        std::cout << "Usage: " << argv[0] << " <port>" << std::endl;
        std::cout << "Please provide a valid port number between 61000 and 65535." << std::endl;
        std::cin.get();

        return 1;
    }

    // Validate Port Number
    int portNum;

    if (!validatePort(argv[1], portNum))
    {
        std::cout << "Invalid command line argument detected: " << argv[1] << std::endl;
        std::cout << "Please check your values and press any key to end program!" << std::endl;
        std::cin.get();

        return 1;
    }

    // Setup Signal Handler for Clean Shutdown
    signal(SIGINT, signalHandler);

    // Create Socket
    int server_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (server_socket == -1)
    {
        std::cerr << "Error: Could not create socket." << std::endl;
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();

        return 1;
    }

    // Set Socket Options to Reuse Address
    int opt = 1;

    if (setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
    {
        std::cerr << "Error: Could not set socket options." << std::endl;
        close(server_socket);
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();

        return 1;
    }

    // Bind Socket to Port
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(portNum);

    if (bind(server_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
    {
        std::cerr << "Error: Could not bind to port " << portNum << "." << std::endl;
        close(server_socket);
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();

        return 1;
    }

    // Listen for Connections
    if (listen(server_socket, SOMAXCONN) < 0)
    {
        std::cerr << "Error: Could not listen on socket" << std::endl;
        close(server_socket);
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();

        return 1;
    }

    std::cout << "Server Listening on Port " << portNum << std::endl;
    writeToLog("Server Started on Port " + std::to_string(portNum));

    // Vector to Store Client Threads
    std::vector<std::thread> client_threads;

    // Accept Connections
    while (server_running)
    {
        struct sockaddr_in client_addr;
        socklen_t client_addr_len = sizeof(client_addr);

        // Set Socket to Non-Blocking with Timeout for Accepting Connections
        struct timeval timeout;
        timeout.tv_sec = 1;
        timeout.tv_usec = 0;
        setsockopt(server_socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        int client_socket = accept(server_socket, (struct sockaddr *) &client_addr, &client_addr_len);

        if (client_socket < 0)
        {
            // Check if Timeout or Error
            if (errno == EWOULDBLOCK || errno == EAGAIN)
            {
                continue; // Timeout - Continue to Check server_running Flag
            }
            
            if (server_running)
            {
                std::cerr << "Error: Could not accept connection." << std::endl;
            }

            continue;
        }

        // Create New Thread to Handle Client Connection
        client_threads.push_back(std::thread(handleClient, client_socket, client_addr));
    }

    // Wait for All Client Threads to Finish
    for (auto &thread : client_threads)
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }

    close(server_socket);
    writeToLog("Server Shutdown Completed.");
    std::cout << "Server Shutdown Completed." << std::endl;

    return 0;
}