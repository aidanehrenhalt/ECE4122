/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 11/26/2025

Description: TCP Client implementation. Connects to TCP server and sends messages for logging.
Client validates command line args before connecting to server.
*/

#include <iostream>
#include <string>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <limits>

/**
 * Function: validatePort()
 * Description: Validates given port number. Checks if within valid range (61000 - 65535).
 * 
 * Input: portStr - Port Number as string
 * 
 * Return: True if valid, otherwise False
 */

bool validatePort(const std::string &portStr, int &portNum)
{
    try
    {
        // Check if String is Exclusively Positive and Numeric
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
    catch(...)
    {
        return false;
    }
}



/**
 * Function: validateIpAddress()
 * Description: Validates given IP address string.
 * 
 * Input: ipStr - IP Address as string
 * 
 * Return: True if valid, otherwise False
 */
bool validateIpAddress(const std::string &ipStr)
{
    // Allow localhost
    if (ipStr == "localhost")
    {
        return true;
    }

    // Check for Valid IP Address Format (Basic Validation)
    struct sockaddr_in sa;
    int result = inet_pton(AF_INET, ipStr.c_str(), &(sa.sin_addr));
    
    if (result == 1)
    {
        return true;
    }

    // Try to Resolve as hostname
    struct hostent *host = gethostbyname(ipStr.c_str());
    if (host != NULL)
    {
        return true;
    }

    return false;
}



/**
 * Function: resolveHostname()
 * Description: Resolves a hostname to an IP address.
 * 
 * Input: hostname - Hostname as string
 * 
 * Return: Resolved IP address as string, or empty string on failure
 */
std::string resolveHostname(const std::string &hostname)
{
    // Check if it's Already an IP Address
    struct sockaddr_in sa;

    if (inet_pton(AF_INET, hostname.c_str(), &(sa.sin_addr)) == 1)
    {
        return hostname; // Already an IP Address
    }

    // Try to Resolve Hostname
    struct hostent *host = gethostbyname(hostname.c_str());

    if (host == NULL || host->h_addr_list[0] == NULL)
    {
        return ""; // Resolution Failed
    }

    struct in_addr *address = (struct in_addr *) host->h_addr_list[0];
    return std::string(inet_ntoa(*address));
}



/**
 * Function: main()
 * Description: Main function to start TCP client, connect to server, and send messages.
 * 
 * Input: argc - argument count, argv - argument values
 * 
 * Return: 0 on success, 1 on error
 */
int main(int argc, char *argv[])
{
    // Check Command Line Args
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " <server_ip> <port>" << std::endl;
        std::cout << "Please provide a valid server IP address and port number between 61000 - 65535." << std::endl;
        std::cin.get();

        return 1;
    }

    std::string ip_address = argv[1];
    std::string port_str = argv[2];

    // Validate IP Address
    if (!validateIpAddress(ip_address))
    {
        std::cout << "Invalid command line argument detected: " << ip_address << std::endl;
        std::cout << "Please check your values and press any key to end program!" << std::endl;
        std::cin.get();

        return 1;
    }

    // Validate Port Number
    int port_num;
    if (!validatePort(port_str, port_num))
    {
        std::cout << "Invalid command line argument detected: " << port_str << std::endl;
        std::cout << "Please check your values and press any key to end program!" << std::endl;
        std::cin.get();

        return 1;
    }

    // Resolve Hostname to IP Address
    std::string resolved_ip = resolveHostname(ip_address);
    if (resolved_ip.empty())
    {
        std::cout << "Could not resolve hostname: " << ip_address << " on " << port_num << "." << std::endl;
        std::cout << "Please provide a valid server IP address." << std::endl;
        std::cin.get();

        return 1;
    }

    // Create Socket
    int client_socket = socket(AF_INET, SOCK_STREAM, 0);

    if (client_socket == -1)
    {
        std::cerr << "Error: Could not create socket." << std::endl;
        std::cout << "Press Enter to exit..." << std::endl;
        std::cin.get();

        return 1;
    }

    // Setup Server Address Structure
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port_num);

    if (inet_pton(AF_INET, resolved_ip.c_str(), &server_addr.sin_addr) <= 0)
    {
        std::cout << "Invalid address/ Address not supported: " << ip_address << " on " << port_num << "." << std::endl;
        std::cout << "Please provide a valid server IP address." << std::endl;
        close(client_socket);
        std::cin.get();

        return 1;
    }

    // Connect to Server
    if (connect(client_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
    {
        std::cout << "Connection to " << ip_address << " on port " << port_num << " failed." << std::endl;
        std::cout << "Please ensure the server is running and reachable." << std::endl;
        close(client_socket);
        std::cin.get();

        return 1;
    }

    std::cout << "Connected to server at " << ip_address << " on port " << port_num << "." << std::endl;
    std::cout << "Type 'quit' to exit" << std::endl;

    // Debugging
    // std::cout << "DEBUG: About to enter while loop" << std::endl;  // Add this
    // std::cout << "DEBUG: cin good? " << std::cin.good() << std::endl;  // Add this

    // Clear Input Buffer
    std::cin.sync();

    // Continuously Prompt User for Messages
    while (true)
    {
        // Debugging
        // std::cout << "DEBUG: Top of while loop iteration" << std::endl;
        // std::cout.flush();
        
        std::cout << "Please enter a message: ";
        std::string message;
        std::getline(std::cin, message);

        // Debugging
        // std::cout << "DEBUG: Got message: '" << message << "'" << std::endl;  // Add this
        // std::cout << "DEBUG: cin.eof() = " << std::cin.eof() << std::endl;  // Add this
        
        // Check for Quit Command
        if (message == "quit" || message == "QUIT" || std::cin.eof())
        {
            break;
        }

        // Send Message to Server
        message += "\n"; // Append Newline Character
        int bytes_sent = send(client_socket, message.c_str(), message.length(), 0);

        if (bytes_sent < 0)
        {
            std::cerr << "Error: Could not send message to server." << std::endl;
            break;
        }
    }

    // Close Connection
    close(client_socket);
    std::cout << "Connection closed. Press Enter to exit..." << std::endl;
    std::cin.get();

    return 0;
}