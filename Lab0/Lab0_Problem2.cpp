/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122
Date: September 8th, 2025

Description: Lab 0 Problem 2 - Project Euler
*/

#include <iostream>
#include <vector>
#include <string>
#include <cctype>

/*
Function: isPrime - Checks if a number is prime
Input: long long num - number we're checking
Output: bool - true if prime, otherwise false
*/
bool isPrime(long long num) 
{
    if (num <= 1) 
    {
        return false;
    }

    for (long long i = 2; i < num; i++) 
    {
        if (num % i == 0) 
        {
            return false;
        }
    }

    return true;
}

/*
Function: isValidInput - Validates input only contains integers
Input: const std::string& input - input string to validate
Output: bool - true if valid, otherwise false
*/
bool isValidInput(const std::string& input) 
{
    if (input.empty()) 
    {
        return false;
    }

    for (char c : input) 
    {
        if (!std::isdigit(c)) 
        {
            return false;
        }
    }
    
    return true;
}

/*
Function: main - Main function to find prime sum of consecutive primes
*/
int main() 
{
    std::string input;

    while (true) 
    {
        std::cout << "Please enter a natural number (Input 0 to exit): ";
        std::cin >> input;

        if (!isValidInput(input)) 
        {
            std::cout << "Invalid input. Please enter a natural number." << std::endl;
            continue; // Prompt again for new input
        }

        long long inputNum = std::stoll(input); // Convert string to long long

        if (inputNum == 0) 
        {
            std::cout << "Program terminated." << std::endl;
            std::cout << "Have a nice day!" << std::endl;
            return 0; // Exit if input is 0
        }

        std::vector<long long> primeTerms; // Vector to store prime terms

        for (long long i = 2; i < inputNum; i++) 
        {        
            // If number is prime, add it to the sum
            if (isPrime(i)) {
                primeTerms.push_back(i); // Store prime number in vector
            }
        }

        // Sum the prime terms
        long long sum = 0;
        for (long long prime : primeTerms) 
        {
            sum += prime;
        }

        long long bestPrimeSum = 0;
        int primeSpan = 0;
        int primeStart = 0;

        // Testing various starting primes
        for (int start = 0; start < (int)primeTerms.size(); start++) 
        {
            long long currentSum = 0;
            int currentCount = 0;

            for (int end = start; end < (int)primeTerms.size(); end++) 
            {
                currentSum += primeTerms[end];
                currentCount++;

                if (currentSum > inputNum) 
                {
                    break; // Stop if we exceed input number
                }

                if (isPrime(currentSum) && (end - start + 1) > primeSpan) 
                {
                    bestPrimeSum = currentSum;
                    primeSpan = end - start + 1;
                    primeStart = start;
                }
            }
        }

        if (primeSpan > 0) 
        {
            // Output string of prime numbers
            std::cout << "The answer is " << bestPrimeSum << " with " << primeSpan << " prime terms: ";

            for (int j = 0; j < primeSpan; j++) 
            { // Loop to format output
                if (j > 0) 
                {
                    std::cout << " + "; // Don't print + before first term
                }

                std::cout << primeTerms[primeStart + j];
            }
        }

        

        std::cout << std::endl;
    }
    
    return 0;
}