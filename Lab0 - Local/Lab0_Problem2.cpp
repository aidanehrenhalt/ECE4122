/*
Author: Aidan (Ace) Ehrenhalt
Date: September 8th, 2025

*/

#include <iostream>

int main() {
    
    // Take inputs
    // Put into Array
    // Duplicates?
    // Prime Check
    // Prime Array

    int input[1];
    int primeTerms[100];
    int n = 0; // Index for primeTerms
    int primeSum = 0; // Initial prime sum

    std::cout << "Please enter a natural number (Input 0 to exit): ";
    std::cin >> input[0];

    for (int i = 0; i < input[0]; i++) {        
        // Check if number is prime
        if (isPrime(i)) {
            primeSum += i; // Add prime number to prime sum
            primeTerms[n] = i; // Store prime number in index n
            n++; // Increment index each time a prime number is stored
        }
    }

    // Output string of prime numbers
    std::cout << "The answer is " << primeSum << " with " << n << " prime terms:" ;
    
    for (int j = 0; j < n - 1; j++) { // Loop to format output
        if (j > 0) {
            std::cout << " + "; // Don't print + before first term
        }

        std::cout << primeTerms[j];
    }

    return 0;
}


// Basic Prime Check - Inspired by Geeks for Geeks
bool isPrime(int num) {
    if (num <= 1) {
        return false;
    }

    for (int i = 2; i < num; i++) {
        if (num % i == 0) {
            return false;
        }
    }

    return true;
}