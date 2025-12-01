/**
 * Author: Aidan (Ace) Ehrenhalt
 * Last Date Modified: 11/29/2025 -- Correct Before Submission
 * 
 * Description:
 * Utilizes OpenMPI to estimate definite integrals using the Monte Carlo method.
 * Distributes random sampling across multiple processes to improve efficiency.
 * Uses collective communication to compile results for final integral estimate.
 */

// Make sure to do: 'module load openmpi' on PACE-ICE
// Installed manually for Local - 'sudo apt-get install openmpi-bin openmpi-common libopenmpi-dev'
#include <mpi.h> // Ignore Error on Local

#include <cmath>
#include <ctime>
#include <random>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using namespace std;

/**
 * Function: integrand1()
 * Description: Computes x^2 for 1st integral
 * 
 * Input: x - point at which to evaluate the integrand
 * Output: x^2 
 */
double integrand1(double x)
{
    return x * x;
}



/**
 * Function: integrand2()
 * Description: Computes e^(-x^2) for 2nd integral
 * 
 * Input: x - point at which to evaluate the integrand
 * Output: e^(-x^2)
 */
double integrand2(double x)
{
    return exp(-x * x);
}



/**
 * Function: parseCmdLineArgs()
 * Description: Parses command line args to get problem number and sample count
 * 
 * Input: argc - argument count, argv = argument values
 * Return: True if parsing was successful, otherwise False
 */
bool parseCmdLineArgs(int argc, char *argv[], int &problemNum, long long &numSamples)
{
    problemNum = 1;
    numSamples = 1000000; // Default 1 million samples

    for (int i = 1; i < argc; i++)
    {
        if (string(argv[i]) == "-P" && i + 1 < argc)
        {
            problemNum = atoi(argv[i + 1]);
            i++;
        }
        else if (string(argv[i]) == "-N" && i + 1 < argc)
        {
            numSamples = atoll(argv[i + 1]);
            i++;
        }
    }

    // Validate Cmd Line Inputs
    if (problemNum < 1 || problemNum > 2)
    {
        return false;
    }
    
    if (numSamples <= 0)
    {
        return false;
    }

    return true;
}



/**
 * Function: monteCarloIntegration()
 * Description: Performs Monte Carlo integration using distributed sampling and OpenMPI to split processes
 * 
 * Input: problemNum - which integral to compute, totalSamples - total num of samples
 * Output: Monte Carlo estimate value of the integral
 */
double monteCarloIntegration(int problemNum, long long totalSamples, int rank, int numProcesses)
{
    // Determine # of Samples per Process
    long long samplesPerProcess = totalSamples / numProcesses;
    long long remainderSamples = totalSamples % numProcesses;

    // Extra Samples to Lower Rank Processes
    long long mySamples = samplesPerProcess;

    if (rank < remainderSamples)
    {
        mySamples++;
    }

    // Init Random Number Generator
    unsigned int seed = time(NULL) + rank;
    mt19937 rng(seed);
    uniform_real_distribution<double> distribution(0.0, 1.0);

    // Select Integrand Function
    double (*integrand)(double);

    if (problemNum == 1)
    {
        integrand = integrand1;
    }
    else if(problemNum == 2)
    {
        integrand = integrand2;
    }

    // Perform Local Monte Carlo Sampling
    double localSum = 0.0;

    for (long long i = 0; i < mySamples; i++)
    {
        double x = distribution(rng);
        localSum += integrand(x);
    }

    // Calculate Local Monte Carlo Avg
    double localAvg = (mySamples > 0) ? (localSum / mySamples) : 0.0; // A ? B : C = If A then B else C 

    // Collect Local Avgs
    double *allAvgs = nullptr;
    long long *allSampleCounts = nullptr;

    if (rank == 0)
    {
        allAvgs = new double[numProcesses];
        allSampleCounts = new long long[numProcesses];
    }

    // MPI_Gather to Collect Local Avgs and Sample Counts
    MPI_Gather(&localAvg, 1, MPI_DOUBLE, allAvgs, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Gather(&mySamples, 1, MPI_LONG_LONG, allSampleCounts, 1, MPI_LONG_LONG, 0, MPI_COMM_WORLD);

    double finalEstimate = 0.0;

    // Rank 0 Calculates Final Weighted Avg
    if (rank == 0)
    {
        double totalSum = 0.0;
        long long totalCount = 0;

        for (int i = 0; i < numProcesses; i++)
        {
            totalSum += allAvgs[i] * allSampleCounts[i];
            totalCount += allSampleCounts[i];
        }

        double avgValue = totalSum / totalCount;
        finalEstimate = avgValue * 1.0; // Interval Width = 1.0

        // Clean Up
        delete[] allAvgs;
        delete[] allSampleCounts;
    }

    // Broadcast Final Estimate to All Processes
    MPI_Bcast(&finalEstimate, 1, MPI_DOUBLE, 0, MPI_COMM_WORLD);

    return finalEstimate;
}



/**
 * Function: main()
 * Description: Main function to init OpenMPI, parse cmd line args, and perform Monte carlo integration
 * 
 */

int main(int argc, char *argv[])
{
    MPI_Init(&argc, &argv);

    int rank;
    int numProcesses;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &numProcesses);

    // Parse Command Line Args
    int problemNum;
    long long numSamples;

    if (!parseCmdLineArgs(argc, argv, problemNum, numSamples))
    {
        if (rank == 0)
        {
            cerr << "Error: Invalid command line arguments." << endl;
            cerr << "Usage: " << argv[0] << " [-P problemNum (1 or 2)] [-N numSamples (> 0)]" << endl;
        }

        MPI_Finalize();
        
        return 1;
    }

    // Perform Monte Carlo Integration
    double estimate = monteCarloIntegration(problemNum, numSamples, rank, numProcesses);

    // Output Result from Rank 0
    if (rank == 0)
    {
        cout << fixed << setprecision(8);
        cout << "The Monte Carlo estimate of the integral for problem " << problemNum << " is " << estimate << endl;   
    }
    
    MPI_Finalize();

    return 0;
}