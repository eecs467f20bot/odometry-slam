#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <getopt.h>

typedef struct differenceCalculator_type
{
    double buffer[20];
    double diffCoefficients[20];
    int bufferSize;
    int currentFilled;
} difference;

typedef struct stdevCalculator_type
{
    double buffer[20];
    int bufferSize;
    int currentFilled;
    double currentEV;
    double currentSM;

} stdev_calc;

void stdev_calc_init(stdev_calc *diff, int bufferSize) {
    for (int i = 0; i < 20; i++)
    {
        diff->buffer[i] = -1; // Set dummy val; might be invalid.
    }
    diff->bufferSize = bufferSize; // Set current buffer size for differences
    diff->currentFilled = 0;       // Set number of current filled
    diff->currentEV = 0;
    diff->currentSM = 0;
}

double stdev_calc_march(stdev_calc *diff, double next_val) {
    diff->currentEV = diff->currentEV + (next_val/(diff->bufferSize));
    diff->currentSM = diff->currentSM + ((next_val*next_val)/(diff->bufferSize));
     if (diff->currentFilled < diff->bufferSize + 3)
    {
        diff->currentFilled = diff->currentFilled + 1;
    }
    // Secondly, add the next_val to the buffer
    // If this is less than the buffer, we know we still haven't gotten enough data
    if (diff->currentFilled < diff->bufferSize)
    {
        diff->buffer[diff->currentFilled - 1] = next_val;
    }
    // If we're equal to the bufferSize, we just need to add it to the end
    else if (diff->currentFilled == diff->bufferSize)
    {
        diff->buffer[diff->currentFilled - 1] = next_val;
    }
    else
    {
        diff->currentSM = diff->currentSM - ((diff->buffer[0]*diff->buffer[0])/diff->bufferSize);
        diff->currentEV = diff->currentEV - (diff->buffer[0]/diff->bufferSize);
        for (int i = 0; i < diff->bufferSize - 1; i++)
        {
            diff->buffer[i] = diff->buffer[i + 1];
        }
        diff->buffer[diff->bufferSize - 1] = next_val;
    }
    if(diff->currentFilled < diff->bufferSize) {
        return -9999;
    }
    double output = 0;
    for(int i = 0; i < diff->bufferSize; i++) {
        output += (diff->buffer[i])/diff->bufferSize;
    }
    return sqrt(diff->currentSM - (diff->currentEV*diff->currentEV));
}

double DifferenceCoefficients[2][4][6] = {
                                            {
                                                {-1.0, 1.0, 0.0, 0.0,0.0,0.0},
                                                {0.5, -2, 1.5, 0.0,0.0,0.0},
                                                {-1.0 / 3, 1.5, -3, 11.0 / 6,0.0,0.0},
                                                {0.0,0.0,0.0,0.0,0.0,0.0}
                                            },
                                            { 
                                                {1.0, -2.0, 1.0, 0.0, 0.0, 0.0},
                                                {-1.0, 4.0, -5.0, 2.0, 0.0, 0.0},
                                                {11.0 / 12.0, -14.0 / 3.0, 19.0 / 2, -26.0 / 3, 35.0 / 12.0, 0.0},
                                                {-5.0 / 6.0, 61.0 / 12.0, -13.0, 107.0 / 6, -77.0 / 6, 15.0 / 4}
                                            }
                                        };
// Supported buffersize, derivative number pairs:
// (1,2),(1,3),(1,4),(2,4),(2,5),(2,6)
void difference_init(difference *diff, int bufferSize, int derivativeNo)
{
    for (int i = 0; i < 20; i++)
    {
        diff->buffer[i] = -1; // Set dummy val; might be invalid.
    }
    diff->bufferSize = bufferSize; // Set current buffer size for differences
    diff->currentFilled = 0;       // Set number of current filled
    // Sets coefficients for interpolation based on bufferSize
    if (derivativeNo == 1)
    {
        if (bufferSize < 2 || bufferSize > 4)
        {
            printf("WARNING: Unused buffersize for this derivative number.");
        }
    }
    else if (derivativeNo == 2)
    {
        if (bufferSize < 3 || bufferSize > 6)
        {
            printf("WARNING: Unused buffersize for this derivative number.");
        }
    }
    for (int i = 0; i < bufferSize; i++)
    {
        diff->diffCoefficients[i] = DifferenceCoefficients[derivativeNo - 1][bufferSize - derivativeNo - 1][i];
    }
}

double difference_march(difference *diff, double next_val)
{
    // Firstly, make currentFilled the number of elements once next_val is added
    if (diff->currentFilled < diff->bufferSize + 3)
    {
        diff->currentFilled = diff->currentFilled + 1;
    }
    // Secondly, add the next_val to the buffer
    // If this is less than the buffer, we know we still haven't gotten enough data
    if (diff->currentFilled < diff->bufferSize)
    {
        diff->buffer[diff->currentFilled - 1] = next_val;
    }
    // If we're equal to the bufferSize, we just need to add it to the end
    else if (diff->currentFilled == diff->bufferSize)
    {
        diff->buffer[diff->currentFilled - 1] = next_val;
    }
    // If we're greater than bufferSize, then we need to rotate values accordingly
    else
    {
        for (int i = 0; i < diff->bufferSize - 1; i++)
        {
            diff->buffer[i] = diff->buffer[i + 1];
        }
        diff->buffer[diff->bufferSize - 1] = next_val;
    }

    //Okay - the buffer works. Now, I need to return the output:
    if(diff->currentFilled < diff->bufferSize) {
        return -9999;
    }
    else{
        double output = 0;
        for(int i = 0; i < diff->bufferSize; i++) {
            output = output + diff->diffCoefficients[i]*diff->buffer[i];
        }
        return output;
    } 

}

