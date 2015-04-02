/*
 * Hand.h
 *
 *  Created on: Jun 13, 2012
 *      Author: Christian Dondrup
 *
 *      Just for testing. Not used at the moment.
 */

#ifndef HAND_H_
#define HAND_H_

#include <math.h>
#include <string>

#define VARIANCE_RESET_THRESHOLD 5

using namespace std;

/**
 * This class is not used at the moment.
 */
class Hand
{
private:
    double mean_dist;
    double var[VARIANCE_RESET_THRESHOLD];
    unsigned long long n;
    unsigned long long var_n;
    bool looming;
    string name;
    double getNewMean(double dist_hand);
    double getVariance();
    bool checkVarianceCounter();
    void resetVariance();

public:
    Hand()
    :mean_dist(0.0), n(0), var_n(0), looming(false)
    {
//    	var = {0.0,0.0,0.0,0.0,0.0};
    }

    void calculateLooming(double dist_hand);

    string getName() const
    {
        return name;
    }

    void setName(string name)
    {
        this->name = name;
    }

    bool isLooming() const
    {
        return looming;
    }
};




#endif /* HAND_H_ */
