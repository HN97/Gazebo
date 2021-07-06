#include "Kalmanfilter.h"

/**********************************
* Constructor functions
**********************************/

KalmanPID::KalmanPID(double vKgain_, double vfMeasurement_, double vEest_)
{
    init();
    vKgain = vKgain_;
    result = vfMeasurement_;
    vEest = vEest_;
}

void KalmanPID::init()
{
    vKgain = 0;
    result = 0;
    vEest = 0;
    vEmeasurement = 1;

}
/**
*
*
*
*
*
*/
void KalmanPID::setMeasurement(double value)
{
    vEmeasurement = value;
    checkParam();
}
/**
*
*
*
*
*
*/
double KalmanPID::update_Error_Estimate(double vKgain_, double oldEest)
{
    double newEest = 0;
    newEest = (1-vKgain_)*oldEest;
    return newEest;
}
/**
*
*
*
*
*
*/
double KalmanPID::update_Kalman_Again(double oldEest)
{
    double newKg = 0;
    newKg = oldEest/(oldEest+vEmeasurement);
    return newKg;
}
/**
*
*
*
*
*
*/
double KalmanPID::new_estimate(double oldEST, double vKgain_, double valueMean)
{
    double newEST = 0;
    newEST = oldEST + (vKgain_*(valueMean - oldEST));
    return newEST;
}

double KalmanPID::getValueKF(double value)
{
    vKgain = update_Kalman_Again(vEest);
    result = new_estimate(result, vKgain, value);
    vEest = update_Error_Estimate(vKgain, vEest);

    return result;
}

void KalmanPID::checkParam()
{
    /* do something */
}