#ifndef KALMAN_H
#define KALMAN_H

class KalmanPID
{
    public:
        KalmanPID(double vKgain_, double vfMeasurement_, double vEest_);
        void setMeasurement(double);
        double update_Error_Estimate(double vKgain_, double oldEest);
        double update_Kalman_Again(double oldEest);
        double new_estimate(double oldEST, double vKgain_, double valueMean);
        double getValueKF(double value);

    private:
        void init();
        void checkParam();
        double vEest;
        double result;
        double vKgain;
        double vEmeasurement;
};

#endif /* Kalman filer */
