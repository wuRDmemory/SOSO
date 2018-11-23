#include <robot.h>
#include <optimizer.h>

Robot::Robot(float x, float y, float theta, float sensor_noise, float forward_noise, float turn_noise) {
    mX = x;
    mY = y;
    mTheta = theta;
    mForwardNoise = forward_noise;
    mTurnNoise = turn_noise;
    mSensorNoise = sensor_noise;

}

bool Robot::setPose(float x, float y, float theta) {
    mX = x;
    mY = y;
    mTheta = theta;
}

bool Robot::move(float forward, float turn) {
    std::default_random_engine generator;
    std::normal_distribution<float> forward_noise(0, mForwardNoise);
    std::normal_distribution<float> turn_noise(0, mTurnNoise);

    // add noise
    mTheta = (mTheta + turn);
    if (mTheta > 2*M_PI)  {
        mTheta = mTheta-2*M_PI;
    }
    if (mTheta < 0) {
        mTheta = 2*M_PI+mTheta;
    }
    float forward_dis = forward;
    mX = mX + forward_dis*cos(mTheta);
    mY = mY + forward_dis*sin(mTheta);
    return true;
}

pair<double, double> Robot::measurement(double *landmark) {
    assert(landmark != NULL);
    std::default_random_engine generator;
    std::normal_distribution<double> sensor_noise(0, mSensorNoise);
    double ldmk_x = landmark[0] + sensor_noise(generator);
    double ldmk_y = landmark[1] + sensor_noise(generator);

    double odom_dis   = sqrt((ldmk_x-mX)*(ldmk_x-mX)+(ldmk_y-mY)*(ldmk_y-mY));
    double odom_theta = getAngle(ldmk_y-mY, ldmk_x-mX) - mTheta;
//    ROS_INFO(">>> odom_theta: %lf", )
    if (odom_theta > 2*M_PI)  {
        odom_theta = odom_theta - 2*M_PI;
    }
    if (odom_theta < 0) {
        odom_theta = 2*M_PI + odom_theta;
    }

    return make_pair(odom_theta, odom_dis);
}


