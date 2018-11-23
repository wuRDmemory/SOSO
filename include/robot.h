#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <math.h>

using namespace std;
using namespace cv;

#define PI 3.1415926f

class Robot {
public:
    Robot(float x=0, float y=0, float theta=0, float sensor_noise=0, float forward_noise=0, float turn_noise=0);
    bool setPose(float x, float y, float theta);
    bool move(float forward, float turn);
    pair<double, double> measurement(double* landmark);

    float mX, mY, mTheta, mForwardNoise, mTurnNoise, mSensorNoise;
};
