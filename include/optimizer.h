#pragma once

//#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <random>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/solver.h>

using namespace std;
using namespace Eigen;

int evaluateLeastSquare(double* x, double* y, double* k, double* theta, int nb, double* initbeta,
                        double tol, int itermax, double iterstep, double* finalpos,
                        const double* dis_noise = NULL, const double* theta_noise = NULL);

double getAngle(double y, double x);

class PoseParameterization : public ceres::LocalParameterization {
public:
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];
        if (x_plus_delta[2] > 2*M_PI)  {
            x_plus_delta[2] = x_plus_delta[2] - 2*M_PI;
        }
        if (x_plus_delta[2] < 0) {
            x_plus_delta[2] = 2*M_PI + x_plus_delta[2];
        }
//        double delta_dis = sqrt(delta[0]*delta[0]+delta[1]*delta[1]);
//        x_plus_delta[0] = x[0] + delta_dis*cos(x_plus_delta[2]);
//        x_plus_delta[1] = x[1] + delta_dis*sin(x_plus_delta[2]);
//        ROS_INFO(">>> ENTER (%lf, %lf, %lf)+(%lf, %lf, %lf)=(%lf, %lf, %lf)", x[0], x[1], x[2], delta[0], delta[1], delta[2],
//                 x_plus_delta[0], x_plus_delta[1], x_plus_delta[2]);

        return true;
    }

    virtual bool ComputeJacobian(const double *x, double *jacobian) const {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor> > j(jacobian);
        j.setIdentity();
        return true;
    }

    virtual int GlobalSize() const {
        return 3;
    }

    virtual int LocalSize() const {
        return 3;
    }
};

class ProjectionFactor : public ceres::SizedCostFunction<2, 3, 2> {
public:
    ProjectionFactor(double measure_dis, double measure_theta) {
        m_dis = measure_dis;
        m_theta = measure_theta;
        m_sqrt_info.setIdentity();
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
        const double robot_x = parameters[0][0], robot_y = parameters[0][1], robot_theta = parameters[0][2];
        const double ldmk_x = parameters[1][0], ldmk_y = parameters[1][1];

        double head_theta = getAngle(ldmk_y-robot_y, ldmk_x-robot_x)-robot_theta;
        double head_dis = sqrt((robot_y-ldmk_y)*(robot_y-ldmk_y)+(robot_x-ldmk_x)*(robot_x-ldmk_x));
        if (head_theta > 2*M_PI)  {
            head_theta = head_theta - 2*M_PI;
        }
        if (head_theta < 0) {
            head_theta = 2*M_PI + head_theta;
        }

        Eigen::Map<Eigen::Vector2d > residual(residuals);
        residual[0] = head_dis - m_dis;
        residual[1] = head_theta - m_theta;
        if (residual[1] > M_PI)  {
            residual[1] = residual[1] - 2*M_PI;
        }
        if (residual[1] < -M_PI) {
            residual[1] = 2*M_PI + residual[1];
        }

        residual = m_sqrt_info*residual;

//        printf(">>> [evaluate] state: (%lf, %lf, %lf), lm: (%lf, %lf), calu: (%lf, %lf), meas: (%lf, %lf)\n", robot_x, robot_y, robot_theta,
//             ldmk_x, ldmk_y, head_theta, head_dis, m_theta, m_dis);

        if (jacobians) {
            double y2x = (ldmk_y-robot_y)/(ldmk_x-robot_x);
            double grad_atan = 1.0/(1.0+y2x*y2x);
            double delta_x = ldmk_x-robot_x;
            if (jacobians[0]) {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > jacobian(jacobians[0]);
                jacobian(0, 0) = (robot_x-ldmk_x)/head_dis;
                jacobian(0, 1) = (robot_y-ldmk_y)/head_dis;
                jacobian(0, 2) = 0;
                jacobian(1, 0) =  1*grad_atan*(ldmk_y-robot_y)/((ldmk_x-robot_x)*(ldmk_x-robot_x));
                jacobian(1, 1) = -1*grad_atan/delta_x;
                jacobian(1, 2) = -1;

                jacobian = m_sqrt_info*jacobian;
            }

            if (jacobians[1]) {
                Eigen::Map<Eigen::Matrix<double, 2, 2, Eigen::RowMajor> > jacobian(jacobians[1]);
                jacobian(0, 0) = -1*(robot_x-ldmk_x)/head_dis;
                jacobian(0, 1) = -1*(robot_y-ldmk_y)/head_dis;
                jacobian(1, 0) = -1*grad_atan*y2x/delta_x;
                jacobian(1, 1) =  1*grad_atan/delta_x;

                jacobian = m_sqrt_info*jacobian;
            }
        }
        return true;
    }
    bool setInformation(double dis_noise, double theta_noise) {
        m_sqrt_info(0, 0) = theta_noise;
        m_sqrt_info(1, 1) = dis_noise;
        return true;
    }

private:
    double m_dis, m_theta;
    Eigen::Matrix2d m_sqrt_info;
};

