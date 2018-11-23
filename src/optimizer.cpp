#include <optimizer.h>

static double init_x[3];
static double ldmks[30][2];

int evaluateLeastSquare(double* x, double* y, double* k, double* theta,
                        int nb, double* initbeta, double tol, int itermax,
                        double iterstep, double* finalpos, const double* dis_noise, const double* theta_noise)
{
    assert(x != NULL && y != NULL && k != NULL && theta != NULL && initbeta != NULL);

    init_x[0] = initbeta[0];
    init_x[1] = initbeta[1];
    init_x[2] = initbeta[2];
    ceres::Problem problem;
    ceres::LossFunction* loss = new ceres::HuberLoss(1);
    ceres::LocalParameterization* localParameterization = new PoseParameterization();
    problem.AddParameterBlock(init_x, 3, localParameterization);
    problem.SetParameterBlockVariable(init_x);

    // all landmark and measurement
    for (int i = 0; i < nb; ++i) {
        ldmks[i][0] = x[i];
        ldmks[i][1] = y[i];
        problem.AddParameterBlock(ldmks[i], 2);
        problem.SetParameterBlockConstant(ldmks[i]);
        ProjectionFactor* factor = new ProjectionFactor(k[i], theta[i]);
        if (dis_noise != NULL && theta_noise != NULL) {
            factor->setInformation(dis_noise[i], theta_noise[i]);
        }
        problem.AddResidualBlock(factor, loss, init_x, ldmks[i]);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
//    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
//    options.max_solver_time_in_seconds = 0.050f;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

//    ROS_INFO(">>> [optimizer] iteration size %d, final cost %lf", (int)summary.iterations.size(), summary.final_cost);
//    for (int i = 0; i < nb; ++i) {
//        ROS_INFO("after lm (%lf, %lf)", ldmks[i][0], ldmks[i][1]);
//    }
    finalpos[0] = init_x[0];
    finalpos[1] = init_x[1];
    finalpos[2] = init_x[2];
    return 1;
}

double getAngle(double y, double x) {
    // first coordination
    if (y >= 0) {
        return atan2(y, x);
    } else {
        return 2*M_PI+atan2(y, x);
    }
}