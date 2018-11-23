#include "ros/ros.h"
#include "robot.h"
#include "visualization.h"
#include "optimizer.h"
#include "geometry_msgs/Twist.h"

#define FACTOR 1.0f
// landmarks
double landmarks[][2]={{1, 4}, {14, 2}, {1, 15}, {14, 14}, {1, 30}, {14, 30}, {1, 40}, {14, 40}, {1, 50}, {14, 55}, {26, 40},
                       {26, 55}, {37, 40}, {38, 53}, {51, 40}, {51, 54}, {64, 40}, {65 ,55}, {75, 35}, {75, 55}, {92, 40}, {92, 54},
                       {75, 25}, {95, 25}, {73, 15}, {92, 15}, {74, 4}, {92, 3}};

// robots
Robot real_robot(8, 10, M_PI/2.0f, 0.05);
Robot curr_robot(8, 10, M_PI/2.0f, 0.05, 0.1f, 5/57.3f);
bool has_data = false;

/*    ROS    */
ros::Subscriber subVelCmd;
geometry_msgs::Twist vel_cmd;

void velocity_callback(const geometry_msgs::Twist::ConstPtr vel_cmd_ptr) {
    vel_cmd.linear.x = max(0.0, vel_cmd_ptr->linear.x)*FACTOR;
    vel_cmd.linear.y = max(0.0, vel_cmd_ptr->linear.y)*FACTOR;
    vel_cmd.linear.z = max(0.0, vel_cmd_ptr->linear.z)*FACTOR;
    vel_cmd.angular.x = max(0.0, vel_cmd_ptr->angular.x)*FACTOR;
    vel_cmd.angular.y = max(0.0, vel_cmd_ptr->angular.y)*FACTOR;
    vel_cmd.angular.z = vel_cmd_ptr->angular.z*FACTOR;

    has_data = true;
}

bool find_landmark(Robot& robot, vector<int>& ldmk_index, vector<pair<double, double> >& measures) {
    ldmk_index.clear();
    measures.clear();
    float x = robot.mX, y = robot.mY, z = robot.mTheta;
    for (int i = 0; i < sizeof(landmarks)/ sizeof(landmarks[0]); ++i) {
        float min_x = x - 20, max_x = x + 20;
        float min_y = y - 20, max_y = y + 20;
        if (min_x < landmarks[i][0] && landmarks[i][0] < max_x &&
            min_y < landmarks[i][1] && landmarks[i][1] < max_y) {
            ldmk_index.emplace_back(i);
            auto meas = robot.measurement(landmarks[i]);
            measures.emplace_back(meas);
        }
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "rp_node");
    ros::NodeHandle n("rp_node");
    register_pub(n);

    subVelCmd = n.subscribe("/turtle1/cmd_vel", 100, velocity_callback);
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        // one step
        if (has_data) {
            has_data = false;
            real_robot.move((float)vel_cmd.linear.x*0.1f, (float)vel_cmd.angular.z*0.1f);
            // ROS_INFO(">>> robot: %f,%f,%f", real_robot.mX, real_robot.mY, real_robot.mTheta);
        }
        vector<int> indexes;
        vector<pair<double, double> > measures;
        find_landmark(real_robot, indexes, measures);
        ROS_ASSERT(indexes.size() == measures.size());
        double* ldmk_x = new double[(int)indexes.size()];
        double* ldmk_y = new double[(int)indexes.size()];
        double* meas_d = new double[(int)indexes.size()];
        double* meas_t = new double[(int)indexes.size()];
        // ROS_INFO(">>> [main] find landmark %d", (int)indexes.size());
        for (int j=0; j<(int)indexes.size(); j++) {
            ldmk_x[j] = landmarks[indexes[j]][0];
            ldmk_y[j] = landmarks[indexes[j]][1];
            meas_t[j] = measures[j].first;
            meas_d[j] = measures[j].second;
//            ROS_INFO(">>> [main] find landmark (%lf, %lf, %lf)-(%lf, %lf)=(%lf, %f)", real_robot.mX, real_robot.mY, real_robot.mTheta, ldmk_x[j], ldmk_y[j], measures[j].first*57.3f, measures[j].second);
        }
        double* initalPose = new double[3]{curr_robot.mX, curr_robot.mY, curr_robot.mTheta};
        double* finalPose  = new double[3];
        evaluateLeastSquare(ldmk_x, ldmk_y, meas_d, meas_t, (int)indexes.size(), initalPose, 0, 0, 0, finalPose);
        curr_robot.mX = (float)finalPose[0];
        curr_robot.mY = (float)finalPose[1];
        curr_robot.mTheta = (float)finalPose[2];
//        printf(">>> [main] robot pose (%f, %f, %f)->(%f, %f, %f)\r", real_robot.mX, real_robot.mY, real_robot.mTheta*57.3f, curr_robot.mX, curr_robot.mY, curr_robot.mTheta*57.3f);
        ROS_INFO(">>> [main] robot pose (%f, %f, %f)->(%f, %f, %f)", real_robot.mX, real_robot.mY, real_robot.mTheta*57.3f, curr_robot.mX, curr_robot.mY, curr_robot.mTheta*57.3f);
        std_msgs::Header header;
        pub_odometry(real_robot, curr_robot, header);
        pub_keyposes(real_robot, curr_robot, header);
        pub_TF(curr_robot, header);

        rate.sleep();
        delete(ldmk_x);
        delete(ldmk_y);
        delete(meas_d);
        delete(meas_t);
        delete(finalPose);
    }
    return 1;
}
