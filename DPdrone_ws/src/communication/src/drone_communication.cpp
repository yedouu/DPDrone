#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class Communication {
public:
    Communication()
        : current_yaw(0), hover_flag(0),
          coordinate_frame(1), arm_state(false), motion_type(0), flight_mode(""), mission(""), last_cmd("") {

        target_motion.coordinate_frame = coordinate_frame;

        local_pose_sub = nh.subscribe("/iris_0/mavros/local_position/pose", 1,
                                       &Communication::localPoseCallback, this);
        cmd_sub = nh.subscribe("/cmd", 3,
                               &Communication::cmdCallback, this);
        cmd_sub_test = nh.subscribe("/cmd_sub_test", 3,
                                    &Communication::cmd_test_Callback, this);
        // cmd_pose_flu_sub = nh.subscribe("/cmd_pose_flu", 1,
        //                                  &Communication::cmdPoseFluCallback, this);
        // cmd_pose_enu_sub = nh.subscribe("/cmd_pose_enu", 1,
        //                                  &Communication::cmdPoseEnuCallback, this);
        // cmd_vel_flu_sub = nh.subscribe("/cmd_vel_flu", 1,
        //                                 &Communication::cmdVelFluCallback, this);
        // cmd_vel_enu_sub = nh.subscribe("/cmd_vel_enu", 1,
        //                                 &Communication::cmdVelEnuCallback, this);
        // cmd_accel_flu_sub = nh.subscribe("/cmd_accel_flu", 1,
        //                                   &Communication::cmdAccelFluCallback, this);
        // cmd_accel_enu_sub = nh.subscribe("/cmd_accel_enu", 1,
        //                                   &Communication::cmdAccelEnuCallback, this);

        // 订阅当前飞行状态
        state_sub = nh.subscribe<mavros_msgs::State>
            ("/iris_0/mavros/state", 10, &Communication::state_cb,this);
        target_motion_pub = nh.advertise<mavros_msgs::PositionTarget>(
            "/iris_0/mavros/setpoint_raw/local", 1);

        armService = nh.serviceClient<mavros_msgs::CommandBool>(
            "/iris_0/mavros/cmd/arming");
        flightModeService = nh.serviceClient<mavros_msgs::SetMode>(
            "/iris_0/mavros/set_mode");
        set_param_srv = nh.serviceClient<mavros_msgs::ParamSet>(
            "/iris_0/mavros/param/set");

        
    }

    void OffBoard_Request(ros::Rate rate){
        ros::Time last_request = ros::Time::now();
        arm_cmd.request.value = true;
        while(ros::ok()){
            target_motion_pub.publish(target_motion);
            if(current_state.mode != "OFFBOARD"){
                ROS_INFO("WAIT FOR OFFBOARD MODE......");
            } else {
                if( !current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( armService.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            ros::spinOnce();
            rate.sleep();
        }


    }

    //程序入口，其他操作全靠回调函数完成
    void start() {
        ros::Rate rate(30);
        OffBoard_Request(rate);
        while (ros::ok()) {
            target_motion_pub.publish(target_motion);
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    mavros_msgs::State current_state;
    mavros_msgs::CommandBool arm_cmd;
    ros::Subscriber state_sub;
    geometry_msgs::Point current_position;      //无人机当前位置(Local)
    double current_yaw;     //无人机当前航向角
    int hover_flag;     //是否悬停的标志
    int coordinate_frame;   //无人机飞行时参考坐标系的标志
    mavros_msgs::PositionTarget target_motion;      //无人机的期望位姿
    bool arm_state;
    int motion_type;    //无人机的飞行方式
    std::string flight_mode;    //飞行模式
    std::string mission;    //没用到
    std::string last_cmd;   //上一条指令

    ros::Subscriber local_pose_sub;     
    ros::Subscriber cmd_sub;
    ros::Subscriber cmd_sub_test;
    ros::Subscriber cmd_pose_flu_sub;
    ros::Subscriber cmd_pose_enu_sub;
    ros::Subscriber cmd_vel_flu_sub;
    ros::Subscriber cmd_vel_enu_sub;
    ros::Subscriber cmd_accel_flu_sub;
    ros::Subscriber cmd_accel_enu_sub;

    ros::Publisher target_motion_pub;
    ros::ServiceClient armService;
    ros::ServiceClient flightModeService;
    ros::ServiceClient set_param_srv;
    uint16_t type_mask[5] = {0b101111111000, 0b011111111000, 0b101111000111, 0b011111000111, 0b011000111111};
    //获取与飞控的状态
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void localPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        current_position = msg->pose.position;
        current_yaw = q2yaw(msg->pose.orientation);
    }

    void cmd_test_Callback(const mavros_msgs::PositionTarget::ConstPtr& msg) {
        mavros_msgs::PositionTarget target;
        //筛选可行的坐标系
        if(msg->coordinate_frame == 1 || msg->coordinate_frame == 8){
            target.coordinate_frame = msg->coordinate_frame;
            //筛选可行的控制方式
            if(msg->type_mask == type_mask[0] || msg->type_mask == type_mask[1] || msg->type_mask == type_mask[2] || msg->type_mask == type_mask[3] || msg->type_mask == type_mask[4]){
                target.type_mask = msg->type_mask;
                target.position.x = msg->position.x;
                target.position.y = msg->position.y;
                target.position.z = msg->position.z;
                target.velocity.x = msg->velocity.x;
                target.velocity.y = msg->velocity.y;
                target.velocity.z = msg->velocity.z;
                target.acceleration_or_force.x = msg->acceleration_or_force.x;
                target.acceleration_or_force.y = msg->acceleration_or_force.y;
                target.acceleration_or_force.z = msg->acceleration_or_force.z;
                target.yaw = msg->yaw;
                target.yaw_rate = msg->yaw_rate;
                target_motion = target;
                
            }
            else{
                ROS_WARN("INVALID TYPE_MASK!!!");
            }
            
        }
        else{
            ROS_WARN("INVALID COORDINATE_FRAME!!!");
        }
    }

    // //FLU系下进行位置控制(BODY)（经测试1.13版本下“coordinate_frame = 9”失效）
    // void cmdPoseFluCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    //     coordinate_frame = 9;
    //     motion_type = 0;
    //     double yaw = q2yaw(msg->orientation);
    //     target_motion = constructTarget(msg->position.x, msg->position.y, msg->position.z, 0, 0, 0, 0, 0, 0, yaw, 0);
    // }
    // //ENU系下进行位置控制(LOCAL)
    // void cmdPoseEnuCallback(const geometry_msgs::Pose::ConstPtr& msg) {
    //     coordinate_frame = 1;
    //     motion_type = 0;
    //     double yaw = q2yaw(msg->orientation);
    //     target_motion = constructTarget(msg->position.x, msg->position.y, msg->position.z, 0, 0, 0, 0, 0, 0, yaw, 0);
    // }
    // //FLU(BODY)系下进行速度控制
    // void cmdVelFluCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     // hoverStateTransition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    //     if (hover_flag == 0) {
    //         coordinate_frame = 8;
    //         motion_type = 1;
    //         target_motion = constructTarget(0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, 0, 0, 0, msg->angular.z);
    //     }
    // }
    // //ENU(LOCAL)系下进行速度控制
    // void cmdVelEnuCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     // hoverStateTransition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    //     if (hover_flag == 0) {
    //         coordinate_frame = 1;
    //         motion_type = 1;
    //         target_motion = constructTarget(0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, 0, 0, 0, msg->angular.z);
    //     }
    // }

    // void cmdAccelFluCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     hoverStateTransition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    //     if (hover_flag == 0) {
    //         coordinate_frame = 8;
    //         motion_type = 2;
    //         target_motion = constructTarget(0, 0, 0, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, msg->angular.z);
    //     }
    // }

    // void cmdAccelEnuCallback(const geometry_msgs::Twist::ConstPtr& msg) {
    //     hoverStateTransition(msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.z);
    //     if (hover_flag == 0) {
    //         coordinate_frame = 1;
    //         motion_type = 2;
    //         target_motion = constructTarget(0, 0, 0, 0, 0, 0, msg->linear.x, msg->linear.y, msg->linear.z, 0, msg->angular.z);
    //     }
    // }

    // void hoverStateTransition(double x, double y, double z, double w) {
    //     if (fabs(x) > 0.02 || fabs(y) > 0.02 || fabs(z) > 0.02 || fabs(w) > 0.005) {
    //         hover_flag = 0;
    //         flight_mode = "OFFBOARD";
    //     } else if (flight_mode != "HOVER") {
    //         hover_flag = 1;
    //         flight_mode = "HOVER";
    //         hover();
    //     }
    // }

    void cmdCallback(const std_msgs::String::ConstPtr& msg) {
        if (msg->data == last_cmd || msg->data.empty() || msg->data == "stop controlling") {
            return;
        }
        // Arm the vehicle
        else if (msg->data == "ARM") {
            arm_state = arm();
            ROS_INFO("%s: Armed %s", arm_state ? "true" : "false");
        } else if (msg->data == "DISARM") {
            arm_state = !disarm();
            ROS_INFO("%s: Armed %s", arm_state ? "true" : "false");
        } else {
            flight_mode = msg->data;
            flightModeSwitch();
        }
        last_cmd = msg->data;
    }

    double q2yaw(const geometry_msgs::Quaternion& q) {
        tf2::Quaternion quaternion(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw; // 定义滚转角、俯仰角、航向角
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw); // 从四元数中提取出滚转角、俯仰角、航向角
        return yaw;
    }

    bool arm() {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        if (armService.call(srv) && srv.response.success) {
            return true;
        } else {
            ROS_ERROR("%s: Arming failed!");
                        return false;
        }
    }

    bool disarm() {
        mavros_msgs::CommandBool srv;
        srv.request.value = false;
        if (armService.call(srv) && srv.response.success) {
            return true;
        } else {
            ROS_ERROR("%s: Disarming failed!");
            return false;
        }
    }

    void hover() {
        target_motion.position.x = current_position.x;
        target_motion.position.y = current_position.y;
        target_motion.position.z = current_position.z;
        target_motion.velocity.x = 0;
        target_motion.velocity.y = 0;
        target_motion.velocity.z = 0;
        target_motion.acceleration_or_force.x = 0;
        target_motion.acceleration_or_force.y = 0;
        target_motion.acceleration_or_force.z = 0;
    }

    void flightModeSwitch() {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = flight_mode;
        if (flightModeService.call(srv) && srv.response.mode_sent) {
            ROS_INFO("%s: Flight mode set to %s", flight_mode.c_str());
        } else {
            ROS_ERROR("%s: Failed to set flight mode to %s", flight_mode.c_str());
        }
    }

    mavros_msgs::PositionTarget constructTarget(double x, double y, double z,  
                                                double vx = 0, double vy = 0, double vz = 0, 
                                                double afx = 0, double afy = 0, double afz = 0,double yaw =0,double yaw_rate = 0) {
        mavros_msgs::PositionTarget target;
        target.coordinate_frame = coordinate_frame;
        target.position.x = x;
        target.position.y = y;
        target.position.z = z;

        target.velocity.x = vx;
        target.velocity.y = vy;
        target.velocity.z = vz;

        target.acceleration_or_force.x = afx;
        target.acceleration_or_force.y = afy;
        target.acceleration_or_force.z = afz;

        target.yaw = yaw;
        target.yaw_rate = yaw_rate;


        // 根据 motion_type 设置 type_mask
        if (motion_type == 0) { // 位置控制 (x, y, z, yaw)
            target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | 
                               mavros_msgs::PositionTarget::IGNORE_VY | 
                               mavros_msgs::PositionTarget::IGNORE_VZ | 
                               mavros_msgs::PositionTarget::IGNORE_AFX | 
                               mavros_msgs::PositionTarget::IGNORE_AFY | 
                               mavros_msgs::PositionTarget::IGNORE_AFZ | 
                               mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        } else if (motion_type == 1) { // 速度控制 (vx, vy, vz, yaw_rate)
            target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                               mavros_msgs::PositionTarget::IGNORE_PY | 
                               mavros_msgs::PositionTarget::IGNORE_PZ | 
                               mavros_msgs::PositionTarget::IGNORE_AFX | 
                               mavros_msgs::PositionTarget::IGNORE_AFY | 
                               mavros_msgs::PositionTarget::IGNORE_AFZ | 
                               mavros_msgs::PositionTarget::IGNORE_YAW;
        } else if (motion_type == 2) { // 加速度控制
            target.type_mask = mavros_msgs::PositionTarget::IGNORE_PX | 
                               mavros_msgs::PositionTarget::IGNORE_PY | 
                               mavros_msgs::PositionTarget::IGNORE_PZ | 
                               mavros_msgs::PositionTarget::IGNORE_VX | 
                               mavros_msgs::PositionTarget::IGNORE_VY | 
                               mavros_msgs::PositionTarget::IGNORE_VZ | 
                               mavros_msgs::PositionTarget::IGNORE_YAW;
        }

        return target;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, std::string("drone_communication"));
    Communication comm;
    comm.start();
    return 0;
}
