/*****************************************************************************************
 * 自定义控制器跟踪egoplanner轨迹
 * 本代码采用的mavros的速度控制进行跟踪
 * 编译成功后直接运行就行，遥控器先position模式起飞，然后rviz打点，再切offborad模式即可
 ******************************************************************************************/
#include <ros/ros.h> // 引入ROS头文件
#include <visualization_msgs/Marker.h> // 引入可视化标记消息头文件
#include <geometry_msgs/PoseStamped.h> // 引入带时间戳的姿态消息头文件
#include <geometry_msgs/Twist.h> // 引入速度消息头文件
#include <sensor_msgs/Joy.h> // 引入控制手柄消息头文件
#include <mavros_msgs/CommandBool.h> // 引入布尔命令消息头文件
#include <mavros_msgs/CommandLong.h> // 引入长整型命令消息头文件
#include <mavros_msgs/SetMode.h> // 引入设置模式命令消息头文件
#include <mavros_msgs/State.h> // 引入无人机状态消息头文件
#include <mavros_msgs/PositionTarget.h> // 引入位置目标消息头文件
#include <mavros_msgs/RCIn.h> // 引入遥控器输入消息头文件
#include "quadrotor_msgs/PositionCommand.h" // 引入四旋翼位置命令消息头文件
#include <nav_msgs/Odometry.h> // 引入里程计消息头文件
#include <tf/transform_datatypes.h> // 引入TF变换数据类型头文件
#include <tf/transform_broadcaster.h> // 引入TF变换广播器头文件
#include <tf2_ros/transform_listener.h> // 引入TF2变换监听器头文件
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 引入TF2几何消息头文件

#define VELOCITY2D_CONTROL 0b101111111000 // 设置掩码，控制哪些字段，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
// 设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.

class Ctrl
{
    public:
        Ctrl(); // 构造函数
        void state_cb(const mavros_msgs::State::ConstPtr &msg); // 状态回调函数
        void position_cb(const nav_msgs::Odometry::ConstPtr &msg); // 位置回调函数
        void target_cb(double x,double y,double z,double yaw);
        void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg); // 轨迹命令回调函数
        void control(const ros::TimerEvent&); // 控制函数
        ros::NodeHandle nh; // 节点句柄
        visualization_msgs::Marker trackpoint; // 可视化标记（没用到）
        quadrotor_msgs::PositionCommand ego; // EGO规划器的信息
        tf::StampedTransform ts; // 用来发布无人机当前位置的坐标系坐标轴
        tf::TransformBroadcaster tfBroadcasterPointer; // 广播坐标轴
        unsigned short velocity_mask = VELOCITY2D_CONTROL; // 控制掩码
        mavros_msgs::PositionTarget current_goal; // 当前目标
        nav_msgs::Odometry position_msg; // 位置消息
        geometry_msgs::PoseStamped target_pos; // 目标位置消息
        mavros_msgs::State current_state; // 当前状态
        geometry_msgs::Twist vel_pub; // 速度消息
        geometry_msgs::Pose local_enu_pos;
        float position_x, position_y, position_z; // 无人机当前位置
        float now_x, now_y, now_yaw; // 当前参考点位置和航向
        float current_yaw; // 当前航向
        float targetpos_x, targetpos_y; // 目标位置
        float ego_pos_x = 0, ego_pos_y = 0, ego_pos_z = 1; // EGO规划器位置
        float ego_vel_x, ego_vel_y, ego_vel_z; // EGO规划器速度
        float ego_a_x, ego_a_y, ego_a_z; // EGO规划器加速度
        float ego_yaw, ego_yaw_rate; // EGO规划器航向和航向速率
        bool receive, get_now_pos; // 触发轨迹的条件判断
        ros::Subscriber state_sub, twist_sub, target_sub, position_sub,yaw_change_sub; // 订阅者
        ros::Publisher local_pos_pub, local_pos_pub2, pubMarker; // 发布者
        ros::Publisher local_vel_pub; // vel发布者
        ros::Timer timer; // 定时器
};

// 构造函数
Ctrl::Ctrl()
{
    timer = nh.createTimer(ros::Duration(0.02), &Ctrl::control, this); // 创建一个定时器，每0.02秒调用一次 control 函数
    state_sub = nh.subscribe("/iris_0/mavros/state", 10, &Ctrl::state_cb, this); // 订阅无人机状态消息
    position_sub = nh.subscribe("/iris_0/mavros/local_position/odom", 10, &Ctrl::position_cb, this); // 订阅无人机位置消息
    // yaw_change_sub = nh.subscribe("/yaw_change_msg", 10, &Ctrl::yaw_change_cb, this); // 订阅yaw角改变信息
    local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/cmd_sub_test", 1); // 发布无人机目标位置
    get_now_pos = false; // 初始化标志，表示是否获取了当前位置信息
    receive = false; // 初始化标志，表示是否接收到目标位置
}

// 状态回调函数（没用到）
void Ctrl::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg; // 更新当前状态
}

// 位置回调函数，更新当前位置信息
void Ctrl::position_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    position_msg = *msg; // 更新位置消息
    tf2::Quaternion quat; // 创建四元数对象
    tf2::convert(msg->pose.pose.orientation, quat); // 将姿态中的四元数转换到 tf2::Quaternion
    double roll, pitch, yaw; // 定义滚转角、俯仰角、航向角
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw); // 从四元数中提取出滚转角、俯仰角、航向角
    ts.stamp_ = msg->header.stamp; // 设置时间戳
    ts.frame_id_ = "world"; // 设置父坐标系为“world”
    ts.child_frame_id_ = "drone_frame"; // 设置子坐标系为“drone_frame”
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w)); // 设置坐标系旋转
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z)); // 设置坐标系原点位置
    // tfBroadcasterPointer.sendTransform(ts); // 广播坐标变换
    position_x = position_msg.pose.pose.position.x; // 更新当前位置X坐标
    position_y = position_msg.pose.pose.position.y; // 更新当前位置Y坐标
    position_z = position_msg.pose.pose.position.z; // 更新当前位置Z坐标
    current_yaw = yaw; // 更新当前航向角
}
//设置目标点
void Ctrl::target_cb(double x,double y,double z,double yaw)
{
    current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED; // 设置坐标系为LOCAL_NED
    current_goal.header.stamp = ros::Time::now(); // 设置时间戳
    current_goal.type_mask = velocity_mask; // 设置掩码
    current_goal.position.x = x; // 设置X坐标
    current_goal.position.y = y; // 设置Y坐标
    current_goal.position.z = z; // 设置Z坐标
    current_goal.yaw = yaw; // 设置航向角
}
// 控制函数
void Ctrl::control(const ros::TimerEvent&)
{   
    local_pos_pub.publish(current_goal); // 发布目标位置
    
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_test"); // 初始化ROS节点
    setlocale(LC_ALL,""); // 设置本地化（确保中文显示正常）
    Ctrl ctrl; // 创建Ctrl对象
    std::cout<<"========================start_ctrl========================"<<std::endl; // 输出启动信息
    //设置目标点
    ctrl.target_cb(0,0,1,0);
    ros::spin(); // 进入ROS事件循环
    return 0; // 程序结束
}
