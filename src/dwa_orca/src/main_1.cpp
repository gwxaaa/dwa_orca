#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "KinematicModel.h" // 引入你的运动学模型头文件

geometry_msgs::Pose modelPose;
geometry_msgs::Twist Twist;
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    std::string targetModelName = "model2.3"; 
    int modelIndex = -1;
    // 找到目标模型的索引
    for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == targetModelName) {
            modelIndex = i;
            break;
        }
    }
    if (modelIndex >= 0) {
        // 获取特定模型的位姿信息
        modelPose = msg->pose[modelIndex];
    }

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "main_1");
    ros::NodeHandle nh;
    // 创建一个订阅器，用于获取模型状态信息
    ros::Subscriber modelStateSub = nh.subscribe("/gazebo/model_states", 1, modelStateCallback);
    // 循环以接收消息
    ros::Rate loop_rate(100);  // 
    ros::Publisher modelStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1);
    Twist.linear.x = -10.0;  // 新的线速度
    Twist.angular.z = -1.0; // 新的角速度

    while (ros::ok()) {
        // 使用运动学模型计算新的位置
        double time = 0.01;  
        KinematicModel kinematicModel(modelPose, Twist);  // 创建运动学模型对象
        geometry_msgs::Pose newPose = kinematicModel.calculateNewPose(time);
        // 创建一个 gazebo_msgs::ModelState 消息，用于更新模型状态
        gazebo_msgs::ModelState modelState;
        modelState.model_name = "model2.3";  // 模型名称
        modelState.pose = newPose;  

        modelStatePub.publish(modelState);

        //对信息进行输出展示
        ROS_INFO("pose: x = %f, y = %f, z = %f", modelPose.position.x, modelPose.position.y, modelPose.position.z);
        double  initial_yaw=atan2(2.0*(modelPose.orientation.w*modelPose.orientation.z+modelPose.orientation.x*modelPose.orientation.y),
                         1.0-2.0*(modelPose.orientation.y*modelPose.orientation.y+modelPose.orientation.z*modelPose.orientation.z));
        ROS_INFO("Yaw: %f degrees", initial_yaw);
        ROS_INFO("newpose: x = %f, y = %f, z = %f", newPose.position.x, newPose.position.y, newPose.position.z);
        double yaw = atan2(2.0 * (newPose.orientation.w * newPose.orientation.z + newPose.orientation.x * newPose.orientation.y),
                           1.0 - 2.0 * (newPose.orientation.y * newPose.orientation.y + newPose.orientation.z * newPose.orientation.z));
        ROS_INFO("newYaw: %f degrees", yaw);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


//关键点，创建运动学模型对象的代码放置位置，放置不正确就会导致相关的信息传入错误。
//传出的时候，需要考虑速度信息的传送，否则就会导致瞬间达到。