#include<ros/ros.h>
#include<std_msgs/Bool.h>
#include<sensor_msgs/Joy.h>

#define LOAD_RING_BUTTON 5 //R1ボタン
#define CATCH_RING_BUTTON 4 //L1ボタン

bool isPressed = false;
std_msgs::Bool arm_status_msg;

//ジョイコンのコールバック関数
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    arm_status_msg.data = joy->buttons[LOAD_RING_BUTTON];
}

int main(int argc, char **argv)
{
    //ROSを初期化
    ros::init(argc, argv, "joy");

    //ROSノードハンドルを作成
    ros::NodeHandle nh;

    //ジョイコンのトピックを購読
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

    
    //Arduinoにメッセージを送信
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("load_ring", 1);

    //ROSのメインループを開始
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        
        // Arduinoにメッセージを送信
        pub.publish(arm_status_msg);

        loop_rate.sleep();
    }

    return 0;
}
