#include<ros/ros.h>
#include<std_msgs/Int16.h>
#include<sensor_msgs/Joy.h>

#define CLOSE_HAND_BUTTON 5 //R1ボタン
#define OPEN_HAND_BUTTON 4 //L1ボタン

std_msgs::Int16 arm_status_msg = 0;

//ジョイコンのコールバック関数
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    if(joy->buttons[OPEN_HAND_BUTTON]){
        arm_status_msg.data = 1;
    }
    else if(joy->buttons[CLOSE_HAND_BUTTON]){
        arm_status_msg.data = 2;
    }else{
        arm_status_msg.data = 0;
    }
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
    ros::Publisher pub = nh.advertise<std_msgs::Int16>("hand_state", 1);

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
