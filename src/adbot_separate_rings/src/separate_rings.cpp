#include<ros/ros.h>
#include<adbot_msgs/SprMsg.h>
#include<sensor_msgs/Joy.h>

#define SEPARATE_BUTTON 3 //ボタン
#define SPR_DUTY 100

adbot_msgs::SprMsg spr_msg;

//ジョイコンのコールバック関数
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    //両方押してるときは手は停止
    spr_msg.do = joy->buttons[SEPARATE_BUTTON];
}
int main(int argc, char **argv)
{
    //ROSを初期化
    ros::init(argc, argv, "joy");

    //ROSノードハンドルを作成
    ros::NodeHandle nh;

    spr_msg.duty = SPR_DUTY;

    //ジョイコンのトピックを購読
    ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

    
    //Arduinoにメッセージを送信
    ros::Publisher pub = nh.advertise<adbot_msgs::SprMsg>("separate", 1);

    //ROSのメインループを開始
    ros::Rate loop_rate(10);
    while(ros::ok()) {
        ros::spinOnce();
        
        // Arduinoにメッセージを送信
        pub.publish(spr_msg);

        loop_rate.sleep();
    }

    return 0;
}