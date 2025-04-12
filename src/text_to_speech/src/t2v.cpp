#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "unistd.h"

#include "text_to_speech/qtts.h"
#include "text_to_speech/msp_cmn.h"
#include "text_to_speech/msp_errors.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

enum VOICEFLAG{
    wait_wakeup,
    wakeup,
    audio_play,
};
// 订阅文本数据，播放订阅的文本数据
class T2V: public rclcpp::Node{
    public:
        T2V(): Node("text_to_voice"){
            // 订阅语音数据
            voice_sub_ = this->create_subscription<custom_msgs::msg::voice_msg>("voice_data", 10, std::bind(&V2T::voice_callback, this, std::placeholders::_1));
            
        }

        string audio_play_chose(const string text){
            string res;
            // 判断逻辑

            return res;
        }

        void voice_callback(const custom_msgs::msg::voice_msg::ConstPtr &msg){
            while(msg->voice_flag_i == audio_play){
                // 逻辑判断要播放的语音
                msg->play_data_s=audio_play_chose(msg->hear_data_s);
                // 播放文本

                // 修改标志位为wait_wakeup
                msg->voice_flag_i = wait_wakeup;
                // 发布文本数据
                voice_pub_->publish(msg);
            }

        }

    private:
        // 订阅语音msg
        rclcpp::Subscription<custom_msgs::msg::voice_msg>::SharedPtr voice_sub_;
        // 发布语音msg
        rclcpp1::Publisher<custom_msgs::msg::voice_msg>::SharedPtr voice_pub_;
}


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::Rate loop_rate(10);
    rclcpp::spin(std::make_shared<T2V>());
    rclcpp::shutdown();

    return 0;
}