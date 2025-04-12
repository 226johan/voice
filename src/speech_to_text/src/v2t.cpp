#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "unistd.h"
#include "termio.h"

#include "speech_to_text/qisr.h"
#include "speech_to_text/msp_cmn.h"
#include "speech_to_text/msp_errors.h"
#include "speech_to_text/speech_recognizer.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "custom_msgs/msg/voice_msg.hpp"

// 监听语音识别标志位，接收到后开始语音识别，识别完成后发布唤醒标志位
// 语音识别完成后，应该发布文本数据

enum VOICEFLAG{
    wait_wakeup,
    wakeup,
    audio_play,
};


class V2T: public rclcpp::Node{
    public:
        V2T(): Node("voice_to_text"){
            // 订阅语音数据
            voice_sub_ = this->create_subscription<custom_msgs::msg::voice_msg>("voice_data", 10, std::bind(&V2T::voice_callback, this, std::placeholders::_1));
            // 判断状态
            // 发布文本数据
            
        }

        void voice_callback(const custom_msgs::msg::voice_msg::ConstPtr &msg){
            while(msg->voice_flag_i == wakeup){
                // 开始语音识别

                // 发布文本数据
                msg->hear_data_s = text;
                // 修改标志位为audio_play
                msg->voice_flag_i = audio_play;

                // 发布数据
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
    rclcpp::spin(std::make_shared<V2T>());
    rclcpp::shutdown(); 
    return 0;
}