#include "rclcpp/rclcpp.hpp"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "msp_types.h"
#include "qivw.h"
#include <alsa/asoundlib.h>


enum VOICEFLAG{
    wait_wakeup,
    wakeup,
    audio_play,
};
// 监听唤醒词，唤醒后发送语音识别标志位，并等待唤醒标志位

class WakeUp : public rclcpp::Node {
public:
    WakeUp() : Node("voice_wave") {
        // 订阅语音数据
        voice_sub_ = this->create_subscription<custom_msgs::msg::voice_msg>("voice_data", 10, std::bind(&V2T::voice_callback, this, std::placeholders::_1));
    }

    bool wait_wakeup_is_ok()
    {

        return true;
    }
    void voice_callback(const custom_msgs::msg::voice_msg::ConstPtr &msg){
        while(msg->voice_flag_i == wait_wakeup){
            // 等待唤醒
            while(wait_wakeup_is_ok())
            {
                msg->voice_flag_i == wakeup;
                // 发布语音数据
                voice_pub_->publish(msg);
            }
            // 修改标志位为wait_wakeup
            msg->voice_flag_i = wait_wakeup;           
        }

    }

private:
    // 订阅语音msg
    rclcpp::Subscription<custom_msgs::msg::voice_msg>::SharedPtr voice_sub_;
    // 发布语音msg
    rclcpp1::Publisher<custom_msgs::msg::voice_msg>::SharedPtr voice_pub_;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WakeUp>());
    rclcpp::shutdown();
    return 0;
}
