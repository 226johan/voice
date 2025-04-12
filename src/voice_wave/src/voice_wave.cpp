#include "rclcpp/rclcpp.hpp"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "msp_types.h"
#include "qivw.h"
#include <alsa/asoundlib.h>

#define IVW_AUDIO_FILE_NAME "/home/penghm/dev/ros_package/src/voice_wave/bin/audio/xbxb.pcm"
#define FRAME_LEN 640
unsigned int SAMPLE_RATE = 16000;
#define BUFFER_SIZE 1024

// 音频捕获设备
snd_pcm_t *handle;

int cb_ivw_msg_proc(const char *sessionID, int msg, int param1, int param2, const void *info, void *userData) {
    auto node = static_cast<rclcpp::Node *>(userData);
    if (MSP_IVW_MSG_ERROR == msg) {
        RCLCPP_ERROR(node->get_logger(), "MSP_IVW_MSG_ERROR errCode:%d", param1);
    } else if (MSP_IVW_MSG_WAKEUP == msg) {
        RCLCPP_INFO(node->get_logger(), "MSP_IVW_MSG_WAKEUP result = %s", static_cast<const char*>(info));
        RCLCPP_INFO(node->get_logger(), "语音唤醒成功！");
    }
    return 0;
}

void run_ivw(rclcpp::Node::SharedPtr node, const char *grammar_list, const char* session_begin_params) {
    const char* session_id = NULL;
    int err_code = MSP_SUCCESS;
    int audio_stat = MSP_AUDIO_SAMPLE_CONTINUE;
    char audio_buffer[BUFFER_SIZE];

    session_id = QIVWSessionBegin(grammar_list, session_begin_params, &err_code);
    if (MSP_SUCCESS != err_code) {
        RCLCPP_ERROR(node->get_logger(), "QIVWSessionBegin failed, error code:%d", err_code);
        return;
    }

    err_code = QIVWRegisterNotify(session_id, cb_ivw_msg_proc, node.get());
    if (MSP_SUCCESS != err_code) {
        RCLCPP_ERROR(node->get_logger(), "QIVWRegisterNotify failed, error code:%d", err_code);
        return;
    }

    // 打开音频捕获设备
    if (snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0) < 0) {
        RCLCPP_ERROR(node->get_logger(), "无法打开音频捕获设备");
        return;
    }

    // 设置音频参数
    snd_pcm_hw_params_t *params;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(handle, params);
    snd_pcm_hw_params_set_access(handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(handle, params, 1);
    snd_pcm_hw_params_set_rate_near(handle, params, &SAMPLE_RATE, 0);
    snd_pcm_hw_params(handle, params);

    // 提示用户呼喊唤醒词
    RCLCPP_INFO(node->get_logger(), "请呼喊唤醒词以唤醒系统...");

    int first = 1;
    while (1) {
        ssize_t frames = snd_pcm_readi(handle, audio_buffer, BUFFER_SIZE / 2);
        if (frames < 0) {
            frames = snd_pcm_recover(handle, frames, 0);
        }
        if (frames < 0) {
            RCLCPP_ERROR(node->get_logger(), "读取音频数据失败");
            break;
        }

        if (first) {
            audio_stat = MSP_AUDIO_SAMPLE_FIRST;
            first = 0;
        } else {
            audio_stat = MSP_AUDIO_SAMPLE_CONTINUE;
        }

        err_code = QIVWAudioWrite(session_id, audio_buffer, frames * 2, audio_stat);
        if (MSP_SUCCESS != err_code) {
            RCLCPP_ERROR(node->get_logger(), "QIVWAudioWrite failed, error code:%d", err_code);
            break;
        }
    }

    // 关闭音频捕获设备
    snd_pcm_drain(handle);
    snd_pcm_close(handle);

    QIVWSessionEnd(session_id, NULL);
}

class VoiceWave : public rclcpp::Node {
public:
    VoiceWave() : Node("voice_wave") {
        int ret = MSP_SUCCESS;
        const char* lgi_param = "appid = cc42f548";
        ret = MSPLogin(NULL, NULL, lgi_param);
        if (MSP_SUCCESS != ret) {
            RCLCPP_ERROR(this->get_logger(), "MSPLogin failed, error code:%d", ret);
        }
    }

    ~VoiceWave() {
        MSPLogout();
    }

    void start_ivw(const char* session_begin_params) {
        run_ivw(shared_from_this(), NULL, session_begin_params);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoiceWave>();
    const char* ssb_param = "ivw_threshold=0:1450,sst=wakeup,ivw_res_path =fo|/home/penghm/dev/ros_package/src/voice_wave/bin/msc/res/ivw/wakeupresource.jet";
    node->start_ivw(ssb_param);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
