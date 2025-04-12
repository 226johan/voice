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

// 基于按键唤醒的语音识别交互

int wakeupFlag = 0;
int resultFlag = 0;

#define FRAME_LEN 640
#define BUFFER_SIZE 4096

static int upload_userwords()
{
    char *userwords = NULL;
    size_t len = 0;
    size_t read_len = 0;
    FILE *fp = NULL;
    int ret = -1;

    fp = fopen("userwords.txt", "rb");
    if (fp == NULL)
    {
        printf("open userwords.txt failed\n");
        goto upload_exit;
    }

    fseek(fp, 0, SEEK_END);
    len = ftell(fp);
    fseek(fp, 0, SEEK_SET);

    userwords = (char *)malloc(len + 1);
    if (NULL == userwords)
    {
        printf("\nout of memory\n");
        goto upload_exit;
    }

    read_len = fread((void *)userwords, 1, len, fp);
    if (read_len != len)
    {
        printf("\nread [userwords.txt] failed\n");
        goto upload_exit;
    }
    userwords[len] = '\0';

    MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret);
    if (MSP_SUCCESS != ret)
    {
        printf("MSPUploadData failed, errcode=%d\n", ret);
        goto upload_exit;
    }

upload_exit:
    if (NULL != fp)
    {
        fclose(fp);
        fp = NULL;
    }

    if (NULL != userwords)
    {
        free(userwords);
        userwords = NULL;
    }

    return ret;
}

static void show_result(char *string, char is_over)
{
    resultFlag = 1;
    printf("\rResult: [ %s ]", string);
    if (is_over)
        putchar('\n');
}

static char *g_result = NULL;
static unsigned int g_buffersize = BUFFER_SIZE;

void on_result(const char *result, char is_last)
{
    if (result)
    {
        // 动态扩容机制，防止内存溢出
        size_t left = g_buffersize - 1 - strlen(g_result);
        size_t size = strlen(result);
        if (left < size)
        {
            g_result = (char *)realloc(g_result, g_buffersize + size + 1);
            if (g_result)
            {
                g_buffersize += BUFFER_SIZE;
            }
            else
            {
                printf("mem alloc failed\n");
                return;
            }
        }
        strncat(g_result, result, size);
        show_result(g_result, is_last);
    }
}

// 语音识别请求开始回调函数
// 应该先确认结果是否为空，若不为空，则先释放掉
// 然后向结果buffer申请内存，并初始化内存
void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);
    }
    g_result = (char *)malloc(BUFFER_SIZE);
    g_buffersize = BUFFER_SIZE;
    memset(g_result, 0, g_buffersize);
    printf("Start Listening...\n");
}

void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        printf("\nSpeaking done \n");
    else
        printf("\nError: %d\n", reason);
}

static void demo_file(const char *audio_file, const char *session_begin_params)
{
    int errcode = 0;
    FILE *f_pcm = NULL;
    char *p_pcm = NULL;
    unsigned long pcm_count = 0;
    unsigned long pcm_size = 0;
    unsigned long read_size = 0;

    struct speech_rec iat;
    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end};

    if (NULL == audio_file)
    {
        goto iat_exit;
    }
    f_pcm = fopen(audio_file, "rb");
    if (NULL == f_pcm)
    {
        printf("\nopen %s failed\n", audio_file);
        goto iat_exit;
    }
    fseek(f_pcm, 0, SEEK_END);
    pcm_size = ftell(f_pcm);
    fseek(f_pcm, 0, SEEK_SET);
    p_pcm = (char *)malloc(pcm_size);
    if (NULL == p_pcm)
    {
        printf("\nout of memory\n");
        goto iat_exit;
    }

    read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
    if (read_size != pcm_size)
    {
        printf("\nread %s failed\n", audio_file);
        goto iat_exit;
    }

    errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
    if (errcode)
    {
        printf("speech recognizer init failed\n");
        goto iat_exit;
    }

    errcode = sr_start_listening(&iat);
    if (errcode)
    {
        printf("start listening failed %d\n", errcode);
        goto iat_exit;
    }

    while (1)
    {
        unsigned int len = 10 * FRAME_LEN;
        int ret = 0;
        if (pcm_size < 2 * len)
        {
            len = pcm_size;
        }
        if (len <= 0)
            break;
        ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);
        if (0 != ret)
        {
            printf("\n write audio data failed ! error code = %d\n", ret);
            goto iat_exit;
        }
        pcm_count += (long)len;
        pcm_size -= (long)len;
    }

    errcode = sr_stop_listening(&iat);
    if (errcode)
    {
        printf("stop listening failed %d\n", errcode);
        goto iat_exit;
    }

iat_exit:
    if (NULL != f_pcm)
    {
        fclose(f_pcm);
        f_pcm = NULL;
    }
    if (NULL != p_pcm)
    {
        free(p_pcm);
        p_pcm = NULL;
    }
    sr_stop_listening(&iat);

    sr_uninit(&iat);
}

static void demo_mic(const char *session_begin_params)
{
    int errcode;
    int i = 0;

    struct speech_rec iat;
    struct speech_rec_notifier recnotifier = {
        on_result,
        on_speech_begin,
        on_speech_end};

    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
    if (errcode)
    {
        printf("speech recognizer init failed\n");
        return;
    }
    errcode = sr_start_listening(&iat);
    if (errcode)
    {
        printf("start listening failed %d\n", errcode);
    }

    int ch;
    while (1)
    {
        ch = getchar();
        if (ch == 32)
        {
            printf("\nSpeaking done \n");
            break;
        }
    }
    errcode = sr_stop_listening(&iat);
    if (errcode)
    {
        printf("stop listening failed %d\n", errcode);
    }

    sr_uninit(&iat);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::Rate loop_rate(10);

    auto node = std::make_shared<rclcpp::Node>("speech_to_text_ndoe");
    node->declare_parameter("appid", "cc42f548");

    auto voiceWordsPub = node->create_publisher<std_msgs::msg::String>("voicewords", 10);

    termios tms_old, tms_new;
    tcgetattr(0, &tms_old);
    tms_new = tms_old;
    tms_new.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(0, TCSANOW, &tms_new);

    RCLCPP_INFO(node->get_logger(), "Press \"Space\" key to Start,Press \"Enter\" key to Exit.");
    int count = 0;
    int ch;
    while (rclcpp::ok())
    {
        ch = getchar();

        printf("Pressed key Value: %d\n", ch);
        if (ch == 32)
        {
            // space key
            wakeupFlag = 1;
        }
        if (ch == 10)
        {
            // enter key
            RCLCPP_INFO(node->get_logger(), "node Exiting...");
            break;
        }
        if (wakeupFlag == 1)
        {
            int ret = MSP_SUCCESS;
            int upload_on = 1;
            // Want to upload the user words ? \n0: No.\n1: Yes\n
            std::string lp = "appid = " + node->get_parameter("appid").as_string() + ", work_dir = .";
            const char *login_params = lp.c_str();
            int aud_src = 0;
            // "0: From a audio file.\n1: From microphone.\n"
            const char *session_begin_params =
                "sub = iat, domain = iat, language = zh_cn, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";
            ret = MSPLogin(NULL, NULL, login_params);
            if (ret != MSP_SUCCESS)
            {
                MSPLogout();
                printf("MSPLogin failed, ret=%d\n", ret);
            }
            printf("Demo recognizing the speech from microphone...\n");

            demo_mic(session_begin_params);

            wakeupFlag = 0;
            MSPLogout();
        }
        if (resultFlag)
        {
            resultFlag = 0;
            std_msgs::msg::String msg;
            msg.data = g_result;
            voiceWordsPub->publish(msg);
        }
        rclcpp::spin_some(node);
        loop_rate.sleep();
        count++;
    }

exit:
    tcsetattr(0, TCSANOW, &tms_old);
    MSPLogout();
    return 0;
}