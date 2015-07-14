#include <dji_sdk_node.h>
#include <dji_sdk/app_act.h>
#include <dji_sdk/dji_act_open.h>
#include <dji_sdk/dji_publishers.h>
#include <dji_sdk/dji_variable.h>

namespace pre_act
{
	
void activation_ack_cmd_callback(ProHeader *header)
{
    uint16_t ack_data;
    printf("Sdk_ack_cmd0_callback,sequence_number=%d,session_id=%d,data_len=%d\n", header->sequence_number,
           header->session_id, header->length - EXC_DATA_SIZE);
    memcpy((uint8_t *) &ack_data, (uint8_t *) &header->magic, (header->length - EXC_DATA_SIZE));

    if (is_sys_error(ack_data)) {
        printf("[DEBUG] SDK_SYS_ERROR!!! ACTIVATION\n");
        std_msgs::Float32 msg;
        msg.data = NO_AUTHORITY;
        publishers::activation_status_pub.publish(msg);
        dji_variable::flag_success = false;
        dji_variable::flag_visit = true;
    }
    else {
        char result[][50] = {{"ACTIVATION_SUCCESS"},
                             {"PARAM_ERROR"},
                             {"DATA_ENC_ERROR"},
                             {"NEW_DEVICE_TRY_AGAIN"},
                             {"DJI_APP_TIMEOUT"},
                             {" DJI_APP_NO_INTERNET"},
                             {"SERVER_REFUSED"},
                             {"LEVEL_ERROR"}};
        printf("[ACTIVATION] Activation result: %s \n", *(result + ack_data));
        if (ack_data == 0)
            dji_variable::flag_success = true;
        else {
            dji_variable::flag_success = false;
        }
        std_msgs::Float32 msg;
        msg.data = (float) ack_data;
        publishers::activation_status_pub.publish(msg);

        if (ack_data == 0) {
            Pro_Config_Comm_Encrypt_Key(key);
            printf("[ACTIVATION] set key %s\n", key);
        }
        dji_variable::flag_visit = true;
    }
}
void activation()
{

    App_Send_Data(2, 0, MY_ACTIVATION_SET, API_USER_ACTIVATION, (uint8_t *) &activation_msg, sizeof(activation_msg),
                  activation_ack_cmd_callback, 1000, 1);

    printf("[ACTIVATION] send acticition msg: %d %d %d %d \n", activation_msg.app_id, activation_msg.app_api_level,
           activation_msg.app_ver, activation_msg.app_bundle_id[0]);
}

void nav_open_close_callback(ProHeader *header)
{
    uint16_t ack_data;
    memcpy((uint8_t *) &ack_data, (uint8_t *) &header->magic, (header->length - EXC_DATA_SIZE));

    std_msgs::Float32 msg;
    if (is_sys_error(ack_data)) {
        printf("[DEBUG] SDK_SYS_ERROR!!! OPEN :%d \n" , dji_variable::opened);
        msg.data = NO_AUTHORITY;
        publishers::nav_ctrl_status_pub.publish(msg);
    }
    else {
        msg.data = (float) ack_data;
        if (msg.data == 2)
        {
            if (dji_variable::flag_open_or_close)
            {
                printf("opened!!!\n");
                dji_variable::opened = true;
            }
            else 
            {
                printf("clsoed!!!\n");
                dji_variable::opened = false;
            }
            dji_variable::flag_success = true;
        }
        else {
            dji_variable::flag_success = false;
            publishers::nav_ctrl_status_pub.publish(msg);
        }
    }
    dji_variable::flag_visit = true;
}

void ros_nav_open_close()
{
    uint8_t send_data;
    if (dji_variable::flag_open_or_close)
    {
        send_data = 1;
        printf("send open nav %d\n", send_data);
    }
    else 
    {
        send_data = 0;
        printf("send close nav %d\n", send_data);
    }
    App_Send_Data(1, 1, MY_CTRL_CMD_SET, API_OPEN_SERIAL, (uint8_t *) &send_data, sizeof(send_data),
                  nav_open_close_callback, 1000, 0);
}

}