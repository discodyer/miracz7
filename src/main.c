#include <zephyr/kernel.h>
#include "HelloWorld.h"
#include "microxrce_transports.h"

#include <uxr/client/client.h>
#include <ucdr/microcdr.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define STREAM_HISTORY  8
#define BUFFER_SIZE     UXR_CONFIG_CUSTOM_TRANSPORT_MTU * STREAM_HISTORY

void main(void)
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor/encoder_stm32.h>
#include <zephyr/drivers/motor.h>

#define MOTOR_DEVICE DT_NODELABEL(motor_controller)

/* 定义电机控制线程栈 */
#define MOTOR_THREAD_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(motor_thread_stack, MOTOR_THREAD_STACK_SIZE);
static struct k_thread motor_thread_data;

/* 电机控制标志位 */
static volatile bool run_motor = true;

/* 电机控制线程函数 */
void motor_thread(void *, void *, void *)
{
    const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor_controller));
    
    if (!device_is_ready(motor)) {
        printk("Motor device not ready\n");
        return;
    }

    while (run_motor) {
        // 通道A正向50%速度
        set_motor(motor, MOTOR_CHANNEL_A, MOTOR_FORWARD, 20);
        // 通道B正向50%速度
        set_motor(motor, MOTOR_CHANNEL_B, MOTOR_FORWARD, 20);
        k_sleep(K_SECONDS(2));
        
        // 停止两个电机
        set_motor(motor, MOTOR_CHANNEL_A, MOTOR_STOP, 0);
        set_motor(motor, MOTOR_CHANNEL_B, MOTOR_STOP, 0);
        
        k_sleep(K_SECONDS(2));

        // 通道A反向50%速度
        set_motor(motor, MOTOR_CHANNEL_A, MOTOR_REVERSE, 20);
        // 通道B反向50%速度
        set_motor(motor, MOTOR_CHANNEL_B, MOTOR_REVERSE, 20);
        k_sleep(K_SECONDS(2));
        
        // 停止两个电机
        set_motor(motor, MOTOR_CHANNEL_A, MOTOR_STOP, 0);
        set_motor(motor, MOTOR_CHANNEL_B, MOTOR_STOP, 0);
        
        k_sleep(K_SECONDS(2));
    }
}

int main(void)
{

    /* 启动电机控制线程 */
    k_thread_create(&motor_thread_data,
                   motor_thread_stack,
                   K_THREAD_STACK_SIZEOF(motor_thread_stack),
                   motor_thread,
                   NULL, NULL, NULL,
                   CONFIG_MAIN_THREAD_PRIORITY + 1,  // 比主线程高1级
                   0, K_NO_WAIT);

    const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
    const struct device *const encoder2 = DEVICE_DT_GET(DT_ALIAS(encoder2));

    /* 检查设备是否就绪 */
    if (!device_is_ready(encoder1) || !device_is_ready(encoder2)) {
        printk("Encoders not ready\n");
        return 0;
    }

    uxrCustomTransport transport;
    uxr_set_custom_transport_callbacks(&transport,
                                        true,
                                        zephyr_transport_open,
                                        zephyr_transport_close,
                                        zephyr_transport_write,
                                        zephyr_transport_read);
    // Transport
    if(!uxr_init_custom_transport(&transport, &default_params))
    {
        printf("Error at create transport.\n");
        while(1){};
    }

    // Session
    uxrSession session;
    uxr_init_session(&session, &transport.comm, 0xAAAABBBB);
    if(!uxr_create_session(&session))
    {
        printf("Error at create session.\n");
        while(1){};
    }

    // Streams
    uint8_t output_reliable_stream_buffer[BUFFER_SIZE];
    uxrStreamId reliable_out = uxr_create_output_reliable_stream(&session, output_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    uint8_t input_reliable_stream_buffer[BUFFER_SIZE];
    uxr_create_input_reliable_stream(&session, input_reliable_stream_buffer, BUFFER_SIZE, STREAM_HISTORY);

    // Create entities
    uxrObjectId participant_id = uxr_object_id(0x01, UXR_PARTICIPANT_ID);
    const char* participant_xml = "<dds>"
                                      "<participant>"
                                          "<rtps>"
                                              "<name>default_xrce_participant</name>"
                                          "</rtps>"
                                      "</participant>"
                                  "</dds>";
    uint16_t participant_req = uxr_buffer_create_participant_xml(&session, reliable_out, participant_id, 0, participant_xml, UXR_REPLACE);

    uxrObjectId topic_id = uxr_object_id(0x01, UXR_TOPIC_ID);
    const char* topic_xml = "<dds>"
                                "<topic>"
                                    "<name>HelloWorldTopic</name>"
                                    "<dataType>HelloWorld</dataType>"
                                "</topic>"
                            "</dds>";
    uint16_t topic_req = uxr_buffer_create_topic_xml(&session, reliable_out, topic_id, participant_id, topic_xml, UXR_REPLACE);

    uxrObjectId publisher_id = uxr_object_id(0x01, UXR_PUBLISHER_ID);
    const char* publisher_xml = "";
    uint16_t publisher_req = uxr_buffer_create_publisher_xml(&session, reliable_out, publisher_id, participant_id, publisher_xml, UXR_REPLACE);

    uxrObjectId datawriter_id = uxr_object_id(0x01, UXR_DATAWRITER_ID);
    const char* datawriter_xml = "<dds>"
                                     "<data_writer>"
                                         "<topic>"
                                             "<kind>NO_KEY</kind>"
                                             "<name>HelloWorldTopic</name>"
                                             "<dataType>HelloWorld</dataType>"
                                         "</topic>"
                                     "</data_writer>"
                                 "</dds>";
    uint16_t datawriter_req = uxr_buffer_create_datawriter_xml(&session, reliable_out, datawriter_id, publisher_id, datawriter_xml, UXR_REPLACE);

    // Send create entities message and wait its status
    uint8_t status[4];
    uint16_t requests[4] = {participant_req, topic_req, publisher_req, datawriter_req};
    if(!uxr_run_session_until_all_status(&session, 1000, requests, status, 4))
    {
        printf("Error at create entities: participant: %i topic: %i publisher: %i darawriter: %i\n", status[0], status[1], status[2], status[3]);
        while(1){};
    }

    // Write topics
    bool connected = true;
    uint32_t count = 0;
    while(connected)
    {
        HelloWorld topic = {++count, "Hello DDS world!"};

        ucdrBuffer ub;
        uint32_t topic_size = HelloWorld_size_of_topic(&topic, 0);
        uxr_prepare_output_stream(&session, reliable_out, datawriter_id, &ub, topic_size);
        HelloWorld_serialize_topic(&ub, &topic);

        printf("Send topic: %s, id: %i\n", topic.message, topic.index);
        connected = uxr_run_session_time(&session, 1000);
    }

    // while (1) {
    //     struct sensor_value val1_rotation, val2_rotation;
    //     struct sensor_value val1_revolution, val2_revolution;
        
    //     /* 读取编码器数据 */
    //     if (sensor_sample_fetch(encoder1) != 0 || 
    //         sensor_sample_fetch(encoder2) != 0) {
    //         printk("Failed to fetch encoder samples\n");
    //         continue;
    //     }

    //     sensor_channel_get(encoder1, SENSOR_CHAN_ROTATION, &val1_rotation);
    //     sensor_channel_get(encoder1, SENSOR_CHAN_REVOLUTIONS, &val1_revolution);
    //     sensor_channel_get(encoder2, SENSOR_CHAN_ROTATION, &val2_rotation);
    //     sensor_channel_get(encoder2, SENSOR_CHAN_REVOLUTIONS, &val2_revolution);

    //     /* 计算并打印角度 */
    //     double angle1 = (val1_rotation.val1 + val1_rotation.val2 / 1000000.0) 
    //                   + val1_revolution.val1 * 360;
    //     double angle2 = (val2_rotation.val1 + val2_rotation.val2 / 1000000.0) 
    //                   + val2_revolution.val1 * 360;
        
    //     printf("%.2f, %.2f\n", angle1, angle2);
        
    //     // /* 根据编码器数据调整电机（示例逻辑） */
    //     // if (angle1 > 360.0) {
    //     //     run_motor = false; // 停止电机
    //     // }
        
    //     k_sleep(K_MSEC(100));
    // }

    // Delete resources
    uxr_delete_session(&session);
    uxr_close_custom_transport(&transport);

    while(1){};
}
