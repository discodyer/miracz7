#include <zephyr/kernel.h>
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

    while (1) {
        struct sensor_value val1_rotation, val2_rotation;
        struct sensor_value val1_revolution, val2_revolution;
        
        /* 读取编码器数据 */
        if (sensor_sample_fetch(encoder1) != 0 || 
            sensor_sample_fetch(encoder2) != 0) {
            printk("Failed to fetch encoder samples\n");
            continue;
        }

        sensor_channel_get(encoder1, SENSOR_CHAN_ROTATION, &val1_rotation);
        sensor_channel_get(encoder1, SENSOR_CHAN_REVOLUTIONS, &val1_revolution);
        sensor_channel_get(encoder2, SENSOR_CHAN_ROTATION, &val2_rotation);
        sensor_channel_get(encoder2, SENSOR_CHAN_REVOLUTIONS, &val2_revolution);

        /* 计算并打印角度 */
        double angle1 = (val1_rotation.val1 + val1_rotation.val2 / 1000000.0) 
                      + val1_revolution.val1 * 360;
        double angle2 = (val2_rotation.val1 + val2_rotation.val2 / 1000000.0) 
                      + val2_revolution.val1 * 360;
        
        printf("%.2f, %.2f\n", angle1, angle2);
        
        // /* 根据编码器数据调整电机（示例逻辑） */
        // if (angle1 > 360.0) {
        //     run_motor = false; // 停止电机
        // }
        
        k_sleep(K_MSEC(100));
    }
    return 0;
}