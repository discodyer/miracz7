#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

int main(void)
{
    const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
    const struct device *const encoder2 = DEVICE_DT_GET(DT_ALIAS(encoder2));
    /* 检查设备是否就绪 */
    if (!device_is_ready(encoder1)) {
        printk("Error: Encoder1 device is not ready\n");
        return 0;
    }
    if (!device_is_ready(encoder2)) {
        printk("Error: Encoder2 device is not ready\n");
        return 0;
    }

    /* 配置编码器参数（可选，设备树已配置） */
    // struct encoder_config cfg = {
    //     .mode = ENCODER_MODE_RUNTIME,
    //     .steps_per_revolution = 2000,
    // };
    // ret = encoder_configure(encoder1, &cfg);

    while (1) {
        struct sensor_value val1, val2;
        double angle1, angle2;
        
        int rc;
        rc = sensor_sample_fetch(encoder1);
        if (rc != 0) {
            printk("Failed to fetch sample from encoder1 (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder1, SENSOR_CHAN_ROTATION, &val1);
        if (rc != 0) {
            printk("Failed to get data1 (%d)\n", rc);
            return 0;
        }

        rc = sensor_sample_fetch(encoder2);
        if (rc != 0) {
            printk("Failed to fetch sample from encoder2 (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder2, SENSOR_CHAN_ROTATION, &val2);
        if (rc != 0) {
            printk("Failed to get data2 (%d)\n", rc);
            return 0;
        }

        /* 计算角度和转速 */
        angle1 = fmodf((float)val1.val1 + (float)val1.val2 / 1000000.0f, 2000.0f);
        angle2 = fmodf((float)val2.val1 + (float)val2.val2 / 1000000.0f, 2000.0f);

        /* 通过串口打印结果 */
        printk("TIM2_ENCODER: Angle: %.2f° | "
               "TIM3_ENCODER: Angle: %.2f° \n",
               (double)angle1, (double)angle2);

        /* 每100ms更新一次 */
        k_sleep(K_MSEC(100));
    }
    return 0;
}