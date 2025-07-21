#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/sensor/encoder_stm32.h>

int main(void)
{
    const struct device *const encoder1 = DEVICE_DT_GET(DT_ALIAS(encoder1));
    const struct device *const encoder2 = DEVICE_DT_GET(DT_ALIAS(encoder2));
    /* 检查设备是否就绪 */
    if (!device_is_ready(encoder1)) {
        printf("Error: Encoder1 device is not ready\n");
        return 0;
    }
    if (!device_is_ready(encoder2)) {
        printf("Error: Encoder2 device is not ready\n");
        return 0;
    }

    while (1) {
        struct sensor_value val1_rotation, val2_rotation, val1_revolution, val2_revolution;
        double angle1, angle2;
        
        int rc;
        rc = sensor_sample_fetch(encoder1);
        if (rc != 0) {
            printf("Failed to fetch sample from encoder1 (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder1, SENSOR_CHAN_ROTATION, &val1_rotation);
        if (rc != 0) {
            printf("Failed to get val1_rotation (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder1, SENSOR_CHAN_REVOLUTIONS, &val1_revolution);
        if (rc != 0) {
            printf("Failed to get val1_revolution (%d)\n", rc);
            return 0;
        }

        rc = sensor_sample_fetch(encoder2);
        if (rc != 0) {
            printf("Failed to fetch sample from encoder2 (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder2, SENSOR_CHAN_ROTATION, &val2_rotation);
        if (rc != 0) {
            printf("Failed to get val2_rotation (%d)\n", rc);
            return 0;
        }

        rc = sensor_channel_get(encoder2, SENSOR_CHAN_REVOLUTIONS, &val2_revolution);
        if (rc != 0) {
            printf("Failed to get val2_revolution (%d)\n", rc);
            return 0;
        }

        /* 计算角度和转速 */
        angle1 = (val1_rotation.val1 + val1_rotation.val2 / 1000000.0f) + val1_revolution.val1 * 360;
        angle2 = (val2_rotation.val1 + val2_rotation.val2 / 1000000.0f) + val2_revolution.val1 * 360;

        /* 通过串口打印结果 */
        printf("%.2f, %.2f\n",
               (double)angle1, (double)angle2);

        /* 每100ms更新一次 */
        k_sleep(K_MSEC(100));
    }
    return 0;
}