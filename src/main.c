#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

#define UPDATE_INTERVAL_MS 10  // 500ms update interval

static const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(bmi08x_accel));
static const struct device *gyro = DEVICE_DT_GET(DT_NODELABEL(bmi08x_gyro));

int main(void)
{
    if (!device_is_ready(accel)) {
        printf("Accelerometer device not ready\n");
        return -1;
    }

    if (!device_is_ready(gyro)) {
        printf("Gyroscope device not ready\n");
        return -1;
    }

    printf("BMI088 sensor initialization completed, starting data output...\n");
    printf("----------------------------------------\n");

    while (1) {
        struct sensor_value accel_data[3], gyro_data[3];
        int ret;

        /* Read accelerometer data */
        ret = sensor_sample_fetch(accel);
        if (ret == 0) {
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_data[0]);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_data[1]);
            sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_data[2]);

            printf("Accelerometer data: X=%.2f m/s², Y=%.2f m/s², Z=%.2f m/s²\n",
                   sensor_value_to_double(&accel_data[0]),
                   sensor_value_to_double(&accel_data[1]),
                   sensor_value_to_double(&accel_data[2]));
        } else {
            printf("Failed to read accelerometer data: %d\n", ret);
        }

        /* Read gyroscope data */
        ret = sensor_sample_fetch(gyro);
        if (ret == 0) {
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_X, &gyro_data[0]);
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_Y, &gyro_data[1]);
            sensor_channel_get(gyro, SENSOR_CHAN_GYRO_Z, &gyro_data[2]);

            printf("Gyroscope data:   X=%.2f rad/s, Y=%.2f rad/s, Z=%.2f rad/s\n",
                   sensor_value_to_double(&gyro_data[0]),
                   sensor_value_to_double(&gyro_data[1]),
                   sensor_value_to_double(&gyro_data[2]));
        } else {
            printf("Failed to read gyroscope data: %d\n", ret);
        }

        printf("----------------------------------------\n");
        k_msleep(UPDATE_INTERVAL_MS);
    }

    return 0;
}