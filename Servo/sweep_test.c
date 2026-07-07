/** 舵机左右扫描测试 */
#include <stdio.h>
#include <unistd.h>
#include "pca9685.h"

#define I2C_DEV "/dev/i2c-4"
#define PAN     0

int main(void) {
    printf("=== Servo Sweep Test ===\n");

    if (pca9685_init(I2C_DEV, 0x40) != 0) {
        printf("[FAIL] PCA9685 not found at %s addr 0x40\n", I2C_DEV);
        printf("Check: is the PCA9685 board connected to I2C?\n");
        return -1;
    }
    printf("[OK] PCA9685 detected!\n");

    /* 先归中 */
    pca9685_set_servo(PAN, 1500);
    printf("Center 1500us ...\n");
    sleep(1);

    /* 左右扫 3 轮 */
    for (int i = 0; i < 3; i++) {
        printf("Sweep LEFT  (800us)  ...\n");
        pca9685_set_servo(PAN, 800);
        usleep(800000);

        printf("Sweep RIGHT (2200us) ...\n");
        pca9685_set_servo(PAN, 2200);
        usleep(800000);
    }

    /* 归中 */
    printf("Center 1500us ...\n");
    pca9685_set_servo(PAN, 1500);
    sleep(1);

    pca9685_close();
    printf("=== Done ===\n");
    return 0;
}
