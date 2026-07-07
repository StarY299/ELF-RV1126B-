#ifndef PCA9685_H
#define PCA9685_H
#include <stdint.h>
int  pca9685_init(const char *dev, uint8_t addr);
void pca9685_close(void);
void pca9685_set_servo(uint8_t ch, int pulse_us);
void pca9685_disable_servo(uint8_t ch);
#endif
