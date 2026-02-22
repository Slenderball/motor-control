#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define MPU_OUT_H    0x3B 

int mpu_wake_up(int file) {
    uint8_t data[14] = {PWR_MGMT_1, 0};
    if(write(file, data, 2)!=2){
        return -1;
    }
    return 0;
}

void mpu_data(int file, int16_t* segmented_data) {
    uint8_t reg[1] = {MPU_OUT_H};
    write(file, reg, 1);
    uint8_t data[14] = {0};
    read(file, data, 14);
    for (int i = 0; i < 7; i++)
    {
        segmented_data[i]= (int16_t)((data[i*2] << 8) | data[i*2+1]);
    }
}

int main() {
    int file;
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0) {
        printf("Error opening bus!\n");
        return 1;
    }
    FILE *log_file = fopen("/tmp/imu_data.csv", "w");
    
    if (log_file == NULL) {
        perror("Error opening log file");
        return 1;
    }

    ioctl(file, I2C_SLAVE, MPU_ADDR);

    if (mpu_wake_up(file) == -1)
    {
        return 1;
    }
    printf("Sensor is awake! Reading data...\n");
    float temp, acc_vibration, gyro_vibration;
    float acc_data[3];
    float gyro_data[3];
    int j;
    int16_t data[7] = {0};
    while (1) {
        j = 0;
        mpu_data(file, data);
        for (int i = 0; i < 7; i++)
        {
            if (i < 3)
            {
                acc_data[j] = data[i]/16384.0;
                printf("%.2f \n ", acc_data[j]);
                j++;
            }
            else if (i>3)
            {
                gyro_data[j] = data[i]/131.0;
                j++;
                printf("%.2f \n ", gyro_data[j-1]);
            }
            else
            {
                temp = (data[i]/ 340.0) + 36.53;
                j = 0;
                printf("%.2f \n ", temp);
            }        
        }
        acc_vibration = sqrt(acc_data[0]* acc_data[0]+acc_data[1]* acc_data[1]+acc_data[2]* acc_data[2]);
        acc_vibration = fabs (acc_vibration - 1.0);

        gyro_vibration = sqrt(gyro_data[0]* gyro_data[0]+gyro_data[1]* gyro_data[1]+gyro_data[2]* gyro_data[2]);

        rewind(log_file);
        ftruncate(fileno(log_file), 0);
        fprintf(log_file, "%.2f|%.2f|%.2f\n", acc_vibration, gyro_vibration, temp);
        fflush(log_file);
        usleep(500000);
    }

    close(file);
    return 0;
}