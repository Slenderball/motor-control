#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MOTOR_PATH "/home/slend/robot_data/motor"
#define IMU_PATH   "/home/slend/robot_data/imu"
#define SPEED_PATH "/home/slend/robot_data/speed"
#define CALIB_PATH "/home/slend/robot_data/calib.csv"

float collect_samples(const char *label, int pwm, float ambient_acc, float ambient_gyro, FILE *f_calib, FILE *f_motor){
    float mean_speed, variance_speed, std_speed,
          mean_acc, variance_acc, std_acc,
          mean_gyro, variance_gyro, std_gyro,
          mean_temp, variance_temp, std_temp;
    char line_buffer[128];
    char command_buffer[16];

    snprintf(command_buffer, sizeof(command_buffer), "f%03d", pwm);
    rewind(f_motor);
    fprintf(f_motor, "%s", command_buffer);
    fflush(f_motor);
    
    if (pwm < 20) usleep(500000);
    else usleep(1000000);

    int samples = 30, valid_samples = 0;
    long sum_speed = 0;
    long long sum_speed_sq = 0;
    float sum_acc = 0.0, sum_acc_sq = 0.0, sum_gyro = 0.0,
        sum_gyro_sq = 0.0, sum_temp = 0.0, sum_temp_sq  = 0.0;

    for (int s = 0; s < samples; s++) {
        usleep(100000);

        int current_speed = 0;
        float current_acc = 0.0, current_gyro = 0.0, current_temp = 0.0;

        FILE *f_speed = fopen(SPEED_PATH, "r");
        if (f_speed) {
            if (fgets(line_buffer, sizeof(line_buffer), f_speed) != NULL) {
                current_speed = (int)strtol(line_buffer, NULL, 10);
            }
            fclose(f_speed);
        }
        if (current_speed < 0 || current_speed > 10000) continue;
        valid_samples++;

        FILE *f_imu = fopen(IMU_PATH, "r");
        if (f_imu) {
            char temp_buf[128];
            while(fgets(temp_buf, sizeof(temp_buf), f_imu) != NULL) {
                 if(strlen(temp_buf) > 5) {
                     strcpy(line_buffer, temp_buf);
                 }
            }
            line_buffer[strcspn(line_buffer, "\n")] = 0;
            int items = sscanf(line_buffer, "%f|%f|%f", &current_acc, &current_gyro, &current_temp);
            current_acc -= ambient_acc;
            current_gyro -= ambient_gyro;
            if (items < 3) {
                current_acc = 0; current_gyro = 0; current_temp = 0;
            }
            fclose(f_imu);
        }

            sum_speed += current_speed;
            sum_speed_sq += current_speed * current_speed;
            sum_acc += current_acc;
            sum_acc_sq += current_acc * current_acc;
            sum_gyro += current_gyro;
            sum_gyro_sq += current_gyro * current_gyro;
            sum_temp += current_temp;
            sum_temp_sq += current_temp * current_temp;
    }
    if (valid_samples == 0) {
        fprintf(f_calib, "%d,%s,0.0000,0.0100,0.0000,0.0100,0.0000,0.0100,0.0000,0.0100\n", pwm, label);
        return 0.0f;
    }

    mean_speed = (float)sum_speed / valid_samples;
    variance_speed = ((float)sum_speed_sq / valid_samples) - (mean_speed * mean_speed);
    mean_acc = sum_acc / valid_samples;
    variance_acc = (sum_acc_sq / valid_samples) - (mean_acc * mean_acc);
    mean_gyro = sum_gyro / valid_samples;
    variance_gyro = (sum_gyro_sq / valid_samples) - (mean_gyro * mean_gyro);
    mean_temp = sum_temp / valid_samples;
    variance_temp = (sum_temp_sq / valid_samples) - (mean_temp * mean_temp);
    if (variance_speed < 0) variance_speed = 0;
    if (variance_acc < 0) variance_acc = 0;
    if (variance_gyro < 0) variance_gyro = 0;
    if (variance_temp < 0) variance_temp = 0;

    std_speed = sqrt(variance_speed);
    std_acc = sqrt(variance_acc);
    std_gyro= sqrt(variance_gyro);
    std_temp = sqrt(variance_temp);

    if (std_speed<0.01)
    {
        std_speed=0.01;
    }
    if (std_acc<0.01)
    {
        std_acc=0.01;
    }
    if (std_gyro<0.01)
    {
        std_gyro=0.01;
    }
    if (std_temp<0.01)
    {
        std_temp=0.01;
    }
    fprintf(f_calib, "%d,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
        pwm, label, mean_speed, std_speed, mean_acc, std_acc,
        mean_gyro, std_gyro, mean_temp, std_temp);
    return mean_speed;
}


int main() {
    char line_buffer[128];
    float acc_vib = 0.0, gyro_vib = 0.0, temp = 0.0, ambient_acc = 0.0, ambient_gyro = 0.0, ambient_data[3] = {0.0, 0.0, 0.0}, speed = 0.0;
    int start_pwm = -1;
    for (int i = 0; i < 50; i++)
    {
        FILE *f_imu = fopen(IMU_PATH, "r");
        if (f_imu) {
            char temp_buf[128];
            while(fgets(temp_buf, sizeof(temp_buf), f_imu) != NULL) {
                 if(strlen(temp_buf) > 5) {
                     strcpy(line_buffer, temp_buf);
                 }
            }
            line_buffer[strcspn(line_buffer, "\n")] = 0;
            int items = sscanf(line_buffer, "%f|%f|%f", &acc_vib, &gyro_vib, &temp);
            if (items < 3) {
                acc_vib = 0; gyro_vib = 0; temp = 0;
            }
            ambient_data[0] +=acc_vib;
            ambient_data[1] +=gyro_vib;
            ambient_data[2] +=temp;
            fclose(f_imu);
            usleep(100000);
            printf("Calibrating IMU... %d%%\n", (i+1)*100/50);
            fflush(stdout);            
        }
    } 
    ambient_acc = ambient_data[0]/50.0;
    ambient_gyro = ambient_data[1]/50.0;

    FILE *f_motor = fopen(MOTOR_PATH, "w");
    FILE *f_calib = fopen(CALIB_PATH, "w");

    if (f_motor == NULL || f_calib == NULL) {
        perror("File Error");
        return 1;
    } else {
        printf("Started calibration process\n");
    }

  
    fprintf(f_calib, "sep=,\n");
    fprintf(f_calib, "PWM,Direction,SpeedMean,SpeedStd,AccMean,AccStd,GyroMean,GyroStd,TempMean,TempStd\n");
    
    printf("Calibrating ascending...\n");
    for (int i = 0; i <= 100; i++)
    {
        speed = collect_samples("up", i, ambient_acc, ambient_gyro, f_calib, f_motor);
        if (speed > 5.0f && start_pwm == -1) start_pwm = (i / 5) * 5;
        
        printf("Progress: %d%%\n", (i));
        fflush(stdout);
    }
    

    printf("Calibrating descending...\n");
    for (int i = 100; i >= 0; i--)
    {
        speed = collect_samples("down", i, ambient_acc, ambient_gyro, f_calib, f_motor);
        printf("Progress: %d%%\n", (i));
        fflush(stdout);
    }


    if (start_pwm == -1) start_pwm = 25;

    FILE *f_meta = fopen("/home/slend/robot_data/motor_meta.csv", "w");
    fprintf(f_meta, "start_pwm,%d\n", start_pwm);
    fclose(f_meta);
    printf("start_pwm=%d\n", start_pwm);

    fprintf(f_motor, "s000");
    fflush(f_motor);
    
    fclose(f_motor);
    fclose(f_calib);    

    printf("\nCalibration completed successfully! File calib.csv updated.\n");
    return 0;
}