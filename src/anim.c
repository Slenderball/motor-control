#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <stdatomic.h>  
#include <stdbool.h>
#include <string.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>

#define HIDE_CURSOR()  printf("\033[?25l")
#define SHOW_CURSOR()  printf("\033[?25h")
#define COLOR_RED    "\033[31m"
#define COLOR_GREEN  "\033[32m"
#define COLOR_YELLOW "\033[33m"
#define COLOR_RESET  "\033[0m"
#define MOTOR_PATH "/home/slend/robot_data/motor"
#define IMU_PATH   "/home/slend/robot_data/imu"
#define SPEED_PATH "/home/slend/robot_data/speed"
#define CALIB_PATH "/home/slend/robot_data/calib.csv"

typedef enum {
    MOTOR_IDLE = 0,
    MOTOR_OK = 1,
    MOTOR_WARNING = 2,
    MOTOR_ERROR = 3
} MotorStatus;

_Atomic MotorStatus motor_status;
atomic_bool is_running = true;
typedef struct {
    char *f1;
    char *f2;
} Frames;

typedef struct {
    int pwm;   
    float speed_mean;
    float speed_std;
    float acc_mean;
    float acc_std;
    float gyro_mean;
    float gyro_std;
    float temp_mean;
    float temp_std;
} motor;

motor calib_up[101];
motor calib_down[101];
MotorStatus last_sent_status = MOTOR_IDLE;
pthread_mutex_t print_mutex = PTHREAD_MUTEX_INITIALIZER;
int start_pwm = 25, grace_period = 20;
char current_msg[64] = "";

void send_msg(const char* msg, MotorStatus current_status) {
    if (current_status != last_sent_status) {
        last_sent_status = current_status;
        pthread_mutex_lock(&print_mutex);
        strncpy(current_msg, msg, sizeof(current_msg) - 1);
        pthread_mutex_unlock(&print_mutex);
    }
}

void update_sensor_status(float index, float warn_thresh, float error_thresh, int* error_counter, MotorStatus* out_status, const char** color, const char* warn_msg, const char* error_msg) {
    if (fabs(index) >= error_thresh) {
        (*error_counter)++;
        *out_status = MOTOR_ERROR;
        *color = COLOR_RED;
        send_msg(error_msg, MOTOR_ERROR);
    } else if (fabs(index) >= warn_thresh) {
        *error_counter = 0;
        *out_status = MOTOR_WARNING;
        *color = COLOR_YELLOW;
        send_msg(warn_msg, MOTOR_WARNING);
    } else {
        *error_counter = 0;
        *out_status = MOTOR_OK;
        *color = COLOR_RESET;
        send_msg("                                          ", MOTOR_OK);
    }
}

void emergency_stop(FILE *f_motor, const char *reason) {
    motor_status = MOTOR_ERROR;
    send_msg(reason, MOTOR_ERROR);
    fprintf(f_motor, "s000");
    fflush(f_motor);
}

void read_calibration(){
    FILE *f_calib = fopen(CALIB_PATH, "r");
    if (!f_calib) { perror("File Error Calib"); return; }
    char line[128], dir[8];
    int pwm;
    fgets(line, sizeof(line), f_calib);
    fgets(line, sizeof(line), f_calib);
    while (fgets(line, sizeof(line), f_calib) != NULL) {
        motor m;
        sscanf(line, "%d,%7[^,],%f,%f,%f,%f,%f,%f,%f,%f",
            &pwm, dir,
            &m.speed_mean, &m.speed_std,
            &m.acc_mean,   &m.acc_std,
            &m.gyro_mean,  &m.gyro_std,
            &m.temp_mean,  &m.temp_std);
        m.pwm = pwm;
        if (strcmp(dir, "up") == 0)
            calib_up[pwm] = m;
        else if (strcmp(dir, "down") == 0)
            calib_down[pwm] = m;
    }
    fclose(f_calib);
    FILE *f_meta = fopen("/home/slend/robot_data/motor_meta.csv", "r");
    if (f_meta) {
        fscanf(f_meta, "start_pwm,%d", &start_pwm);
        fclose(f_meta);
    }
}

int kbhit(void) {
    struct termios oldt, newt;
    int ch, oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

char* load_frame_to_ram(const char* filename) {
    FILE *f = fopen(filename, "r");
    if (!f) return NULL;
    fseek(f, 0, SEEK_END);
    long length = ftell(f);
    fseek(f, 0, SEEK_SET);
    char *buffer = malloc(length + 1);
    if (buffer) {
        fread(buffer, 1, length, f);
        buffer[length] = '\0'; 
    }
    fclose(f);
    return buffer;
}

void smart_update(const char *old_frame, const char *new_frame, const char* color) {
    int row = 1, col = 1;
    for (int i = 0; new_frame[i] != '\0' && old_frame[i] != '\0'; i++) {
        if (new_frame[i] == '\n') {
            row++; col = 1; continue;
        }
        if (new_frame[i] != old_frame[i]) {
            printf("\033[%d;%dH%s%c%s", row, col, color, new_frame[i], COLOR_RESET);
        }
        col++;
    }
}


void* animation_thread(void* arg) {
    Frames* frames = (Frames*)arg;
    const char *color = COLOR_GREEN, *last_color = color;
    int i = 0;

    printf("\033[2J");
    HIDE_CURSOR(); 
    printf("\033[H%s%s%s", color, frames->f1, COLOR_RESET);
    fflush(stdout);

    while (atomic_load(&is_running)) {
        pthread_mutex_lock(&print_mutex);
        MotorStatus current_status = atomic_load(&motor_status);
        if (current_status == MOTOR_IDLE) color = COLOR_RESET;
        else if (current_status == MOTOR_OK) color = COLOR_GREEN;
        else if (current_status == MOTOR_WARNING) color = COLOR_YELLOW;
        else if (current_status == MOTOR_ERROR) color = COLOR_RED;

        if (color != last_color) {
            printf("\033[H%s%s%s", color, (i % 2 == 0) ? frames->f1 : frames->f2, COLOR_RESET);
            last_color = color;
        } else {
            if (i % 2 == 0) smart_update(frames->f2, frames->f1, color);
            else smart_update(frames->f1, frames->f2, color);
        }
        MotorStatus status = atomic_load(&motor_status);
        const char *msg_color;
        if (status == MOTOR_WARNING) msg_color = COLOR_YELLOW;
        else if (status == MOTOR_ERROR) msg_color = COLOR_RED;
        else msg_color = COLOR_RESET;
        
        printf("\033[44;70H\033[K\033[1m%s[ GRACE PERIOD: %-3d ]%s\033[0m", msg_color, grace_period > 0 ? grace_period : 0, COLOR_RESET);
        printf("\033[45;70H\033[K\033[1m%s[ MESSAGE     ]%s\033[0m", msg_color, COLOR_RESET);
        printf("\033[46;70H\033[K\033[1m%s%-42s%s\033[0m", msg_color, current_msg, COLOR_RESET);
        
        fflush(stdout);
        i++;
        pthread_mutex_unlock(&print_mutex);
        usleep(500000);
    }
    return NULL;
}

int main() {
    struct termios orig_termios, new_termios;
    tcgetattr(STDIN_FILENO, &orig_termios);
    new_termios = orig_termios;
    new_termios.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

    Frames frames;
    frames.f1 = load_frame_to_ram("art_1.txt");
    frames.f2 = load_frame_to_ram("art_2.txt");
    FILE *f_log = fopen("log.csv", "a");
    MotorStatus local_status = MOTOR_IDLE;
    if (!frames.f1 || !frames.f2) return 1;
    pthread_t ui_thread_id;
    if (pthread_create(&ui_thread_id, NULL, animation_thread, &frames) != 0) {
        printf("Thread creation error!\n");
        return 1;
    }
    read_calibration();
    
    float acc_vib, gyro_vib, temp, speed_index = 0.0, acc_index = 0.0, gyro_index = 0.0, temp_index = 0.0,
        ambient_acc = 0.0, ambient_gyro = 0.0, ambient_data[3] = {0.0, 0.0, 0.0};
    int speed, i=0, temp_error_time = 0, gyro_error_time = 0, acc_error_time = 0, speed_error_time = 0;
    char dir = 's', command_buffer[16], line_buffer[128];
    static bool going_up = true, motor_running = false;
    FILE *f_motor = fopen(MOTOR_PATH, "w");
    const char *speed_color = COLOR_RESET, *acc_vib_color = COLOR_RESET, 
        *gyro_vib_color = COLOR_RESET, *temp_color = COLOR_RESET;
    MotorStatus temp_status = MOTOR_OK;
    
    if (f_motor == NULL) {
        perror("File Error Motor");
        return 1;
    }
    
    usleep(1000000);
    motor_status = MOTOR_OK;
    
    for (int i = 0; i < 50; i++)
    {
        FILE *f_imu = fopen(IMU_PATH, "r");
        if (f_imu) {
            char temp_buf[128];
            line_buffer[0] = '\0';
            while(fgets(temp_buf, sizeof(temp_buf), f_imu) != NULL) {
                 if(strlen(temp_buf) > 5) {
                    strncpy(line_buffer, temp_buf, sizeof(line_buffer) - 1);
                    line_buffer[sizeof(line_buffer) - 1] = '\0';
                 }
            }
            if (strlen(line_buffer) == 0) {
                acc_vib = gyro_vib = temp = 0;
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
            if (i%10==0)
            {
                printf("Calibrating IMU... %d%%\n", 5-(i+1)/10);
                fflush(stdout);
            }
            
        }else {
            motor_status = MOTOR_ERROR;
            send_msg("IMU DATA LOST!\n", MOTOR_ERROR);
            continue;
        }
    } 
    ambient_acc = ambient_data[0]/50.0;
    ambient_gyro = ambient_data[1]/50.0;
    printf("Entering main loop\n");
    fflush(stdout);

    while (1)
    {   
        if (i<0) i=0;
        if (i>100) i=100;

        if (i==0){
            motor_running = false;
        }
        else if(i>0 && !motor_running) {
            if (i < start_pwm) i = start_pwm;
            if (speed > 5) motor_running = true;
        }
        else if (motor_running && !going_up && i < start_pwm) {
            i = 0;
            motor_running = false;
            rewind(f_motor);
            
            fprintf(f_motor, "s000");
            fflush(f_motor);
        }
       
        motor *active_calib = going_up ? calib_up : calib_down;
        if (i>0) dir = 'f';
        else dir = 's';
        snprintf(command_buffer, sizeof(command_buffer), "%c%03d", dir, i);
        rewind(f_motor);
        fprintf(f_motor, "%s", command_buffer);
        fflush(f_motor);

        FILE *f_speed = fopen(SPEED_PATH, "r");
        if (f_speed) {
            if (fgets(line_buffer, sizeof(line_buffer), f_speed) != NULL) {
                speed = (int)strtol(line_buffer, NULL, 10);
            }
            fclose(f_speed);
        } else {
            speed = 0;
        }

        FILE *f_imu = fopen(IMU_PATH, "r");
        if (f_imu) {
            char temp_buf[128];
            line_buffer[0] = '\0';
            while(fgets(temp_buf, sizeof(temp_buf), f_imu) != NULL) {
                 if(strlen(temp_buf) > 5) {
                    strncpy(line_buffer, temp_buf, sizeof(line_buffer) - 1);
                    line_buffer[sizeof(line_buffer) - 1] = '\0';
                 }
            }
            if (strlen(line_buffer) == 0) {
                acc_vib = gyro_vib = temp = 0;
            }
            
            line_buffer[strcspn(line_buffer, "\n")] = 0;
            int items = sscanf(line_buffer, "%f|%f|%f", &acc_vib, &gyro_vib, &temp);
            acc_vib -= ambient_acc;
            gyro_vib -= ambient_gyro;
            if (items < 3) {
                acc_vib = 0; gyro_vib = 0; temp = 0;
            }
            fclose(f_imu);
        }
        else {
            emergency_stop(f_motor, "IMU DATA LOST!");
            break;
        }
        static float filtered_acc = 0;
        filtered_acc = 0.9 * filtered_acc + 0.1 * acc_vib;
        static float filtered_gyro = 0;
        filtered_gyro = 0.9 * filtered_gyro + 0.1 * gyro_vib;

        float safe_speed = active_calib[i].speed_std;
        if (safe_speed < 0.1f) safe_speed = 0.1f;
        float safe_acc = active_calib[i].acc_std;
        if (safe_acc < 0.01f) safe_acc = 0.01f;
        float safe_gyro = active_calib[i].gyro_std;
        if (safe_gyro < 0.01f) safe_gyro = 0.01f;
        float safe_temp = active_calib[i].temp_std;
        if (safe_temp < 0.5f) safe_temp = 0.5f;

        speed_index = (speed - active_calib[i].speed_mean) / safe_speed;
        acc_index = (filtered_acc - active_calib[i].acc_mean) / safe_acc;
        gyro_index = (filtered_gyro - active_calib[i].gyro_mean) / safe_gyro;
        temp_index = (temp - active_calib[i].temp_mean) / safe_temp;

        

        if (grace_period > 0) {
            grace_period--;
            speed_error_time = acc_error_time = gyro_error_time = temp_error_time = 0;
        } else {
            printf("\033[43;1H\033[K"); 
            fflush(stdout);

            update_sensor_status(speed_index, 2.0, 3.0, &speed_error_time, &local_status, &speed_color,
                                "Speed out of safe range!", "Speed critically high!");
            update_sensor_status(acc_index, 2.5, 4.0, &acc_error_time, &local_status, &acc_vib_color,
                                "ACC out of safe range!", "ACC critically!");
            update_sensor_status(gyro_index, 2.5, 4.0, &gyro_error_time, &local_status, &gyro_vib_color,
                                "GYRO out of safe range!",  "GYRO critical!");
            if (temp >= 70 || temp <= 0) {
                temp_error_time++;
                temp_status = MOTOR_ERROR;
                temp_color = COLOR_RED;
                send_msg("ERROR TEMPERATURE! IMMEDIATE STOP!", MOTOR_ERROR);
            } else if ((temp >= 50 && temp < 70) || (temp > 0 && temp <= 10)) {
                temp_error_time = 0;
                temp_status = MOTOR_WARNING;
                temp_color = COLOR_YELLOW;
                send_msg("Temperature out of safe range!", MOTOR_WARNING);
            } else {
                temp_error_time = 0;
                temp_status = MOTOR_OK;
                temp_color = COLOR_RESET;
                send_msg("                                          ", MOTOR_OK);
            }

            if (local_status == MOTOR_ERROR || temp_status == MOTOR_ERROR) motor_status = MOTOR_ERROR;
            else if (local_status == MOTOR_WARNING || temp_status == MOTOR_WARNING) motor_status = MOTOR_WARNING;
            else motor_status = MOTOR_OK;
        }
        

        printf("\033[10;50H\033[K\033[1m[ SYSTEM METRICS ]\033[0m");        
        printf("\033[12;50H\033[K\033[1mSPEED     : %s%d\033[0m", speed_color,speed);
        printf("\033[14;50H\033[K\033[1mVIB ACCEL : %s%.4f\033[0m", acc_vib_color, filtered_acc);
        printf("\033[15;50H\033[K\033[1mVIB GYRO  : %s%.4f\033[0m", gyro_vib_color, filtered_gyro);    
        printf("\033[17;50H\033[K\033[1mTEMP      : %s%.2f °C\033[0m", temp_color, temp);
        printf("\033[19;50H\033[KPOWER: %d", i);
        fflush(stdout);

        if (speed_error_time >= 20 || acc_error_time >= 10 || gyro_error_time >= 10 || temp_error_time >= 10) {
            emergency_stop(f_motor, "CRITICAL SENSOR FAILURE");
            usleep(5000000);
            break; 
        }
        
        if (kbhit()) {
            char c = getchar();
            if (c == 's' || c == 'S' || c == ' ' ) {
                fprintf(f_motor, "s000");
                fflush(f_motor);
                atomic_store(&is_running, false);
                motor_status = MOTOR_IDLE;
                break;
            }
            if (c == '\033' ) {
                char c1 = getchar();
                char c2 = getchar();
                if (c1 == '[' ) {
                    if (c2 =='A')
                    {
                        i=i+5;
                        going_up = true;
                        grace_period = 5;
                    }
                    else if (c2 =='B')
                    {
                        i=i-5;
                        going_up = false;
                        grace_period = 5;
                    }                    
                }
            }
        }
        fprintf(f_log,"%d,%d,%.3f,%.3f,%.3f,%.3f,%d\n",
            i,
            speed,
            speed_index,
            acc_index,
            gyro_index,
            temp_index,
            motor_status);
        fflush(f_log);
        usleep(300000);
    }
    atomic_store(&is_running, false);
    pthread_join(ui_thread_id, NULL);
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios);
    fprintf(f_motor, "s000");
    fflush(f_motor);
    fclose(f_motor);
    fclose(f_log);
    free(frames.f1);
    printf("\033[2J\033[H\033[?25h");
    fflush(stdout);
    system("reset");
    SHOW_CURSOR();
    free(frames.f2);
    return 0;
}