#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 0
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY 950    // Boundary value to distinguish between black and white readings
#define IR_SENSOR_PIN GPIO_NUM_0
const int weights[5] = {-5, -3, 1, 3, 5};

// Motor value bounds
int optimum_duty_cycle = 57;
int lower_duty_cycle = 45;
int higher_duty_cycle = 60;
float left_duty_cycle = 0, right_duty_cycle = 0;
int left_turn_flag = 0;
int right_turn_flag = 0;
int u_turn_flag = 0;
int all_black_flag = 1;
// Line Following PID Variables
float error = 0, prev_error = 0, difference, cumulative_error, correction;
#define NOR 15


int objectflag = 1;

#define REQUIRED_WHITE_COUNT 19
int cwhitecount = 0;
// Sensor history tracking for sensors 0 and 4
int sensor_0_history[NOR] = {0};
int sensor_4_history[NOR] = {0};
int history_index = 0;

// Union containing line sensor readings
line_sensor_array line_sensor_readings;

void store_sensor_history() {
    // Store binary values indicating black (1) or white (0) for sensors 0 and 4
    sensor_0_history[history_index] = (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY) ? 1 : 0;
    sensor_4_history[history_index] = (line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY) ? 1 : 0;
    history_index = (history_index + 1) % NOR; // Circular buffer
}

void calculate_correction() {
    difference = error - prev_error;
    cumulative_error += error;
    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error +
                 read_pid_const().ki * cumulative_error +
                 read_pid_const().kd * difference;
    prev_error = error;
}

float calculate_average(int sensor_history[])
{
    int sum = 0;
    for (int i = 0; i < NOR; i++)
    {
        sum += sensor_history[i];
    }
    return (float)sum / NOR; // Result represents the proportion of black readings
}

void calculate_error() {
    all_black_flag = 1;
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    bool left_reading = line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY;
    bool right_reading = line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY;

    for (int i = 0; i < 5; i++) {
        int k = line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY ? 1 : 0;
        weighted_sum += (float)(weights[i]) * k;
        sum += k;

        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY) {
            all_black_flag = 0;
        }
    }

    if (left_reading == 1 ) {
        left_turn_flag = 1;
        right_turn_flag = 0;
    } else if (left_reading == 0 && right_reading == 1) {
        left_turn_flag = 0;
        right_turn_flag = 1;
    } else {
        left_turn_flag = right_turn_flag = 0;
    }

    if (all_black_flag == 1) {
        u_turn_flag = 1;
    } else {
        u_turn_flag = 0;
    }

    if (sum != 0) {
        pos = (weighted_sum - 1) / sum;
    }

    if (all_black_flag == 1) {
        error = prev_error > 0 ? 10 : -10;
    } else {
        error = pos;
    }
}

void line_follow_task(void* arg) {
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    ESP_ERROR_CHECK(enable_bar_graph());


    //ir
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << IR_SENSOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
    //
    #ifdef CONFIG_ENABLE_OLED
        ESP_ERROR_CHECK(init_oled());
        vTaskDelay(100);
        lv_obj_clean(lv_scr_act());
    #endif

    while (true) {


        int ir_state = gpio_get_level(IR_SENSOR_PIN);
        if (ir_state == 0) {
            ESP_LOGI("debug", "Obstacle detected!");
            // Handle obstacle detection logic here
            // You could stop or change the behavior when the obstacle is detected
        } else {
            ESP_LOGI("debug", "No obstacle detected.");
        }


        line_sensor_readings = read_line_sensor(line_sensor);
        
        for (int i = 0; i < 5; i++) {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - line_sensor_readings.adc_reading[i];
        }
        //END
        int all_white = 1;
        if(line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[3]> BLACK_BOUNDARY  && 
        line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY ){
            all_white = 1;
        }
        else{
            all_white = 0;
        }

        // Update consecutive white count
        if (all_white) {
            cwhitecount++;
        } else {
            cwhitecount = 0;
        }

        if(all_white && ir_state == 0){
            set_motor_speed(motor_a_0, MOTOR_FORWARD, higher_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_BACKWARD, higher_duty_cycle);
            vTaskDelay(1100 / portTICK_PERIOD_MS);
        }

        // Stop the bot if consecutive white count exceeds threshold
        if (cwhitecount >= REQUIRED_WHITE_COUNT) {
            set_motor_speed(motor_a_0, MOTOR_STOP, 0);
            set_motor_speed(motor_a_1, MOTOR_STOP, 0);
            ESP_LOGI("debug", "End of line detected. Stopping bot.");
            break;
        }


        //
        calculate_error();
        calculate_correction();
        store_sensor_history(); // Record sensor history for sensors 0 and 4

        float leftavg = calculate_average(sensor_0_history);
        float rightavg = calculate_average(sensor_4_history);


        left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        //inverted
         if(line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY && 
        line_sensor_readings.adc_reading[3]> BLACK_BOUNDARY  && 
        line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY ){
            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, left_duty_cycle);
        }


        if (left_turn_flag == 1) {
            set_motor_speed(motor_a_0, MOTOR_BACKWARD, right_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            leftavg = 0;

            ESP_LOGI("debug", "LF == 1");
        } else if (right_turn_flag == 1 && left_turn_flag == 0) {
            if (line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY) {
                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
            ESP_LOGI("debug", "STRAIGHT + RIGHT");

            }
            else{
                ESP_LOGI("debug", "ONLY RIGHT");

                set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, left_duty_cycle);
            }

        }
        else if(left_turn_flag == 1 && right_turn_flag == 1){
                ESP_LOGI("debug", "TTTTTTTTTTTT");

            set_motor_speed(motor_a_0, MOTOR_BACKWARD, higher_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, higher_duty_cycle);
        }
         else if (u_turn_flag) {
            
            if(rightavg > 0.1 && leftavg < 0.1){
                ESP_LOGI("debug", "UUU RIGHT");

                set_motor_speed(motor_a_0, MOTOR_FORWARD, higher_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_BACKWARD, higher_duty_cycle);
            }
            else{
                ESP_LOGI("debug", "UUU LEFT");

                set_motor_speed(motor_a_0, MOTOR_BACKWARD, higher_duty_cycle);
                set_motor_speed(motor_a_1, MOTOR_FORWARD, higher_duty_cycle);
            }
            vTaskDelay(400 / portTICK_PERIOD_MS);
            leftavg = 0;
            rightavg = 0;
        } else {
                ESP_LOGI("debug", "PID");

            set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        }

        // if(all_black_flag == 1){
        //     if(rightavg > 0.05 && leftavg < 0.05){
        //         set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
        //         set_motor_speed(motor_a_1, MOTOR_BACKWARD, right_duty_cycle);
        //     }
        // }

        #ifdef CONFIG_ENABLE_OLED
            if (read_pid_const().val_changed) {
                display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
                reset_val_changed_pid_const();
            }
        #endif
        ESP_LOGI("debug", "leftavg: %f, rightavg: %f",leftavg, rightavg);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main() {   
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}