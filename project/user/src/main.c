/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2019,逐飞科技
 * All rights reserved.
 *
 * 以下所有内容版权均属逐飞科技所有，未经允许不得用于商业用途，
 * 欢迎各位使用并传播本程序，修改内容时必须保留逐飞科技的版权声明。
 *
 * @file             main
 * @company          成都逐飞科技有限公司
 * @author           逐飞科技(QQ3184284598)
 * @version          查看doc内version文件 版本说明
 * @Software         IAR 8.32.4 or MDK 5.28
 * @Target core      MM32F327X_G9P
 * @Taobao           https://seekfree.taobao.com/
 * @date             2022-04-11
 ********************************************************************************************************************/
// hhhhhh
#include "zf_common_headfile.h"

#define KEY1 (G0)  // G0按键存取GPS数据
#define KEY2 (G1)  // G1按键暂停存取
#define KEY3 (G2)  // G2按键开跑

#define KEY1_EXTI (EXTI0_IRQn)  // 对应外部中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体
#define KEY2_EXTI (EXTI1_IRQn)  // 对应外部中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体
#define KEY3_EXTI (EXTI2_IRQn)  // 对应外部中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

#define SERVO_MOTOR_TIM TIM_2             // 定义主板上舵机对应TIM
#define SERVO_MOTOR_PWM TIM2_PWM_CH1_A15  // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ 200              // 定义主板上舵机频率  请务必注意范围 50-300

#define SERVO_MOTOR_L_MAX -10  // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX 120  // 定义主板上舵机活动范围 角度

#define SERVO_MOTOR_DUTY(x) (PWM_DUTY_MAX / (1000 / SERVO_MOTOR_FREQ) * (1 + x / 180))

#define PWM_CH1 TIM2_PWM_CH1_A0
#define DIR_CH1 A1

#if (SERVO_MOTOR_FREQ < 50 || SERVO_MOTOR_FREQ > 300)
#error "SERVO_MOTOR_FREQ ERROE!"
#endif

float servo_motor_duty = 0;  // 舵机动作角度
float servo_motor_dir = 1;   // 舵机动作状态

void gps_data_get(void);         //获取GPS数据
void success_get_gps(void);      //成功获取GPS数据的响应
void success_read_gps(uint8 i);  //成功从Flash读取GPS
void key1_exti_handler(void);    // Key1触发
void key2_exti_handler(void);    // Key1触发
void key3_exti_handler(void);    // Key3触发

void go_to(float lat2, float long2);  // A点到B点的控制

void wireless(void);  //无线串口通讯函数

//用于串口通讯
uint16 send_index = 0;
uint8 buff[64];

//一页即1024字节 double为8字节 则一页可以存128个double
#define FLASH_SECTION_INDEX0 (FLASH_SECTION_126)  // 存储经度用的扇区 倒数第二个扇区
#define FLASH_SECTION_INDEX1 (FLASH_SECTION_127)  // 存储维度用的扇区 倒数第一个扇区
#define FLASH_PAGE_INDEX (FLASH_PAGE_3)           // 存储数据用的页码 倒数第一个页码

uint8 exti_state[4] = {0};
uint8 cnt = 0;  //存取GPS点数
uint32 buffer0[512] = {0};
uint32 buffer1[512] = {0};
// **************************** 代码区域 ****************************
int main(void) {
    clock_init(SYSTEM_CLOCK_120M);       // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                        // 初始化默认 Debug UART
    gps_init();                          // GPS初始化
    ips200_init(IPS200_TYPE_PARALLEL8);  // 显示屏观察数据状态

    exti_init(KEY1, EXTI_TRIGGER_RISING);  // 初始化 KEY1 为外部中断输入 上升沿触发
    exti_init(KEY2, EXTI_TRIGGER_RISING);  // 初始化 KEY2 为外部中断输入 上升沿触发
    exti_init(KEY3, EXTI_TRIGGER_RISING);  // 初始化 KEY3 为外部中断输入 上升沿触发

    // interrupt_set_priority(KEY1_EXTI, 0);  // 设置 KEY1 对应外部中断的中断有先级为 0
    // interrupt_set_priority(KEY2_EXTI, 1);  // 设置 KEY2 对应外部中断的中断有先级为 1
    // interrupt_set_priority(KEY3_EXTI, 2);  // 设置 KEY3 对应外部中断的中断有先级为 2

    wireless_uart_init();
    pwm_init(PWM_CH1, 1000, 0);
    gpio_init(DIR_CH1, GPO, GPIO_LOW, GPO_PUSH_PULL);

    // pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty));  //舵机设置

    // GPS数据存储
    ips200_show_string(0, 0, "GPS test");

    if (flash_check(FLASH_SECTION_INDEX0, FLASH_PAGE_INDEX))       // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX0, FLASH_PAGE_INDEX);  // 擦除这一页
    if (flash_check(FLASH_SECTION_INDEX1, FLASH_PAGE_INDEX))       // 判断是否有数据
        flash_erase_page(FLASH_SECTION_INDEX1, FLASH_PAGE_INDEX);  // 擦除这一页

    while (1) {
        gps_data_get();
        if (exti_state[0]) {
            exti_state[0] = 0;
            buffer0[cnt] = gps_tau1201.latitude;   //经度 数据
            buffer1[cnt] = gps_tau1201.longitude;  //纬度 数据
            success_get_gps();
            cnt++;
        }
        if (exti_state[1]) {
            exti_state[1] = 0;
            flash_write_page(FLASH_SECTION_INDEX0, FLASH_PAGE_INDEX, buffer0,
                             cnt);  // 向指定 Flash 扇区的页码写入缓冲区数据
            flash_write_page(FLASH_SECTION_INDEX1, FLASH_PAGE_INDEX, buffer1,
                             cnt);  // 向指定 Flash 扇区的页码写入缓冲区数据
            break;
        }
        system_delay_ms(10);
    }
    while (1) {
        if (exti_state[2]) {
            exti_state[2] = 0;
            //读flash数据，纯GPS跑
            while (1) {
                for (uint8 i = 0; i < cnt; i++) {
                    flash_read_page(FLASH_SECTION_INDEX0, FLASH_PAGE_INDEX, buffer0,
                                    256);  // 将数据从 flash 读取
                    flash_read_page(FLASH_SECTION_INDEX1, FLASH_PAGE_INDEX, buffer1,
                                    256);  // 将数据从 flash 读取
                    success_read_gps(i);
                    float lat2 = buffer0[i];
                    float long2 = buffer1[i];
                    go_to(lat2, long2);
                }
            }
        }
        system_delay_ms(100);
    }
}

void key1_exti_handler(void) {
    exti_state[0] = 1;  // 外部中断触发 标志位置位
}

void key2_exti_handler(void) {
    exti_state[1] = 1;  // 外部中断触发 标志位置位
}

void key3_exti_handler(void) {
    exti_state[2] = 1;  // 外部中断触发 标志位置位
}

void gps_data_get() {
    if (gps_tau1201_flag) {
        gps_tau1201_flag = 0;
        gps_data_parse();  //开始解析数据

        ips200_show_string(0, 16 * 2, "latitude");   // 显示字符串
        ips200_show_string(0, 16 * 4, "longitude");  // 显示字符串

        ips200_show_float(120, 16 * 2, gps_tau1201.latitude, 4, 6);
        ips200_show_float(120, 16 * 4, gps_tau1201.longitude, 4, 6);
    }
}

void success_get_gps() {
    ips200_show_string(0, 16 * 6, "success get");    // 显示字符串
    ips200_show_string(0, 16 * 7, "get_latitude");   // 显示字符串
    ips200_show_string(0, 16 * 8, "get_longitude");  // 显示字符串

    ips200_show_float(120, 16 * 7, buffer0[cnt], 4, 6);
    ips200_show_float(120, 16 * 8, buffer1[cnt], 4, 6);
}

void success_read_gps(uint8 i) {
    ips200_show_string(0, 16 * 9, "success read");    // 显示字符串
    ips200_show_string(0, 16 * 10, "get_latitude");   // 显示字符串
    ips200_show_string(0, 16 * 11, "get_longitude");  // 显示字符串

    ips200_show_float(120, 16 * 10, buffer0[cnt], 4, 6);
    ips200_show_float(120, 16 * 11, buffer1[cnt], 4, 6);
}

void go_to(float lat2, float long2) {
    while (1) {
        ips200_show_string(0, 16 * 13, "Here we go");  // 显示字符串
        pwm_set_duty(PWM_CH1, PWM_DUTY_MAX / 4);
        gps_data_get();
        float lat1 = gps_tau1201.latitude;
        float long1 = gps_tau1201.longitude;
        float distance = get_two_points_distance(lat1, long1, lat2, long2);
        if (distance < 1) break;
        float dir1 = gps_tau1201.direction;
        float dir2 = get_two_points_azimuth(lat1, long1, lat2, long2);
        servo_motor_duty = dir2 - dir1;
        pwm_set_duty(SERVO_MOTOR_PWM, SERVO_MOTOR_DUTY(servo_motor_duty));  //设置舵机角度为A与B的差值
    }
}

// **************************** 代码区域 ****************************
