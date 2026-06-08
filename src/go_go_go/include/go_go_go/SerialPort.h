// #ifndef _SERIAL_PORT_H
// #define _SERIAL_PORT_H
// #pragma once

// #include <iostream>
// #include <deque>
// #include <chrono>
// #include <mutex>
// using namespace std;
// extern std::mutex ds_mutex;

// #define SERIAL_PATH "/dev/ttyACM0"
// #define SERIAL_VMIN 16
// #define SERIAL_VTIME 0
// #define READ_SIZE 24

// struct Data_Save {
//     double camera_yaw_angle;
//     double camera_pit_angle;  
//     double camera_roll_angle;
//     int Buff_or_Armor;       // 自瞄或大符
//     bool buff_mode;          // 大符或小符
//     int color;               // 敌方颜色
// };
// extern Data_Save ds;

// struct Data_Get
// {
//     int size;
//     u_char raw_data[21];
//     void Vision2Control(int32_t x, int32_t y);
// };
// extern Data_Get dg;

// class SerialPort
// {
// public:
//     SerialPort(const char *filename);

//     void restart_serial(void); // 尝试重连的函数
//     int read_data();
//     void send_data(const struct Data_Get &data);
    
//     int fd;
//     int last_fd;
//     bool success_;

// private:
//     const char *file_name_;
//     int buadrate_;
//     float last_bullet_speed;
    
// };
// /*--------------- serial data transmission ----------------*/
// static SerialPort serial_(SERIAL_PATH);
// /*--------------- serial data transmission ----------------*/
// #endif

#ifndef _SERIAL_PORT_H
#define _SERIAL_PORT_H
#pragma once

#include <iostream>
#include <deque>
#include <chrono>
#include <mutex>
using namespace std;
extern std::mutex ds_mutex;

#define SERIAL_PATH "/dev/ttyACM0"
#define SERIAL_VMIN 16
#define SERIAL_VTIME 0
#define READ_SIZE 24

struct Data_Save {
    double camera_yaw_angle;
    double camera_pit_angle;  
    double camera_roll_angle;
    int Buff_or_Armor;       // 自瞄或大符
    bool buff_mode;          // 大符或小符
    int color;               // 敌方颜色
};
extern Data_Save ds;

struct Data_Get
{
    int size;
    u_char raw_data[21];
    void Vision2Control(int16_t x, int16_t y);
};
extern Data_Get dg;

class SerialPort
{
public:
    SerialPort(const char *filename);

    void restart_serial(void); // 尝试重连的函数
    int read_data();
    void send_data(const struct Data_Get &data);
    
    int fd;
    int last_fd;
    bool success_;

private:
    const char *file_name_;
    int buadrate_;
    float last_bullet_speed;
    
};
/*--------------- serial data transmission ----------------*/
static SerialPort serial_(SERIAL_PATH);
/*--------------- serial data transmission ----------------*/
#endif