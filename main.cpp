#include "mbed.h"
#include "m3pi.h"
#include "APDS9960_I2C.h"
#include <stdlib.h>
#include <math.h>
#include "SDFileSystem.h"

m3pi m3pi;
APDS9960_I2C apds(p28,p27);
SDFileSystem sd(p11, p12, p13, p20, "sd"); // the pinout on the mbed Cool Components workshop board

// KalmanFilter 类定义
double C = 5206.8473;
double B = 0.22514;
double bias = 0.002184;
class KalmanFilter {
public:
    KalmanFilter(double initial_estimate, double initial_est_error, double process_noise, double measurement_noise) {
        x = initial_estimate;
        P = initial_est_error;
        Q = process_noise;
        R = measurement_noise;
    }

    void predict() {
        P = P + Q;
        x = x + B*0.5 - bias;
    }

    double update(double L) {
        double k = P*C / (C*P*C + R);
        x = x + k * (L - C*x);
        P = (1 - k*C) * P;
        return x;
    }

private:
    double x, P, Q, R;
};

KalmanFilter kf(0.0, 0.0, 0.000001464, 388867.96);  // 初始估计、估计误差、过程噪声和测量噪声


Ticker sensorTicker;
Ticker speedTicker;

int count = 1;  // 全局变量，用于记录采集次数

/*float speed = 0.2; // 初始速度
bool forward = true;  // 初始方向为前进
*/

//匀速的
float speed = 0.5;  // 设置前进速度为0.3米/秒
float max_speed = 1.0;  // 假设机器人最大速度为1米/秒

/*
// 匀加速运动参数
float acceleration = 0.02;  // 加速度
float initial_velocity = 0.2;  // 初速度 0.2 米/秒
float velocity = 0.0;  // 当前速度
float max_time = 10.0;  // 总时间 10 秒
float max_speed = 1.0;  // 最大速度为 1 米/秒
float elapsed_time = 0.0;  // 记录经过的时间
float time_step = 1.0;  // 每 1 秒更新一次速度
*/

/*
// sin运动参数
float A = 0.942;           // 计算出的速度振幅
float T = 5.0;             // 每段时间为5秒
float time_step = 0.1;     // 每0.1秒更新一次速度
float velocity = 0.0;      // 当前速度
float pi = 3.1415927;
float elapsed_time = 0.0;  // 记录经过的时间
bool forward = true;       // 当前运动方向
*/

// 用于存储最新的传感器数据和位置估计
volatile bool newDataAvailable = false;
float L_latest = 0.0;
float pos_latest = 0.0;

void updateSensorReadings() {
    uint16_t cl = 0;  // 读取光线传感器的值
    float totalL = 0; // 累加所有光强的总和
    const int numReadings = 10;  // 读取的次数

    // 循环读取20次光强信息
    for (int i = 0; i < numReadings; i++) {
        apds.readAmbientLight(cl);  // 读取传感器
        totalL += cl * 40;          // 将每次读取的光强值累加
    }

    // 计算平均值
    double L = totalL / numReadings;
    /*uint16_t cl = 0;  // 读取光线传感器
    apds.readAmbientLight(cl);
    float L = cl*40;*/
    kf.predict();  // 执行预测步骤
    double pos = kf.update(L);  // 更新位置估计
    /*uint16_t cl = 0;  // 读取光线传感器
    apds.readAmbientLight(cl);
    float L = cl*20;
    kf.predict();  // 执行预测步骤
    float pos = kf.update(L);  // 更新位置估计*/

    // 显示数据在机器人的LCD屏幕上
    m3pi.cls();  // 清除屏幕内容
    /*m3pi.locate(0, 0);  // 定位到第一行
    m3pi.printf("l: %.2f", L);  // 显示传感器值
    m3pi.locate(0, 1);  // 定位到第二行
    m3pi.printf("Po: %.2f", pos);  // 显示更新的位置*/

    // 打开文件以追加数据
    FILE *fp = fopen("/sd/data.txt", "a");
    if (fp != NULL) {
        // 写入时间戳、传感器数据和位置信息到SD卡
        fprintf(fp, "L: %.5f, Po: %.5f, Count: %d\n", L, pos, count);
        fclose(fp);
        count++;
    } else {
        error("Could not open file for write\n");
    }
}

/*void updateSpeed() {
    // 根据预定逻辑调整速度和方向
    if (speed >= 0.6) {
        speed = 0.1;  // 重置速度
        forward = !forward;  // 改变方向
    } else {
        speed += 0.02;  // 增加速度
    }

    // 根据方向和速度控制机器人
    if (forward) {
        m3pi.forward(speed);
    } else {
        m3pi.backward(speed);
    }
}*/

// 匀速的机器人的速度和方向
void updateSpeed() {
    // 将速度归一化到最大速度的比例
    float normalized_speed = speed / max_speed;

    // 控制电机速度，设定左右电机相同的速度以保持直线运动
    m3pi.forward(normalized_speed);
}


 /*//匀加速
void updateSpeed() {
    // 计算当前速度：v(t) = v_0 + a * t
    velocity = initial_velocity + acceleration * elapsed_time;

    // 将速度归一化到最大速度的范围
    float normalized_speed = velocity / max_speed;

    // 确保速度不超过最大速度
    if (normalized_speed > 1.0) {
        normalized_speed = 1.0;
    }

    // 设置机器人前进的速度
    m3pi.forward(normalized_speed);

    // 打印当前速度（调试用）
    printf("Time: %.2f, Speed: %.2f\n", elapsed_time, normalized_speed);

    // 增加经过的时间
    elapsed_time += time_step;

    // 如果超过最大时间，停止机器人并停止定时器
    if (elapsed_time >= max_time) {
        speedTicker.detach();  // 停止定时器
        m3pi.stop();  // 停止机器人
    }
}
*/

/*
//sin
void updateSpeed() {
    // 计算当前时间内的速度：v(t) = A * sin(pi * t / T)
    velocity = A * sin(pi * elapsed_time / T);

    // 控制机器人的运动状态
    if (forward) {
        m3pi.forward(fabs(velocity));
    } else {
        m3pi.backward(fabs(velocity));
    }

    // 增加经过的时间
    elapsed_time += time_step;

    // 当时间超过T秒时，改变方向
    if (elapsed_time >= T) {
        forward = !forward;  // 改变方向
        elapsed_time = 0.0;  // 重置时间
    }

    // 如果经过的总时间超过10秒，停止机器人
    if (elapsed_time >= 2 * T) {
        speedTicker.detach();  // 停止速度更新
        m3pi.stop();  // 停止机器人
    }
}
*/

// 定义全局计时器
Timer runtimeTimer;

int main() {
     // 初始化传感器
    apds.init();
    apds.enablePower();
    apds.enableLightSensor();
    apds.setAmbientLightGain(3);

    m3pi.printf("Starting robot control\n");

    sensorTicker.attach(&updateSensorReadings, 0.2);  // 每0.1秒更新一次传感器读数和位置
    speedTicker.attach(&updateSpeed, 0.5);  // 每0.5秒调整一次速度和方向

    while (true) {
        wait(0.05);  // 主循环中的延迟时间
    }
}
