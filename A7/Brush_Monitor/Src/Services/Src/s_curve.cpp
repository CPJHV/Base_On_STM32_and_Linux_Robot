#include "s_curve.h"
#include <math.h>
#include "motor.h"
#include "pid.h"

float uart_buf[10] = {0};
#define CURRENT_YAW  (uart_buf[2])

// 车体状态（兼容写法）
Car_State car_state;

S_Curve motor1_smooth;
S_Curve motor2_smooth;
S_Curve motor3_smooth;
S_Curve motor4_smooth;

#define CAR_WHEEL_WIDTH    0.18f
#define WHEEL_RADIUS       0.035f
#define SPEED_SCALE_COEFF  1.0f
//对于多处地方需要修改的，一定要加入临界区或者信号量
//停止后要将数据及时清空
//=============================================
// S曲线初始化
//=============================================
void S_Curve_Init(S_Curve *sc, float dt, float max_jerk)
{
    sc->target = 0;
    sc->now    = 0;
    sc->dt     = dt;
    sc->max_jerk = max_jerk;
}

//=============================================
// 一阶斜坡限速，建议换成S型曲线
//=============================================
float S_Curve_Calc(S_Curve *sc, float target)
{
    sc->target = target;
    float err = sc->target - sc->now;
    float step = sc->max_jerk * sc->dt;

    if(fabsf(err) < 0.5f)
    {
        sc->now = sc->target;
        return sc->now;
    }

    if(err > 0)
    {
        sc->now += step;
        if(sc->now > sc->target) sc->now = sc->target;
    }
    else
    {
        sc->now -= step;
        if(sc->now < sc->target) sc->now = sc->target;
    }
    return sc->now;
}

//=============================================
// 线速度 → 电机速度
//=============================================
float LineSpeed_To_MotorSpeed(float line_spd)
{
    float wheel_angular = line_spd / WHEEL_RADIUS;
    return wheel_angular * SPEED_SCALE_COEFF;
}

//=============================================
// 差速运动学
//=============================================
void Car_Kinematics_Calc(float v, float omega)
{
	//公式速度左=v-w*W/2.W是左右轮距
	//w是车体绕中心旋转角度
	//v是直线前进的速度
    float v_left  = v - (omega * CAR_WHEEL_WIDTH / 2.0f);
    float v_right = v + (omega * CAR_WHEEL_WIDTH / 2.0f);

    float spd_l = LineSpeed_To_MotorSpeed(v_left);
    float spd_r = LineSpeed_To_MotorSpeed(v_right);

    float smooth_lf = S_Curve_Calc(&motor1_smooth, spd_l);
    float smooth_rf = S_Curve_Calc(&motor2_smooth, spd_r);
    float smooth_lb = S_Curve_Calc(&motor3_smooth, spd_l);
    float smooth_rb = S_Curve_Calc(&motor4_smooth, spd_r);

    monitor_contr.g_motor1_data.speed = smooth_lf;
    monitor_contr.g_motor2_data.speed = smooth_rf;
    monitor_contr.g_motor3_data.speed = smooth_lb;
    monitor_contr.g_motor4_data.speed = smooth_rb;
}

//=============================================
// 角度PID初始化
//=============================================
void Angle_PID_Param_Init(void)
{
    pidx.angle_pid.SetPoint     = 0.0f;
    pidx.angle_pid.ActualValue  = 0.0f;
    pidx.angle_pid.SumError     = 0.0f;
    pidx.angle_pid.Error        = 0.0f;
    pidx.angle_pid.LastError    = 0.0f;
    pidx.angle_pid.PrevError    = 0.0f;

    pidx.angle_pid.Proportion  = 2.0f;
    pidx.angle_pid.Integral    = 0.05f;
    pidx.angle_pid.Derivative  = 0.1f;

    // 初始化车体状态
    car_state.mode = CAR_STOP;
    car_state.target_speed = 0;
    car_state.target_yaw = 0;
    car_state.current_yaw = 0;
}

//=============================================
// 计算转向角速度
//=============================================
float Get_Car_Turn_Omega(float target_yaw, float now_yaw)
{
    float err = target_yaw - now_yaw;

    if(err > 180.0f)  err -= 360.0f;
    if(err < -180.0f) err += 360.0f;

    float out = pidx.increment_pid_ctrl(&pidx.angle_pid, err);

    if(out > 30.0f)  out = 30.0f;
    if(out < -30.0f) out = -30.0f;

    return out * 0.0174533f;
}

//=============================================
// 目标角度转向
//=============================================
void Car_Move_To_Target_Angle(float base_speed, float target_angle)
{
    float now_yaw = CURRENT_YAW;
    float omega = Get_Car_Turn_Omega(target_angle, now_yaw);

    float vl = base_speed - (omega * CAR_WHEEL_WIDTH / 2.0f);
    float vr = base_speed + (omega * CAR_WHEEL_WIDTH / 2.0f);

    float set_l = S_Curve_Calc(&motor1_smooth, LineSpeed_To_MotorSpeed(vl));
    float set_r = S_Curve_Calc(&motor2_smooth, LineSpeed_To_MotorSpeed(vr));

    pidx.g1_speed_pid.SetPoint = set_l;
    pidx.g3_speed_pid.SetPoint = set_l;
    pidx.g2_speed_pid.SetPoint = set_r;
    pidx.g4_speed_pid.SetPoint = set_r;
}

//=============================================
// 直线行走 + 航向保持
//=============================================
void Car_Straight_Hold_Head(float speed)
{
    car_state.mode = CAR_STRAIGHT_HOLD;
    car_state.target_speed = speed;
    car_state.target_yaw = CURRENT_YAW;
}

//=============================================
// 10ms 运动调度
//=============================================
void Car_Run_10ms(void)
{
    car_state.current_yaw = CURRENT_YAW;

    switch(car_state.mode)
    {
        case CAR_STOP:
            pidx.g1_speed_pid.SetPoint = 0;
            pidx.g2_speed_pid.SetPoint = 0;
            pidx.g3_speed_pid.SetPoint = 0;
            pidx.g4_speed_pid.SetPoint = 0;
            break;

        case CAR_STRAIGHT_HOLD:
        {
            float err = car_state.target_yaw - car_state.current_yaw;
            if(err > 180) err -= 360;
            if(err < -180) err += 360;

            float omega = pidx.increment_pid_ctrl(&pidx.angle_pid, err);

            if(omega > 25) omega = 25;
            if(omega < -25) omega = -25;

            float v = car_state.target_speed;
            float w = omega * 0.01745f;

            float vl = v - w * CAR_WHEEL_WIDTH / 2.0f;
            float vr = v + w * CAR_WHEEL_WIDTH / 2.0f;

            float sl = S_Curve_Calc(&motor1_smooth, LineSpeed_To_MotorSpeed(vl));
            float sr = S_Curve_Calc(&motor2_smooth, LineSpeed_To_MotorSpeed(vr));

            pidx.g1_speed_pid.SetPoint = sl;
            pidx.g3_speed_pid.SetPoint = sl;
            pidx.g2_speed_pid.SetPoint = sr;
            pidx.g4_speed_pid.SetPoint = sr;
            break;
        }

        case CAR_TURN_ANGLE:
            Car_Move_To_Target_Angle(0, car_state.target_yaw);
            break;
    }
}

//=============================================
// 设置模式
//=============================================
void Car_Set_Mode(Car_Mode mode)
{
    car_state.mode = mode;
}

//=============================================
// 停止
//=============================================
void Car_Stop(void)
{
    car_state.mode = CAR_STOP;
    pidx.angle_pid.SumError = 0;
}