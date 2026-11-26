using System;

namespace WarehouseGridSimulation.Core.Motion
{
    /// <summary>
    /// 通用一维连续闭环 PID 控制器。
    /// 支持输出限幅与积分抗饱和，用于机器人平滑循迹与高精度到站对准。
    /// </summary>
    public class PidController
    {
        public float Kp { get; set; }
        public float Ki { get; set; }
        public float Kd { get; set; }
        public float MinOutput { get; set; }
        public float MaxOutput { get; set; }

        private float _integral;
        private float _previousError;
        private bool _isFirstStep = true;

        public PidController(float kp, float ki, float kd, float minOutput = float.MinValue, float maxOutput = float.MaxValue)
        {
            // 初始化闭环控制器增益参数与输出幅值饱和阈值。
            // kp: 比例增益系数; ki: 积分增益系数; kd: 微分增益系数; minOutput: 控制器下限输出幅值; maxOutput: 控制器上限输出幅值。
            Kp = kp;
            Ki = ki;
            Kd = kd;
            MinOutput = minOutput;
            MaxOutput = maxOutput;
            Reset();
        }

        public void Reset()
        {
            // 重置控制器内部累积积分与历史微分误差状态。
            _integral = 0f;
            _previousError = 0f;
            _isFirstStep = true;
        }

        public float Compute(float error, float dt)
        {
            // 依据当前输入误差微元与时间步长计算受限控制输出。
            // error: 当前状态与设定目标之间的误差量; dt: 本次控制循环的时间微元步长。
            if (dt <= 0f) return 0f;

            _integral += error * dt;

            float derivative = 0f;
            if (!_isFirstStep)
            {
                derivative = (error - _previousError) / dt;
            }
            else
            {
                _isFirstStep = false;
            }

            _previousError = error;

            float output = Kp * error + Ki * _integral + Kd * derivative;
            return Math.Clamp(output, MinOutput, MaxOutput);
        }
    }
}
