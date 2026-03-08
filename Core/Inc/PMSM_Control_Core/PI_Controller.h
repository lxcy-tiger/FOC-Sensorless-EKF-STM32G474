//
// Created by lxcy on 2025/10/9.
//

#ifndef FOC_SENSORLESS_PI_CONTROLLER_H
#define FOC_SENSORLESS_PI_CONTROLLER_H
#include <stdbool.h>

typedef struct PI_Controller_t {
    float Set;//设定值
    float Measure;//实测值
    float error;//误差值
    //error=Set-Measure
    float Output;//输出值
    float AddUp;//积分值
};

//使用宏定义生成代码:
//(NAME函数名称,P_VAL比例参数,I_Ts_VAL积分参数(I*Ts类型),
//   MAX_VAL输出最大值,MIN_VAL输出最小值,CalculateError,为1时计算error=Set-Measure,为0时error由用户给定)
#define GenerateFunction_PIController(NAME, P_VAL, I_Ts_VAL, MAX_VAL, MIN_VAL, CalculateError) \
    static inline void NAME##_PI_update(struct PI_Controller_t* NAME){ \
        if(CalculateError)NAME->error = NAME->Set - NAME->Measure; \
        float output_unsat = (float)P_VAL * NAME->error + (float)I_Ts_VAL * NAME->AddUp; \
        float output; \
        bool saturated = false; \
        if (output_unsat > MAX_VAL) { \
            output = MAX_VAL; \
            saturated = true; \
        } else if (output_unsat < MIN_VAL) { \
            output = MIN_VAL; \
            saturated = true; \
        } else { \
            output = output_unsat; \
        } \
        if (!saturated) { \
            NAME->AddUp += NAME->error; \
        } \
        NAME->Output = output; \
    } \
    extern struct PI_Controller_t NAME##_PIstate;

GenerateFunction_PIController(Id,12.566f,1.727825f,6.5f,-6.5f,1)
GenerateFunction_PIController(Iq,12.566f,1.727825f,6.5f,-6.5f,1)
#define Speed_I_Ts_VAL 2.138e-6f
GenerateFunction_PIController(Speed,0.008712368f, Speed_I_Ts_VAL,1.0f,-1.0f,1)


GenerateFunction_PIController(FluxObserver_PLLSpeed,80435.9f,312.45f,3000,-3000,0)

GenerateFunction_PIController(SMO_PLLSpeed,1398.38,48.89,3000,-3000,0)

GenerateFunction_PIController(ST_SMO_PLLSpeed,1398.38,48.89,3000,-3000,0)



#endif //FOC_SENSORLESS_PI_CONTROLLER_H