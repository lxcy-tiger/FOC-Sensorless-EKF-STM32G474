//
// Created by lxcy on 2025/10/9.
//

#ifndef FOC_SENSORLESS_PI_CONTROLLER_H
#define FOC_SENSORLESS_PI_CONTROLLER_H
#include <stdbool.h>
#include <stdint.h>
struct PI_Controller_t {
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
        /*CalculateError为0时，用户给定error，直接用用户给定的error计算，我们不会计算并覆盖掉它*/\
        if(CalculateError)NAME->error = NAME->Set - NAME->Measure; \
        float output_unsat = (float)P_VAL * NAME->error + (float)I_Ts_VAL * NAME->AddUp; \
        float output; \
        int8_t saturated = 0; \
        if (output_unsat > MAX_VAL) { \
            output = MAX_VAL; \
            saturated = 1; \
        } else if (output_unsat < MIN_VAL) { \
            output = MIN_VAL; \
            saturated = -1; \
        } else { \
            output = output_unsat; \
        } \
        /* 积分不饱和时允许积分，或者误差符号有助于减少积分饱和时也允许积分 */\
        /*下一行代码等价于 if (saturated==0 || (saturated==1 && NAME->error<0) || (saturated==-1&&NAME->error>0)) {*/ \
        if(saturated * NAME->error <= 0.0f){\
            NAME->AddUp += NAME->error; \
        } \
        NAME->Output = output; \
    } \
    extern struct PI_Controller_t NAME##_PIstate;
#define Current_P_VAL 12.566f
#define Current_I_Ts_VAL 1.727825f
GenerateFunction_PIController(Id,Current_P_VAL,Current_I_Ts_VAL,6.5f,-6.5f,1)
GenerateFunction_PIController(Iq,Current_P_VAL,Current_I_Ts_VAL,6.5f,-6.5f,1)
#define Speed_P_VAL 0.008712368f
#define Speed_I_Ts_VAL 2.138e-6f
GenerateFunction_PIController(Speed,Speed_P_VAL, Speed_I_Ts_VAL,1.0f,-1.0f,1)


GenerateFunction_PIController(FluxObserver_PLLSpeed,64404.56f,200.14f,3000,-3000,0)

GenerateFunction_PIController(SMO_PLLSpeed,1398.38,48.89,3000,-3000,0)

GenerateFunction_PIController(ST_SMO_PLLSpeed,1398.38,48.89,3000,-3000,0)



#endif //FOC_SENSORLESS_PI_CONTROLLER_H