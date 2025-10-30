//
// Created by lxcy on 2025/10/9.
//

#ifndef FOC_SENSORLESS_PI_CONTROLLER_H
#define FOC_SENSORLESS_PI_CONTROLLER_H

typedef struct PI_Controller_t {
    float Set;//设定值
    float Measure;//实测值
    //error=Set-Measure
    float Output;//输出值
    float AddUp;//积分值
};

//使用宏定义生成代码:
//(NAME函数名称,P_VAL比例参数,I_Ts_VAL积分参数(I*Ts类型),MAX_VAL输出最大值,MIN_VAL输出最小值)
#define GenerateFunction_PIController(NAME,P_VAL,I_Ts_VAL,MAX_VAL,MIN_VAL) \
   static inline void NAME##_PI_update(struct PI_Controller_t* NAME){\
        float error=NAME->Set-NAME->Measure;\
        float output_unsat = P_VAL * error + I_Ts_VAL * NAME->AddUp;\
        float output;                       \
        if (output_unsat > MAX_VAL) {       \
            output = MAX_VAL;               \
        } else if (output_unsat < MIN_VAL) {\
            output = MIN_VAL;               \
        } else {                            \
        output = output_unsat;              \
        }                                   \
        if (output == output_unsat) {       \
            NAME->AddUp += error;           \
        }                                   \
        NAME->Output=output;                \
   }\
  extern struct PI_Controller_t NAME##_PIstate;

GenerateFunction_PIController(Id,10.f,0.01f,6.5f,-6.5f)
GenerateFunction_PIController(Iq,10.f,0.01f,6.5f,-6.5f)
GenerateFunction_PIController(Speed,0.0005f,0.0000003f,1.f,-1.f)
#endif //FOC_SENSORLESS_PI_CONTROLLER_H