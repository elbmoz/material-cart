#include "zf_common_headfile.h"
#include "balance_control.h"
cascade_value_struct balance_cascade;
cascade_value_struct balance_cascade_resave;

cascade_value_struct turn_cascade;
cascade_value_struct turn_cascade_resave;

cascade_value_struct steer_balance_cascade;
cascade_value_struct steer_balance_cascade_resave;

//--------
//函数简介          一阶互补滤波
//参数说明      filter_value    滤波参数结构体
//参数说明      gyro_raw_data   陀螺仪原始数据
//参数说明      acc_raw_data    加速度原始数据
//返回参数      void
//使用示例          first_order_complementary_filtering(&balance_cascade.cascade_value,*balance_cascade.cascade_value.gyro_raw_data,*balance_cascade.cascade_value.acc_raw_data);
//备注信息
//------
void first_order_complementary_filtering(cascade_common_value_struct *filter_value,int16 gyro_raw_data,int16 acc_raw_data)
{
    float gyro_temp;            //角速度计算临时变量f
    float acc_temp;          //加速度计算临时变量

    gyro_temp =gyro_raw_data *filter_value->gyro_ration;    //角速度数据*角速度置信度(一般给4)

    acc_temp =(acc_raw_data -filter_value->angle_temp)*filter_value->acc_ration;//加速度微分数据*加速度置信度(一般给4)

    filter_value->angle_temp +=((gyro_temp +acc_temp)*filter_value->call_cycle);  //两数之和*调用周期并积分到角度输出

    filter_value->filtering_angle=filter_value->angle_temp +filter_value->mechanical_zero;  //最终滤波角度成去零点位置即可
}
//-----
//函数简介      PID闭环计算(位置式)
//参数说明      pid_cycle               PID参数结构体
//参数说明      target                  目标值
//参数说明      real                    当前值
//返回参数      void
//使用示例      pid_control(&balance cascade speed cycle,θ,(left_motor.encoder_data+right_motor.encoder_data)/2);
//备注信息
//
void pid_control (pid_cycle_struct *pid_cycle,float target,float real)
{
    float   proportion_value=0;     //比例量

    float   differential_value=0;   //微分量

    proportion_value =target -real; //比例量=目标值-实际值

    pid_cycle->i_value +=(proportion_value *pid_cycle->i_value_pro);    //积分量=积分量+比例量*积分程度

    pid_cycle->i_value =func_limit_ab(pid_cycle->i_value,-pid_cycle->i_value_max,pid_cycle->i_value_max);//积分量限幅

    differential_value =proportion_value -pid_cycle->p_value_last;      //微分量=比例量-上一次比例量

    pid_cycle->out =(int16)(pid_cycle->p*proportion_value +pid_cycle->i*pid_cycle->i_value +pid_cycle->d*differential_value);//PIOM合

    pid_cycle->out =func_limit_ab(pid_cycle->out,-pid_cycle->out_max,pid_cycle->out_max);   //PID输出限幅

    pid_cycle->p_value_last =proportion_value;  //保存比例量
}
//
//函数简介      PID闭环计算(增量式)
//参数说明      pid_cycle           PID参数结构体
//参数说明      target              目标值
//参数说明      real                当前值
//返回参数      void
//使用示例      pid_control_incremental（&balancecascade.speed cycle,0,（left_motor.encoder_data+right_motor.encoder_data)/ 2);
//备注信息
void pid_control_incremental(pid_cycle_struct *pid_cycle,float target,float real)
{
    float   proportion_value=0;//比例璧

    float   differential_value=0;//微分量

    pid_cycle->i_value=target-real;//积分里=目标值-实际值增里式PIDP--->I

    differential_value=proportion_value-2*pid_cycle->incremental_data[0]+pid_cycle->incremental_data[1];//微分量增量式PIDI--->D

    proportion_value=proportion_value-pid_cycle->incremental_data[0];//比例量增量式PIDD--->P

    pid_cycle->incremental_data[1]= pid_cycle->incremental_data[0];//增量式PID保存参数

    pid_cycle->incremental_data[0]=proportion_value;

    pid_cycle->out +=(int16)(pid_cycle->p *proportion_value +pid_cycle->i*pid_cycle->i_value +pid_cycle->d * differential_value);// PID拟合

    pid_cycle->out=func_limit_ab(pid_cycle->out,-pid_cycle->out_max,pid_cycle->out_max);//PID输出限幅

}

//函数简介      串级平衡算法初始化
//返回参数      void
//使用示例      balance_cascade_init();
//备注信息      设置串级平衡参数
//-----
void balance_cascade_init(void)
{
    balance_cascade.cascade_value.gyro_raw_data     =           &imu660ra_gyro_y;
    balance_cascade.cascade_value.acc_raw_data      =           &imu660ra_acc_x;
    balance_cascade.cascade_value.gyro_ration       =           4.0f;
    balance_cascade.cascade_value.acc_ration        =           4.0f;
    balance_cascade.cascade_value.call_cycle        =           0.005f;
    balance_cascade.cascade_value.mechanical_zero   =           -200;//-670

    turn_cascade.cascade_value.gyro_raw_data        =           &imu660ra_gyro_z;
    turn_cascade.cascade_value.acc_raw_data         =           &imu660ra_acc_z;
    turn_cascade.cascade_value.gyro_ration          =           4.0f;
    turn_cascade.cascade_value.acc_ration           =           4.0f;
    turn_cascade.cascade_value.call_cycle           =           0.005f;
    turn_cascade.cascade_value.mechanical_zero      =           0;

    turn_cascade.cascade_value.filtering_angle      =   -turn_cascade.cascade_value.mechanical_zero;
    turn_cascade.cascade_value.angle_temp           =   -turn_cascade.cascade_value.mechanical_zero;

    balance_cascade.cascade_value.filtering_angle   =   -balance_cascade.cascade_value.mechanical_zero;
    balance_cascade.cascade_value.angle_temp        =   -balance_cascade.cascade_value.mechanical_zero;

    balance_cascade.angular_speed_cycle.i_value_max =           1000;
    balance_cascade.angular_speed_cycle.i_value_pro =           0.0f;
    balance_cascade.angular_speed_cycle.out_max     =           10000;
    balance_cascade.angle_cycle.i_value_max         =           4000;
    balance_cascade.angle_cycle.i_value_pro         =           0.05f;
    balance_cascade.angle_cycle.out_max             =           8000;
    balance_cascade.speed_cycle.i_value_max         =           4000;
    balance_cascade.speed_cycle.i_value_pro         =           0.05f;
    balance_cascade.speed_cycle.out_max             =           1500;

    balance_cascade.angular_speed_cycle.p           =           1.4f;//1.4
    balance_cascade.angular_speed_cycle.i           =           0.0f;
    balance_cascade.angular_speed_cycle.d           =           0.1f;//0.1

    balance_cascade.angle_cycle.p                   =           8.0f;//8.0
    balance_cascade.angle_cycle.i                   =           0.0f;
    balance_cascade.angle_cycle.d                   =           0.5f;

    balance_cascade.speed_cycle.p                   =           4.7f;//5.5
    balance_cascade.speed_cycle.i                   =           0.0f;
    balance_cascade.speed_cycle.d                   =           1.0f;//1.0

    turn_cascade.turn_cycle.p                       =           0.0f;
    turn_cascade.turn_cycle.i                       =           0.0f;
    turn_cascade.turn_cycle.d                       =           0.0f;

    steer_balance_cascade.cascade_value.gyro_raw_data           =&imu660ra_gyro_x;
    steer_balance_cascade.cascade_value.acc_raw_data            =&imu660ra_acc_y;
    steer_balance_cascade.cascade_value.gyro_ration             =4.0f;
    steer_balance_cascade.cascade_value.acc_ration              =4.0f;
    steer_balance_cascade.cascade_value.call_cycle              =0.005f;
    steer_balance_cascade.cascade_value.mechanical_zero         =0;
    steer_balance_cascade.cascade_value.filtering_angle         =-steer_balance_cascade.cascade_value.mechanical_zero;
    steer_balance_cascade.cascade_value.angle_temp              =-steer_balance_cascade.cascade_value.mechanical_zero;

    steer_balance_cascade.angular_speed_cycle.i_value_max       =1000;
    steer_balance_cascade.angular_speed_cycle.i_value_pro       =0.1f;
    steer_balance_cascade.angular_speed_cycle.out_max           =10000;
    steer_balance_cascade.angle_cycle.i_value_max               =4000;
    steer_balance_cascade.angle_cycle.i_value_pro               =0.05f;
    steer_balance_cascade.angle_cycle.out_max                   =300;
    steer_balance_cascade.speed_cycle.i_value_max               =4000;
    steer_balance_cascade.speed_cycle.i_value_pro               =0.05f;
    steer_balance_cascade.speed_cycle.out_max                   =1500;

    steer_balance_cascade.angular_speed_cycle.p                 =0.0f;//角速度环
    steer_balance_cascade.angular_speed_cycle.i                 =0.0f;
    steer_balance_cascade.angular_speed_cycle.d                 =0.0f;

    steer_balance_cascade.angle_cycle.p                         =0.05f;//角度环
    steer_balance_cascade.angle_cycle.i                         =0.000f;//单边桥
    steer_balance_cascade.angle_cycle.d                         =0.00f;

    steer_balance_cascade.speed_cycle.p                         =0.0f;//速度环
    steer_balance_cascade.speed_cycle.i                         =0.0f;
    steer_balance_cascade.speed_cycle.d                         =0.0f;


    memcpy(&balance_cascade_resave,&balance_cascade,sizeof(balance_cascade_resave));
    memcpy(&steer_balance_cascade_resave,&steer_balance_cascade,sizeof(steer_balance_cascade_resave));
    memcpy(&turn_cascade_resave,&turn_cascade,sizeof(turn_cascade_resave));


}
