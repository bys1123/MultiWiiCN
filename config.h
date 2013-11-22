/*************************************************************************************************/
/****           可配置参数                                                                    ****/
/*************************************************************************************************/

/* 本文件由几部分组成
 * 用于创建一个可用的组合，你必须至少在第1部分做出你的选择。
 * 1 - 基本设置 - 你必须在每个块中选择一个选项。
 *      这里假定你的板子连接了4个通道还有标准电调和舵机。
 * 2 - 飞行器分类详细选项 - 你可能需要为你的飞行器分类检查选项
 * 3 - 遥控系统设置
 * 4 - 替代CPU和板子 - 如果你有
 * 5 - 替代设置 - 在此选择替代发射机（SBUS，PPM等），替代电调范围等
 * 6 - 可选特性 - 在此启用锦上添花的特性（飞行模式，LCD，遥测，电池监控等）
 * 7 - 调试 & 开发者 - 如果你知道你在做什么；已经警告过你了
 */

/* 提示：
 * 1. 在注释中用(*)标记的参数被储存在eeprom中，并且可以通过串口监控器或LCD调节。
 *    在config.h中修改这些值并上传需要在GUI中“重置”才会生效
 */


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 1 部分 - 基本设置                                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************    多旋翼飞行器种类    ****************************/
    //#define GIMBAL
    //#define BI
    //#define TRI
    //#define QUADP
    //#define QUADX
    //#define Y4
    //#define Y6
    //#define HEX6
    //#define HEX6X
    //#define HEX6H  // New Model
    //#define OCTOX8
    //#define OCTOFLATP
    //#define OCTOFLATX
    //#define FLYING_WING
    //#define VTAIL4
    //#define AIRPLANE
    //#define SINGLECOPTER
    //#define DUALCOPTER
    //#define HELI_120_CCPM
    //#define HELI_90_DEG

  /****************************       电机最小油门      *******************************/
    /* 设定发送至电调（ESC，Electronic Speed Controller）的最小油门命令
       该最小值允许电机运行在怠速上  */
    //#define MINTHROTTLE 1300 // 用于Turnigy Plush ESCs 10A
    //#define MINTHROTTLE 1120 // 用于Super Simple ESCs 10A
    //#define MINTHROTTLE 1064 // 特殊的ESC (simonk)
    //#define MINTHROTTLE 1050 // 用于brushed ESC比如ladybird
    #define MINTHROTTLE 1150 // (*)

  /****************************        电机最大油门     *******************************/
    /* ESC在全功率是的最大值，该值最大可增至2000 */
    #define MAXTHROTTLE 1850

  /****************************       最小命令          *******************************/
    /* 该值用于未解锁时的ESC
       在某些情况下，用于一些特殊的电调该值必须降至900，否则电调会初始化失败 */
    #define MINCOMMAND  1000

  /**********************************    I2C速度   ************************************/
    //#define I2C_SPEED 100000L     //100kHz普通模式，正品WPM必须使用该值
    #define I2C_SPEED 400000L   //400kHz快速模式，仅用于一些山寨WPM

  /***************************      内部i2c上拉        ********************************/
    /* 启用内部I2C上拉（在多数情况下，使用外部上拉更佳） */
    #define INTERNAL_I2C_PULLUPS

  /**************************************************************************************/
  /*****************          板子与传感器定义                         ******************/
  /**************************************************************************************/

    /***************************    Combined IMU Boards    ********************************/
      /* 如果你在使用特定的传感器板：
         请提交改动到这个列表。
           来自Alex的提示：我只有其中一些板子，对于其他板子，我不能确保好用，信息由遥控论坛生成，请小心使用 */
      //#define FFIMUv1         // 来自Jussi的第一块9DOF+气压计板，使用HMC5843                   <- 由Alex确认
      //#define FFIMUv2         // 来自Jussi的第二版9DOF+气压计板，使用HMC5883                   <- 由Alex确认
      //#define FREEIMUv1       // 来自Fabio的v0.1 & v0.2 & v0.3版本的9DOF版
      //#define FREEIMUv03      // FreeIMU v0.3与v0.3.1
      //#define FREEIMUv035     // FreeIMU v0.3.5无气压计
      //#define FREEIMUv035_MS  // FreeIMU v0.3.5_MS                                                <- 由Alex确认
      //#define FREEIMUv035_BMP // FreeIMU v0.3.5_BMP
      //#define FREEIMUv04      // FreeIMU v0.4使用MPU6050，HMC5883L，MS561101BA                    <- 由Alex确认
      //#define FREEIMUv043     // same as FREEIMUv04 with final MPU6050（with the right ACC scale）
      //#define NANOWII         // 最小的multiwii飞控基于MPU6050 + pro micro based proc             <- 由Alex确认
      //#define PIPO            // 来自erazz的9DOF板
      //#define QUADRINO        // full FC board 9DOF+baro board from witespy  with BMP085 baro     <- 由Alex确认
      //#define QUADRINO_ZOOM   // full FC board 9DOF+baro board from witespy  second edition
      //#define QUADRINO_ZOOM_MS// full FC board 9DOF+baro board from witespy  second edition       <- 由Alex确认
      //#define ALLINONE        // full FC board or standalone 9DOF+baro board from CSG_EU
      //#define AEROQUADSHIELDv2
      //#define ATAVRSBIN1      // Atmel 9DOF (Contribution by EOSBandi). requires 3.3V power.
      //#define SIRIUS          // Sirius Navigator IMU                                             <- 由Alex确认
      //#define SIRIUSGPS       // Sirius Navigator IMU  using external MAG on GPS board            <- 由Alex确认
      //#define SIRIUS600       // Sirius Navigator IMU  using the WMP for the gyro
      //#define SIRIUS_AIR      // Sirius Navigator IMU 6050 32U4 from MultiWiiCopter.com
      //#define SIRIUS_AIR_GPS  // Sirius Navigator IMU 6050 32U4 from MultiWiiCopter.com with GPS/MAG remote located
      //#define MINIWII         // Jussi的MiniWii飞行控制器                                <- 由Alex确认
      //#define MICROWII        // MicroWii 10DOF使用ATmega32u4, MPU6050, HMC5883L, MS561101BA，来自http://flyduino.net/
      //#define CITRUSv2_1      // CITRUS，来自qcrc.ca
      //#define CHERRY6DOFv1_0
      //#define DROTEK_10DOF    // Drotek 10DOF使用ITG3200, BMA180, HMC5883, BMP085, w or w/o LLC
      //#define DROTEK_10DOF_MS // Drotek 10DOF使用ITG3200, BMA180, HMC5883, MS5611, LLC
      //#define DROTEK_6DOFv2   // Drotek 6DOF v2
      //#define DROTEK_6DOF_MPU // Drotek 6DOF使用MPU6050
      //#define DROTEK_10DOF_MPU//
      //#define MONGOOSE1_0     // mongoose 1.0    http://store.ckdevices.com/
      //#define CRIUS_LITE      // Crius MultiWii Lite
      //#define CRIUS_SE        // Crius MultiWii SE
      //#define OPENLRSv2MULTI  // OpenLRS v2 Multi Rc Receiver board including ITG3205 and ADXL345
      //#define BOARD_PROTO_1   // with MPU6050 + HMC5883L + MS baro
      //#define BOARD_PROTO_2   // with MPU6050 + slave  MAG3110 + MS baro
      //#define GY_80           // 中国的10 DOF with  L3G4200D ADXL345 HMC5883L BMP085, LLC
      //#define GY_85           // 中国的9 DOF with  ITG3205 ADXL345 HMC5883L LLC
      //#define GY_86           // 中国的10 DOF with  MPU6050 HMC5883L MS5611, LLC
      //#define GY_521          // 中国的6  DOF with  MPU6050, LLC
      //#define INNOVWORKS_10DOF // with ITG3200, BMA180, HMC5883, BMP085 available here http://www.diymulticopter.com
      //#define INNOVWORKS_6DOF // with ITG3200, BMA180 available here http://www.diymulticopter.com
      //#define MultiWiiMega    // MEGA + MPU6050+HMC5883L+MS5611 available here http://www.diymulticopter.com
      //#define PROTO_DIY       // 10DOF mega board
      //#define IOI_MINI_MULTIWII// www.bambucopter.com
      //#define Bobs_6DOF_V1     // BobsQuads 6DOF V1 with ITG3200 & BMA180
      //#define Bobs_9DOF_V1     // BobsQuads 9DOF V1 with ITG3200, BMA180 & HMC5883L
      //#define Bobs_10DOF_BMP_V1 // BobsQuads 10DOF V1 with ITG3200, BMA180, HMC5883L & BMP180 - BMP180 is software compatible with BMP085
      //#define FLYDUINO_MPU
      //#define CRIUS_AIO_PRO_V1
      //#define DESQUARED6DOFV2GO  // DEsquared V2 with ITG3200 only
      //#define DESQUARED6DOFV4    // DEsquared V4 with MPU6050
      //#define LADYBIRD
      //#define MEGAWAP_V2_STD     // 可在这里买到： http://www.multircshop.com                    <- 由Alex确认
      //#define MEGAWAP_V2_ADV
      //#define HK_MultiWii_SE_V2  // Hobbyking board with MPU6050 + HMC5883L + BMP085
      //#define HK_MultiWii_328P   // Also labeled "Hobbybro" on the back.  ITG3205 + BMA180 + BMP085 + NMC5583L + DSM2 Connector (Spektrum Satellite)  
      //#define RCNet_FC           // RCNet FC with MPU6050 and MS561101BA  http://www.rcnet.com
      //#define RCNet_FC_GPS       // RCNet FC with MPU6050 + MS561101BA + HMC5883L + UBLOX GPS http://www.rcnet.com
      //#define FLYDU_ULTRA        // MEGA+10DOF+MT3339 FC

      
    /***************************    独立的传感器    ********************************/
      /* 如果你已在上方选择了相应的板子，保持注释状态即可 */
      /* I2C陀螺仪 */
      //#define WMP
      //#define ITG3200
      //#define L3G4200D
      //#define MPU6050       //combo + ACC

      /* I2C加速度计 */
      //#define NUNCHUCK  // 如果你要将nunckuk连接到WMP使用
      //#define MMA7455
      //#define ADXL345
      //#define BMA020
      //#define BMA180
      //#define NUNCHACK  // 如果你要将nunckuk作为单独的I2C加速度计使用，不连接到WMP
      //#define LIS3LV02
      //#define LSM303DLx_ACC
      //#define MMA8451Q

      /* I2C气压计 */
      //#define BMP085
      //#define MS561101BA

      /* I2C磁力计 */
      //#define HMC5843
      //#define HMC5883
      //#define AK8975
      //#define MAG3110

      /* 声呐 */ // 目前用作显示用途 - 无控制代码支持
      //#define SRF02 // 使用Devantech SRF i2c传感器
      //#define SRF08
      //#define SRF10
      //#define SRF23

      /* ADC加速度计 */ // 用于来自sparkfun的5DOF，使用模拟针脚A1/A2/A3
      //#define ADCACC

      /* 强制你私有的传感器方向 - 甚至覆盖板子特定的默认值 */
      //#define FORCE_ACC_ORIENTATION(X, Y, Z)  {accADC[ROLL]  =  Y; accADC[PITCH]  = -X; accADC[YAW]  = Z;}
      //#define FORCE_GYRO_ORIENTATION(X, Y, Z) {gyroADC[ROLL] = -Y; gyroADC[PITCH] =  X; gyroADC[YAW] = Z;}
      //#define FORCE_MAG_ORIENTATION(X, Y, Z)  {magADC[ROLL]  =  X; magADC[PITCH]  =  Y; magADC[YAW]  = Z;}

      /* 板子方向转移 */
      /* 如果你的机架设计仅用于+模式，并且你不能物理上将飞控旋转至用于X模式飞行（反之亦然）
       * 你可以使用其中一个选项虚拟旋转传感器45度，然后通过飞行模式设定多旋翼飞行器的类型。
       * 检查电机顺序与旋转方向是否与新的“前方”匹配！仅接触其中一项注释！ */
      //#define SENSORS_TILT_45DEG_RIGHT        // 将“前方”顺时针旋转45度
      //#define SENSORS_TILT_45DEG_LEFT         // 将“前方”逆时针旋转45度


/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 2 部分 - 飞行器分类详细选项                                          *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /********************************    三轴    *********************************/
    #define YAW_DIRECTION 1
    //#define YAW_DIRECTION -1 // 如果你要翻转yaw修正方向
    /* 你可以在此修改三轴飞行器舵机行程 */
      #define TRI_YAW_CONSTRAINT_MIN 1020
      #define TRI_YAW_CONSTRAINT_MAX 2000
      #define TRI_YAW_MIDDLE 1500 // (*) 尾舵机中点。 - 使用此项进行初步修正；之后通过LCD修正中点

  /********************************    两轴    *********************************/
    /* 你可以在此修改两轴飞行器舵机行进方向 */     
    //#define BI_PITCH_DIRECTION 1
     #define BI_PITCH_DIRECTION -1

   /********************************    锁定/解锁    *********************************/
   /* 可以禁止使用摇杆组合进行锁定/解锁电机。
     * 在多数情况下，选择其中一种通过发射机摇杆锁定/解锁电机的选项即可 */
    #define ALLOW_ARM_DISARM_VIA_TX_YAW
    //#define ALLOW_ARM_DISARM_VIA_TX_ROLL

  /***********************          相机稳定             ***********************/
    /* 以下几行仅用于pitch/roll倾斜稳定系统。接触注释第一或第二行来激活它 */
    //#define SERVO_MIX_TILT
    //#define SERVO_TILT
    #define TILT_PITCH_MIN    1020    //最低舵机行程，不要低于1020
    #define TILT_PITCH_MAX    2000    //最高舵机行程，最大值=2000
    #define TILT_PITCH_MIDDLE 1500    //舵机中立值
    #define TILT_PITCH_PROP   10      //舵机比例（与角度相关联）；可以为负反转转动方向
    #define TILT_PITCH_AUX_CH AUX3    //用AUX通道重写相机的pitch（AUX1-AUX4），注释以手动输入并释放AUX通道
    #define TILT_ROLL_MIN     1020
    #define TILT_ROLL_MAX     2000
    #define TILT_ROLL_MIDDLE  1500
    #define TILT_ROLL_PROP    10
    #define TILT_ROLL_AUX_CH  AUX4    //用AUX通道重写相机的Roll（AUX1-AUX4），注释以手动输入并释放AUX通道

    /* 相机快门功能：通过GUI中的遥控选项激活，舵机输出=promini上的A2 */
    //#define CAMTRIG
    #define CAM_SERVO_HIGH 2000  // 舵机高位位置
    #define CAM_SERVO_LOW 1020   // 舵机低位位置
    #define CAM_TIME_HIGH 1000   // 舵机高位持续时间，用ms表示
    #define CAM_TIME_LOW 1000    // 舵机低位持续时间，用ms表示

  /***********************          飞翼                   ***********************/
    /* 你可以再次修改舵机方向与舵机最大/最小值
       对所有飞行模式都生效，即使为passThrough模式
       需要在此设置舵机方向；不必在交换接收机的舵机通道 */
    #define PITCH_DIRECTION_L 1 //  左舵机 - pitch方向
    #define PITCH_DIRECTION_R -1  // 右舵机 - pitch方向（如果舵机是镜像安装的，与PITCH_DIRECTION_L符号相反）
    #define ROLL_DIRECTION_L 1 // 左舵机 - roll方向
    #define ROLL_DIRECTION_R 1  // 右舵机 - roll方向（如果舵机是镜像安装的，与ROLL_DIRECTION_L符号相同）
    #define WING_LEFT_MID  1500 // (*) 左舵机中点 - 用此进行初步修正；之后通过LCD修正中点
    #define WING_RIGHT_MID 1500 // (*) 右舵机中点 - 用此进行初步修正；之后通过LCD修正中点
    #define WING_LEFT_MIN  1020 // 限制舵机行程，必须在[1020;2000]之内
    #define WING_LEFT_MAX  2000 // 限制舵机行程，必须在[1020;2000]之内
    #define WING_RIGHT_MIN 1020 // 限制舵机行程，必须在[1020;2000]之内
    #define WING_RIGHT_MAX 2000 // 限制舵机行程，必须在[1020;2000]之内

  /***********************          飞机                       ***********************/
    //#define USE_THROTTLESERVO // 用于在油门上使用标准50Hz舵机。
    #define SERVO_RATES      {100, 100, 100, 100, 100, 100, 100, 100} // Rates在0-100%之间
    #define SERVO_DIRECTION  { -1,   1,   1,   -1,  1,   1,   1,   1 } // 通过设为-1反转舵机

    //#define FLAPPERONS    AUX4          // 混合襟翼与副翼。
    #define FLAPPERON_EP   { 1500, 1700 } // 用于襟翼双向切换的端点，另外可设为{1020,2000}并在遥控上编程。
    //#define FLAPPERON_EP   { 1200, 1500 } // 或襟副翼 up for CrowMix 
    #define FLAPPERON_INVERT { 1, -1 }    // 改变襟副翼的方向{ Wing1, Wing2 }
    
    //#define FLAPS         AUX4          // A2上的传统襟翼，通过SERVO_DIRECTION舵机[2)反转。
    #define FLAP_EP      { 1500, 1900 }   // 用于襟翼双向切换的端点，另外可设为{1020,2000}并在遥控上编程。

    //#define FLAPSPEED     3             // 使襟翼移动变慢，值越高速度越快。

  /***********************          直升机与飞机公用           ***********************/
    //#define D12_POWER      // 使用PROMINI上的D12连接功率传感器。将会禁用D12上的舵机[4]
    #define SERVO_OFFSET     {  0,   0,   0,  0,   0,   0,  0,   0 } // (*) 调整舵机中点偏移与摆动角度
    // 可选通道:=    ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4

    /* 调节器：试图通过螺距和电压的改变维持转速
     * 预测方法：观察输入信号与电压并猜测适当的修正。
     * （油门曲线必须为调节器留有空间，所以0-50-75-80-80是可以的，不可以为0-50-95-100-100。
     * 可以通过aux开关切换
     */
    //#define GOVERNOR_P 7     // (*) 比例因子。更大的值 -> 更大的油门增量。必须>=1；0 = 关闭
    //#define GOVERNOR_D 4     // (*) 衰减时间。更大的值 -> 油门回到正常需要更长时间。 必须>=1；
    //#define GOVERNOR_R 10    // (*) 电压冲击修正大小，以0.1为单位。更大的值 -> 更大的电压降补偿。正常值为10 <=> 1.0；0为关闭

  /***********************          直升机                           ***********************/
    /* 控制总距的通道 */
    #define COLLECTIVE_PITCH      THROTTLE
    /* 设置舵机最大可移动范围。取决于模型 */
    #define SERVO_ENDPOINT_HIGH {2000,2000,2000,2000,2000,2000,2000,2000};
    #define SERVO_ENDPOINT_LOW  {1020,1020,1020,1020,1020,1020,1020,1020};

    /* 限制总距的范围。100%为每个方向的最大范围，还有零螺距的位置 */
    #define COLLECTIVE_RANGE { 80, 0, 80 }// {最小%,从1500开始的零螺距偏移,最大%}。
    #define YAW_CENTER             1500      // 使用舵机[5]的SERVO_ENDPOINT_HIGH/LOW作为端点。
    #define YAWMOTOR                 0       // 如果一个电机用作YAW则设为1，否则设为0。

    /* 用于120直升机的舵机混控，使用分数1/10（例.5 = 5/10 = 1/2）
                         {Coll,Nick,Roll} */
    #define SERVO_NICK   { +10, -10, -0 }
    #define SERVO_LEFT   { +10, +5, +10 } 
    #define SERVO_RIGHT  { +10, +5, -10 } 

    /* 用于90直升机的舵机混控
                            {Coll,Nick,Roll} */
    #define SERVO_DIRECTIONS { +1, -1, -1 } // -1会反转舵机

    /* 限制用于Roll & Nick最大控制，范围0-100% */
    #define CONTROL_RANGE   { 100, 100 }      //  { ROLL,PITCH }

    /* 使用舵机代码驱动油门输出。用模拟舵机驱动IC引擎上的油门时，你会需要此项。
       如果不启用，油门输出会被看做电机输出，所以它可以驱动电调 */
    //#define HELI_USE_SERVO_FOR_THROTTLE

  /***********************      单轴与共轴双桨飞行器设置  ***********************/
    /* 改为-1以反转每轴的舵机移动方向
       用于单轴飞行器的舵机设置 */
    #define SINGLECOPTRER_YAW   {1, 1, 1, 1} // 左，右，前，后
    #define SINGLECOPTRER_SERVO {1,-1, 1,-1} // Pitch,Pitch,Roll, Roll    
  
    /* 用于共轴双桨飞行器的舵机设置 */
     #define DUALCOPTER_SERVO {1,1} //Pitch,Roll
    /* 使用在直升机与飞机部分的SERVO_OFFSET与SERVO_RATES作为中点和端点 */

  /***********************      你的个人混控              ***********************/
    /* 如果你想要覆盖一个选存的混合表中的条目，你可能想要避免
     * 在每个版本一遍又一遍的编辑mixTable()函数
     * 操作方式：http://www.multiwii.com/wiki/index.php?title=Config.h#Individual_Mixing
     */
    //#define MY_PRIVATE_MIXING "filename.h"
    //#define LEAVE_HEADROOM_FOR_MOTORS 4 // 预留空间给陀螺仪修正，仅用于前四个电机

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 3 部分 - 遥控系统设置                                                *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /* 提示：如果你使用的是标准接收机，不必解除本节的一些注释 */

  /**************************************************************************************/
  /********                       特殊接收机类型                     ********************/
  /**************************************************************************************/

    /****************************    PPM Sum接收机       ***********************************/
      /* 下列几行仅用于特定的仅有一个PPM sum信号的接收机，接在数字针脚2上
         根据你的遥控品牌选择相应的行。当你的PPM顺序不同时，你可以随意修改顺序 */
      //#define SERIAL_SUM_PPM         PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //用于Graupner/Spektrum
      //#define SERIAL_SUM_PPM         ROLL,PITCH,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //用于Robe/Hitec/Futaba
      //#define SERIAL_SUM_PPM         ROLL,PITCH,YAW,THROTTLE,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //用于Multiplex
      //#define SERIAL_SUM_PPM         PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,AUX3,AUX4,8,9,10,11 //用于一些韩国的/日本三和/其它

      // 解除下面这行注释以允许连接PPM_SUM接收机至MEGA板上的标准油门针脚（例.CRIUS AIO上的A8）
      //#define PPM_ON_THROTTLE

    /**********************    Spektrum卫星接收机    *******************************/
      /* 以下几行仅用于Spektrum卫星接收机
         Spektrum卫星系列是3V设备。不要连接至5V！
         对于MEGA板，将灰线连接到RX1，19针脚上。黑线接地。橙线连接到Mega板的3.3V上（或其他3V至3.3V的电源）。
         对于PROMINI，将灰线连接到RX0。黑线接地。 */
      //#define SPEKTRUM 1024
      //#define SPEKTRUM 2048
      //#define SPEK_SERIAL_PORT 1    // Pro Mini与其他单串口的板子上只能设为0；在所有基于Mega的板子上设为你选择的0，1，2（在Mega上默认为1）。
      //**************************
      // 定义此项允许Spektrum或兼容机远程接收机（也就是卫星）通过配置GUI对频。
      //   对频模式与上述的相同，只要你的发射机支持。
      //   接地，电源，信号必须来自三个邻近的针脚。
      //   默认下，它们为接地=4，电源=5，信号=6。这些针脚在多数MultiWii扩展板上都为一排。可在下方覆盖针脚。
      //   通常需要在电源针脚上使用3.3V稳压器！！如果你的卫星在对频时停摆（闪烁，但不会常亮停止闪烁），将所有的针脚连接至5V。
      //**************************
      //   对于Pro Mini，用于卫星的属于FTDI的连接器可以拔掉，并移至那三个相邻针脚。
      //#define SPEK_BIND             //解除注释以开启Spektrum卫星对频支持。没有它代码可节省约420字节。
      //#define SPEK_BIND_GROUND 4
      //#define SPEK_BIND_POWER  5
      //#define SPEK_BIND_DATA   6

    /*******************************    SBUS接收机        ************************************/
      /* 下面这行仅用于Futaba S-Bus接收机在MEGA板上的RX1的情况（串口1）。
         你必须反转S-Bus-串口信号，例如使用十六进制反相器像是IC SN74 LS 04 */
      //#define SBUS

    /*******************   通过Multiwii串口协议的来自串口的遥控信号      *********************/
      //#define RCSERIAL




/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 4 部分 - 替代CPU和板子                                               *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /**************************************************************************************/
  /********                      Promini专用设置                     ********************/
  /**************************************************************************************/

    /**************************       六轴电机 5 & 6 针脚    *******************************/
      /* 用A0与A1针脚代替D5与D6针脚，用于6个电机配置与promini配置
         该模式允许在promini上标准接收机的使用
         （不必使用PPM sum接收机） */
      //#define A0_A1_PIN_HEX

    /*********************************    Aux 2 针脚     ***********************************/
      /* 让你可以使用针脚8或针脚12作为遥控的AUX2输入（只可启用一个，不可全部启用）
         如果启用它会使功率针脚（针脚12）或蜂鸣针脚（针脚8）失效 */
      //#define RCAUXPIN8
      //#define RCAUXPIN12


  /**************************************************************************************/
  /*****************             Teensy 2.0 支持                       ******************/
  /**************************************************************************************/
    /* 解除此项如果你使用的是使用teensyduino的teensy 2.0
       它需要运行在16MHz */
    //#define TEENSY20


  /**************************************************************************************/
  /********   用于ProMicro，Leonardo和其他Atmega32u4板子的设置                ***********/
  /**************************************************************************************/

    /*********************************    针脚布局       **********************************/
      /* 如果所有针脚都能使用，激活此项可获得更好的针脚布局 => 在ProMicro上不可用 */
      //#define A32U4ALLPINS

    /**********************************    PWM设置       **********************************/
      /* 激活全部6个硬件PWM输出，电机5 = D11，电机6 = D13。 
         提示：不可用于sparkfun promicro（针脚11 & 13未被引出）
         如果激活：
         电机1-6 = 10位硬件PWM
         电机7-8 = 8位软件PWM
         舵机    = 8位软件PWM
         如果未激活：
         电机1-4 = 10位硬件PWM
         电机5-8 = 10位软件PWM
         舵机    = 10位软件PWM */
      //#define HWPWM6

    /**********************************    Aux 2 针脚    **********************************/
      /* AUX2针脚在RXO针脚上 */
      //#define RCAUX2PINRXO

      /* aux2针脚在D17针脚上（RXLED） */
      //#define RCAUX2PIND17

    /**********************************    蜂鸣针脚      **********************************/
      /* 此项将蜂鸣针脚从TX0移动至D8以使用ppm sum或spectrum sat.接收机（如果启用了A32U4ALLPINS则不需此项） */
      //#define D8BUZZER

    /***********************      Promicro版本相关             ****************************/
      /* 反转状态LED用于Promicro版本10 */
      //#define PROMICRO10


  /**************************************************************************************/
  /********                      覆盖默认针脚分配                    ********************/
  /**************************************************************************************/

  /* 仅在你必须改变默认针脚分配时才启用其中一项，例：你的板子没有特定针脚 */
  /* 你可能需要依据期望的针脚给PINx与PORTx加上#移位！ */
  //#define OVERRIDE_V_BATPIN                   A0 // 代替A3    // 模拟针脚3

  //#define OVERRIDE_LEDPIN_PINMODE             pinMode (A1, OUTPUT); // 使用A1代替d13
  //#define OVERRIDE_LEDPIN_TOGGLE              PINC |= 1<<1; // PINB |= 1<<5;     //切换LED针脚状态（数字针脚13）
  //#define OVERRIDE_LEDPIN_OFF                 PORTC &= ~(1<<1); // PORTB &= ~(1<<5);
  //#define OVERRIDE_LEDPIN_ON                  PORTC |= 1<<1;    // was PORTB |= (1<<5);

  //#define OVERRIDE_BUZZERPIN_PINMODE          pinMode (A2, OUTPUT); // 使用A2代替d8
  //#define OVERRIDE_BUZZERPIN_ON               PORTC |= 1<<2 //PORTB |= 1;
  //#define OVERRIDE_BUZZERPIN_OFF              PORTC &= ~(1<<2); //PORTB &= ~1;

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 5 部分 - 替代设置                                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /******                串行通讯速度    *********************************/
    /* 此为串行接口速度 */
    #define SERIAL0_COM_SPEED 115200
    #define SERIAL1_COM_SPEED 115200
    #define SERIAL2_COM_SPEED 115200
    #define SERIAL3_COM_SPEED 115200

    /* 在WMP+NK配置中读取WMP/NK两者之间的交错延迟以微秒为单位
       如果加速度计校准时间非常长（20或30秒），尝试增大此延迟至4000
       它仅与有NK（鸡腿柄）的配置相关 */
    #define INTERLEAVING_DELAY 3000

    /* 当I2C总线有错误时，我们可在很短的时间内中立化相关值。用微秒表示
       它仅与至少有一个WMP的配置相关 */
    #define NEUTRALIZE_DELAY 100000


  /**************************************************************************************/
  /********                              陀螺仪滤波器                ********************/
  /**************************************************************************************/

    /*********************    一些陀螺仪的低通滤波器    ****************************/
      /* ITG3200 & ITG3205 低通滤波器设置。假如你不能消除陀螺仪的所有震动，你可以尝试
         减少低通滤波器（LPF）的频率，每次只要减少一档。一旦抽动消失，就保持这个设置。
         它对回馈引起的摆动不起作用，所以只在飞行器随机抽动并且所有抑制和
         平衡设置失效的时候才修改它。只取消注释其中一项！
         重要！改变低通滤波器设置将会改变PID的行为，所以在改变LPF后重新调整你的PID。*/
      //#define ITG3200_LPF_256HZ     // 此为默认设置，不需要取消注释，只作为参考
      //#define ITG3200_LPF_188HZ
      //#define ITG3200_LPF_98HZ
      //#define ITG3200_LPF_42HZ
      //#define ITG3200_LPF_20HZ
      //#define ITG3200_LPF_10HZ      // 只在极端情况下使用此项，更应该换电机和/或螺旋桨

      /* MPU6050 低通滤波器设置。假如你不能消除陀螺仪的所有震动，你可以尝试
         减少低通滤波器（LPF）的频率，每次只要减少一档。一旦抽动消失，就保持这个设置。
         它对回馈引起的摆动不起作用，所以只在飞行器随机抽动并且所有抑制和
         平衡设置失效的时候才修改它。只取消注释其中一项！
         重要！改变低通滤波器设置将会改变PID的行为，所以在改变LPF后重新调整你的PID。*/
      //#define MPU6050_LPF_256HZ     // 此为默认设置，不需要取消注释，只作为参考
      //#define MPU6050_LPF_188HZ
      //#define MPU6050_LPF_98HZ
      //#define MPU6050_LPF_42HZ
      //#define MPU6050_LPF_20HZ
      //#define MPU6050_LPF_10HZ
      //#define MPU6050_LPF_5HZ       // 只在极端情况下使用此项，更应该换电机和/或螺旋桨

    /******                陀螺仪平滑化            **********************************/
      /* GYRO_SMOOTHING.在你不能消除振动的情况下，_并且_是在尝试了低通滤波器选项_之后_，你
         可尝试此通过平均化的陀螺仪平滑化。不适用于多旋翼飞行器！
         在有很多振动的直升机，飞机和飞翼（泡沫的）上可获得良好结果。*/
      //#define GYRO_SMOOTHING {20, 20, 3}    // (*) 分别为roll, pitch, yaw的平均化范围

    /************************    移动平均陀螺仪    **********************************/
      //#define MMGYRO 10                      // (*) 激活用于陀螺仪的移动平均函数
      //#define MMGYROVECTORLENGTH 15          // 移动平均向量的长度（用于可调节的MMGYRO的最大值
      /* 移动平均舵机云台信号输出 */
      //#define MMSERVOGIMBAL                  // 激活用于舵机云台的输出移动平均函数
      //#define MMSERVOGIMBALVECTORLENGHT 32   // 移动平均向量的长度




/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 6 部分 - 可选特性                                                    *******/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************************        连续陀螺仪校准                     ********************/
  /* 如果在校准过程中飞行器被移动，陀螺仪校准将会重复。 */
    //#define GYROCALIBRATIONFAILSAFE

  /************************        AP飞行模式           **********************************/
    /* 临时禁用GPS_HOLD_MODE（GPS保持模式），让移动摇杆时可以调整定点位置。*/
    #define AP_MODE 40  // Create a deadspan for GPS.
        
  /************************    辅助特技练习器         ************************************/
    /* 在自动复原辅助下训练特技。该值设定ANGLE_MODE接管的点。
       记住首先激活ANGLE_MODE！...
       值为200将会给你一个很明显的转换 */
    //#define ACROTRAINER_MODE 200   // http://www.multiwii.com/forum/viewtopic.php?f=16&t=1944#p17437


  /********                          失控保护设置                     ********************/
    /* 失控保护检查四个控制通道CH1-CH4的脉冲。如果脉冲丢失或低于985us（在这四个通道的任意一个上）
       失控保护程序就会启动。从失控保护检测到，再经过FAILSAFE_DELAY的时间，自稳模式就会开启（如果加速度或鸡腿柄可用），
       PITCH，ROLL和YAW被置中，油门设为FAILSAFE_THR0TTLE的值。你必须设定该值使下降速度在1m/s左右
       以获得最佳结果。该值取决于你的配置，总重量和一些其他参数。接下来，在FAILSAFE_OFF_DELAY之后，飞行器会被锁定，
       并且电机会停止。如果遥控脉冲在到达FAILSAFE_OFF_DELAY时间之前恢复，在很短的保护时间之后遥控就会恢复正常。 */
    //#define FAILSAFE                                // 解除注释以激活failsafe函数
    #define FAILSAFE_DELAY     10                     // 用于丢失信号之后失控保护激活之前的保护时间。1步=0.1秒 - 示例中为1秒
    #define FAILSAFE_OFF_DELAY 200                    // 用于电机停止前的着陆时间，以0.1秒为单位。1步=0.1秒 - 示例中为20秒
    #define FAILSAFE_THROTTLE  (MINTHROTTLE + 200)    // (*) 用于降落的油门级别 - 可与MINTHROTTLE相关联 - 如本例所示


  /*****************                DFRobot LED光环     *********************************/
    /* I2C DFRobot LED光环通讯 */
    //#define LED_RING

  /********************************    LED闪光灯      ***********************************/
    //#define LED_FLASHER
    //#define LED_FLASHER_DDR DDRB
    //#define LED_FLASHER_PORT PORTB
    //#define LED_FLASHER_BIT PORTB4
    //#define LED_FLASHER_INVERT
    //#define LED_FLASHER_SEQUENCE        0b00000000      // leds关闭
    //#define LED_FLASHER_SEQUENCE_ARMED  0b00000101      // 创建双闪
    //#define LED_FLASHER_SEQUENCE_MAX    0b11111111      // 全照明
    //#define LED_FLASHER_SEQUENCE_LOW    0b00000000      // 无照明


  /*******************************    着陆灯            *********************************/
  /* 着陆灯
     使用一个输出针脚控制着陆灯。
     它与从声纳获得的高度数据结合时
     可以自动开关。 */
    //#define LANDING_LIGHTS_DDR DDRC
    //#define LANDING_LIGHTS_PORT PORTC
    //#define LANDING_LIGHTS_BIT PORTC0
    //#define LANDING_LIGHTS_INVERT

    /* 依据声纳传来的数在地面之上的高度（以cm为单位） */
    //#define LANDING_LIGHTS_AUTO_ALTITUDE 50

    /* 让闪光灯的样式应用于着陆灯LED */
    //#define LANDING_LIGHTS_ADOPT_LED_FLASHER_PATTERN

  /*************************    飞行时加速度计校准           *****************************/
    /* 此项会激活加速度计飞行时校准 */
    //#define INFLIGHT_ACC_CALIBRATION

  /**************************    禁用WMP电源针脚           *******************************/
    /* 禁止使用电源阵脚
       （如果RCAUXPIN12选项已被选则此项已启用） */
    //#define DISABLE_POWER_PIN

  /*******************************    OSD切换        *************************************/
    // 此项会添加一个可被OSD解读的激活状态的选框（比如说开关覆盖物）
  //#define OSD_SWITCH

  /**************************************************************************************/
  /***********************              发射机-相关            **************************/
  /**************************************************************************************/

    /* 在摇杆中点周围引入一个死区（译者注：无作用控制区）
       必须大于零，如果你不需要在roll，pitch和yaw上的死区就注释掉它 */
    //#define DEADBAND 6

    /* 在定高时定义一个油门摇杆中立区域，默认设置为+/-40
       如果你想改变它，解除注释并在下面更改值。 */
    //#define ALT_HOLD_THROTTLE_NEUTRAL_ZONE 40 


  /**************************************************************************************/
  /***********************                  GPS                **************************/
  /**************************************************************************************/

    /* GPS使用一个串口
       如果启用，在此定义Arduino串口号与UART速度
       注：如在NMEA模式只有RX针脚是被使用的，GPS不可被multiwii配置
       在NMEA模式下，GPS必须配置为输出GGA与RMC NMEA语句（在大部分GPS设备中通常为默认配置）
       至少为5Hz更新速率。解除第一行注释来选择用于GPS的arduino串口 */
    //#define GPS_SERIAL 2 // flyduino v2应设为2。此为arduino MEGA上的串口号
    //#define GPS_BAUD   57600
    #define GPS_BAUD   115200


   /* GPS协议 
       NMEA  - 标准NMEA协议。需要GGA，GSA与RMC语句
       UBLOX - U-Blox二进制协议，使用来自源码树的ublox配置文件（u-blox-config.ublox.txt）
       MTK_BINARY16 与 MTK_BINARY19 - 基于MTK3329芯片的GPS，使用DIYDrones二进制固件（v1.6 或 v1.9）
       在使用UBLOX与MTK_BINARY时你不需要在multiwii代码中使用GPS_FILTERING!!! */

    
    //#define NMEA
    //#define UBLOX
    //#define MTK_BINARY16
    //#define MTK_BINARY19
    //#define INIT_MTK_GPS        // 初始化MTK GPS。使其使用选定的速度，5Hz更新速率与GGA & RMC语句或二进制的设置

    //#define GPS_PROMINI_SERIAL    57600 //  当ardu（ino）启动时如果已连接了GPS，将会自动检测
   
    /* I2C GPS设备，使用一个独立的arduino + GPS设备制作
       包含一些导航函数
       由EOSBandi贡献   http://code.google.com/p/i2c-gps-nav/ 
       你必须使用I2CGpsNav r33以上版本 */
    //#define I2C_GPS

    /* I2C GPS设备，使用独立的ATTiny[24]313 + GPS设备与
       可选的声呐设备制作。    https://github.com/wertarbyte/tiny-gps/ */
    /* 从Tiny-GPS获取GPS数据 */
    //#define TINY_GPS
    /* 从Tiny-GPS获取声呐数据 */
    //#define TINY_GPS_SONAR

    /* 从Misio-OSD中读取GPS数据 - GPS模块连接至OSD，然后MultiWii从OSD中读取GPS数据 - 已测试并且运行正常! */
    //#define GPS_FROM_OSD

    /* 通过LED闪烁表明GPS搜到了至少5颗有效的卫星 - 由MIS修改 - 使用常亮的LED（CRIUS AIO上为黄色）led作为星数指示器工作
      - GPS无定位 -> LED闪烁速度为收到GPS帧的速度
      - 定位并且星数小于5 -> LED关闭
      - 定位并且星数 >= 5 -> LED闪烁，闪一下表示5颗星，闪两下表示6颗星，三下表示7 ... */
    #define GPS_LED_INDICATOR

    //#define USE_MSP_WP                        // 启用MSP_WP命令，用于WinGUI显示与记录家与定点的位置

    //#define DONT_RESET_HOME_AT_ARM             // 家（HOME）的地点会在每次解锁时重置，解除注释此项来禁用它（你可以通过校准陀螺仪来设置家的地点）

    /* GPS navigation can control the heading */
    
    #define NAV_CONTROLS_HEADING       true      // 飞行器面对着航电飞行，磁场保持必须为此开启
    #define NAV_TAIL_FIRST             false     // true - 飞行器以尾部首先飞来
    #define NAV_SET_TAKEOFF_HEADING    true      // true - 当飞行器到达家的位置时他会旋转至起飞时的角度
    
    
    /* 从这里获取你的磁偏角：http://magnetic-declination.com/
       转换度+分至小数的角度，通过 ==> 度+分*(1/60)
       注意磁偏角的符号，它可为负或正（西或东） */
    //#define MAG_DECLINIATION  3.96f              // 用于匈牙利布达佩斯。
    #define MAG_DECLINIATION  0.0f

    #define GPS_LEAD_FILTER                      // 添加向前预测滤波以补偿GPS延迟。代码基于Jason Short领导的滤波器实现
    
    //#define GPS_FILTERING                        // 添加5元素移动平均滤波器至GPS坐标，帮助消除GPS噪波但会增加延时，注释以禁用
    #define GPS_WP_RADIUS              200       // 如果我们与航点在此距离以内，我们则认为已到达航点（以cm为单位）
    #define NAV_SLEW_RATE              30        // 将速率控制添加至导航输出，可磨平导航角度毛刺


  /**************************************************************************************/
  /***********************        LCD/OLED - 显示设置       *********************/
  /**************************************************************************************/

    /* http://www.multiwii.com/wiki/index.php?title=Extra_features#LCD_.2F_OLED */

    /*****************************    LCD种类              **********************************/
      /* 选择用于配置和遥测的LCD，见下方注解 */
      //#define LCD_DUMMY       // 无物理LCD附加。通过定义此与LCD_CONF，发射机遥杆可用于设置增益，通过观察LED闪烁。  
      //#define LCD_SERIAL3W    // Alex的初始变体使用3条导线，使用rx针脚进行传输@固定的9600波特率
      //#define LCD_TEXTSTAR    // 串口LCD：Cat's Whisker品牌的LCD_TEXTSTAR模块CW-LCD-02（拥有4个输入按键用于选择菜单）
      //#define LCD_VT100       // 串口LCD：vt100兼容终端仿真（blueterm，putty等）
      //#define LCD_TTY         // 串口LCD：用于通过线缆与arduino IDE“串口监视器”连接调整参数
      //#define LCD_ETPP        // I2C LCD：Eagle Tree品牌的Power Panel LCD，使用i2c（非串口）
      //#define LCD_LCD03       // I2C LCD：LCD03，使用i2c
      //#define OLED_I2C_128x64 // I2C LCD：OLED http://www.multiwii.com/forum/viewtopic.php?f=7&t=1350

    /******************************   显示设置            ***********************************/
      #define LCD_SERIAL_PORT 0    // 在Pro Mini以及其他单串口板上只能设为0，在任何基于Mega的板子上可设置为你的选择

      //#define SUPPRESS_OLED_I2C_128x64LOGO  // 禁用OLED logo显示来节省储存

    /* 为获得更好的可读性，使用双倍字体高度。减少一半可见#行。
     * 每个页面的下半部分以按住shift的键盘文字作为名字：
     * 1 - ! , 2 - @ , 3 - # , 4 - $ , 5 - % , 6 - ^ , 7 - & , 8 - * , 9 - (
     * 你必须同时添加到你的lcd.遥测.*序列中
     */
      //#define DISPLAY_FONT_DSIZE //目前只能应用于OLED_I2C_128x64

    /* 显示风格 - 通过LCD_ setting自动检测 - 仅在覆盖默认时激活 */
      //#define DISPLAY_2LINES
      //#define DISPLAY_MULTILINE
      //#define MULTILINE_PRE 2  // 多行配置菜单#之前的行
      //#define MULTILINE_POST 6 // 多行配置菜单#之后的行
    /********************************    导航             ***********************************/
    /* 用来导航LCD配置菜单的按键 */
      #define LCD_MENU_PREV 'p'
      #define LCD_MENU_NEXT 'n'
      #define LCD_VALUE_UP 'u'
      #define LCD_VALUE_DOWN 'd'

      #define LCD_MENU_SAVE_EXIT 's'
      #define LCD_MENU_ABORT 'x'

  /**************************************************************************************/
  /***********************             LCD配置菜单             **************************/
  /**************************************************************************************/

    /* 如果你准备将LCD或OLED用于调整参数，那么解除本行注释
     * http://www.multiwii.com/wiki/index.php?title=Extra_features#Configuration_Menu */
      //#define LCD_CONF

    /* 用于包含通过LCD进行AUX1 -> AUX4辅助开关切换的设置 */
      //#define LCD_CONF_AUX

    /* 可选排除一些功能 - 解除注释以禁用一些不需要的遥测页面 */
      //#define SUPPRESS_LCD_CONF_AUX34

  /**************************************************************************************/
  /***********************              LCD遥测                **************************/
  /**************************************************************************************/

    /* 通过LCD监控系统数据（电池电压，周期时间等）
     * http://www.multiwii.com/wiki/index.php?title=LCD_Telemetry */

    /********************************    激活     ***********************************/
    //#define LCD_TELEMETRY

    /* 在解除注释于此的一个遥测页面组合中启用自动跳转。 */
    //#define LCD_TELEMETRY_AUTO "123452679" // 升序显示1至9页
    //#define LCD_TELEMETRY_AUTO  "212232425262729" // 着重显示第2页

    /* 手动步进序列；序列的第一页在启动时加载以允许无交互时显示 */
    //#define LCD_TELEMETRY_STEP "0123456789" // 应包含一个0以允许关闭。

    /* 可选地排除一些功能 - 解除注释以禁用一些不需要的遥测页面 */
    //#define SUPPRESS_TELEMETRY_PAGE_1
    //#define SUPPRESS_TELEMETRY_PAGE_2
    //#define SUPPRESS_TELEMETRY_PAGE_3
    //#define SUPPRESS_TELEMETRY_PAGE_4
    //#define SUPPRESS_TELEMETRY_PAGE_5
    //#define SUPPRESS_TELEMETRY_PAGE_6
    //#define SUPPRESS_TELEMETRY_PAGE_7
    //#define SUPPRESS_TELEMETRY_PAGE_8
    //#define SUPPRESS_TELEMETRY_PAGE_9

  /********************************************************************/
  /****                      接收信号强度显示（RSSI）              ****/
  /********************************************************************/
    //#define RX_RSSI
    //#define RX_RSSI_PIN A3

  /********************************************************************/
  /****                      蜂鸣器（BUZZER）                      ****/
  /********************************************************************/
    //#define BUZZER
    //#define RCOPTIONSBEEP         // 如果你想在遥控选项在通道Aux1至Aux4改变时让蜂鸣器响起，解除注释此项
    //#define ARMEDTIMEWARNING 330  // (*) 在解锁一段时间[s]后触发警报以保护锂电。（如果你的发射机没有倒计时）
    //#define PILOTLAMP             //如果你在使用X-Arcraft导航灯那么解除注释

  /********************************************************************/
  /****                      电池电压监控                          ****/
  /********************************************************************/
    /* 用于V BAT（电池电压）监控
       在电阻分压后，我们在模拟V_BAT针脚上应获得[0V;5V]->[0;1023]
       通过R1=33k和R2=51k
       vbat = [0;1023]*16/VBATSCALE
       必须与#define BUZZER结合! */
    //#define VBAT              // 解除注释本行以激活vbat代码
    #define VBATSCALE       131 // (*) 如果读取到的电池电压与真实电压不同，修改该值
    #define VBATNOMINAL     126 // 12,6V满电标准电压 - 仅用于lcd.遥测
    #define VBATLEVEL_WARN1 107 // (*) 10,7V
    #define VBATLEVEL_WARN2  99 // (*) 9.9V
    #define VBATLEVEL_CRIT   93 // (*) 9.3V - 临界情况：如果vbat持续低于该值，就会触发警报长响
    #define NO_VBAT          16  // (*) 避免在没有电池时响起


  /********************************************************************/
  /****                      功率计（电池容量监控）                ****/
  /********************************************************************/

    /* 启用电池能量消耗监控（以mAh考虑）
       允许在GUI中或通过LCD设置警戒值
       全部描述与操作方法请见此 http://www.multiwii.com/wiki/index.php?title=Powermeter
       有两个选项：
       1 - 硬件: - （使用硬件传感器，配置后将获得相当不错的结果）
       2 - 软件: - （使用plush与mystery电调可获得+-5%的料号结果，使用SuperSimple电调结果不佳）    */
    //#define POWERMETER_SOFT
    //#define POWERMETER_HARD
    /* PLEVELSCALE是你用于设置警告的步进大小 */
    #define PLEVELSCALE 50 // 如果你将这些值修改为其他间隔大小，你必须在代码中查找注释进行相应的修改
    /* 更大的PLEVELDIV会让你得到更小的功率（就是mAh）值 */
    #define PLEVELDIV 5000 // (*) 默认用于软件 - 如果你使用了更低的PLEVELDIV，当心溢出uint32功率计范围
    #define PLEVELDIVSOFT PLEVELDIV // 用于软件使用等于PLEVELDIV；用于硬件设为5000
    #define PSENSORNULL 510 // (*) 设置0电流时analogRead()的值；I=0A时，我的传感器得到1/2 Vss；约为2.49伏；
    #define PINT2mA 13 // (*) 用于遥测显示：一个用在arduino模拟转换为mA时的整数（例4.9 / 37 * 100

  /********************************************************************/
  /****                      高度保持                              ****/
  /********************************************************************/

    /* 解除注释以禁用高度保持特性。
     * 此项可用于所有下列应用
     * + 你有一个气压传感器
     * + 想要高度值输出
     * + 不需要使用高度保持特性
     * + 想要节省储存空间
     */
    //#define SUPPRESS_BARO_ALTHOLD

  /* 用于快速操纵的自然高度变化。当油门摇杆移出ALT_HOLD_THROTTLE_NEUTRAL_ZONE定义的死区时，临时关闭定高
   * 但如果它被注释掉：平滑高度变化程序被激活，用于慢速自动模式与航拍模式（来自alexmos的通用解决方案）。可缓慢地升高/降低
   * 高度与摇杆移动成比例（+/-100油门在周期时间为3-4ms时，1秒产生约+/-50cm的高度变化）
   */
  #define ALTHOLD_FAST_THROTTLE_CHANGE

  /********************************************************************/
  /****                     高度爬升率测定器                       ****/
  /********************************************************************/

    /* 启用以获得来自上升/下降中的飞行器/飞机的声频反馈。
     * 需要工作中的气压计。
     * 目前，输出会通过串行线发送至启用中的vt100终端程序。
     * 有两种方式可选（启用其中一个或同时启用）
     * 方式1：使用来自气压计的短期移动（更大的代码尺寸）
     * 方式2：使用来自气压计的长期高度观察（更小的代码尺寸）
     */
    //#define VARIOMETER 12            // 可用值：12 = 方式 1 & 2 ；1 = 方式 1；2 = 方式 2
    //#define SUPPRESS_VARIOMETER_UP   // 如果不期望有用于向上移动的信号
    //#define SUPPRESS_VARIOMETER_DOWN // 如果不期望有用于向下移动的信号
    //#define VARIOMETER_SINGLE_TONE   // 仅使用一个声调（响铃）；对未打补丁的vt100终端是必需的

  /********************************************************************/
  /****                     板子命名                               ****/
  /********************************************************************/

    /*
     * 这个名字会与MultiWii版本号共同显示
     * 在打开电源时显示在LCD上。
     * 如果你没有显示设备那么你可以启用LCD_TTY并
     * 使用arduino IDE的串口监控器来查看此信息。
     *
     * 你必须保持此处文本的格式！
     * 它必须总共有16个字母，
     * 最后4个字母将会被版本号覆盖。
     */
    #define BOARD_NAME "MultiWii   V-.--"
    //                  123456789.123456

  /*************      在EEPROM中支持多个配置参数文件      ************/
    //#define MULTIPLE_CONFIGURATION_PROFILES

/*************************************************************************************************/
/*****************                                                                 ***************/
/****************  第 7 部分 - 调试 & 开发者                                       **************/
/*****************                                                                 ***************/
/*************************************************************************************************/

  /************ 实验性的：强制一个稳固（高）的周期时间       **********/
    /* 当此项激活，在GUI中显示的周期时间将会不正确。
     * 可通过LCD配置菜单调整。
     * 值为0关闭此项特性。
     */
    //#define CYCLETIME_FIXATED 9000 // (*)

  /**************************************************************************************/
  /********          使用扩展范围[0-2000]微秒的特殊电调              ********************/
  /**************************************************************************************/
    //#define EXT_MOTOR_RANGE

  /**************************************************************************************/
  /***********************     电机，舵机和其他的预置             ***********************/
  /**************************************************************************************/
    /* 当油门命令在低位时电机将不会旋转
       这是立即停止电机的替代方案 */
    #define MOTOR_STOP

    /* 一些遥控器的中立点不是1500。可以在此修改 */
    #define MIDRC 1500

  /***********************         舵机刷新率                   ***********************/
    /* 默认50Hz舵机刷新率 */
    #define SERVO_RFR_50HZ

    /* 升至160Hz舵机刷新率 .. 用于多数模拟舵机 */
    //#define SERVO_RFR_160HZ

    /* 升至300Hz刷新率，它越快越好（100-300Hz取决于使用的舵机和舵机状态）。
       用于数字舵机
       不要用于模拟舵机！它们可能遭到破坏。（一些可以使用，但请非常小心） */
    //#define SERVO_RFR_300HZ
    
  /***********************             硬件PWM舵机              ***********************/ 
    /* 硬件PWM舵机输出用于Arduino Mega..移动至：
      Pitch   = pin 44
      Roll    = pin 45
      CamTrig = pin 46
      SERVO4  = pin 11 (在配置为飞行器时指定给PPM或SPECTRUM通道9)
      SERVO5  = pin 12 (在配置为飞行器时指定给PPM或SPECTRUM通道10)
      此选项禁用其他用于舵机的软件PWM - 仅有五个硬件控制舵机可用
      */ 
    //#define MEGA_HW_PWM_SERVOS

  /********************************************************************/
  /****           串行命令处理 - MSP或其他命令                     ****/
  /********************************************************************/

    /* 如需减少储存空间需求，可以通过禁用串口命令处理来实现。
     * 它_不会_对RXserial，Spektrum，GPS的处理产生影响。这些不会受到影响，仍可以照常工作。
     * 启用下列选项中其中一项或两项  */

    /* 移除所有新MultiWii串行协议命令的处理。
     * 这将会禁用GUI，winGUI，android应用以及其他所有使用MSP的程序。
     * 你必须找到其他调试参数的方法（如LCD_CONF）或保持默认。
     * 如果你是通过i2c或串口/蓝牙使用LCD/OLED，可以放心使用 */
    //#define SUPPRESS_ALL_SERIAL_MSP // 节省约2700字节

    /* 移除其他串行命令处理。
     * 包含通过串口操作lcd.配置菜单，lcd.遥测与永久.日志。
     * 通过在发射机上摇杆输入进行操作不会受到影响，操作起来是一样的。  */
    //#define SUPPRESS_OTHER_SERIAL_COMMANDS // 节省约0至100字节，取决于启用的特性

  /********************************************************************/
  /****           诊断                                             ****/
  /********************************************************************/

    /* 记录像最大周期时间与其他可能的值
       记录值可通过LCD配置看到
       设为1，启用'R'选项来重置值，最大电流，最大高度
       设为2，添加最大/最小周期时间
       设为3，以每个电机为单位添加额外的功耗（它使用一个很大的数组并且很吃储存，如果POWERMETER <> PM_SOFT） */
    //#define LOG_VALUES 1

    /* 永久记录至eeprom - 可在（多数）升级与参数重置中保留下来。
     * 常用于追踪控制板生命周期中的飞行次数等。
     * 写入至eeprom末端 - 不应与已储存的参数冲突。
     * 记录的值：累积的生存时间，#重启/重置/初始化事件，#解锁事件，#锁定事件，最后解锁时间，
     *                #失控保护@锁定，#i2c_errs@锁定
     * 设置你的mcu的eeprom的尺寸以激活：promini 328p：1023；2560：4095。
     * 启用一项或更多选项以显示记录
     */
    //#define LOG_PERMANENT 1023
    //#define LOG_PERMANENT_SHOW_AT_STARTUP // 启用以在启动时显示记录
    //#define LOG_PERMANENT_SHOW_AT_L // 启用以在接收到'L'时显示记录
    //#define LOG_PERMANENT_SHOW_AFTER_CONFIG // 启用以在退出LCD配置菜单之后显示记录
    //#define LOG_PERMANENT_SERVICE_LIFETIME 36000 // 以秒为单位；在10小时的解锁时间之后，在启动时响起服务警告

    /* 添加调试代码
       不需要并且也不推荐在平常运行时开启
       将会额外添加代码，可能会使主循环变慢或使飞行器不可飞行 */
    //#define DEBUG

    /* 使用此项在没有发射机时触发LCD配置 - 仅用于调试 - 不要在此项激活的情况下飞行 */
    //#define LCD_CONF_DEBUG

    /* 使用此项在没有发射机时触发遥测 - 仅用于调试 - 不要在此项激活的情况下飞行 */
    //#define LCD_TELEMETRY_DEBUG    //该形式在所有的屏幕间轮换，LCD_TELEMETRY_AUTO必须同时被定义。
    //#define LCD_TELEMETRY_DEBUG 6  //该形式停在特定的屏幕上。

    /* 启用从飞行器到GUI的字符串传送 */
    //#define DEBUGMSG


  /********************************************************************/
  /****           电调校准                                         ****/
  /********************************************************************/

    /* 同时校准所有连接到MWii的电调（可以避免来回连接每一个电调）
       警告：这将产生一个特别版本的MultiWii代码
       这个特殊的版本是不可以用来飞行的。它只可以用来校准电调
       使用方法详见 http://code.google.com/p/multiwii/wiki/ESCsCalibration */
    #define ESC_CALIB_LOW  MINCOMMAND
    #define ESC_CALIB_HIGH 2000
    //#define ESC_CALIB_CANNOT_FLY  // 解除注释激活此项

  /****           内部频率                                         ****/
    /* 在主循环中的稀有循环操作的频率，取决于周期时间
       时间基数为主循环周期时间 - 值为6意味着每六个主循环触发一次操作
       示例：周期时间大约在3ms，执行操作就在每 6*3ms=18ms
       取值范围 [1; 65535] */
    #define LCD_TELEMETRY_FREQ 23       // 通过串口发送遥测数据 23 <=> 60ms <=> 16Hz （只发送隔行数据，8Hz上传速率）
    #define LCD_TELEMETRY_AUTO_FREQ 967 // 翻到下一个遥测页面 967 <=> 3s
    #define PSENSORFREQ 6               // 读取硬件功率传感器 6 <=> 18ms
    #define VBATFREQ PSENSORFREQ        // 读取电池电压 - 保持与 PSENSORFREQ 相等 除非你知道你自己在做什么

  /********************************************************************/
  /****           回归测试                                         ****/
  /********************************************************************/

    /* 只用作开发用途:
       考虑到测试编译时，不同的config参数是保持在一起的，所以可以更简单地重复测试config设置，
       它的意义是可以帮助检测编译时的错误，让多种不同的特性以协调的方式运作。
       这并不是用来制作你自己的飞行固件的。
       使用方法：
       - 不要在config.h中做任何设置，
       - 启用#define COPTERTEST 1，然后编译
       - 如果可能的话，检查程序大小
       - 重复测试其他值2, 3, 4等。
        */
    //#define COPTERTEST 1

/*************************************************************************************************/
/****           可配置参数结束                                                                ****/
/*************************************************************************************************/
