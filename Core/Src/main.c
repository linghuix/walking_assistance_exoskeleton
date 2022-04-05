
#include "main.h"

#define ECON
#define ECON_L
#define ECON_R

/* safe */
#define TH_BOUND 40.0
#define PERIOD 50

/* 控制周期 */
uint32_t inc            = 0;              // 2ms 周期计时器
int      CONTROL_PERIOD = PERIOD / 2;     // ms 实际控制周期 CONTROL_PERIOD*2 ms
float    dt             = PERIOD / 1000;  // 控制周期 50 ms

/**
 * @para 用于检测人体是否静止
 */
int16_t stopCounter[2] = {0};  // 停止判断计数器;
int16_t stopFlag[2]    = {0};  // 站立判断指示 1-站立
#define TH_W 8.0               // 角速度阈值
#define TH_D 15.0  // 角度阈值

/*  交互力模块 */
uint16_t Interaction_force = 0;

/* 外部IMU采集 */
float hip1_w, hip1_d, I1;
float hip2_w, hip2_d, I2;
float hip1_raww, hip1_rawd;
float hip2_raww, hip2_rawd;

/* 滤波 */
#define Buffsize 2  // 原始数据滤波窗口
WIN         acc1win_w, acc2win_w, acc1win_d, acc2win_d;
ElementType acc1WinArray_d[Buffsize] = {0};
ElementType acc2WinArray_d[Buffsize] = {0};
ElementType acc1WinArray_w[Buffsize] = {0};
ElementType acc2WinArray_w[Buffsize] = {0};
float       weights[Buffsize] = {0.15, 0.85};    // data_now = 0.95 x data_pre + 0.05 x data_now
float       phaseOffset       = -0.10 * PI * 2;  // phase = phase-phaseOffset

/*  峰值检测模块 */
#define TH_PERIOD 20
uint32_t peaktimestamp[2]   = {0};       // 峰值对应时间记录
uint16_t period[2]          = {20, 20};  // 人体步态周期估计
int32_t  peak_delay_time[2] = {0};

WIN         d1minwin_w, d2minwin_w;  // 极小值检测窗口
ElementType d1Win[3] = {0};
ElementType d2Win[3] = {0};

/* 相位模块 */
float phase[2], predictPhase[2];  // 0左，1右
float AOoffset[2] = {0};          // AO 相位补偿器

/*advanced PO*/
struct APO apohip1;  // po.h
struct APO apohip2;

/* 助力模式 */
extern int16_t delaySwitch[2];
int8_t         assive_mode[2] = {0};  // 当前助力模式
int            state[2]       = {0};  // 0-stop 1-walking

/* 助力值计算 */
// float AssisTor = 0.5;
//#define RightTorRatio 1	   右侧的 assist gain 更大一些
#define D_area 2.0  // 2.0	for eliminate chattering
#define W_area 2.0  // 1.0
#define MAX_D_area 50.0  // for safety
float left_k = 0, right_k = 0;
float kkkk = 0;

/* 助力值优化参数 */
float tao_Ep = 2.23;   // 5-10 Nm  // 0.1-0.8
float fai_Ep = 0.202;  // 0.2-0.3
float fai_Er = 0.112;  // 0.1-0.2
float fai_Ef = 0.176;  // 0.1-0.2
float tao_Fp, fai_Fp, fai_Fr, fai_Ff;
float a[3];
float b[3];

// Jscope 调试
int     debug_hip1_d, debug_hip1_rawd, debug_hip2_d, debug_hip2_rawd;
int     debug_hip1_w, debug_hip1_raww, debug_hip2_w, debug_hip2_raww;
uint8_t debug_peak[2]  = {0, 0};
uint8_t found_peak[2]  = {0, 0};
int     debug_AOphase1 = 0, debug_AOoutput1 = 0, debug_AOpre1 = 0, debug_phase1 = 0, debug_AOw1,
    debug_AOphasePre1;
int debug_AOphase2 = 0, debug_AOoutput2 = 0, debug_AOpre2 = 0, debug_phase2 = 0, debug_AOw2,
    debug_AOphasePre2;
int debug_AOIndex      = 0;
int debug_assisTorque1 = 0, debug_assisTorque2 = 0;  // 临时查看变量
int debug_tmp;

extern int   idleflag;
extern float floatabs(float x);
int          main(void)
{
  /* Basic initalization */
  Core_Config();
  Jlink_Init();
  debug_init();

  /******* test code *******/
  //  test_USART1_communication();
  //	test_win_buff();
  //	test_HC05_communication();
  //	test_AOs();


  /* Advanced PO */
  apohip1.APO_updateinterval = 20;
  apohip2.APO_updateinterval = 20;
  apohip1.kk                 = 1.0;
  apohip1.alpha              = 0.0;
  apohip1.beta               = 0.0;
  apohip2.kk                 = 1.0;
  apohip2.alpha              = 0.0;
  apohip2.beta               = 0.0;
  apohip1.stopflag           = 1;
  apohip2.stopflag           = 1;
  apohip1.begin              = 1;
  apohip2.begin              = 1;
  apohip1.once               = 1;
  apohip2.once               = 1;

  //	right_current_control();
  //	left_current_control();

  /* IMU initalization */
  Acc1_Init();
  Acc2_Init();
  winBuffer(&acc1win_d, acc1WinArray_d, Buffsize);
  winBuffer(&acc2win_d, acc2WinArray_d, Buffsize);
  winBuffer(&acc1win_w, acc1WinArray_w, Buffsize);
  winBuffer(&acc2win_w, acc2WinArray_w, Buffsize);

  /* Assistive Curve */
  tao_Fp = tao_Ep;
  fai_Fp = 0.5 + fai_Ep;
  fai_Fr = fai_Er;  // 0.1-0.2
  fai_Ff = fai_Ef;  // 0.1-0.2

  /* 峰值检测 Peak Detection */
  winBuffer(&d1minwin_w, d1Win, 3);
  winBuffer(&d2minwin_w, d2Win, 3);

  /* Control Loop */
  MX_TIM_PWMOUT(TIM4, 50000, 100);
  HAL_TIM_Base_Start_IT(&htim4);

#ifdef ECON
  ECON_I_init();
  ECON_action();
#endif

  /* AO */
  AO_Init(period[0] * dt, 1);
  AO_Init(period[1] * dt, 2);

  /*启动外设*/
  Acc1_Start();
  Acc2_Start();

  printf("\r\nABOUT ANGLE AND SPEED couterclock is postive from outside.\r\n");
  printf(
      "\r\nthe acc1 of left hip - d w | the acc2 of right hip - d w | I1 "
      ",I2\r\n");

  //	FSR_Init();
  HC05_RcvCmd();

  float tmp1 = (fai_Er * fai_Er);
  float tmp2 = (fai_Ep * fai_Ep);
  float tmp3 = (fai_Ef * fai_Ef);
  a[0]       = -tao_Ep / tmp1;
  a[1]       = 2 * tao_Ep * fai_Ep / tmp1;
  a[2]       = tao_Ep - tao_Ep * tmp2 / tmp1;
  b[0]       = -tao_Ep / tmp3;
  b[1]       = 2 * tao_Ep * fai_Ep / tmp3;
  b[2]       = tao_Ep - tao_Ep * tmp2 / tmp3;


  while (1) {
    /* idle 串口数据解析 */
    if (idleflag == 1) {
      commandPrase();
      idleflag = 0;
    }

    /* 左髋关节 加速度信号采集  采样周期 100Hz */
    if (flag_1 == 1 && flag_2 == 1 && flag_3 == 1) {
      //			printf("L\r\n");
      flag_1 = 0;
      flag_2 = 0;
      flag_3 = 0;

      hip1_rawd = -angle1[0] / 32768.0 * 180;
      hip1_raww = -w1[0] / 32768.0 * 2000;

      if (hip1_rawd > 0) {
        hip1_rawd = hip1_rawd - 360;
      }
      hip1_rawd = hip1_rawd + 180;

      addToBuff(&acc1win_d, hip1_rawd);
      addToBuff(&acc1win_w, hip1_raww);
      changeLastestValue(&acc1win_d, avergeWin(&acc1win_d, weights, Buffsize));
      changeLastestValue(&acc1win_w, avergeWin(&acc1win_w, weights, Buffsize));

      hip1_d = getLastestValue(acc1win_d);
      hip1_w = getLastestValue(acc1win_w);

      addToBuff(&d1minwin_w, hip1_d);
      debug_peak[0] = findpeak(&d1minwin_w);  // 髋关节伸展角度极值检测
      if (debug_peak[0] == 1) {
        found_peak[0] = 1;
      }

      /* detect the stop state */

      if (floatabs(hip1_w) < TH_W && floatabs(hip1_d) < TH_D) {
        stopCounter[0]++;

        if (stopCounter[0] >= 50) {
          stopCounter[0] = 50;
        }
      }
      else {
        stopCounter[0]--;
        if (stopCounter[0] <= -50) {
          stopCounter[0] = -50;
        }
      }

      if (stopCounter[0] > 0) {  // 连续检测到10次，为 stop state
        stopFlag[0] = 1;
      }
      else if (stopCounter[0] < 0) {
        stopFlag[0] = 0;
      }
    }

    /* 右髋关节 加速度信号采集  采样周期 100Hz */
    if (flag_11 == 1 && flag_22 == 1 && flag_33 == 1) {
      //			printf("R\r\n");
      flag_11 = 0;
      flag_22 = 0;
      flag_33 = 0;

      hip2_rawd = angle2[0] / 32768.0 * 180;
      hip2_raww = w2[0] / 32768.0 * 2000;

      if (hip2_rawd > 0) {
        hip2_rawd = hip2_rawd - 360;
      }
      hip2_rawd = hip2_rawd + 180;

      addToBuff(&acc2win_d, hip2_rawd);
      addToBuff(&acc2win_w, hip2_raww);
      changeLastestValue(&acc2win_d, avergeWin(&acc2win_d, weights, Buffsize));
      changeLastestValue(&acc2win_w, avergeWin(&acc2win_w, weights, Buffsize));

      hip2_d = getLastestValue(acc2win_d);
      hip2_w = getLastestValue(acc2win_w);

      addToBuff(&d2minwin_w, hip2_d);
      debug_peak[1] = findpeak(&d2minwin_w);  // 髋关节伸展角度极值检测
      if (debug_peak[1] == 1) {
        found_peak[1] = 1;
      }

      /* detect the stop state */
      if (floatabs(hip2_w) < TH_W && floatabs(hip2_d) < TH_D) {
        stopCounter[1]++;
        if (stopCounter[1] >= 50) {
          stopCounter[1] = 50;
        }
      }
      else {
        stopCounter[1]--;
        if (stopCounter[1] <= -50) {
          stopCounter[1] = -50;
        }
      }

      if (stopCounter[1] > 0) {
        stopFlag[1] = 1;
      }
      else if (stopCounter[1] < 0) {
        stopFlag[1] = 0;
      }
    }

    /* 控制周期 2ms x CONTROL_PERIOD */
    if (inc % CONTROL_PERIOD == 0) {
      // promote the code is running
      //			if(inc % (CONTROL_PERIOD * 50) == 0){
      //				printf("Ctrl\r\n");
      //			}

      // print IMU information

      IMUMonitor("acc1rawd\t%.2f\tw\t%.2f\t", hip1_rawd, hip1_raww);
      IMUMonitor("acc2rawd\t%.2f\tw\t%.2f\t", hip2_rawd, hip2_raww);
      IMUMonitor("%.2f\t%.2f\t", hip1_d, hip1_w);
      IMUMonitor("%.2f\t%.2f\t", hip2_d, hip2_w);

      // human exo Interaction force
      //			Interaction_force = GetFSRForce();
      //			INTERFORCE_Monitor("F %d\t", Interaction_force);

      Aoindex++;  // 控制周期的序号
      // debug_AOIndex++; if( debug_AOIndex > 100 ){ debug_AOIndex = 0;}//FOR
      // TEST

      /**
              @name 左
      */
      {
        /* AO iteration */
        AO(hip1_d, 1);

        /* Peak detection and period estimation*/
        if (found_peak[0] == 1) {  // 检测峰值，估计人体步态周期并记录需要补偿的相位值
          period[0]        = Aoindex - peaktimestamp[0];  // gait period estimation
          peaktimestamp[0] = Aoindex;  // peaktimestamp 记录上一个峰值对应的 Aoindex
          AOoffset[0]      = hip1.phase[1];
          found_peak[0]    = 0;
        }

        /* Mode selection */
        assive_mode[0] = switch_task(&hip1, hip1_d, hip1_w, 1);  // 模式切换
        assive_mode[0] = POMODE;

        /* Get phase*/
        if (assive_mode[0] == POMODE) {
          phase[0] = APOPhase(&apohip1, hip1_d, hip1_w);
          phase[0] = -phase[0] + PI - phaseOffset;
          phase[0] = fitIn(phase[0], 2 * PI, 0);
        }
        else if (assive_mode[0] == AOMODE) {
          phase[0] = hip1.predictedBasicPhase - AOoffset[0];
          phase[0] = fitIn(phase[0], 2 * PI, 0);
        }
        else {
          while (1) {
            MSG_ERR(123, "assive_mode error\r\n", 123);
          }
        }

        /* No assistance conditions */
        left_k = 1.0;
        // 防高频率抖动
        //			if(period[0] < TH_PERIOD){
        //				left_k = 0.0;
        //			}

        // 站立时不助力
        // if stop state is detected
        if (stopFlag[0] == 1) {
          left_k = 0.0;
        }

        // 角度过大时的安全防护
        //			if(floatabs(hip1_rawd) > TH_BOUND){
        //				left_k = 0.0;
        //			}

        /* Assistive Torque */
        // c1-2-4-1-3-5-1

        // I1  = sin(phase[0]);

        phase[0] = phase[0] / 2.0 / PI;
        if (phase[0] < fai_Ep - fai_Er ||
            ((phase[0] > fai_Ep + fai_Ef) && (phase[0] < fai_Fp - fai_Fr)) ||
            phase[0] > (fai_Fp + fai_Ff)) {
          // printf("c1");
          I1 = 0;
        }
        else if ((phase[0] >= fai_Ep - fai_Er) && (phase[0] <= fai_Ep)) {
          // printf("c2");
          I1 = (a[0] * phase[0] + a[1]) * phase[0] + a[2];
        }
        else if ((phase[0] >= fai_Ep - fai_Er + 0.5) && phase[0] <= (fai_Ep + 0.5)) {
          // printf("c3");
          float phi = phase[0] - 0.5;
          I1        = -((a[0] * phi + a[1]) * phi + a[2]);
        }
        else if (phase[0] >= fai_Ep && (phase[0] <= fai_Ep + fai_Ef)) {
          // printf("c4");
          I1 = (b[0] * phase[0] + b[1]) * phase[0] + b[2];
        }
        else if ((phase[0] >= fai_Ep + 0.5) && (phase[0] <= fai_Ep + fai_Ef + 0.5)) {
          // printf("c5");
          float phi = phase[0] - 0.5;
          I1        = -((b[0] * phi + b[1]) * phi + b[2]);
        }

        I1 = I1 * left_k;

#ifdef ECON_L
        set_I_direction(1, -I1);  // 正表明是往前
#endif

        AssisMonitor("I1 %.2f\t", I1);
      }

      // -------------------------------------------------------------------//

      /**
              @name 右
      */
      {
        AO(hip2_d, 2);

        /* Summit Detect */

        if (found_peak[1] == 1) {  // 检测峰值，估计人体步态周期并记录需要补偿的相位值
          period[1]        = Aoindex - peaktimestamp[1];  // gait period get
          peaktimestamp[1] = Aoindex;
          AOoffset[1]      = hip2.phase[1];
          found_peak[1]    = 0;
        }

        assive_mode[1] = switch_task(&hip2, hip2_d, hip2_w, 2);
        assive_mode[1] = POMODE;

        /* get phase */
        //			right_k = AssisTor*RightTorRatio;
        if (assive_mode[1] == POMODE) {
          phase[1] = APOPhase(&apohip2, hip2_d, hip2_w);
          phase[1] = -phase[1] + PI - phaseOffset;
          phase[1] = fitIn(phase[1], 2 * PI, 0);
        }
        else if (assive_mode[1] == AOMODE) {
          phase[1] = hip2.predictedBasicPhase - AOoffset[1];
          phase[1] = fitIn(phase[1], 2 * PI, 0);
        }
        else {
          while (1) {
            MSG_ERR(123, "assive_mode error\r\n", 123);
          }
        }

        right_k = 1.0;
        //			if(period[1] < TH_PERIOD){
        //				right_k = 0.0;
        //			}
        //
        if (stopFlag[1] == 1) {
          right_k = 0.0;
        }

        //			if(floatabs(hip2_rawd) > TH_BOUND){
        //				right_k = 0.0;
        //			}

        // I2 = sin(phase[1]);

        phase[1] = phase[1] / 2.0 / PI;
        if (phase[1] < fai_Ep - fai_Er ||
            ((phase[1] > fai_Ep + fai_Ef) && (phase[1] < fai_Fp - fai_Fr)) ||
            phase[1] > (fai_Fp + fai_Ff)) {
          // printf("c1");
          I2 = 0;
        }
        else if ((phase[1] >= fai_Ep - fai_Er) && (phase[1] <= fai_Ep)) {
          // printf("c2");
          I2 = (a[0] * phase[1] + a[1]) * phase[1] + a[2];
        }
        else if ((phase[1] >= fai_Ep - fai_Er + 0.5) && phase[1] <= (fai_Ep + 0.5)) {
          // printf("c3");
          float phi = phase[1] - 0.5;
          I2        = -((a[0] * phi + a[1]) * phi + a[2]);
        }
        else if (phase[1] >= fai_Ep && (phase[1] <= fai_Ep + fai_Ef)) {
          // printf("c4");
          I2 = (b[0] * phase[1] + b[1]) * phase[1] + b[2];
        }
        else if ((phase[1] >= fai_Ep + 0.5) && (phase[1] <= fai_Ep + fai_Ef + 0.5)) {
          // printf("c5");
          float phi = phase[1] - 0.5;
          I2        = -((b[0] * phi + b[1]) * phi + b[2]);
        }

        I2 = I2 * right_k;

#ifdef ECON_R
        if (I2 > 0) {
          set_I_direction(2, -(I2 + 0.3));  //负是往前， 补偿右侧驱动器的减速器的摩擦力
        }
        else {
          set_I_direction(2, -(I2 - 0.3));  //负是往前，
        }
#endif

        AssisMonitor("I2 %.2f\t", I2);
      }
      //			INF("\r\n");
    }

    debug_assisTorque1 = 1000.0 * I1;
    debug_assisTorque2 = 1000.0 * I2;

    debug_hip1_d    = (int)hip1_d;
    debug_hip1_rawd = (int)hip1_rawd;
    debug_hip2_d    = (int)hip2_d;
    debug_hip2_rawd = (int)hip2_rawd;
    debug_hip1_w    = (int)hip1_w;
    debug_hip1_raww = (int)hip1_raww;
    debug_hip2_w    = (int)hip2_w;
    debug_hip2_raww = (int)hip2_raww;

    debug_AOphase1  = 1000.0 * hip1.phase[1];
    debug_AOpre1    = (int)hip1.predict;
    debug_AOoutput1 = (int)hip1.output;

    debug_AOphase2  = 1000.0 * hip2.phase[1];
    debug_AOpre2    = (int)hip2.predict;
    debug_AOoutput2 = (int)hip2.output;

    debug_AOw1        = 1000.0 * hip1.ww;
    debug_AOphasePre1 = 1000.0 * hip1.predictedBasicPhase;

    debug_AOw2        = 1000.0 * hip2.ww;
    debug_AOphasePre2 = 1000.0 * hip2.predictedBasicPhase;

    debug_phase1 = 1000 * phase[0];
    debug_phase2 = 1000 * phase[1];

    debug_tmp = 1000.0;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4) {
    inc++;
  }
}
