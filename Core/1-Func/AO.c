#include "AO.h"

#define GAIN 5                     //最优学习参数增益，0.2~5
int             PREDICT_TIME = 4;  // 4*50ms ahead
extern uint16_t period[2];


uint64_t                    Aoindex = 0;
struct Adaptive_Oscillators hip1, hip2;
extern int                  CONTROL_PERIOD;
/**
 * @date   2022/4/6
 * @author lhx
 * @brief  AO结构体参数设置,简化版
 * @param
  T - 步态周期;
  va - 幅值学习参数
  vw - 频率学习参数
  vph - 相位学习参数
  order - 振荡器池中的振荡器个数，除了没有频率的offset振荡器
 (非零频率的联级振荡器数量,不包含频率=0),因此总共 order+1个振荡器
  dt - 时间间隔/采样间隔
  ww - 基频频率初值
  aa[] - 幅值初值,振荡器当前幅值order+1阶
  pphh[] - 相位初值,振荡器当前相位order+1阶 ,第一个 pphh[0] 强制为0
  step - 预测步长
 * @note   none
 */
void AO_Init(struct Adaptive_Oscillators* AO, float T)
{
  float va, vw, vph, dt;  // va - 幅值学习参数,vw - 频率学习参数,vph - 相位学习参数
  float aa[3] = {5.0, 10.0, 20.0}, pphh[3] = {0, PI / 3, PI / 6};
  int   step;
  int   order = 2;

  dt = (float)CONTROL_PERIOD * 2.0 /
       1000.0;  // 1/100 = 0   1.0/100 = 0.01 先按 int 类型计算，然后强制转化为 float
  va   = 2.0 / (GAIN * T);
  vw   = va;
  vph  = sqrt(24.2 * vw);
  step = 5;

  // set AO structure
  AO->va      = va;
  AO->vw      = vw;
  AO->vph     = vph;
  AO->order   = order;       // order+1 AOs
  AO->ww      = 3 * PI / T;  // basic frequency
  AO->init_ww = 3 * PI / T;
  AO->dt      = dt;
  AO->step    = step;  // predict step

  for (uint8_t i = 0; i < order + 1; i++) {
    AO->aa[i]        = aa[i];  // amplitude
    AO->init_aa[i]   = aa[i];
    AO->pphh[i]      = pphh[i];  // phase
    AO->init_pphh[i] = pphh[i];
    // TESTOUT("amplify[%d] = %.3f\r\n", i,aa[i]);
    // TESTOUT("phase[%d] = %.3f\r\n", i,pphh[i]);
  }
  // TESTOUT("inital w = %.3f\r\n",ww);

  for (uint8_t i = 0; i < AO->order + 1; i++) {
    /* w_real[0]=0; w_real[1]=w_now; w_real[2]=2*w_now ... */
    AO->w_real[i] = i * AO->ww;
  }

  //存储下标
  AO->index = 0;
}

/**
* @date   2022/4/7
* @author lhx
* @brief  AO结构体参数设置/配置
* @param
 va - 幅值学习参数
 vw - 频率学习参数
 vph - 相位学习参数
 order - 振荡器池中的振荡器个数，除了没有频率的offset振荡器，因此总共order+1个振荡器
 dt - 时间间隔/采样间隔
 ww - 基频频率初值
 aa[] - 幅值初值,振荡器当前幅值 order+1个阶数
 pphh[] - 相位初值,振荡器当前相位 order+1 ,第一个 ph_now[0] 强制为 0
 step - 预测步长
* @note   usually the special usage need to be noticed
*/
void AO_set(struct Adaptive_Oscillators* AO, float vw, float va, float vph, int order, float dt,
            float ww, float aa[], float pphh[], int step)
{
  AO->va      = va;  // learning parameter
  AO->vw      = vw;
  AO->vph     = vph;
  AO->order   = order;  // order+1 AOs
  AO->ww      = ww;     // basic frequency
  AO->init_ww = ww;
  AO->dt      = dt;
  AO->step    = step;  // predict step

  for (uint8_t i = 0; i < order + 1; i++) {
    AO->aa[i]        = aa[i];  // amplitude
    AO->init_aa[i]   = aa[i];
    AO->pphh[i]      = pphh[i];  // phase
    AO->init_pphh[i] = pphh[i];
    // TESTOUT("amplify[%d] = %.3f\r\n", i,aa[i]);
    // TESTOUT("phase[%d] = %.3f\r\n", i,pphh[i]);
  }
  // TESTOUT("inital w = %.3f\r\n",ww);

  for (uint8_t i = 0; i < AO->order + 1; i++) {
    // w_real[0]=0; w_real[1]=w_now; w_real[2]=2*w_now ...
    AO->w_real[i] = i * AO->ww;
  }

  //存储下标
  AO->index = 0;
}


void  Oscillators(struct Adaptive_Oscillators* AO, float e, int sync, float vw_sync);
float curvePredict(struct Adaptive_Oscillators* AO, float delta_t);
void  phasePredict(struct Adaptive_Oscillators* AO, float delta_t);
float fitIn(float data, float up, float down);
/**
 * @date   2022/4/6
 * @author lhx
 * @brief  AO迭代，实时输入关节角度，进行AO迭代
 * @param
    y_now -     关节角度
    t_now -     关节角度对应采集时间
    sync -      是否同步, 0-不同步, 1-同步
    vw_sync -   同步的学习参数
 * @note   AO_Iteration(AO, d, Aoindex, 0, 0);
 */
void AO_Iteration(struct Adaptive_Oscillators* AO, float y_now, float t_now, int sync,
                  float vw_sync)
{
  AO->output = curvePredict(AO, 0);
  float e    = y_now - AO->output;
  Oscillators(AO, e, sync, vw_sync);

  for (int i = 0; i < AO->order + 1; i++) {
    AO->phase[i] = AO->pphh[i];
    AO->phase[i] = fitIn(AO->phase[i], 2 * PI, 0);
  }

  AO->predict = curvePredict(AO, AO->step * AO->dt);  // step*2*CONTROL_PERIOD ms

  phasePredict(AO, PREDICT_TIME * AO->dt);  // predict 200 ms

  AO->outputSave[AO->index]        = AO->output;
  AO->predictedSaveData[AO->index] = AO->predict;

  if (++(AO->index) == MaxSize) {
    AO->index = 0;
  }
}


/**
 * @date   2022/4/6
 * @author lhx
 * @brief  fit data into a certain zone
 * @param  parameters definition
 *      data            input data
 *      up            	up bound
 *      down         	down bound
 * @note   fitIn(x,0,2PI)
 */
float fitIn(float data, float up, float down)
{
  float period = up - down;
  if (data <= up) {
    if (data >= down) {
      return data;
    }
    else {
      while (data < down) {
        data += period;
      }
    }
  }
  else {
    while (data > up) {
      data -= period;
    }
  }
  return data;
}

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  Oscillators differential function
 * @param
 *      e               外部输入-估计值
 *      sync            Adaptive Oscillators
 *      vw_sync
 * @note   it is called in AO_Iteration() function
  ref to Zheng E, Manca S, Yan T, et al. Gait Phase Estimation Based on Noncontact Capacitive Sensing and Adaptive Oscillators[J]. IEEE Transactions on Biomedical Engineering, 2017, 64(10): 2419–2430.
 */
void Oscillators(struct Adaptive_Oscillators* AO, float e, int sync, float vw_sync)
{
  float va = AO->va, vw = AO->vw, vph = AO->vph;
  float Dph, Da, Dw;
  float dt = AO->dt;
  float sum_a;

  sum_a = 0;
  for (int j = 0; j < AO->order + 1; j++) {
    sum_a += AO->aa[j];
  }

  for (int i = 0; i < AO->order + 1; i++) {
    if (i == 0) { /* w=0 */
      Da  = AO->va * e;
      Dph = 0;
    }
    else { /* w!=0 */
      Dph         = AO->w_real[i] + vph * e * cos(AO->pphh[i]) / sum_a;
      AO->pphh[i] = dt * Dph + AO->pphh[i];
      Da          = va * e * sin(AO->pphh[i]);
    }
    AO->aa[i] = dt * Da + AO->aa[i];

    if (i == 1 && sync == 0) {
      Dw     = vw * e * cos(AO->pphh[i]) / sum_a;
      AO->ww = dt * Dw + AO->ww;
      // TESTOUT("dw = %f",Dw);
    }
    else if (sync == 1)  // multi oscillators freq sync
      AO->ww = dt * vw_sync + AO->ww;
  }

  for (int i = 0; i < AO->order + 1; i++) {
    AO->w_real[i] = i * AO->ww;
  }
}


/**
 * @date   2022/4/7
 * @author lhx
 * @brief  curve value Predict using AO
 * @param
 delta_t	- Predict Time interval. It can maxmize independence of this function.
 AO - Adaptive Oscillators structure
 * @note  called in AO_Iteration() function
 */
float curvePredict(struct Adaptive_Oscillators* AO, float delta_t)
{
  int   order = AO->order;
  float y_pre = 0;

  for (int i = 0; i < order + 1; i++) {
    if (i == 0)
      y_pre = AO->aa[i];
    else
      y_pre += AO->aa[i] * sin(AO->w_real[i] * delta_t + AO->pphh[i]);
  }
  return y_pre;
}

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  AO basic phase prediction (phase[1] is corresponding to ww's phase)
 * @param
 *      delta_t	-	Phase Predict Time interval
 *      AO 		-	Adaptive Oscillators structure
 * @note   usually the special usage need to be noticed
 */
void phasePredict(struct Adaptive_Oscillators* AO, float delta_t)
{
  //	float delta_t = AO->step*AO->dt;
  AO->predictedBasicPhase = AO->phase[1] + AO->ww * delta_t;
  AO->predictedBasicPhase = fitIn(AO->predictedBasicPhase, 2 * 3.1416, 0);
}


/**
 * @date   2022/4/7
 * @author lhx
 * @brief  For showing AO structure information
 * @param  AO -	Adaptive Oscillators structure
 * @note   usually the special usage need to be noticed
 */
void show(struct Adaptive_Oscillators* AO)
{
  TESTOUT("\r\nva - %.2f\r\n", AO->va);
  TESTOUT("vw - %.2f\r\n", AO->vw);
  TESTOUT("vph - %.2f\r\n", AO->vph);
  TESTOUT("order - %d\r\n", AO->order);
  TESTOUT("dt - %.4f\r\n", AO->dt);
  TESTOUT("phase - %.2f\r\n", AO->phase[1]);
  TESTOUT("step - %d\r\n", AO->step);
  TESTOUT("ww = %.5f\r\n", AO->ww);
  TESTOUT("output - %.5f\r\n", AO->output);
  TESTOUT("predict = %.5f\r\n", AO->predict);

  for (int i = 0; i < AO->order + 1; i++) {
    TESTOUT("phi[%d] = %.5f\r\n", i, AO->pphh[i]);
  }

  for (int i = 0; i < AO->order + 1; i++) {
    TESTOUT("amplify[%d] = %.5f\r\n", i, AO->aa[i]);
  }

  for (int i = 0; i < AO->order + 1; i++) {
    TESTOUT("real w[%d] = %.5f\r\n", i, AO->w_real[i]);
  }
}


// -------------------*---------------------------*----------TEST---------*----------------------*-----------------//
// -------------------*---------------------------*----------TEST---------*----------------------*-----------------//

#define TEST_ON

#ifdef TEST_ON
/**
 * @date   2022/4/7
 * @author lhx
 * @brief  AO information configuration test
 * @param
 * @note
 */
void test_Osc(void)
{
  struct Adaptive_Oscillators hip;

  float va, vw, vph, dt;
  float aa[2] = {0.0, 5.0}, pphh[2] = {0, 0};
  int   step;
  float pi = 3.1415926;

  va   = 2 * pi / 5 * 5;  // 5T
  vw   = va;
  vph  = sqrt(24.2 * vw);
  dt   = 0.01;
  step = 10;

  AO_Init(&hip, 2);
  show(&hip);
  AO_set(&hip, vw, va, vph, 1, dt, 5, aa, pphh, step);
  show(&hip);
  Oscillators(&hip, 0.125, 0, 0);
  show(&hip);
}

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  AO prediction function test
 * @param
 * @note
 */
void test_Pre(void)
{
  float                       pre;
  struct Adaptive_Oscillators hip;
  float                       va, vw, vph, dt, aa[2] = {1.25, 3.523}, pphh[2] = {0, 0};
  int                         step;
  float                       pi = 3.1415926;

  va   = 2 * pi / 5 * 5;
  vw   = va;
  vph  = sqrt(24.2 * vw);
  dt   = 0.01;
  step = 10;

  AO_set(&hip, vw, va, vph, 1, dt, 4, aa, pphh, step);
  show(&hip);
  pre = curvePredict(&hip, 10);
  TESTOUT("pre = %.5f, truth value = 4.466306118313432", pre);

  (void)pre;  // avoid compiler warning
}

int test_phase, test_w, test_output, test_input, test_predict, test_phasePredict, test_j;  // jscope
/**
 * @date   2022/4/7
 * @author lhx
 * @brief  AO Iteration test for single sin function
 * @param
 * @note
 */
void test_AO(void)
{
  struct Adaptive_Oscillators hip;
  float                       va, vw, dt, T, aa[2] = {0.0, 15.0}, pphh[2] = {PI / 5.0, PI / 3.0};
  int                         step;

  TESTOUT("AO test for sine wave\r\n");
  dt   = 0.005;
  T    = 2 * PI / (23.0);
  va   = 2.0 / (T * 1.0);
  vw   = va;
  step = 2;

  AO_set(&hip, vw, va, sqrt(24.2 * vw), 1, dt, 15, aa, pphh, step);
  // show(&hip);

  float    y;
  uint64_t j = 0;
  while (j++ < 5000000000) {
    y = 30 * sin(23 * (j * dt) + 56) + 20;
    AO_Iteration(&hip, y, j, 0, 0);
    TESTOUT("y\t%.2f\tpredict\t%.2f\t%.2f\r\n", y, hip.predict, hip.phase[1]);
    // show(&hip);
    HAL_Delay(5);
    test_phase        = 1000.0 * hip.phase[1];
    test_w            = 1000.0 * hip.ww;
    test_output       = 1000.0 * hip.output;
    test_input        = 1000.0 * y;
    test_predict      = 1000.0 * hip.predict;
    test_phasePredict = 1000.0 * hip.predictedBasicPhase;
    test_j            = j % 100;
  }
}

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  AO Iteration test for multiple sin functions
 * @param
 * @note
 */
void test_AOs(void)
{
  struct Adaptive_Oscillators hip;
  float va, vw, dt, T, aa[3] = {0.0, 15.0, 1.0}, pphh[3] = {PI * 0.2, PI / 3.0, PI / 3.0};
  int   step;

  TESTOUT("AO test for sine wave\r\n");
  dt   = 0.005;
  T    = 2.0 * PI / 23.0;
  va   = 2.0 / (T * 1.0);
  vw   = va;
  step = 2;

  AO_set(&hip, vw, va, sqrt(24.2 * vw), 2, dt, 15, aa, pphh, step);
  // show(&hip);

  float    y;
  uint64_t j = 0;
  while (j++ < 5000000000) {
    y = 30 * sin(23 * j * dt + 56) + 20 + 10 * sin(35 * j * dt + 56);
    AO_Iteration(&hip, y, j, 0, 0);
    TESTOUT("y\t%.2f\tpredict\t%.2f\t%.2f\r\n", y, hip.predict, hip.phase[1]);
    // show(&hip);
    HAL_Delay(5);
    test_phase        = 1000.0 * hip.phase[1];
    test_w            = 1000.0 * hip.ww;
    test_output       = 1000.0 * hip.output;
    test_input        = 1000.0 * y;
    test_predict      = 1000.0 * hip.predict;
    test_phasePredict = 1000.0 * hip.predictedBasicPhase;
  }
}

#endif

// -------------------*---------------------------*--------AOPOswitching-------*----------------------*-----------------//
// -------------------*---------------------------*--------AOPOswitching-------*----------------------*-----------------//


#define SwitchMonitor(...)  // printf

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  find the minimum peak in a piece of time series data 关节角度最低值查找程序
 * @param
 WINp - windows structure contains a piece of time series data
 * @return 1 - this piece has minimum; 0 - this piece has NO minimum
 * @note   windows structure is defined in win.c
 */
uint8_t findpeak(WINp win)
{
  ElementType end1 = getValue(win, 1);
  ElementType end2 = getValue(win, 3);
  ElementType mid  = getValue(win, 2);

  if (mid < end1 && mid < end2 && mid < 5) {
    return 1;
  }
  return 0;
}


float floatabs(float x);
/**
 * @date   2022/4/7
 * @author lhx
 * @brief  判断两者数值是否相等
 * @param  i/j - 用于判断是否相等的数值，可以为AO预测的髋关节角度与实际的角度
 * @return 1 - equal; 0 - NOT equal
 * @note
 */
uint8_t Isequal(float i, float j)
{
  if (i == 0) i = i + 0.001;  //计算相对误差时除数不能为 0
  if (floatabs(i - j) < 8) {  //尝试绝对值，相对误差，最小二乘
    return 1;
  }
  return 0;
}


#define StableThreshold 5
/**
 * @date   2022/4/7
 * @author lhx
 * @brief  判断数组中的数据波动程度
 * @param
 float angle[] - 装载数据片段
 * @return
 0 - 波动巨大，超过 StableThreshold;
 1 - 波动微弱，小于 StableThreshold;
 * @note   未测试
 */
uint8_t Isstable(float angle[])
{
  for (uint8_t i = 0; i < MaxSize - 1; i++) {
    if (angle[i] - angle[i + 1] > StableThreshold) {
      return 0;
    }
  }
  return 1;
}


/**
 * @date   2022/4/7
 * @author lhx
 * @brief  取浮点数的绝对值，由于库中的abs只针对int类型，因此特别创建一个关于浮点数的
 * @param  x - float value
 * @return x - absolute x value
 * @note
 */
float floatabs(float x) { return x > 0 ? x : -x; }


#include "PO.h"
#define DELAY_TIME 15  //斯密特触发器的宽度
#define RESET_TIME 400  // AO reinit 判断条件

int32_t PO_time[2]         = {0};     // PO 运行时间长度度量，决定AO初始化
int32_t debug_preOffset[2] = {0};     // 决定AO初始化
int16_t delaySwitch[2]     = {0, 0};  // Schmitt trigger.当值>DELAY_TIME为AO模式,<0为PO模式
int8_t  assive[2]          = {POMODE, POMODE};  //助力模式,[0]左髋关节,[1]右髋关节

extern float   dt;
extern int16_t stopFlag[2];  // standing state
extern int     state[2];     // 1-walking, 0-stopping state
/**
 * @date   2022/4/7
 * @author lhx
 * @brief  不同助力模式的切换策略
 * @param  AO
 * d - 关节角度，用于输入到AO中进行迭代，PO助力模式时被PO用于计算相位
   w - 关节角速度，PO助力模式时被PO用于计算相位
 * node - 电机的编号，左为0，右为1
 * @return 助力模式
 * @note
 */
int8_t switch_task(struct Adaptive_Oscillators* AO, float d, float w, uint8_t node)
{
  int8_t i = AO->index;
  int8_t j;

  //站立情况下助力模式为PO
  if (stopFlag[node - 1] == 1) {
    assive[node - 1]      = POMODE;
    delaySwitch[node - 1] = 0;
    PO_time[node - 1] =
        RESET_TIME - 140;  // 重置PO 运行时间长度。站到走从非零开始，加快AO reset;
                           // 由于频繁走停时从站到走AO可能会立刻收敛，所以这里不直接对AO reset
    return assive[node - 1];
  }

  state[node - 1]           = 1;  // set to walking state
  j                         = i - 1 - AO->step < 0 ? i - 1 - AO->step + MaxSize
                                                   : i - 1 - AO->step;  // get prediction value index
  debug_preOffset[node - 1] = AO->predictedSaveData[j];                 // for debug
  // if(Isequal(d ,AO->predictedSaveData[j])&&(Isstable(AO->outputSave)==0)){//&& floatabs(w)>1)
  // AO预测是否成功
  if (Isequal(d, AO->predictedSaveData[j])) {
    delaySwitch[node - 1] = delaySwitch[node - 1] + 1;  // if succee, smith trigger+1
  }
  else {
    delaySwitch[node - 1] = delaySwitch[node - 1] - 1;
  }

  //根据触发器设置助力模式
  if (assive[node - 1] == POMODE && delaySwitch[node - 1] >= DELAY_TIME) {
    assive[node - 1] = AOMODE;
  }
  else if (assive[node - 1] == AOMODE && delaySwitch[node - 1] <= 0) {
    assive[node - 1] = POMODE;
  }

  // PO 运行时间计算
  if (assive[node - 1] == POMODE) {
    PO_time[node - 1] = PO_time[node - 1] + 1;
  }
  else if (assive[node - 1] == AOMODE) {
    PO_time[node - 1] = PO_time[node - 1] - 0;
  }

  // PO 运行时间达到RESET_TIME后进行 AO reset
  if (PO_time[node - 1] > RESET_TIME) {
    AO_Init(AO, period[node - 1] * dt);  // AO reset 	float w0 = 21*dt;
                                         // 21*dt
    PO_time[node - 1] = 0;               // AO reset time
  }
  else if (PO_time[node - 1] < 0) {
    PO_time[node - 1] = 0;
  }


  if (delaySwitch[node - 1] > DELAY_TIME) {
    delaySwitch[node - 1] = DELAY_TIME;
  }
  if (delaySwitch[node - 1] < 0) {
    delaySwitch[node - 1] = 0;
  }

  SwitchMonitor("PO_time\t%d\t", PO_time);
  SwitchMonitor("delaySwitch\t%d\t", delaySwitch);

  if (node == 1) {
    SwitchMonitor("preErr1\t%.1f\tassiveMode1\t%d\t", AO->predict_10steps_save[j] - d, assive);
  }
  if (node == 2) {
    SwitchMonitor("preErr2\t%.1f\tassiveMode2\t%d\t", AO->predict_10steps_save[j] - d, assive);
  }

  return assive[node - 1];
}

/**
 * @date   2022/4/7
 * @author lhx
 * @brief  利用P控制提供助力将下肢关节角度控制AO预测的未来角度
 * @param  AO
   d - IMU采集的关节真实角度 degree
 * @return
 * @note   usually the special usage need to be noticed
 */
float assive_torque(struct Adaptive_Oscillators* AO, float d)
{
  float assistive_torque;
  if (floatabs(AO->predict - d) < 3)  //当两者差距很小时，不提供助力
    assistive_torque = 0;
  else {
    assistive_torque = (AO->predict - d) * 0.05;
  }
  return assistive_torque;
}
