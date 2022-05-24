#include "conf_global.h"


/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  HAL_StatusTypeDef  ret = HAL_OK;
  RCC_OscInitTypeDef RCC_OscInitStructure;
  RCC_ClkInitTypeDef RCC_ClkInitStructure;

  RCC_OscInitStructure.OscillatorType = RCC_OSCILLATORTYPE_HSE;  //ʱ��ԴΪHSE
  RCC_OscInitStructure.HSEState       = RCC_HSE_ON;              //��HSE
  RCC_OscInitStructure.HSEPredivValue = RCC_HSE_PREDIV_DIV1;     // HSEԤ��Ƶ
  RCC_OscInitStructure.PLL.PLLState   = RCC_PLL_ON;              //��PLL
  RCC_OscInitStructure.PLL.PLLSource  = RCC_PLLSOURCE_HSE;  // PLLʱ��Դѡ��HSE
  RCC_OscInitStructure.PLL.PLLMUL     = RCC_PLL_MUL9;       //��PLL��Ƶ����
  ret = HAL_RCC_OscConfig(&RCC_OscInitStructure);           //��ʼ��

  if (ret != HAL_OK)
    while (1)
      ;

  //ѡ��PLL��Ϊϵͳʱ��Դ��������HCLK,PCLK1��PCLK2
  RCC_ClkInitStructure.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStructure.SYSCLKSource =
      RCC_SYSCLKSOURCE_PLLCLK;  //����ϵͳʱ��ʱ��ԴΪPLL
  RCC_ClkInitStructure.AHBCLKDivider  = RCC_SYSCLK_DIV1;  // AHB��Ƶϵ��Ϊ1
  RCC_ClkInitStructure.APB1CLKDivider = RCC_HCLK_DIV2;    // APB1��Ƶϵ��Ϊ2
  RCC_ClkInitStructure.APB2CLKDivider = RCC_HCLK_DIV1;    // APB2��Ƶϵ��Ϊ1
  ret                                 = HAL_RCC_ClockConfig(
      &RCC_ClkInitStructure,
      FLASH_LATENCY_2);  //ͬʱ����FLASH��ʱ����Ϊ2WS��Ҳ����3��CPU���ڡ�

  if (ret != HAL_OK)
    while (1)
      ;
}

void tick_init(uint32_t period_us)  // 1miu_s
{
  /**Configure the Systick interrupt time */
  HAL_SYSTICK_Config(SystemCoreClock / (1000000 / period_us));  // 10e5=1x10^6

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
 * Initializes the Global MSP.
 */
void NVIC_Init(void)
{
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);

  /* System interrupt init*/

  /*HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);

  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);

  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);

  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);

  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);*/
}

#ifndef PREFETCH_ENABLE
#define PREFETCH_ENABLE 1
#endif

void Flash_prefetch(void)
{
  /* Configure Flash prefetch */
#if (PREFETCH_ENABLE != 0)
#if defined(STM32F101x6) || defined(STM32F101xB) || defined(STM32F101xE) || \
    defined(STM32F101xG) || defined(STM32F102x6) || defined(STM32F102xB) || \
    defined(STM32F103x6) || defined(STM32F103xB) || defined(STM32F103xE) || \
    defined(STM32F103xG) || defined(STM32F105xC) || defined(STM32F107xC)

  /* Prefetch buffer is not available on value line devices */
  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
#endif
#endif /* PREFETCH_ENABLE */
}


void Core_Config(void)
{
  Flash_prefetch();
  NVIC_Init();
  tick_init(1000);

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_DISABLE();

  SystemClock_Config();
}
