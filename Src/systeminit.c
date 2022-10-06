#include <stdint.h>
#include "system_stm32f4xx.h"
#include "stm32f4xx.h"
#include "Helpers/logger.h"

LogLevel system_log_level = LOG_LEVEL_DEBUG;
uint32_t SystemCoreClock = 72000000;

// HCLK = 72 MHz
// PLL: M = 4, N = 72, P = 1, Q = 3
// AHB prescaler = 1
// APB1 prescaler = 2
// APB2 prescaler = 1
// MCO1 prescaler = 2
static void configure_clock()
{
	// Modify the flash latency before setting the core clock or we may endup with
	// undefined behaviors.
	MODIFY_REG(FLASH->ACR,
			   FLASH_ACR_LATENCY,
			   _VAL2FLD(FLASH_ACR_LATENCY, 2));

	// Enables HSE
	SET_BIT(RCC->CR, RCC_CR_HSEON);

	// waits until HSE is stable
	while(!READ_BIT(RCC->CR, RCC_CR_HSERDY));

	// Configures PLL: source = HSE, SYSCLK = 72MHz
	MODIFY_REG(RCC->PLLCFGR,
			   RCC_PLLCFGR_PLLM |
			   RCC_PLLCFGR_PLLN |
			   RCC_PLLCFGR_PLLP |
			   RCC_PLLCFGR_PLLSRC |
			   RCC_PLLCFGR_PLLQ,
			   _VAL2FLD(RCC_PLLCFGR_PLLM, 4) |
			   _VAL2FLD(RCC_PLLCFGR_PLLN, 72) |
			   _VAL2FLD(RCC_PLLCFGR_PLLSRC, 1) |
			   _VAL2FLD(RCC_PLLCFGR_PLLQ, 3));

	// Enables PLL module
	SET_BIT(RCC->CR, RCC_CR_PLLON);

	// Waits until PLL is stable
	while (!READ_BIT(RCC->CR, RCC_CR_PLLRDY));

	// Switches system clock to PLL
	MODIFY_REG(RCC->CFGR,
			   RCC_CFGR_SW,
			   RCC_CFGR_SW_PLL);

	// Configures PPRE1
	MODIFY_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_DIV2);

	// Waits until PLL is used
	while(READ_BIT(RCC->CFGR, RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

	// Disable HSI
	CLEAR_BIT(RCC->CR, RCC_CR_HSION);
}

static void configure_mco1()
{
	// Configures MCO1: source = PLLCLK, MCO1PRE = 2.
	MODIFY_REG(RCC->CFGR,
               RCC_CFGR_MCO1 | RCC_CFGR_MCO1PRE,
               _VAL2FLD(RCC_CFGR_MCO1, 3) | _VAL2FLD(RCC_CFGR_MCO1PRE, 4));

	// Enables GPIOA (MCO1 is connected to PA8).
	SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN);

	// Configures PA8 as medium speed.
	MODIFY_REG(GPIOA->OSPEEDR,
               GPIO_OSPEEDR_OSPEED8,
               _VAL2FLD(GPIO_OSPEEDR_OSPEED8, 1));

	// Configures PA8 to work in alternate function mode.
	MODIFY_REG(GPIOA->MODER,
              GPIO_MODER_MODER8,
              _VAL2FLD(GPIO_MODER_MODER8, 2));
}

void SystemInit(void)
{
	// configure_mco1();
	configure_clock();
}
