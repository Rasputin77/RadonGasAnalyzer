/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_threadx.c
  * @author  MCD Application Team
  * @brief   ThreadX applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_threadx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CDC_THREAD_STACK_SIZE 2048
#define CDC_THREAD_PRIORITY 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
TX_THREAD cdc_thread;
UCHAR cdc_thread_stack[CDC_THREAD_STACK_SIZE];
extern UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
//UX_DEVICE_CLASS_CDC_ACM *cdc_acm;  // dichiarato nel file app_usbx_device.c
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void cdc_thread_entry(ULONG arg);
/* USER CODE END PFP */

/**
  * @brief  Application ThreadX Initialization.
  * @param memory_ptr: memory pointer
  * @retval int
  */
UINT App_ThreadX_Init(VOID *memory_ptr)
{
  UINT ret = TX_SUCCESS;
  /* USER CODE BEGIN App_ThreadX_MEM_POOL */

  /* USER CODE END App_ThreadX_MEM_POOL */
  /* USER CODE BEGIN App_ThreadX_Init */

  TX_BYTE_POOL *byte_pool = (TX_BYTE_POOL*)memory_ptr;

      /* 1. Alloco lo stack del thread */
      ret = tx_byte_allocate(byte_pool,
                             (VOID **)&cdc_thread_stack,
                             CDC_THREAD_STACK_SIZE,
                             TX_NO_WAIT);

  if (ret != TX_SUCCESS)
	  return ret;

  /* 2. Creo il thread */
  ret = tx_thread_create(&cdc_thread,
						 "CDC Thread",
						 cdc_thread_entry,  // �? La tua funzione!
						 0,                 // arg
						 cdc_thread_stack,
						 CDC_THREAD_STACK_SIZE,
						 CDC_THREAD_PRIORITY,
						 CDC_THREAD_PRIORITY,
						 TX_NO_TIME_SLICE,
						 TX_AUTO_START);     // Parte subito

  if (ret != TX_SUCCESS)
	  return ret;

  return TX_SUCCESS;

  /* USER CODE END App_ThreadX_Init */

  return ret;
}

  /**
  * @brief  Function that implements the kernel's initialization.
  * @param  None
  * @retval None
  */
void MX_ThreadX_Init(void)
{
  /* USER CODE BEGIN Before_Kernel_Start */

  /* USER CODE END Before_Kernel_Start */

  tx_kernel_enter();

  /* USER CODE BEGIN Kernel_Start_Error */

  /* USER CODE END Kernel_Start_Error */
}

/* USER CODE BEGIN 1 */

void cdc_thread_entry(ULONG arg)
{
    while (1)
    {
        if (cdc_acm != UX_NULL)
        {
            UCHAR msg[] = "Ciao da U585 USBX + ThreadX!\r\n";
            ULONG sent;

            ux_device_class_cdc_acm_write(cdc_acm,
                                          msg,
                                          sizeof(msg)-1,
                                          &sent);
        }

        tx_thread_sleep(100); // ogni 100 ms
    }
}

/* USER CODE END 1 */
