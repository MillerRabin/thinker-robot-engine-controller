#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* ============================= */
/* RP2040 interrupt handlers     */
/* ============================= */

#define vPortSVCHandler isr_svcall
#define xPortPendSVHandler isr_pendsv
#define xPortSysTickHandler isr_systick

/* ============================= */
/* SMP CONFIG                    */
/* ============================= */

#define configNUM_CORES 2
#define configTICK_CORE 0
#define configUSE_CORE_AFFINITY 1
#define configRUN_MULTIPLE_PRIORITIES 1

/* ============================= */
/* Scheduler                     */
/* ============================= */

#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE 0

/* ============================= */
/* Hardware                      */
/* ============================= */

#define configCPU_CLOCK_HZ 133000000
#define configTICK_RATE_HZ 1000

/* ============================= */
/* Task config                   */
/* ============================= */

#define configMAX_PRIORITIES 8
#define configMINIMAL_STACK_SIZE 256
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1

/* ============================= */
/* Synchronization               */
/* ============================= */

#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 3

#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0

/* ============================= */
/* Memory                        */
/* ============================= */

#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE (192 * 1024)

/* ============================= */
/* Debug                         */
/* ============================= */

#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1

#define configASSERT(x)                                                        \
  if ((x) == 0) {                                                              \
    __asm volatile("bkpt #0");                                                 \
  }

/* ============================= */
/* Timers                        */
/* ============================= */

#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY 4
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 512

/* ============================= */
/* Optional features             */
/* ============================= */

#define configQUEUE_REGISTRY_SIZE 10
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5
#define configENABLE_BACKWARD_COMPATIBILITY 0

/* ============================= */
/* API inclusion                 */
/* ============================= */

#define INCLUDE_vTaskPrioritySet 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_vTaskSuspend 1
#define INCLUDE_xResumeFromISR 1
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_xTaskGetIdleTaskHandle 1
#define INCLUDE_eTaskGetState 1
#define INCLUDE_xTaskAbortDelay 1
#define INCLUDE_xTaskGetHandle 1
#define INCLUDE_xTaskResumeFromISR 1
#define INCLUDE_xTimerPendFunctionCall 1

#endif /* FREERTOS_CONFIG_H */