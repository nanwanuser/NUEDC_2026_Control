#ifndef IMU_TASK_H
#define IMU_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief FreeRTOS IMU task entry; updates the IMU and queues large Yaw compensation at 100 Hz.
 * @param argument Unused task argument.
 */
void IMU_Task_App(void *argument);

#ifdef __cplusplus
}
#endif

#endif
