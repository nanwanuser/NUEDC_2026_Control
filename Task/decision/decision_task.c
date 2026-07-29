#include "decision_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include <string.h>

#define DECISION_TASK_PERIOD_MS 1U

volatile DecisionTaskRequest DecisionTask_Input;
volatile uint8_t DecisionTask_RequestPending;
volatile DecisionTaskOutput DecisionTask_Output;

static void copy_request(DecisionTaskRequest *request)
{
    taskENTER_CRITICAL();
    *request = DecisionTask_Input;
    DecisionTask_RequestPending = 0U;
    taskEXIT_CRITICAL();
}

static void publish_output(const DecisionTaskOutput *output)
{
    taskENTER_CRITICAL();
    DecisionTask_Output = *output;
    taskEXIT_CRITICAL();
}

void DecisionTask_Init(void)
{
    (void)memset((void *)&DecisionTask_Input, 0, sizeof(DecisionTask_Input));
    (void)memset((void *)&DecisionTask_Output, 0, sizeof(DecisionTask_Output));
    Decision_GetDefaultConfig((DecisionConfig *)&DecisionTask_Input.config);
    DecisionTask_Output.result = DECISION_RESULT_INVALID_ARGUMENT;
    DecisionTask_RequestPending = 0U;
}

uint8_t DecisionTask_Submit(const DecisionTaskRequest *request)
{
    if (request == NULL) {
        return 0U;
    }

    taskENTER_CRITICAL();
    DecisionTask_Input = *request;
    DecisionTask_RequestPending = 1U;
    taskEXIT_CRITICAL();
    return 1U;
}

void Decision_App(void *argument)
{
    DecisionTaskRequest request;
    DecisionTaskOutput output;

    (void)argument;
    for (;;) {
        if (DecisionTask_RequestPending != 0U) {
            copy_request(&request);
            (void)memset(&output, 0, sizeof(output));
            output.result = Decision_Solve(request.mode,
                                           &request.vision,
                                           &request.fixed_layout,
                                           &request.config,
                                           &output.plan);
            publish_output(&output);
        }
        osDelay(DECISION_TASK_PERIOD_MS);
    }
}
