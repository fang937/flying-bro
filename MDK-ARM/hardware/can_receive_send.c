#include "can_receive_send.h"
#include "can.h"
#include "tim.h"
#include "servo.h"

#define ACTION_TIME_MS       500U
#define CAN_MAX_RX_FRAMES_PER_LOOP  8U
#define CAN_RECOVERY_INTERVAL_MS    100U

/* 舵机角度，根据实际机械结构调整 */
#define FIRE_HOME_ANGLE      90.0
#define FIRE_ACTIVE_ANGLE    180.0

#define RELOAD_SERVO_COUNT        6U
#define RELOAD_MIN_ACTIVE_COUNT   4U
#define RELOAD_MAX_ACTIVE_COUNT   6U
#define MAX_FIRE_COUNT            3U

/*
 * 六个换弹舵机分别配置归位角和动作角。
 * 数组下标与 reload_servos[] 中的舵机顺序一致。
 */
static const double reload_home_angles[RELOAD_SERVO_COUNT] =
{
    90.0, 90.0, 90.0, 90.0, 90.0, 90.0
};

static const double reload_active_angles[RELOAD_SERVO_COUNT] =
{
    150.0, 160.0, 170.0, 180.0, 160.0, 150.0
};

typedef enum
{
    ACTION_NONE = 0,
    ACTION_FIRE,
    ACTION_RELOAD
} ActionType;

typedef struct
{
    TIM_HandleTypeDef *timer;
    uint32_t channel;
} ServoChannel;

typedef struct
{
    uint8_t valid;
    uint8_t sequence;
    uint8_t state;
    uint8_t command;
    uint8_t error;
} PendingStatus;

/* 发射舵机：TIM1_CH1 */
static ServoChannel fire_servo =
{
    &htim1,
    TIM_CHANNEL_1
};

/* 六个换弹舵机 */
static ServoChannel reload_servos[RELOAD_SERVO_COUNT] =
{
    {&htim1, TIM_CHANNEL_2},
    {&htim1, TIM_CHANNEL_3},
    {&htim1, TIM_CHANNEL_4},
    {&htim8, TIM_CHANNEL_1},
    {&htim8, TIM_CHANNEL_2},
    {&htim8, TIM_CHANNEL_3}
};

static ActionType current_action = ACTION_NONE;
static uint8_t current_command = 0U;
static uint8_t current_sequence = 0U;
static uint8_t current_state = STATE_IDLE;
static uint8_t fire_count = 0U;
static uint8_t reload_servo_count = 0U;
static uint32_t action_start_tick = 0U;
static uint32_t can_recovery_tick = 0U;
static PendingStatus pending_status = {0};

static HAL_StatusTypeDef CAN_ConfigFilterAndStart(void)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /*
     * 标准 ID 放在 32 位过滤器的高 16 位。
     * 32 位精确掩码避免 16 位模式下产生隐含的全匹配过滤器。
     */
    filter.FilterIdHigh = CAN_RX_ID << 5;
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0x7FFU << 5;
    filter.FilterMaskIdLow = 0U;

    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_CAN_Start(&hcan1);
}

static void CAN_TrySendPendingStatus(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};
    uint32_t mailbox;

    if (pending_status.valid == 0U ||
        HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_LISTENING ||
        HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        return;
    }

    tx_header.StdId = CAN_TX_ID;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;
// 用于发送状态信息的 CAN 帧数据格式
// 顺序为序列号、状态、命令、错误码，后续字节保留为 0
    data[0] = pending_status.sequence;
    data[1] = pending_status.state;
    data[2] = pending_status.command;
    data[3] = pending_status.error;

    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox) == HAL_OK)
    {
        pending_status.valid = 0U;
    }
}

static void CAN_SendStatus(uint8_t sequence,
                           uint8_t state,
                           uint8_t command,
                           uint8_t error)
{
    pending_status.sequence = sequence;
    pending_status.state = state;
    pending_status.command = command;
    pending_status.error = error;
    pending_status.valid = 1U;

    CAN_TrySendPendingStatus();
}

static void CAN_RecoverIfNeeded(void)
{
    uint32_t error;
    uint32_t now;

    error = HAL_CAN_GetError(&hcan1);
    if (HAL_CAN_GetState(&hcan1) != HAL_CAN_STATE_ERROR &&
        (error & HAL_CAN_ERROR_BOF) == 0U)
    {
        return;
    }

    now = HAL_GetTick();
    if ((now - can_recovery_tick) < CAN_RECOVERY_INTERVAL_MS)
    {
        return;
    }

    can_recovery_tick = now;

    /*
     * DeInit 会清除错误状态和 CAN 外设配置，随后重新初始化并恢复过滤器。
     */
    if (HAL_CAN_DeInit(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    MX_CAN1_Init();

    if (CAN_ConfigFilterAndStart() != HAL_OK)
    {
        Error_Handler();
    }

    CAN_TrySendPendingStatus();
}

static void SetAllServosHome(void)
{
    uint8_t i;

    servo_180_setangle(fire_servo.timer,
                       fire_servo.channel,
                       FIRE_HOME_ANGLE);

    for (i = 0U; i < RELOAD_SERVO_COUNT; i++)
    {
        servo_180_setangle(reload_servos[i].timer,
                           reload_servos[i].channel,
                           reload_home_angles[i]);
    }
}

static void StartFire(uint8_t sequence)
{
    current_action = ACTION_FIRE;
    current_command = CMD_FIRE;
    current_sequence = sequence;
    current_state = STATE_RUNNING;
    action_start_tick = HAL_GetTick();

    servo_180_setangle(fire_servo.timer,
                       fire_servo.channel,
                       FIRE_ACTIVE_ANGLE);

    CAN_SendStatus(sequence,
                   STATE_RUNNING,
                   CMD_FIRE,
                   ERR_NONE);
}

static void StartReload(uint8_t sequence)
{
    uint8_t i;

    if (fire_count == 0U)
    {
        CAN_SendStatus(sequence,
                       STATE_ERROR,
                       CMD_RELOAD,
                       ERR_SEQUENCE);
        return;
    }

    /*
     * fire_count 表示已经完成的发射次数。
     * 1 次发射后换弹 -> 4 个舵机
     * 2 次发射后换弹 -> 5 个舵机
     * 3 次发射后换弹 -> 6 个舵机
     */
    reload_servo_count = RELOAD_MIN_ACTIVE_COUNT + (fire_count - 1U);

    if (reload_servo_count > RELOAD_MAX_ACTIVE_COUNT)
    {
        reload_servo_count = RELOAD_MAX_ACTIVE_COUNT;
    }

    current_action = ACTION_RELOAD;
    current_command = CMD_RELOAD;
    current_sequence = sequence;
    current_state = STATE_RUNNING;
    action_start_tick = HAL_GetTick();

    for (i = 0U; i < reload_servo_count; i++)
    {
        servo_180_setangle(reload_servos[i].timer,
                           reload_servos[i].channel,
                           reload_active_angles[i]);
    }

    CAN_SendStatus(sequence,
                   STATE_RUNNING,
                   CMD_RELOAD,
                   ERR_NONE);
}

static void StopAction(uint8_t sequence)
{
    SetAllServosHome();

    current_action = ACTION_NONE;
    current_command = CMD_STOP;
    current_sequence = sequence;
    current_state = STATE_IDLE;
    fire_count = 0U;
    reload_servo_count = 0U;

    CAN_SendStatus(sequence,
                   STATE_FINISHED,
                   CMD_STOP,
                   ERR_NONE);
}

static void HandleCommand(const CAN_RxHeaderTypeDef *rx_header,
                          uint8_t *data)
{
    uint8_t command;
    uint8_t sequence;

    if (rx_header->IDE != CAN_ID_STD ||
        rx_header->RTR != CAN_RTR_DATA ||
        rx_header->DLC < 2U)
    {
        return;
    }

    command = data[0];
    sequence = data[1];

    if (command == CMD_STOP)
    {
        StopAction(sequence);
        return;
    }

    if (command == CMD_QUERY)
    {
        CAN_SendStatus(sequence,
                       current_state,
                       current_command,
                       ERR_NONE);
        return;
    }

    if (current_action != ACTION_NONE)
    {
        CAN_SendStatus(sequence,
                       STATE_ERROR,
                       command,
                       ERR_BUSY);
        return;
    }

    switch (command)
    {
        case CMD_FIRE:
            StartFire(sequence);
            break;

        case CMD_RELOAD:
            StartReload(sequence);
            break;

        default:
            CAN_SendStatus(sequence,
                           STATE_ERROR,
                           command,
                           ERR_BAD_CMD);
            break;
    }
}

void CAN_App_Init(void)
{
    if (CAN_ConfigFilterAndStart() != HAL_OK)
    {
        Error_Handler();
    }

    current_action = ACTION_NONE;
    current_state = STATE_IDLE;
    current_command = 0U;
    fire_count = 0U;
    reload_servo_count = 0U;
    can_recovery_tick = HAL_GetTick();
    pending_status.valid = 0U;

    SetAllServosHome();
}

void CAN_App_Process(void)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t data[8];
    uint32_t elapsed;
    uint32_t processed_frames = 0U;

    CAN_RecoverIfNeeded();
    CAN_TrySendPendingStatus();

    while (processed_frames < CAN_MAX_RX_FRAMES_PER_LOOP &&
           HAL_CAN_GetRxFifoFillLevel(&hcan1,
                                      CAN_RX_FIFO0) > 0U)
    {
        processed_frames++;

        if (HAL_CAN_GetRxMessage(&hcan1,
                                 CAN_RX_FIFO0,
                                 &rx_header,
                                 data) == HAL_OK)
        {
            if (rx_header.StdId == CAN_RX_ID)
            {
                HandleCommand(&rx_header, data);
            }
        }
        else
        {
            break;
        }
    }

    CAN_TrySendPendingStatus();

    if (current_action == ACTION_NONE)
    {
        return;
    }

    elapsed = HAL_GetTick() - action_start_tick;

    if (elapsed < ACTION_TIME_MS)
    {
        return;
    }

    if (current_action == ACTION_FIRE)
    {
        servo_180_setangle(fire_servo.timer,
                           fire_servo.channel,
                           FIRE_HOME_ANGLE);

        fire_count++;

        if (fire_count > MAX_FIRE_COUNT)
        {
            fire_count = MAX_FIRE_COUNT;
        }
    }
    else if (current_action == ACTION_RELOAD)
    {
        uint8_t i;

        for (i = 0U; i < reload_servo_count; i++)
        {
            servo_180_setangle(reload_servos[i].timer,
                               reload_servos[i].channel,
                               reload_home_angles[i]);
        }

        if (fire_count >= MAX_FIRE_COUNT)
        {
            fire_count = 0U;
        }
    }

    current_action = ACTION_NONE;
    current_state = STATE_FINISHED;

    CAN_SendStatus(current_sequence,
                   STATE_FINISHED,
                   current_command,
                   ERR_NONE);
}
