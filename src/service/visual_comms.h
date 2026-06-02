#ifndef _visual_comms_h_
#define _visual_comms_h_

#include <stdbool.h>

#include "main.h" // IWYU pragma: keep

// ! ========================= 接 口 变 量 / Typedef 声 明 ========================= ! //

/**
 * @brief 视觉通信服务接口单例别名，业务代码可通过 `visual_comms.xxx(...)` 调用
 */
#define visual_comms visual_comms_interface

/**
 * @brief 视觉通信服务状态码表
 * @param OK 操作成功
 * @param INVALID_PARAM 输入参数无效
 * @param NOT_INITIALIZED 服务尚未初始化
 * @param RING_BUFFER_ERROR 接收环形缓冲区操作失败
 * @param UART_ERROR 串口收发或启动接收失败
 * @param FRAME_ERROR 接收到的协议帧格式错误
 */
#define VISUAL_COMMS_STATUS_TABLE             \
    X(OK, "OK")                               \
    X(INVALID_PARAM, "Invalid Parameter")     \
    X(NOT_INITIALIZED, "Not Initialized")     \
    X(RING_BUFFER_ERROR, "Ring Buffer Error") \
    X(UART_ERROR, "UART Error")               \
    X(FRAME_ERROR, "Frame Error")

/**
 * @brief 视觉通信服务状态码
 */
#define X(name, str) VISUAL_COMMS_##name,
typedef enum {
    VISUAL_COMMS_STATUS_TABLE
} VisualCommsStatus;
#undef X

/**
 * @brief 上位机控制命令类型
 * @param VISUAL_CMD_NONE 当前无待处理命令
 * @param VISUAL_CMD_START_RECOGNIZE_A 请求启动 A 区识别
 * @param VISUAL_CMD_START_RECOGNIZE_B 请求启动 B 区识别
 */
typedef enum {
    VISUAL_CMD_NONE = 0,
    VISUAL_CMD_START_RECOGNIZE_A = 1,
    VISUAL_CMD_START_RECOGNIZE_B = 2,
} VisualCommand;

/**
 * @brief 视觉下发的三维坐标
 *
 * 坐标帧格式为：
 * - 0~1 字节：帧头 `0xBB 0x66`
 * - 2~5 字节：x，小端 4 字节浮点
 * - 6~9 字节：y，小端 4 字节浮点
 * - 10~13 字节：z，小端 4 字节浮点
 * - 14~15 字节：帧尾 `0x66 0xBB`
 */
typedef struct {
    float x;
    float y;
    float z;
} VisualCoordinate;

/**
 * @brief 视觉通信服务只读状态快照
 * @param pending_command 尚未被上层取走的命令
 * @param current_command 最近一次被上层确认消费的命令
 * @param latest_coordinate 最近一次成功解析的坐标
 * @param has_new_coordinate 是否存在尚未被上层取走的新坐标
 * @param initialized 服务是否已经初始化完成
 */
typedef struct {
    VisualCommand pending_command;
    VisualCommand current_command;
    VisualCoordinate latest_coordinate;
    bool has_new_coordinate;
    bool initialized;
} VisualComms;

/**
 * @brief 视觉通信服务统一接口表
 */
extern const struct VisualCommsInterface {
#define X(name, str) VisualCommsStatus name;
    struct {
        VISUAL_COMMS_STATUS_TABLE
    };
#undef X

    /**
     * @brief 初始化视觉通信服务
     *
     * 该函数会完成内部状态清零、接收环形缓冲区初始化、UART 回调注册，
     * 并启动 `UART10` 单字节中断接收
     *
     * @param huart 视觉模块使用的串口句柄，当前预期传入 `&huart10`
     * @return VisualCommsStatus 状态码
     */
    VisualCommsStatus (*init)(UART_HandleTypeDef* huart);

    /**
     * @brief 处理当前已接收的视觉串口数据
     *
     * 该函数会从内部环形缓冲区取出字节流，按协议解析控制帧和坐标帧；
     * 建议在主循环中持续调用
     *
     * @return VisualCommsStatus 状态码
     */
    VisualCommsStatus (*process)(void);

    /**
     * @brief 取走一条待处理控制命令
     *
     * 成功取走后，内部 `pending_command` 会被清空，并更新 `current_command`
     *
     * @param command 输出参数，用于接收命令类型
     * @return true 成功取到一条新命令
     * @return false 当前无新命令或参数无效
     */
    bool (*consume_command)(VisualCommand* command);

    /**
     * @brief 取走一组新坐标
     *
     * 成功取走后，内部 `has_new_coordinate` 标志会被清除
     *
     * @param coordinate 输出参数，用于接收坐标值
     * @return true 成功取到一组新坐标
     * @return false 当前无新坐标或参数无效
     */
    bool (*consume_coordinate)(VisualCoordinate* coordinate);

    /**
     * @brief 发送坐标接收应答
     *
     * 协议固定为 8 字节控制/应答帧：
     * - 成功：`AA 55 02 00 00 00 55 AA`
     * - 失败：`AA 55 02 01 00 00 55 AA`
     *
     * @param success true 发送成功应答，false 发送失败应答
     * @return VisualCommsStatus 状态码
     */
    VisualCommsStatus (*send_coordinate_ack)(bool success);

    /**
     * @brief 发送任务结束通知
     *
     * 协议固定帧为：`AA 55 03 00 00 00 55 AA`
     *
     * @return VisualCommsStatus 状态码
     */
    VisualCommsStatus (*send_end_task)(void);

    /**
     * @brief 查询视觉通信服务是否已经完成初始化
     * @return true 服务可用
     * @return false 服务尚未初始化
     */
    bool (*is_ready)(void);

    /**
     * @brief 获取视觉通信服务只读状态视图
     * @return const VisualComms* 只读状态快照指针
     */
    const VisualComms* (*get_state)(void);

    /**
     * @brief 将视觉通信服务状态码转换为静态字符串
     * @param status 状态码
     * @return const char* 状态码名称
     */
    const char* (*status_str)(VisualCommsStatus status);
} visual_comms_interface;

// ! ========================= 接 口 函 数 声 明 ========================= ! //

/**
 * @brief 初始化视觉通信服务
 * @param huart 视觉模块使用的串口句柄，当前预期传入 `&huart10`
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_init(UART_HandleTypeDef* huart);

/**
 * @brief 处理当前已接收的视觉串口数据
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_process(void);

/**
 * @brief 取走一条待处理控制命令
 * @param command 输出参数，用于接收命令类型
 * @return true 成功取到一条新命令
 * @return false 当前无新命令或参数无效
 */
bool visual_comms_consume_command(VisualCommand* command);

/**
 * @brief 取走一组新坐标
 * @param coordinate 输出参数，用于接收坐标值
 * @return true 成功取到一组新坐标
 * @return false 当前无新坐标或参数无效
 */
bool visual_comms_consume_coordinate(VisualCoordinate* coordinate);

/**
 * @brief 发送坐标接收应答
 * @param success true 发送成功应答，false 发送失败应答
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_send_coordinate_ack(bool success);

/**
 * @brief 发送任务结束通知
 * @return VisualCommsStatus 状态码
 */
VisualCommsStatus visual_comms_send_end_task(void);

/**
 * @brief 查询视觉通信服务是否已经完成初始化
 * @return true 服务可用
 * @return false 服务尚未初始化
 */
bool visual_comms_is_ready(void);

/**
 * @brief 获取视觉通信服务只读状态视图
 * @return const VisualComms* 只读状态快照指针
 */
const VisualComms* visual_comms_get_state(void);

/**
 * @brief 将视觉通信服务状态码转换为静态字符串
 * @param status 状态码
 * @return const char* 状态码名称
 */
const char* visual_comms_status_str(VisualCommsStatus status);

#endif
