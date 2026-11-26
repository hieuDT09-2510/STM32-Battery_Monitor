/*
 * 文件名称：bsp_i2c2.h
 *
 * 模块名称：STM32F103 I2C2板级驱动接口
 *
 * 模块职责：
 * 1. 配置PB10(SCL)、PB11(SDA)和I2C2外设；
 * 2. 使用7位从机地址，完成“8位寄存器地址 + 16位寄存器数据”的读写；
 * 3. 为一次完整事务设置总超时，防止传感器或线路异常永久卡住任务；
 * 4. 对被从设备拉住的总线产生恢复时钟并重新初始化I2C2；
 * 5. 保存成功、NACK、超时、总线忙和总线恢复等诊断数据。
 *
 * 数据流：
 * INA226驱动提供：7位设备地址、8位寄存器地址、待写值/接收地址
 *                              ↓
 * 本模块按照STM32F1 I2C时序操作PB10、PB11和I2C2寄存器
 *                              ↓
 * INA226驱动得到：16位原始寄存器值或明确的失败状态
 *
 * 模块边界：
 * - 本模块只认识I2C地址、寄存器地址和原始16位数据；
 * - 本模块不知道0x01代表分流电压，也不知道寄存器值如何换算成mA；
 * - 本模块不发布电流快照、不估算SOC，也不判断BMS故障；
 * - 当前仅由INA226驱动调用，但接口没有写死INA226设备地址。
 *
 * 执行上下文：
 * 当前接口由CurrentTask调用，采用轮询方式完成一次事务。
 * 函数内部有有限超时，但调用期间CurrentTask仍占用CPU，
 * 因此不能在中断服务函数中调用这些接口。
 */

/*
 * 头文件保护宏。
 * 防止同一个.c文件通过不同包含路径多次包含本头文件，
 * 从而造成类型和函数声明重复定义。
 */
#ifndef BSP_I2C2_H
#define BSP_I2C2_H

/* 提供uint8_t、uint16_t和uint32_t等定长无符号整数类型。 */
#include <stdint.h>

/*
 * 一次I2C事务的执行结果。
 *
 * 上层不能只得到“成功/失败”两个结果，因为不同错误对应不同排查方向：
 * NACK通常检查地址、供电或接线；BUS_BUSY通常检查SDA是否被拉低；
 * TIMEOUT表示某个预期硬件标志在限定时间内没有出现。
 */
typedef enum
{
    /* 完整事务按预期结束，写入已经发送或读取值已经复制给调用者。 */
    I2C2_STATUS_OK = 0,

    /*
     * 硬件AF(Acknowledge Failure)位置位。
     * 可能是从机地址无人应答，也可能是从机拒绝接收某个数据字节。
     */
    I2C2_STATUS_NACK,

    /* 等待SB、ADDR、TXE或BTF等硬件标志超过本模块设定的事务期限。 */
    I2C2_STATUS_TIMEOUT,

    /* 新事务开始前SR2.BUSY持续为1，说明I2C总线没有回到空闲状态。 */
    I2C2_STATUS_BUS_BUSY,

    /* 检测到BERR、ARLO、OVR或硬件TIMEOUT等I2C控制器错误标志。 */
    I2C2_STATUS_BUS_ERROR,

    /* 发送恢复时钟并重新初始化后，SCL/SDA仍为低或硬件仍报告BUSY。 */
    I2C2_STATUS_RECOVERY_FAILED,

    /* 调用者传入了空输出指针，或7位地址超出0x00～0x7F范围。 */
    I2C2_STATUS_INVALID_ARGUMENT
} I2C2_Status_t;

/*
 * I2C2底层运行诊断快照。
 *
 * 所有count字段都是从本次上电初始化后开始累计的次数；
 * 计数达到UINT32最大值后保持最大值，不允许回绕到0。
 * CLI的current命令读取该结构，用来区分器件问题、线路问题和时序问题。
 */
typedef struct
{
    uint32_t transfer_ok_count;   /* 成功完成的16位寄存器读写事务总数。 */
    uint32_t transfer_fail_count; /* 未成功完成的读写事务总数，不含单独调用的恢复结果。 */
    uint32_t nack_count;          /* 失败事务中，返回I2C2_STATUS_NACK的次数。 */
    uint32_t timeout_count;       /* 失败事务中，软件总超时到期的次数。 */
    uint32_t bus_busy_count;      /* 失败事务中，开始前总线一直忙的次数。 */
    uint32_t bus_error_count;     /* 失败事务中，控制器报告硬件错误的次数。 */
    uint32_t recovery_ok_count;   /* 手动恢复后SCL、SDA和BUSY检查均正常的次数。 */
    uint32_t recovery_fail_count; /* 手动恢复结束后总线仍不正常的次数。 */
    uint32_t last_sr1;            /* 最近一次事务结束时I2C2->SR1的原始快照。 */
    uint32_t last_sr2;            /* 最近一次事务结束时I2C2->SR2的原始快照。 */
    uint8_t last_status;          /* 最近一次事务/恢复结果，数值对应I2C2_Status_t。 */
} I2C2_Diagnostics_t;

/*
 * 初始化I2C2板级驱动。
 *
 * 作用：
 * 1. 打开GPIOB和I2C2外设时钟；
 * 2. 配置PB10/PB11为复用开漏并释放两条总线；
 * 3. 按项目固定的PCLK1=36MHz配置CCR=180、TRISE=37；
 * 4. 启动100kHz标准模式I2C2。
 *
 * 诊断结构是静态存储对象，上电时由C运行库自动清零。本函数重新
 * 初始化硬件时不会清空诊断历史，因此总线恢复后仍可查看此前错误。
 *
 * 调用位置：CurrentTask第一次运行时，在访问INA226之前调用一次。
 * BSP_I2C2_RecoverBus完成GPIO恢复时序后也会调用本函数重新接管总线。
 */
void BSP_I2C2_Init(void);

/*
 * 向从设备的一个8位寄存器地址写入16位数据。
 *
 * address_7bit：I2C标准7位设备地址，不包含最低位的读写方向位；
 *               INA226当前地址为0x44。
 * register_address：器件内部8位寄存器地址，例如INA226配置寄存器0x00。
 * value：待写入的16位原始值；总线上先发送高8位，再发送低8位。
 *
 * 返回值：I2C2_STATUS_OK表示完整事务成功，其余值指出失败阶段的类别。
 */
I2C2_Status_t BSP_I2C2_WriteRegister16(uint8_t address_7bit,
                                        uint8_t register_address,
                                        uint16_t value);

/*
 * 从从设备的一个8位寄存器地址读取16位数据。
 *
 * address_7bit：不包含读写方向位的7位设备地址。
 * register_address：准备读取的器件内部寄存器地址。
 * value：调用者提供的输出地址；成功时写入“高字节在前”的组合结果。
 *        传入空指针会返回I2C2_STATUS_INVALID_ARGUMENT。
 *
 * 返回值：只有返回I2C2_STATUS_OK时，*value才代表本次新读取的数据。
 */
I2C2_Status_t BSP_I2C2_ReadRegister16(uint8_t address_7bit,
                                       uint8_t register_address,
                                       uint16_t *value);

/*
 * 尝试恢复被拉住的I2C总线。
 *
 * 实现会暂时关闭I2C2，把PB10/PB11切成普通开漏输出，
 * 最多产生9个SCL脉冲并人工产生STOP，然后重新初始化I2C2。
 * 返回OK表示恢复后的引脚电平和BUSY标志正常，不代表下一次器件访问必然成功。
 */
I2C2_Status_t BSP_I2C2_RecoverBus(void);

/*
 * 复制一份完整诊断快照给调用者。
 * diagnostics：接收快照的结构体地址；空指针会被安全忽略。
 * 复制过程使用极短关中断区，避免读取到一半新统计值被写入。
 */
void BSP_I2C2_GetDiagnostics(I2C2_Diagnostics_t *diagnostics);

#endif /* BSP_I2C2_H */
