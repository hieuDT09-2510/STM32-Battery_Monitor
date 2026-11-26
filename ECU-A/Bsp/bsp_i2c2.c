/*
 * 文件名称：bsp_i2c2.c
 *
 * 模块作用：实现bsp_i2c2.h声明的STM32F103 I2C2底层操作。
 *
 * 正常读事务的数据流：
 * 等待总线空闲 → START → 发送写地址 → 发送寄存器地址
 * → ReSTART → 发送读地址 → 按STM32F1双字节时序读取MSB/LSB → STOP。
 *
 * 正常写事务的数据流：
 * 等待总线空闲 → START → 发送写地址 → 发送寄存器地址
 * → 发送16位数据的高字节/低字节 → 等待发送完成 → STOP。
 *
 * 时序原则：
 * I2C2使用100kHz标准模式和轮询收发。轮询不等于无限等待：
 * 一次完整寄存器事务共享同一个3ms截止时间和循环预算，任一阶段
 * 没有等到标志都会退出。这样INA226或线路异常不会永久卡住CurrentTask。
 *
 * 并发边界：
 * 当前只有CurrentTask调用本驱动，因此没有为I2C2增加Mutex。
 * 双字节接收中的极短关中断区只保护STM32F1规定的寄存器操作顺序，
 * 不包含任何等待循环，避免长时间影响FreeRTOS节拍和其他中断。
 */
#include "bsp_i2c2.h"

#include "stm32f10x.h"
#include "bsp_tim.h"

/* 一次完整读/写事务允许占用的最长墙上时间，单位ms。 */
#define BSP_I2C2_TRANSACTION_TIMEOUT_MS 3UL

/*
 * 当GetTick暂时不前进（例如调试停顿或中断异常）时的第二道退出条件。
 * 每轮轮询减1，归零后同样判定超时，避免只依赖系统毫秒计数。
 */
#define BSP_I2C2_LOOP_BUDGET            250000UL

/* 一次事务共享的截止条件，保证多个阶段合计不能分别占用3ms。 */
typedef struct
{
    uint32_t start_ms;   /* 事务开始时的系统毫秒时间戳。 */
    uint32_t loops_left; /* 本次事务剩余的最大轮询次数。 */
} I2C2_Deadline_t;

/*
 * I2C2底层累计诊断状态。
 * 只有本文件写入；CLI通过BSP_I2C2_GetDiagnostics取得一致副本。
 */
static I2C2_Diagnostics_t g_i2c2_diagnostics;

/// @brief 将32位诊断计数器饱和递增，达到0xFFFFFFFF后保持最大值。
/// @param value 待递增计数器的地址；调用者必须传入有效指针。
/// @return 无。
static void I2C2_IncrementSaturated(uint32_t *value)
{
    /* 调用位置传入的都是本模块有效计数器地址，不接受空指针。 */
    if (*value < 0xFFFFFFFFUL)
    {
        (*value)++;
    }
}

/// @brief 在I2C总线恢复期间产生短延时，为SCL和SDA电平变化留出稳定时间。
/// @param 无。
/// @return 无。
static void I2C2_DelayShort(void)
{
    /* volatile阻止编译器删除这个仅用于产生GPIO恢复时序的空循环。 */
    volatile uint32_t count;

    /* 恢复总线时给SCL/SDA电平留出稳定时间，不用于正常事务超时。 */
    for (count = 0U; count < 100U; count++)
    {
        __NOP(); //空操作
    }
}


/// @brief 为一笔新的I2C事务建立统一的毫秒截止时间和轮询预算。
/// @param deadline 接收本次事务起始时间和剩余轮询次数的结构体地址。
/// @return 无。
static void I2C2_StartDeadline(I2C2_Deadline_t *deadline)
{
    /* 保存当前毫秒时间，后续使用无符号减法兼容32位时间戳回绕。 */
    deadline->start_ms = GetTick();

    /* 每个新事务重新获得完整轮询预算。 */
    deadline->loops_left = BSP_I2C2_LOOP_BUDGET;
}

/// @brief 消耗一次轮询预算，并判断当前I2C事务是否已经超时。
/// @param deadline 当前事务共用的截止条件；函数会递减其中的轮询预算。
/// @return 1表示毫秒期限或轮询预算已耗尽，0表示仍可继续等待。
static uint8_t I2C2_DeadlineExpired(I2C2_Deadline_t *deadline)
{
    /* 循环预算已经耗尽，即使毫秒Tick未变化也必须退出。 */
    if (deadline->loops_left == 0U)
    {
        return 1U;
    }

    deadline->loops_left--; /* 本次检查消耗一次轮询预算。 */

    /* 整个事务从start_ms起累计达到3ms，判定统一超时。 */
    if ((uint32_t)(GetTick() - deadline->start_ms) >=
        BSP_I2C2_TRANSACTION_TIMEOUT_MS)
    {
        return 1U;
    }

    return 0U;
}

/// @brief 检查并清除I2C2当前可识别的硬件错误标志。
/// @param 无。
/// @return 无错误返回I2C2_STATUS_OK；收到NACK返回I2C2_STATUS_NACK；其余硬件错误返回I2C2_STATUS_BUS_ERROR。
static I2C2_Status_t I2C2_CheckHardwareError(void)
{
    uint32_t sr1; /* 保存本次检查使用的SR1快照，避免同一判断内多次读取变化。 */

    sr1 = I2C2->SR1;

    /* AF=1表示发送地址或数据后没有收到从机ACK。 */
    if ((sr1 & I2C_SR1_AF) != 0U)
    {
        /* 清除AF，防止该旧错误污染下一次事务。 */
        I2C2->SR1 &= ~I2C_SR1_AF;
        return I2C2_STATUS_NACK;
    }

    /*
     * BERR：非法START/STOP等总线错误；ARLO：多主仲裁丢失；
     * OVR：数据寄存器处理不及时导致溢出/欠载；
     * TIMEOUT：I2C硬件自身检测到超时。
     */
    if ((sr1 & (I2C_SR1_BERR |
                I2C_SR1_ARLO |
                I2C_SR1_OVR |
                I2C_SR1_TIMEOUT)) != 0U)
    {
        /* 清除已识别的错误位，让后续事务从干净状态开始。 */
        I2C2->SR1 &= ~(I2C_SR1_BERR |
                       I2C_SR1_ARLO |
                       I2C_SR1_OVR |
                       I2C_SR1_TIMEOUT);
        return I2C2_STATUS_BUS_ERROR;
    }

    return I2C2_STATUS_OK;
}


/// @brief 在统一截止时间内等待SR1中的指定事件标志置位。
/// @param flag 要等待的I2C_SR1_xxx标志掩码。
/// @param deadline 当前完整事务共用的截止条件。
/// @return 标志到达返回I2C2_STATUS_OK；否则返回NACK、BUS_ERROR或TIMEOUT。
static I2C2_Status_t I2C2_WaitSR1Set(uint32_t flag,
                                      I2C2_Deadline_t *deadline)
{
    I2C2_Status_t status; /* 保存每轮硬件错误检查结果。 */

    /* 等待指定事件位；成功、硬件错误或软件超时三种条件之一会退出。 */
    for (;;)
    {
        /* 先检查错误，避免NACK后仍无意义地等待ADDR/TXE等正常标志。 */
        status = I2C2_CheckHardwareError();
        if (status != I2C2_STATUS_OK)
        {
            return status;
        }

        /* 调用者要求的全部标志位已经置位。 */
        if ((I2C2->SR1 & flag) != 0U)
        {
            return I2C2_STATUS_OK;
        }

        /* 事务共用截止条件到期，返回软件超时。 */
        if (I2C2_DeadlineExpired(deadline) != 0U)
        {
            return I2C2_STATUS_TIMEOUT;
        }
    }
}

/// @brief 等待SR2.BUSY清零，确认上一笔事务已经释放I2C总线。
/// @param deadline 当前完整事务共用的截止条件。
/// @return 总线空闲返回I2C2_STATUS_OK；等待到期返回I2C2_STATUS_BUS_BUSY。
static I2C2_Status_t I2C2_WaitBusFree(I2C2_Deadline_t *deadline)
{
    /* SR2.BUSY=1表示SCL/SDA上仍存在一个正在进行或卡住的事务。 */
    while ((I2C2->SR2 & I2C_SR2_BUSY) != 0U)
    {
        if (I2C2_DeadlineExpired(deadline) != 0U)
        {
            return I2C2_STATUS_BUS_BUSY;
        }
    }

    return I2C2_STATUS_OK;
}

/// @brief 按STM32F1要求依次读取SR1和SR2，以清除ADDR标志。
/// @param 无。
/// @return 无。
static void I2C2_ClearAddressFlag(void)
{
    /* volatile保证编译器严格执行两次寄存器读取，不能优化删除。 */
    volatile uint32_t temporary;

    /* STM32F1规定按顺序读SR1再读SR2才能清除ADDR标志。 */
    temporary = I2C2->SR1;
    temporary = I2C2->SR2;

    /* 显式标记已使用，避免编译器产生“变量赋值后未使用”警告。 */
    (void)temporary;
}

/*
 * STM32F1接收1/2字节时，清ADDR、设置NACK以及STOP后的首个DR读取
 * 都有严格时序要求。这里只屏蔽几个寄存器访问所需的极短时间，
 * 不把等待ADDR/BTF的轮询过程放入关中断区。
 */
/// @brief 保存当前PRIMASK并暂时屏蔽可屏蔽中断，保护极短寄存器操作序列。
/// @param 无。
/// @return 进入原子序列前的PRIMASK值，必须交给I2C2_ExitAtomicSequence恢复。
static uint32_t I2C2_EnterAtomicSequence(void)
{
    uint32_t primask; /* 保存进入前的全局中断屏蔽状态。 */

    /* 先保存状态，再屏蔽可屏蔽中断；返回值交给退出函数恢复。 */
    primask = __get_PRIMASK();
    __disable_irq(); //用于禁用中断
    return primask;
}

/// @brief 根据进入原子序列前的PRIMASK恢复原有中断状态。
/// @param primask I2C2_EnterAtomicSequence返回的原始PRIMASK值。
/// @return 无。
static void I2C2_ExitAtomicSequence(uint32_t primask)
{
    /* 只有进入前中断原本开启时才重新开启，不能破坏调用者原有状态。 */
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/// @brief 保存本次I2C结果和SR1/SR2现场，并更新对应的诊断计数器。
/// @param status 本次事务最终返回的I2C2_Status_t状态。
/// @return 无。
static void I2C2_RecordResult(I2C2_Status_t status)
{
    /* 无论成功失败，都保存最近一次硬件状态和软件返回码。 */
    g_i2c2_diagnostics.last_sr1 = I2C2->SR1;
    g_i2c2_diagnostics.last_sr2 = I2C2->SR2;
    g_i2c2_diagnostics.last_status = (uint8_t)status;

    /* 成功事务只增加成功总数，不进入失败分类。 */
    if (status == I2C2_STATUS_OK)
    {
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.transfer_ok_count);
        return;
    }

    /* 所有非OK结果统一计入失败总数。 */
    I2C2_IncrementSaturated(&g_i2c2_diagnostics.transfer_fail_count);

    /* 再按照最主要的失败原因增加对应分类计数。 */
    if (status == I2C2_STATUS_NACK)
    {
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.nack_count);
    }
    else if (status == I2C2_STATUS_TIMEOUT)
    {
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.timeout_count);
    }
    else if (status == I2C2_STATUS_BUS_BUSY)
    {
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.bus_busy_count);
    }
    else if (status == I2C2_STATUS_BUS_ERROR)
    {
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.bus_error_count);
    }
}

/// @brief 异常结束当前事务，恢复默认接收控制位并记录原始失败原因。
/// @param status 触发终止的原始失败状态。
/// @return 原样返回传入的status，供上层继续判断失败类别。
static I2C2_Status_t I2C2_Abort(I2C2_Status_t status)
{
    /* 请求STOP，尽量释放当前总线事务。 */
    I2C2->CR1 |= I2C_CR1_STOP;

    /* 恢复默认双字节接收控制状态，避免失败时留下POS/NACK影响下一次读取。 */
    I2C2->CR1 &= ~I2C_CR1_POS;
    I2C2->CR1 |= I2C_CR1_ACK;

    /* 保存原始失败原因并原样返回给调用者。 */
    I2C2_RecordResult(status);
    return status;
}

/// @brief 一次完成GPIOB、PB10/PB11、I2C2时钟和100kHz时序初始化。
/// @param 无。
/// @return 无。
void BSP_I2C2_Init(void)
{
    /* 1. 打开GPIOB时钟，随后才能配置PB10和PB11。 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    /*
     * 2. PB10(SCL)和PB11(SDA)配置为50MHz复用开漏输出。
     * CRH中每个引脚占4位，0xF表示MODE=11、CNF=11。
     */
    GPIOB->CRH &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOB->CRH |=  ((0xFUL << 8) | (0xFUL << 12));

    /* 开漏总线写1表示释放引脚，高电平由外部上拉电阻产生。 */
    GPIOB->ODR |= GPIO_Pin_10 | GPIO_Pin_11;

    /* 3. 打开并复位I2C2，清除上一次运行可能遗留的硬件状态。 */
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

    /*
     * 4. 本项目时钟固定为PCLK1=36MHz，I2C2使用100kHz标准模式：
     * CR2.FREQ=36；CCR=36MHz/(2×100kHz)=180；TRISE=36+1=37。
     */
    I2C2->CR1 = 0U;
    I2C2->CR2 = 36U;
    I2C2->CCR = 180U;
    I2C2->TRISE = 37U;
    I2C2->OAR1 = 0x4000U; /* 主机模式下硬件仍要求OAR1位14保持1。 */

    /* 5. 默认允许ACK并使能I2C2，初始化完成。 */
    I2C2->CR1 = I2C_CR1_ACK | I2C_CR1_PE;
}

/// @brief 向指定7位I2C设备的8位寄存器地址写入一个16位原始值。
/// @param address_7bit 不包含读写位的7位从机地址，合法范围0x00～0x7F。
/// @param register_address 从设备内部的8位寄存器地址。
/// @param value 要发送的16位值；总线上先发送高字节，再发送低字节。
/// @return 完整写入返回I2C2_STATUS_OK；失败时返回具体NACK、超时、总线忙、总线错误或参数错误状态。
I2C2_Status_t BSP_I2C2_WriteRegister16(uint8_t address_7bit,
                                        uint8_t register_address,
                                        uint16_t value)
{
    I2C2_Deadline_t deadline; /* 本次完整写事务共用的超时状态。 */
    I2C2_Status_t status;     /* 保存每个步骤返回的状态。 */

    /* 7位地址只能占0x00～0x7F；拒绝已包含读写位的8位地址。 */
    if (address_7bit > 0x7FU)
    {
        I2C2_RecordResult(I2C2_STATUS_INVALID_ARGUMENT);
        return I2C2_STATUS_INVALID_ARGUMENT;
    }

    /* 从这里开始计算整笔事务的3ms期限。 */
    I2C2_StartDeadline(&deadline);

    /* 必须确认上一笔事务已经释放总线，才能产生新的START。 */
    status = I2C2_WaitBusFree(&deadline);
    if (status != I2C2_STATUS_OK)
    {
        I2C2_RecordResult(status);
        return status;
    }

    /* CR1.START=1：硬件在总线上产生起始条件。 */
    I2C2->CR1 |= I2C_CR1_START;

    /* SB=1表示START已经产生、主机模式已经建立。 */
    status = I2C2_WaitSR1Set(I2C_SR1_SB, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /* 7位地址左移一位且最低位保持0，表示“写方向”。 */
    I2C2->DR = (uint8_t)(address_7bit << 1);

    /* ADDR=1表示从机已经应答该写地址。 */
    status = I2C2_WaitSR1Set(I2C_SR1_ADDR, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    /* 清ADDR后硬件才继续进入数据发送阶段。 */
    I2C2_ClearAddressFlag();

    /* TXE=1表示数据寄存器可以接收下一个待发送字节。 */
    status = I2C2_WaitSR1Set(I2C_SR1_TXE, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    I2C2->DR = register_address; /* 先告诉从机要访问哪个内部寄存器。 */

    /* 等待寄存器地址进入移位寄存器后，再写16位数据高字节。 */
    status = I2C2_WaitSR1Set(I2C_SR1_TXE, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    I2C2->DR = (uint8_t)(value >> 8); /* INA226寄存器采用MSB first。 */

    /* 高字节已经移走后写低字节。 */
    status = I2C2_WaitSR1Set(I2C_SR1_TXE, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    I2C2->DR = (uint8_t)value; /* 强制转8位，保留value的低8位。 */

    /* BTF=1表示DR和移位寄存器都空，最后一个字节已经发送完毕。 */
    status = I2C2_WaitSR1Set(I2C_SR1_BTF, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    I2C2->CR1 |= I2C_CR1_STOP; /* 正常结束事务并释放总线。 */
    I2C2_RecordResult(I2C2_STATUS_OK);
    return I2C2_STATUS_OK;
}

/// @brief 从指定7位I2C设备的8位寄存器地址读取一个16位原始值。
/// @param address_7bit 不包含读写位的7位从机地址，合法范围0x00～0x7F。
/// @param register_address 从设备内部的8位寄存器地址。
/// @param value 接收组合后16位数据的地址；只有函数返回OK时内容才是本次新数据。
/// @return 完整读取返回I2C2_STATUS_OK；失败时返回具体NACK、超时、总线忙、总线错误或参数错误状态。
I2C2_Status_t BSP_I2C2_ReadRegister16(uint8_t address_7bit,
                                       uint8_t register_address,
                                       uint16_t *value)
{
    I2C2_Deadline_t deadline; /* 本次“写寄存器地址+读两字节”共用的期限。 */
    I2C2_Status_t status;     /* 保存每个I2C阶段的执行结果。 */
    uint32_t primask;         /* 保存极短原子序列前的中断开关状态。 */
    uint8_t msb;              /* 从总线读取的寄存器高8位。 */
    uint8_t lsb;              /* 从总线读取的寄存器低8位。 */

    /* 输出地址必须有效，设备地址必须是标准7位值。 */
    if ((value == 0) || (address_7bit > 0x7FU))
    {
        I2C2_RecordResult(I2C2_STATUS_INVALID_ARGUMENT);
        return I2C2_STATUS_INVALID_ARGUMENT;
    }

    /* 两个方向阶段共享同一个截止时间，不能各自重新获得3ms。 */
    I2C2_StartDeadline(&deadline);
    status = I2C2_WaitBusFree(&deadline);
    if (status != I2C2_STATUS_OK)
    {
        I2C2_RecordResult(status);
        return status;
    }

    I2C2->CR1 |= I2C_CR1_ACK;  /* 从默认ACK状态开始本次读取。 */
    I2C2->CR1 &= ~I2C_CR1_POS; /* 先清除上次双字节接收可能留下的POS。 */
    I2C2->CR1 |= I2C_CR1_START;/* 产生第一次START，准备写寄存器地址。 */

    /* 等待START完成。 */
    status = I2C2_WaitSR1Set(I2C_SR1_SB, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /* 先以写方向寻址，因为必须先告诉从机内部寄存器地址。 */
    I2C2->DR = (uint8_t)(address_7bit << 1);
    status = I2C2_WaitSR1Set(I2C_SR1_ADDR, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    I2C2_ClearAddressFlag(); /* 按SR1→SR2顺序清除ADDR。 */

    /* 等待DR可写，然后发送内部寄存器地址。 */
    status = I2C2_WaitSR1Set(I2C_SR1_TXE, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }
    I2C2->DR = register_address;

    /* BTF保证寄存器地址已完整发送，随后才能安全产生重复START。 */
    status = I2C2_WaitSR1Set(I2C_SR1_BTF, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /* 不释放总线，直接产生Repeated START并切换到读方向。 */
    I2C2->CR1 |= I2C_CR1_START;
    status = I2C2_WaitSR1Set(I2C_SR1_SB, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /* 两字节接收：在发送读地址前同时置POS和ACK。 */
    I2C2->CR1 |= I2C_CR1_POS | I2C_CR1_ACK;

    /* 地址最低位置1表示读方向。 */
    I2C2->DR = (uint8_t)((address_7bit << 1) | 0x01U);
    status = I2C2_WaitSR1Set(I2C_SR1_ADDR, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /*
     * ST AN2824的EV6_1顺序：先读SR1/SR2清ADDR，再清ACK。
     * 两步之间禁止中断，避免错过第二字节的NACK控制窗口。
     */
    /* 只保护清ADDR和清ACK这两个紧邻寄存器操作。 */
    primask = I2C2_EnterAtomicSequence();
    I2C2_ClearAddressFlag();
    I2C2->CR1 &= ~I2C_CR1_ACK;
    I2C2_ExitAtomicSequence(primask);

    /* BTF=1表示两个接收字节都已经进入硬件缓冲，可连续读取。 */
    status = I2C2_WaitSR1Set(I2C_SR1_BTF, &deadline);
    if (status != I2C2_STATUS_OK) { return I2C2_Abort(status); }

    /*
     * STM32F1勘误要求STOP到读取倒数第二字节之间不得插入中断。
     * BTF已置位，两个字节都在硬件缓冲区中，这里只需连续读取两次DR。
     */
    primask = I2C2_EnterAtomicSequence();
    I2C2->CR1 |= I2C_CR1_STOP; /* 在读取倒数第二字节前预先请求STOP。 */
    msb = (uint8_t)I2C2->DR;   /* 第一次读DR取得高字节。 */
    lsb = (uint8_t)I2C2->DR;   /* 第二次读DR取得低字节。 */
    I2C2_ExitAtomicSequence(primask);

    /* 恢复默认POS=0、ACK=1，为下一笔事务准备。 */
    I2C2->CR1 &= ~I2C_CR1_POS;
    I2C2->CR1 |= I2C_CR1_ACK;

    /* INA226线上高字节在前，这里重新组合成CPU使用的uint16_t。 */
    *value = ((uint16_t)msb << 8) | (uint16_t)lsb;
    I2C2_RecordResult(I2C2_STATUS_OK);
    return I2C2_STATUS_OK;
}

/// @brief 通过最多9个SCL脉冲和人工STOP尝试释放总线，随后重新初始化I2C2。
/// @param 无。
/// @return SCL、SDA和BUSY均恢复正常时返回I2C2_STATUS_OK，否则返回I2C2_STATUS_RECOVERY_FAILED。
I2C2_Status_t BSP_I2C2_RecoverBus(void)
{
    uint8_t pulse; /* 已产生的SCL恢复脉冲序号，范围0～8。 */

    /* 关闭I2C2外设，让GPIO重新取得PB10/PB11控制权。 */
    I2C2->CR1 &= ~I2C_CR1_PE;

    /*
     * 临时把PB10/PB11改成2MHz普通开漏输出：MODE=10、CNF=01，即0x7。
     * 随后BSRR置位表示释放两条线，让外部上拉把它们拉高。
     */
    GPIOB->CRH &= ~((0xFUL << 8) | (0xFUL << 12));
    GPIOB->CRH |=  ((0x7UL << 8) | (0x7UL << 12));
    GPIOB->BSRR = GPIO_Pin_10 | GPIO_Pin_11;
    I2C2_DelayShort();

    /* 从设备若卡在发送阶段，最多9个SCL脉冲使其移出剩余数据位。 */
    for (pulse = 0U; pulse < 9U; pulse++)
    {
        GPIOB->BRR = GPIO_Pin_10;  /* 主动把SCL拉低。 */
        I2C2_DelayShort();
        GPIOB->BSRR = GPIO_Pin_10; /* 释放SCL，由上拉形成高电平。 */
        I2C2_DelayShort();

        /* SDA已被释放为高，说明从设备不再占用数据线，可以提前结束。 */
        if ((GPIOB->IDR & GPIO_Pin_11) != 0U)
        {
            break;
        }
    }

    /* 人工产生STOP：SCL为高时让SDA从低变高。 */
    GPIOB->BRR = GPIO_Pin_11;  /* 先让SDA保持低电平。 */
    I2C2_DelayShort();
    GPIOB->BSRR = GPIO_Pin_10; /* 确保SCL已经释放为高。 */
    I2C2_DelayShort();
    GPIOB->BSRR = GPIO_Pin_11; /* SCL高时释放SDA，形成STOP上升沿。 */
    I2C2_DelayShort();

    /* 把引脚还给复用功能并重新配置I2C2；保留此前诊断记录。 */
    BSP_I2C2_Init();

    /*
     * 恢复成功必须同时满足：SCL为高、SDA为高、SR2.BUSY为0。
     * 任一条件不满足都说明总线仍被硬件或外设占用。
     */
    if (((GPIOB->IDR & GPIO_Pin_10) == 0U) ||
        ((GPIOB->IDR & GPIO_Pin_11) == 0U) ||
        ((I2C2->SR2 & I2C_SR2_BUSY) != 0U))
    {
        /* 单独统计恢复失败；它不是一笔寄存器传输，所以不增加transfer_fail。 */
        I2C2_IncrementSaturated(&g_i2c2_diagnostics.recovery_fail_count);
        g_i2c2_diagnostics.last_status =
            (uint8_t)I2C2_STATUS_RECOVERY_FAILED;
        return I2C2_STATUS_RECOVERY_FAILED;
    }

    /* 引脚和硬件状态均正常，记录一次恢复成功。 */
    I2C2_IncrementSaturated(&g_i2c2_diagnostics.recovery_ok_count);
    g_i2c2_diagnostics.last_status = (uint8_t)I2C2_STATUS_OK;
    return I2C2_STATUS_OK;
}

/// @brief 原子复制I2C2累计诊断信息，供CLI或其他诊断模块读取。
/// @param diagnostics 接收诊断快照的结构体地址；传入空指针时直接返回。
/// @return 无。
void BSP_I2C2_GetDiagnostics(I2C2_Diagnostics_t *diagnostics)
{
    uint32_t primask; /* 保存进入复制动作前的中断屏蔽状态。 */

    /* 空输出地址没有可写位置，直接返回，避免空指针访问。 */
    if (diagnostics == 0)
    {
        return;
    }

    /*
     * 结构体包含多个字段，复制不是单条原子指令。
     * 短暂关中断保证调用者得到同一时刻的一整份诊断快照。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    *diagnostics = g_i2c2_diagnostics;

    /* 若进入前中断处于开启状态才恢复，保持调用者原有PRIMASK语义。 */
    if (primask == 0U)
    {
        __enable_irq();
    }
}
