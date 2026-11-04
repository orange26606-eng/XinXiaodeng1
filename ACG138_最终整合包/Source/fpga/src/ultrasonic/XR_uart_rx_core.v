`timescale 1ns/1ps
// ============================================================================


// 日期：2026-10-07 15:33
// 模块：XR_uart_rx_core
// 版本：1.1.0
// 注释与排版整理：2026-10-10；保持原有逻辑、接口和时序。
// ============================================================================
// 功能：将异步 UART RX 波形转换成带 valid/error 脉冲的并行字节。
// 格式：8N1，即 1 位起始位、8 位数据、无校验位、1 位停止位；低位先收。
// 空闲电平为高，起始位为低，停止位必须为高。
//
// 时钟关系：
//   所有寄存器都由 clk 驱动，tick 是单周期时钟使能，不是另一条时钟。
//   每个串行位对应 16 次 tick；波特率由外部 tick 发生器决定。
//   若 clk=50MHz、每 326 拍产生一次 tick，则波特率约为 9585.9 baud。
//   两级同步器和 IDLE 状态的起始检测按每个 clk 工作；位内计数按 tick 工作。
//
// 接收流程：
// 接收阶段：ARM准备、IDLE空闲、START起始、DATA数据、STOP停止，再回IDLE。
//                     起始异常/停止异常 -> REARM -> IDLE
//   ARM 要求连续 160 次 tick 采到高电平，避免复位在字节中途释放时误收。
//   REARM 要求连续 16 次 tick 采到高电平，用于接收错误后的重新准备。
//   “连续”以 tick 采样结果为准，并非检测两次 tick 之间的所有窄脉冲。
// ============================================================================
module XR_uart_rx_core (
    input            clk,       // 系统时钟；本工程通常为 50MHz。
    input            rst_n,     // 低有效异步复位；释放同步由上层保证。
    input            tick,      // 16 倍波特率使能，要求有效时持续一个 clk 周期。
    input            rx,        // 外部异步串行输入，先经过本模块的两级同步器。
    output reg [7:0] data,       // 最近一次成功接收的字节；只有 valid=1 时表示新数据。
    output reg       valid,     // 成功收到停止位时拉高一个 clk 周期，无反压接口。
    output reg       error,     // 假起始位或停止位错误时拉高一个 clk 周期。
    output wire      busy       // 仅 START/DATA/STOP 为 1；ARM/REARM 期间也不能接收。
);

// rx_sync[0] 接收异步电平，rx_sync[1] 供状态机使用。
// 两级同步用于降低亚稳态传播风险，不等同于多点表决或数字消抖。
reg [1:0] rx_sync;
reg [2:0] state, bit_idx;        // state：接收状态；bit_idx：当前数据位编号 0～7。
reg [3:0] phase;                 // tick 相位计数，0～15；START 使用其中的 0～7。
reg [7:0] shift, idle_ticks;     // shift：接收缓存；idle_ticks：重新准备的高电平计数。

localparam ARM = 0, IDLE = 1, START = 2, DATA = 3, STOP = 4, REARM = 5;

assign busy = (state == START) || (state == DATA) || (state == STOP);

// 异步输入同步：独立于 tick，始终按系统时钟更新。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        rx_sync <= 2'b11;
    else
        rx_sync <= {rx_sync[0], rx};
end

// 非阻塞赋值在本拍末尾更新，条件判断读取的是进入本拍时的寄存器值。
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state      <= ARM;
        bit_idx    <= 0;
        phase      <= 0;
        shift      <= 0;
        data       <= 0;
        valid      <= 0;
        error      <= 0;
        idle_ticks <= 0;
    end
    else begin
        // 每个 clk 默认撤销事件脉冲，不能等到下一个 tick 才撤销。
        // data 则持续保存上一次成功字节，接收方必须依据 valid 取数。
        valid <= 0;
        error <= 0;

        // IDLE 下检测同步后的低电平，并非专门的下降沿检测电路。
        // 本分支优先于下面的 tick 分支；两者同拍出现时先进入 START。
        // phase 清零后，等待后续第 8 次 tick 到达起始位中部附近。
        // 因 tick 自由运行，实际起始确认位置会有不足一个 tick 的相位差。
        if (state == IDLE && !rx_sync[1]) begin
            state <= START;
            phase <= 0;
        end
        else if (tick)
            case (state)
                ARM, REARM: begin
                    // 任意一次 tick 采到低电平都会重新累计高电平等待时间。
                    // 旧值 159/15 对应当前分别是第 160/16 次高电平采样。
                    if (!rx_sync[1])
                        idle_ticks <= 0;
                    else if ((state == ARM && idle_ticks == 159) ||
                             (state == REARM && idle_ticks == 15)) begin
                        state      <= IDLE;
                        idle_ticks <= 0;
                    end
                    else
                        idle_ticks <= idle_ticks + 1'b1;
                end

                START: if (phase == 7) begin
                    phase <= 0;
                    // 半个位宽后仍为低，才认为起始位成立。
                    // 若已经回高，报告一次错误并进入 REARM。
                    if (!rx_sync[1]) begin
                        state   <= DATA;
                        bit_idx <= 0;
                    end
                    else begin
                        state      <= REARM;
                        idle_ticks <= 0;
                        error      <= 1;
                    end
                end
                else
                    phase <= phase + 1'b1;

                DATA: if (phase == 15) begin
                    phase          <= 0;
                    shift[bit_idx] <= rx_sync[1];
                    // 起始位中部之后，每隔 16 次 tick 采样一个数据位。
                    // bit_idx 从 0 增加到 7，实现 UART 的 LSB-first 顺序。
                    // 每位只采样一次，没有 3 点多数表决。
                    if (bit_idx == 7)
                        state <= STOP;
                    else
                        bit_idx <= bit_idx + 1'b1;
                end
                else
                    phase <= phase + 1'b1;

                STOP: if (phase == 15) begin
                    phase <= 0;
                    // 在停止位中部附近检查高电平；此时 shift 的 8 位已完整。
                    // 成功后马上回 IDLE，无需重新经历 ARM/REARM 等待。
                    if (rx_sync[1]) begin
                        data  <= shift;
                        valid <= 1;
                        state <= IDLE;
                    end
                    else begin
                        // 错误字节不更新 data，也不产生 valid。
                        error      <= 1;
                        state      <= REARM;
                        idle_ticks <= 0;
                    end
                end
                else
                    phase <= phase + 1'b1;

                // IDLE 高电平时落入这里并保持状态。
                // 保留原实现：未定义状态也保持，不额外增加状态恢复逻辑。
                default: begin end
            endcase
    end
end

endmodule
