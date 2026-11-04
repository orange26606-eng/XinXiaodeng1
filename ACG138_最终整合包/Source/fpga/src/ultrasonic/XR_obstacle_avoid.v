`timescale 1ns / 1ps
// ============================================================================
// 模块：XR_obstacle_avoid —— 三路距离驱动的差速避障决策

// ============================================================================
// 一、职责和边界
//   输入前/左/右毫米距离，输出建议方向和左右轮目标刻度；不收UART，不产生PWM。
//   输出还需通过 XR_function 的净空、换向等待、斜坡和反馈保护，不能直接当电机引脚。
//   速度参数是PI使用的内部刻度，不是mm/s；此状态机没有编码器角度闭环。
//   无后方距离输入，不自动倒车。原地转向由时间和前方/侧方净空决定，不保证固定角度。
//
// 二、接口和数据有效性
//   clk_sys/rst_n：系统时钟、低有效异步复位；enable由上层自动模式选择产生。
//   enable=0时组合输出立即归零，下一个时钟把自动会话状态和重试预算复位。
//   front_mm/left_mm/right_mm：毫米距离；valid_mask的0/1/2位分别对应前/左/右。
// sample_update 表示对应通道的新样本事件；重复缓存不累计确认次数。
//   当前A02上游只对有效新帧产生更新事件；本模块仍独立检查三路valid_mask。
//   motor_cmd：0000停止、1010双轮前进、0110左原地转、1001右原地转。
//   target_speed_l/r：左右轮独立目标；未满足当前状态净空时两者输出0。
//   state_code：当前状态编码；attempts：当前会话的绕行/原地转向尝试计数。
//   blocked/fault：分别指示BLOCKED和FAULT_STOP，不是所有停车原因的总标志。
//
// 三、状态地图（编码与原接口保持一致）
//   0 WAIT_READY：等待三路有效并收齐更新；5 RECOVER：以低速恢复并确认前方净空；
//   8 SLOW：减速行驶；1 RUN：巡航；2 STOP_DECIDE：停车等待并收齐新的三路样本；
//   9 ARC_LEFT / 10 ARC_RIGHT：保持双轮前进，通过内慢外快画弧绕行；
//   3 TURN_LEFT / 4 TURN_RIGHT：原地转向；6 BLOCKED：无可用路线或耗尽预算后锁定；
//   7 FAULT_STOP：传感器失效停车，三路恢复后退回WAIT_READY重新等待更新。
//   BLOCKED不自行解锁，退出自动模式或系统复位才能重开会话。
//
// 四、安全确认与迟滞
//   距离变近直接限制组合输出或下拍进入停车；距离变远需连续新前向样本确认。
//   巡航减速门限默认600mm，恢复巡航门限700mm，避免同一阈值附近反复切换。
//   clear_count只由满足条件的新前向样本增加；不满足条件的新样本会清零。
//   stop_recheck清除seen，要求重新收集停车之后的各通道更新，不沿用旧确认标记。
//   seen不是三个距离同时采样的证明，只记录当前等待期间哪些通道有过更新。
//   retry在开始弧线/原地转向时增加；成功回到RUN、退出自动模式或复位时清零。
//
// 五、阅读代码时注意
//   上半部always @(*)给出当前状态的组合输出；下半部时序always更新状态寄存器。
//   非阻塞赋值读取更新前的state/timer/clear_count，定时边界以实际比较表达式为准。
//   80mm是保留代码中的原地转向前方最低净空常量，不受P_STOP_MM参数控制。
//   时间参数按50MHz标注且应为正整数；P_CLEAR_SAMPLES应在1..256内，
//   P_MAX_ATTEMPTS应在4位retry可表示范围内；参数之间也需保持合理的门限关系。
// ============================================================================

module XR_obstacle_avoid #(
    parameter integer P_STOP_HOLD_CLKS         = 2500000,  // 停车至少50ms，还需收齐停车后的三路更新
    parameter integer P_TURN_MIN_CLKS          = 5000000,  // 原地转向最短100ms
    parameter integer P_TURN_MAX_CLKS          = 50000000,  // 原地转向上限1s
    parameter integer P_MAX_ATTEMPTS           = 3,  // 弧线和原地转向共享重试预算，防止无限绕圈
    parameter integer P_CLEAR_SAMPLES          = 3,  // 加速/退出绕行需要的连续新前向样本
    parameter integer P_STOP_MM                = 200,  // 前进绝对停车线
    parameter integer P_RELEASE_MM             = 300,  // 重新前进门限，也是弧线最低前方净空
    parameter integer P_CLEAR_MM               = 600,  // 弧线退出/恢复确认门限
    parameter integer P_SIDE_STOP_MM           = 120,  // 所有运动的侧向硬停车线
    parameter integer P_SIDE_OPEN_MM           = 450,  // 选择一侧的开启门限
    parameter integer P_SIDE_TURN_MM           = 250,  // 原地转向选中侧保持门限
    parameter integer P_CHOICE_MARGIN_MM       = 80,  // 左右接近时保留偏好，避免抖动
    // 以下四项是两路PI的目标刻度，具体物理车速依赖轮径、编码器与测速参数。
    parameter integer P_SPEED_FULL             = 612,
    parameter integer P_SPEED_MID              = 408,
    parameter integer P_SPEED_LOW              = 204,
    parameter integer P_SPEED_TURN             = 306,
    parameter integer P_SLOW_ENTER_MM          = 600,  // 巡航进入减速；接近立即生效
    parameter integer P_SLOW_EXIT_MM           = 700,  // 减速退出门限，高于进入门限形成滞回
    parameter integer P_ARC_ENTER_MM           = 450,  // 小于此距离尝试弧线，低于300则先停车
    parameter integer P_ARC_SIDE_MM            = 300,  // 弧线选中侧必须始终保持至少300mm
    parameter integer P_ARC_OTHER_MM           = 200,  // 弧线另一侧至少200mm，严于硬停车线
    parameter integer P_ARC_MAX_CLKS           = 100000000,  // 弧线最多2s，超时停车重评估
    parameter integer P_ARC_MIN_CLKS           = 5000000,  // 至少100ms后才确认退出
    parameter integer P_SPEED_ARC_OUTER        = 306  // 弧线外轮306、内轮204，均为正向
) (
    input                 clk_sys,
    input                 rst_n,
    input                 enable,
    input       [23:0]    front_mm,
    input       [23:0]    left_mm,
    input       [23:0]    right_mm,
    input       [2:0]     valid_mask,
    input       [2:0]     sample_update,
    output reg  [3:0]     motor_cmd,
    output      [3:0]     state_code,
    output      [3:0]     attempts,
    output                blocked,
    output                fault,
    output reg  [10:0]    target_speed_l,
    output reg  [10:0]    target_speed_r
);

// 状态编码对外回传，保留原数值，不按下面源码出现顺序重新编号。
localparam WAIT_READY = 0, RUN = 1, STOP_DECIDE = 2, TURN_LEFT = 3, TURN_RIGHT = 4,
RECOVER = 5, BLOCKED = 6, FAULT_STOP = 7, SLOW = 8, ARC_LEFT = 9, ARC_RIGHT = 10;
// state是当前行为；retry是一次自动会话中已使用的转向/绕行预算。
reg [3:0] state, retry;
// timer在停车、弧线、原地转向阶段复用，进入相应动作前清零。
reg [31:0] timer;
// seen按位记住等待阶段已经观察到的新样本；用于三方向重新确认。
reg [2:0] seen;
// 前向连续净空确认计数；计数的是新样本，不是经过的时钟拍数。
reg [7:0] clear_count;
// 两侧相近时沿用方向偏好，初始偏左；不表示左右距离的测量结果。
reg prefer_left;
// 三路必须同时有效；无效距离即使数值很大也不能通过ready。
wire ready = &valid_mask;
wire sides_safe = left_mm >= P_SIDE_STOP_MM && right_mm >= P_SIDE_STOP_MM;
wire left_open = left_mm >= P_SIDE_OPEN_MM && sides_safe;
wire right_open = right_mm >= P_SIDE_OPEN_MM && sides_safe;
// 左可走且右不可走则选左；两边都可走时比较裕量，相近时采用历史偏好。
wire choose_left = left_open && (!right_open || left_mm > right_mm + P_CHOICE_MARGIN_MM ||
    (!(right_mm > left_mm + P_CHOICE_MARGIN_MM) && prefer_left));
// 弧线要求两侧都有基础净空，且所选内侧满足更严格的弧线侧向门限。
wire arc_space = left_mm >= P_ARC_OTHER_MM && right_mm >= P_ARC_OTHER_MM;
wire arc_safe = front_mm >= P_RELEASE_MM && arc_space &&
    ((state == ARC_LEFT) ? left_mm >= P_ARC_SIDE_MM : right_mm >= P_ARC_SIDE_MM);
assign state_code = state;
assign attempts = retry;
assign blocked = state == BLOCKED;
assign fault = state == FAULT_STOP;
// 【组合输出】先默认停车，再按状态开放；所有分支完整赋值，不推断锁存器。
always @(*) begin
    motor_cmd = 0;
    target_speed_l = 0;
    target_speed_r = 0;
    if (enable && ready && sides_safe) begin
        case (state)
            // 直行时前方至少达到停车门限；即使仍处于RUN，靠近障碍也会直接降目标。
            RUN, SLOW: if (front_mm >= P_STOP_MM) begin
                motor_cmd = 4'b1010;
                target_speed_l = (state == RUN && front_mm >= P_SLOW_ENTER_MM) ? P_SPEED_FULL :
                    ((front_mm >= P_ARC_ENTER_MM) ? P_SPEED_MID : P_SPEED_LOW);
                target_speed_r = target_speed_l;
            end
            // 恢复阶段始终低速，需满足前方恢复门限才输出前进。
            RECOVER: if (front_mm >= P_RELEASE_MM) begin
                motor_cmd = 4'b1010;
                target_speed_l = P_SPEED_LOW;
                target_speed_r = P_SPEED_LOW;
            end
            // 两轮方向均为前进：左弧线左轮慢，右弧线右轮慢，不在弧线中反复换侧。
            ARC_LEFT, ARC_RIGHT: if (arc_safe) begin
                motor_cmd = 4'b1010;
                target_speed_l = (state == ARC_LEFT) ? P_SPEED_LOW : P_SPEED_ARC_OUTER;
                target_speed_r = (state == ARC_LEFT) ? P_SPEED_ARC_OUTER : P_SPEED_LOW;
            end
            // 原地转向采用固定目标刻度，但侧向/前向净空不足时立即输出停车。
            TURN_LEFT: if (left_mm >= P_SIDE_TURN_MM && front_mm >= 80) begin
                motor_cmd = 4'b0110;
                target_speed_l = P_SPEED_TURN;
                target_speed_r = P_SPEED_TURN;
            end
            TURN_RIGHT: if (right_mm >= P_SIDE_TURN_MM && front_mm >= 80) begin
                motor_cmd = 4'b1001;
                target_speed_l = P_SPEED_TURN;
                target_speed_r = P_SPEED_TURN;
            end
            default: begin
            end
        endcase
    end
end

// 统一停车入口：每次停车都丢弃旧的seen，需要停车后的三路新测量。
// task是时序块内复用的寄存器赋值，不是新时钟或延迟执行的线程。
task stop_recheck;
    begin
        state <= STOP_DECIDE;
        timer <= 0;
        seen <= 0;
        clear_count <= 0;
    end
endtask

// 【时序决策】复位/退出自动优先；运行中数据无效先转故障停车，再处理状态分支。
always @(posedge clk_sys or negedge rst_n) begin
    if (!rst_n) begin
        state <= WAIT_READY;
        timer <= 0;
        seen <= 0;
        retry <= 0;
        clear_count <= 0;
        prefer_left <= 1;
    end else if (!enable) begin
        state <= WAIT_READY;
        timer <= 0;
        seen <= 0;
        retry <= 0;
        clear_count <= 0;
        prefer_left <= 1;
    end else if (state != WAIT_READY && state != BLOCKED && !ready) begin
        // WAIT_READY本来就等待有效数据；BLOCKED保持锁定，不被故障恢复旁路解锁。
        state <= FAULT_STOP;
        timer <= 0;
        seen <= 0;
        clear_count <= 0;
    end else begin
        case (state)
            // 合并当前拍sample_update，避免最后一路新样本要多等一拍才能被看见。
            WAIT_READY: begin
                seen <= seen | sample_update;
                if (ready && ((seen | sample_update) == 7)) begin
                    state <= RECOVER;
                    seen <= 0;
                    clear_count <= 0;
                end
            end
            // 先处理近障停车/绕行，再处理巡航迟滞；尝试弧线时必须有剩余预算。
            RUN, SLOW: begin
                if (front_mm < P_STOP_MM || !sides_safe) stop_recheck;
                else if (front_mm < P_ARC_ENTER_MM) begin
                    clear_count <= 0;
                    if (front_mm >= P_RELEASE_MM && arc_space && (left_open || right_open) && retry < P_MAX_ATTEMPTS) begin
                        state <= choose_left ? ARC_LEFT : ARC_RIGHT;
                        prefer_left <= choose_left;
                        timer <= 0;
                        retry <= retry + 1'b1;
                    end else stop_recheck;
                end else if (state == RUN) begin
                    if (front_mm < P_SLOW_ENTER_MM) begin
                        state <= SLOW;
                        clear_count <= 0;
                    end
                end else if (sample_update[0]) begin
                    if (front_mm >= P_SLOW_EXIT_MM) begin
                        if (clear_count >= P_CLEAR_SAMPLES - 1) begin
                            state <= RUN;
                            clear_count <= 0;
                            retry <= 0;
                        end else clear_count <= clear_count + 1'b1;
                    end else clear_count <= 0;
                end
            end
            // 满足停车等待时间并收齐三路更新后才选路；优先恢复，再判断阻塞或转向。
            STOP_DECIDE: begin
                seen <= seen | sample_update;
                if (timer < P_STOP_HOLD_CLKS) timer <= timer + 1'b1;
                else if ((seen | sample_update) == 7) begin
                    seen <= 0;
                    timer <= 0;
                    clear_count <= 0;
                    // 近障区域300~449mm不直接恢复直行；有空间则先原地转开。
                    if (front_mm >= P_ARC_ENTER_MM && sides_safe) state <= RECOVER;
                    else if (retry >= P_MAX_ATTEMPTS || (!left_open && !right_open) || front_mm < 80) state <= BLOCKED;
                    else begin
                        retry <= retry + 1'b1;
                        prefer_left <= choose_left;
                        state <= choose_left ? TURN_LEFT : TURN_RIGHT;
                    end
                end
            end
            // 任一净空破坏或弧线超时则停车重评估；安全退出还需最短动作时间和新样本确认。
            ARC_LEFT, ARC_RIGHT: begin
                timer <= timer + 1'b1;
                if (!arc_safe || !sides_safe || timer >= P_ARC_MAX_CLKS - 1) stop_recheck;
                else if (sample_update[0]) begin
                    if (front_mm >= P_CLEAR_MM && timer >= P_ARC_MIN_CLKS) begin
                        if (clear_count >= P_CLEAR_SAMPLES - 1) begin
                            state <= RECOVER;
                            clear_count <= 0;
                            timer <= 0;
                        end else clear_count <= clear_count + 1'b1;
                    end else clear_count <= 0;
                end
            end
            // 转向失败/超时进入统一停车；转向成功也先停车重新收样，而非直接切高速前进。
            TURN_LEFT, TURN_RIGHT: begin
                timer <= timer + 1'b1;
                if (!sides_safe || front_mm < 80 || (state == TURN_LEFT && left_mm < P_SIDE_TURN_MM) ||
                    (state == TURN_RIGHT && right_mm < P_SIDE_TURN_MM) || timer >= P_TURN_MAX_CLKS - 1) stop_recheck;
                else if (sample_update[0]) begin
                    if (front_mm >= P_CLEAR_MM && timer >= P_TURN_MIN_CLKS) begin
                        if (clear_count >= P_CLEAR_SAMPLES - 1) stop_recheck;
                        else clear_count <= clear_count + 1'b1;
                    end else clear_count <= 0;
                end
            end
            // 低速恢复：过近停车，进入近障区先SLOW，前方连续开阔后也先经过SLOW。
            RECOVER: begin
                if (front_mm < P_RELEASE_MM || !sides_safe) stop_recheck;
                else if (front_mm < P_ARC_ENTER_MM) begin
                    state <= SLOW;
                    clear_count <= 0;
                end
                else if (sample_update[0]) begin
                    if (front_mm >= P_CLEAR_MM) begin
                        if (clear_count >= P_CLEAR_SAMPLES - 1) begin
                            state <= SLOW;
                            clear_count <= 0;
                        end else clear_count <= clear_count + 1'b1;
                    end else clear_count <= 0;
                end
            end
            // 空分支有意保持锁定；持续新距离本身不能解除BLOCKED。
            BLOCKED: begin
            end
            // 数据恢复只回到WAIT_READY，丢弃旧seen后再收齐更新，不立即恢复电机动作。
            FAULT_STOP: if (ready) begin
                state <= WAIT_READY;
                seen <= 0;
                clear_count <= 0;
            end
            default: begin
                state <= WAIT_READY;
                seen <= 0;
                timer <= 0;
            end
        endcase
    end
end

endmodule
