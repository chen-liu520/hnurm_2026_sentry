# PubRobotStatus 节点逻辑分析与模拟

## 一、节点架构概览

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        PubRobotStatus 节点                               │
├─────────────────────────────────────────────────────────────────────────┤
│  定时器 (3个)                                                            │
│  ├── cruise_change_timer_    (10s)    → UpdateCurrentCruiseMode()       │
│  ├── high_frequence_timer_   (200ms)  → high_frequence_callback()       │
│  └── super_high_frequence_timer_ (1ms)→ super_high_frequence_callback() │
├─────────────────────────────────────────────────────────────────────────┤
│  订阅话题 (6个)                                                          │
│  ├── global_position_topic   → global_pose_callback()                   │
│  ├── cmd_vel_topic          → remap_cmd_vel_callback()                  │
│  ├── recv_topic             → recv_callback()  (串口状态)                │
│  ├── send_topic             → send_target_info_callback() (视觉目标)     │
│  ├── back_target_topic      → back_target_callback()                    │
│  └── human_goal_topic       → human_goal_callback() (云台手输入)         │
├─────────────────────────────────────────────────────────────────────────┤
│  发布话题 (3个)                                                          │
│  ├── cmd_vel_remap_topic    → 重映射速度                                │
│  ├── small_gesture_topic    → 姿态指令                                  │
│  └── back_target_rotate...  → 后视相机旋转指令                           │
└─────────────────────────────────────────────────────────────────────────┘
```

## 二、状态机设计

### 2.1 状态优先级（从高到低）
```
┌─────────────────┐
│  pose_from_human │  ← 最高优先级 (云台手打断)
│   (云台打断)     │
└────────┬────────┘
         │
┌────────▼────────┐
│   need_supply   │  ← 第二优先级 (血量/弹量低)
│    (补给模式)    │
└────────┬────────┘
         │
┌────────▼────────┐
│    is_pursue    │  ← 第三优先级 (发现目标)
│    (追击模式)    │
└────────┬────────┘
         │
┌────────▼────────┐
│    is_cruise    │  ← 最低优先级 (默认巡航)
│    (巡航模式)    │
└─────────────────┘
```

### 2.2 状态变量
| 状态变量 | 类型 | 说明 |
|---------|------|------|
| `is_pose_from_human_` | bool | 云台手输入打断 |
| `is_need_supply_` | bool | 需要补给 (HP<=180 或 弹量<=50) |
| `is_pursue_` | bool | 追击模式 (视觉检测到目标) |
| `is_cruise_` | bool | 巡航模式 (默认状态) |

## 三、核心逻辑分析

### 3.1 初始化流程

```cpp
initialization() {
    previous_cruise_mode_ = "cruise_goals_highland_mixed";
    current_cruise_mode_ = "cruise_goals_my_half";  // 默认我方半场巡航
    is_cruise_ = true;
    
    // 根据红蓝方加载不同的巡航路径参数
    if (self_color_ == RED) load("red");
    else load("blue");
}
```

### 3.2 高频回调函数 (200ms)

```cpp
high_frequence_callback() {
    if (颜色已设置 && !已初始化) {
        initialization();  // 延迟初始化
    } else if (已初始化) {
        // 打印调试信息
        打印当前巡航状态、目标点、状态机状态
    }
}
```

### 3.3 超高频回调函数 (1ms) - 【新增】

```cpp
super_high_frequence_callback() {
    // 1. 检查是否到达目标点，更新队列
    CheckIsReached_Update(0.25);
    
    // 2. 检测巡航模式切换，重新装填队列
    if (previous_cruise_mode_ != current_cruise_mode_) {
        FillGoalsDeque();  // 重新装填巡航队列
        // 打印日志: "切换到巡航模式，当前巡航模式为: xxx"
    }
}
```

**⚠️ 潜在问题分析：**
- 1ms周期非常短，频繁调用 `CheckIsReached_Update` 和 `FillGoalsDeque`
- `previous_cruise_mode_` 的更新位置需要注意

### 3.4 Tick 函数逻辑 (每帧调用)

```cpp
tick() {
    if (!is_init_) return FAILURE;
    
    setOutput("current_controller", "Omni");
    
    // 按优先级检查状态
    if (is_pose_from_human_) {
        // 切换到human模式，发布MOVE姿态
        setOutput("pose_from_human", true);
        setOutput("human_goal", ...);
        publish(Gesture::MOVE);
        return SUCCESS;
    }
    
    if (is_need_supply_) {
        // 【注意】这里也有 FillGoalsDeque() 调用
        if (previous_cruise_mode_ != current_cruise_mode_) {
            FillGoalsDeque();
        }
        setOutput("need_supply", true);
        setOutput("supply_goal", ...);
        publish(Gesture::DEFEND);
        return SUCCESS;
    }
    
    if (is_pursue_) {
        setOutput("is_pursue", true);
        setOutput("pursue_goal", ...);
        publish(Gesture::ATTACK);
        return SUCCESS;
    }
    
    if (is_cruise_) {
        // 【注意】这里也有 FillGoalsDeque() 调用
        if (previous_cruise_mode_ != current_cruise_mode_) {
            FillGoalsDeque();
        }
        setOutput("is_cruise", true);
        setOutput("cruise_goal", ...);
        publish(Gesture::MOVE);
        return SUCCESS;
    }
    
    return FAILURE;
}
```

## 四、关键函数分析

### 4.1 CheckIsReached_Update (到达检测与队列更新)

```cpp
CheckIsReached_Update(threshold) {
    // ========== 追击模式处理 ==========
    if (is_pursue_ && !is_pose_from_human_ && !is_need_supply_) {
        if (pursue_goal_deque_.empty()) return ERROR;
        
        current_goal = pursue_goal_deque_.front();
        distance = calc_distance(current_x_y_, current_goal);
        
        if (distance <= threshold) {
            switch (current_pursue_mode_) {
                case PURSUE:
                    if (队列大小 > 1) pop_front();
                    break;
                case STAY:
                    // 保持不动
                    break;
                case STAY_CIRCLE:
                    pop_front();
                    if (队列大小 < 3) create_pursue_stay_circle_path();
                    break;
                case FALLBACK:
                    // 待实现
                    break;
            }
        }
        return;
    }
    
    // ========== 巡航模式处理 ==========
    if (current_cruise_goal_deque_.empty()) {
        FillGoalsDeque();  // 队列为空时重新装填
        return;
    }
    
    current_goal = current_cruise_goal_deque_.front();
    distance = calc_distance(current_x_y_, current_goal);
    
    if (is_pose_from_human_) {
        if (distance <= threshold) {
            // 到达云台手目标点，切换回巡航模式
            change_cruise_mode_aft_human_goal();
        }
    }
    else if (is_need_supply_) {
        if (distance <= threshold) {
            // 【重要】到达补给点后，更新巡航模式
            previous_cruise_mode_ = current_cruise_mode_;
            current_cruise_mode_ = "cruise_goals_my_half";
        }
    }
    else if (is_cruise_) {
        if (distance <= threshold && 不是补给点/堡垒点) {
            current_cruise_goal_deque_.pop_front();
        }
        // 队列即将耗尽时自动重新装填
        if (current_cruise_goal_deque_.size() <= 2) {
            FillGoalsDeque();
        }
    }
}
```

### 4.2 FillGoalsDeque (装填巡航队列)

```cpp
FillGoalsDeque() {
    current_cruise_goal_deque_.clear();
    
    switch (current_cruise_mode_) {
        case "human":
            // 重复5次human_goal_pose_
            for (i=0; i<5; i++) push_back(human_goal_pose_);
            break;
            
        case "cruise_goals_highland_mixed":
            // 高地混合模式：highland + highland_buff_outpost，重复3次
            for (i=0; i<3; i++) {
                for (goal : path_pose_arrays_["cruise_goals_highland"]) push_back(goal);
                for (goal : path_pose_arrays_["cruise_goals_highland_buff_outpost"]) push_back(goal);
            }
            break;
            
        default:
            // 其他模式直接重复3次
            for (i=0; i<3; i++) {
                for (goal : path_pose_arrays_[current_cruise_mode_]) push_back(goal);
            }
    }
}
```

### 4.3 UpdateCurrentCruiseMode (巡航模式更新)

```cpp
UpdateCurrentCruiseMode() {
    previous_cruise_mode_ = current_cruise_mode_;
    
    // 保护supply和human模式不被覆盖
    if (current_cruise_mode_ == "supply" || current_cruise_mode_ == "human") {
        return;
    }
    
    // 30秒超时切换逻辑
    if (当前时间 - current_cruise_mode_start_time >= 30s) {
        if (current_cruise_mode_ == "cruise_goals_my_half")
            current_cruise_mode_ = "cruise_goals_highland_mixed";
        else if (current_cruise_mode_ == "cruise_goals_opponent_half")
            current_cruise_mode_ = "cruise_goals_highland_buff_outpost";
        else if (current_cruise_mode_ == "cruise_goals_highland_mixed")
            current_cruise_mode_ = "cruise_goals_opponent_half";
            
        current_cruise_mode_start_time = 当前时间;
        return;
    }
    
    // 根据血量差值切换模式
    outpost_hp_diff = 我方前哨血量 - 敌方前哨血量;
    base_hp_diff = 我方基地血量 - 敌方基地血量;
    
    if (base_hp_diff == 0) {  // 开始阶段
        if (outpost_hp_diff > 100) 
            current_cruise_mode_ = "cruise_goals_highland_mixed";
        else if (outpost_hp_diff < -100)
            current_cruise_mode_ = "cruise_goals_my_half";
    } else {
        if (base_hp_diff > 2000)
            current_cruise_mode_ = "cruise_goals_opponent_half";
        else if (base_hp_diff < -100)
            current_cruise_mode_ = "cruise_goals_my_half";
        else if (base_hp_diff > 100)
            current_cruise_mode_ = "cruise_goals_highland_mixed";
    }
}
```

## 五、逻辑时序图

### 5.1 正常巡航 → 发现目标 → 追击 → 回到巡航

```
时间轴 ───────────────────────────────────────────────────────────────►

[巡航状态]
   │
   │  is_cruise_=true
   │  current_cruise_goal_deque_ = [A, B, C, ...]
   │
   │     ┌─────────────────────────────────────────┐
   │     │  super_high_frequence_callback (1ms)    │
   │     │  - CheckIsReached_Update(0.25)          │
   │     │  - 检查模式切换                          │
   │     └─────────────────────────────────────────┘
   │                    ▲
   │                    │
   ▼                    │
[视觉检测到目标]        │
   │                    │
   │  send_target_info_callback()
   │  ├── target_state >= 1
   │  ├── target_type != 0/2/6
   │  ├── distance < 3.5 → current_pursue_mode_ = STAY_CIRCLE
   │  └── distance >= 3.5 → current_pursue_mode_ = PURSUE
   │
   │  is_cruise_ = false
   │  is_pursue_ = true
   │
   │  FillGoalsDeque_Pursue()  // 装填追击队列
   │  current_cruise_mode_start_time = now()  // 重置计时器
   │
   ▼
[追击状态]
   │
   │  tick() 返回 pursue_goal
   │  publish(Gesture::ATTACK)
   │
   │     ┌─────────────────────────────────────────┐
   │     │  CheckIsReached_Update (追击分支)       │
   │     │  - 计算到目标点距离                      │
   │     │  - 根据 pursue_mode 处理队列             │
   │     │    * PURSUE: 到达后pop，继续追击         │
   │     │    * STAY_CIRCLE: 到达后pop，补充满圆圈  │
   │     └─────────────────────────────────────────┘
   │
   ▼
[目标丢失]
   │
   │  send_target_info_callback()
   │  └── target_state < 1 或 target_type 无效
   │      is_cruise_ = true
   │      is_pursue_ = false
   │
   │  // 【关键】此时需要重新装填巡航队列！
   │  super_high_frequence_callback()
   │  ├── CheckIsReached_Update()
   │  └── previous_cruise_mode_ != current_cruise_mode_
   │      → FillGoalsDeque()  // 重新装填巡航队列
   │
   ▼
[回到巡航状态]
   │
   │  tick() 返回 cruise_goal
   │  publish(Gesture::MOVE)
```

### 5.2 巡航 → 云台手打断 → 回到巡航

```
[巡航状态]
   │
   ▼
[收到云台手目标点]
   │
   │  human_goal_callback()
   │  ├── human_goal_pose_ = 接收到的点
   │  ├── is_pose_from_human_ = true
   │  ├── previous_cruise_mode_ = current_cruise_mode_  // 保存当前模式
   │  └── current_cruise_mode_ = "human"
   │
   │  // super_high_frequence_callback 检测到模式变化
   │  └── FillGoalsDeque()  // 装填5次human_goal_pose_
   │
   ▼
[云台打断状态]
   │
   │  tick() 返回 human_goal
   │  publish(Gesture::MOVE)
   │
   │     ┌─────────────────────────────────────────┐
   │     │  CheckIsReached_Update (human分支)      │
   │     │  - 到达目标点后                          │
   │     │  └── change_cruise_mode_aft_human_goal()│
   │     │      current_cruise_mode_ = "cruise_goals_highland_mixed"
   │     │      is_pose_from_human_ = false        │
   │     └─────────────────────────────────────────┘
   │
   ▼
[回到巡航状态]
   │
   │  // super_high_frequence_callback 检测到模式变化
   │  └── FillGoalsDeque()  // 重新装填新的巡航队列
```

### 5.3 巡航 → 补给 → 回到巡航

```
[巡航状态]
   │
   ▼
[血量/弹量低触发补给]
   │
   │  recv_callback()
   │  ├── current_hp <= 180 或 allow_fire_amount <= 50
   │  ├── is_need_supply_ = true
   │  └── current_cruise_mode_ = "cruise_goals_supply"
   │
   │  // super_high_frequence_callback 检测到模式变化
   │  └── FillGoalsDeque()  // 装填补给点队列
   │
   ▼
[补给状态]
   │
   │  tick() 返回 supply_goal
   │  publish(Gesture::DEFEND)
   │
   │     ┌─────────────────────────────────────────┐
   │     │  CheckIsReached_Update (补给分支)       │
   │     │  - 到达补给点后                          │
   │     │  ├── previous_cruise_mode_ = current    │
   │     │  └── current_cruise_mode_ = "cruise_goals_my_half"
   │     │      // 补给完成后默认回我方半场巡航     │
   │     └─────────────────────────────────────────┘
   │
   │  recv_callback()
   │  ├── current_hp > 380 且 allow_fire_amount > 100
   │  └── is_need_supply_ = false  // 关闭补给标志
   │
   ▼
[回到巡航状态]
   │
   │  // super_high_frequence_callback 检测到模式变化
   │  └── FillGoalsDeque()  // 装填我方半场巡航队列
```

## 六、潜在问题分析

### 6.1 【严重】`previous_cruise_mode_` 更新不一致

**问题描述：**
- `UpdateCurrentCruiseMode()` 中设置了 `previous_cruise_mode_ = current_cruise_mode_`
- 但 `CheckIsReached_Update()` 的补给分支也有 `previous_cruise_mode_ = current_cruise_mode_`
- `tick()` 的巡航和补给分支检测 `previous_cruise_mode_ != current_cruise_mode_`，但 **没有更新** `previous_cruise_mode_`

**后果：**
```cpp
// 场景：从补给切换到巡航
// 1. UpdateCurrentCruiseMode() 或 recv_callback 改变了 current_cruise_mode_
// 2. super_high_frequence_callback() 检测到变化，调用 FillGoalsDeque()
// 3. 但 previous_cruise_mode_ 没有被更新！
// 4. 下一次 tick() 又检测到变化，又调用 FillGoalsDeque()
// 5. 导致队列被重复装填！
```

**修复建议：**
在 `super_high_frequence_callback()` 中，调用 `FillGoalsDeque()` 后需要更新 `previous_cruise_mode_`：

```cpp
void PubRobotStatus::super_high_frequence_callback()
{
    CheckIsReached_Update(0.25); 
    if (previous_cruise_mode_ != current_cruise_mode_)
    {
        FillGoalsDeque();
        previous_cruise_mode_ = current_cruise_mode_;  // 【缺少这一行！】
        RCLCPP_INFO(node_->get_logger(), "...");
    }
}
```

### 6.2 【中等】FillGoalsDeque 重复调用

**问题描述：**
- `super_high_frequence_callback()` 1ms调用一次，频繁检测和装填
- `tick()` 每次行为树tick也会检测和装填
- 两者都检测 `previous_cruise_mode_ != current_cruise_mode_`

**风险：**
- 如果 `previous_cruise_mode_` 更新逻辑不一致，可能导致重复装填
- 高频调用 `FillGoalsDeque()` 会频繁 `clear()` 和重新填充队列

### 6.3 【轻微】1ms周期可能过于频繁

**分析：**
- `super_high_frequence_timer_` 周期为 1ms
- 每次调用执行 `CheckIsReached_Update(0.25)` 和模式检测
- 如果不需要超高频响应，建议调整为 10ms 或 50ms

### 6.4 【轻微】日志打印过于频繁

**分析：**
- `super_high_frequence_callback()` 中的日志在模式切换时会打印
- 1ms周期下，如果 `previous_cruise_mode_` 不更新，会刷屏

## 七、修复建议总结

### 7.1 必须修复

```cpp
void PubRobotStatus::super_high_frequence_callback()
{
    CheckIsReached_Update(0.25); 
    if (previous_cruise_mode_ != current_cruise_mode_)
    {
        // 刚切换到巡航模式，重新装填
        FillGoalsDeque();
        
        // 【新增】更新 previous_cruise_mode_，防止重复装填
        previous_cruise_mode_ = current_cruise_mode_;
        
        RCLCPP_INFO(node_->get_logger(), "\033[34m[巡航模式切换] 切换到: %s\033[0m", 
                    current_cruise_mode_.c_str());
    }
}
```

### 7.2 可选优化

1. **调整定时器周期：** 1ms → 10ms 或 20ms
2. **统一模式切换逻辑：** 将 `previous_cruise_mode_` 的更新集中到一处
3. **增加防抖动：** 避免短时间内的频繁模式切换

## 八、测试场景设计

### 场景1：正常巡航切换
```
初始: current="my_half", previous="highland_mixed"
触发: 10秒定时器触发 UpdateCurrentCruiseMode()
预期: 
  - current 变为新值
  - super_high_frequence_callback() 检测到变化
  - FillGoalsDeque() 被调用
  - previous 被更新为新值
  - 后续不再重复装填
```

### 场景2：追击后回到巡航
```
初始: is_cruise_=true, current="my_half"
触发: 视觉检测到目标
预期:
  - is_cruise_=false, is_pursue_=true
  - 追击逻辑执行
触发: 目标丢失
预期:
  - is_cruise_=true, is_pursue_=false
  - super_high_frequence_callback() 可能需要重新装填队列
```

### 场景3：补给完成
```
初始: is_need_supply_=true, current="supply"
触发: 血量恢复
预期:
  - is_need_supply_=false
  - current="my_half"
  - super_high_frequence_callback() 装填新的巡航队列
  - previous 被更新
```
