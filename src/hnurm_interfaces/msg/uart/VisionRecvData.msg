std_msgs/Header header

SelfColor self_color
WorkMode work_mode
BulletSpeed bullet_speed
Gesture gesture

float32 roll
float32 pitch
float32 yaw

float32 control_id

float32 game_progress # 0：未开始比赛 1：准备阶段 2：十五秒裁判系统自检阶段 3：五秒倒计时 4：比赛中 5：比赛结算中
float32 current_hp
float32 current_base_hp 		
float32 allow_fire_amount  	
float32 current_outpost_hp  	
float32 current_enemy_base_hp   
float32 current_enemy_outpost_hp  

float32 remain_time 
float32 center_ctrl # 0 为未被占领，1 为被己方占领，2 为被对方占领，3 为被双方占领
float32 cmd_x 
float32 cmd_y 