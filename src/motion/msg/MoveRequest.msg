# 默认槽位沿用官方路由：原地转向使用 turn 模型，其余使用标准 walk 模型。
int8 WALK_SLOT_DEFAULT=-1
int8 walk_slot
# 机身坐标系下的期望速度，交给选中的行走模型作为命令输入。
float64 forward
float64 left
float64 turn
