# 清洁任务消息
# 描述一个需要清洁的目标点

# 任务ID
uint32 task_id

# 目标点在地图坐标系中的位置
geometry_msgs/Point target_position

# 清洁类型 (0: 普通清洁, 1: 深度清洁, 2: 定点清洁)
uint8 cleaning_type

# 任务优先级 (0-255, 越大优先级越高)
uint8 priority

# 任务状态 (0: 待执行, 1: 执行中, 2: 已完成, 3: 失败)
uint8 status

# 时间戳
builtin_interfaces/Time timestamp

# 附加信息
string description

