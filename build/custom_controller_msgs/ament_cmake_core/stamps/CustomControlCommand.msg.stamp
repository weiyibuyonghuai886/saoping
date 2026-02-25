# CustomControlCommand.msg
std_msgs/Header header
string[] joint_names

# 每个关节的控制命令
float64[] position      # 位置命令
float64[] velocity     # 速度命令
float64[] effort        # 力矩命令
float64[] kp       # 比例增益
float64[] kd       # 微分增益

# 可选：时间戳或执行时间
builtin_interfaces/Duration time_from_start