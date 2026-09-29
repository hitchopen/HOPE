# 必要项，标识调用方来源
string pkg_name

# 必要项，优先级(1~10, 默认6)
uint32 priority

# 可选项，(1~100) priority + priority_weight% is final priority
uint32 priority_weight