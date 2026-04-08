# UART 颜色调试修改记录

## 修改目的
查看当 `self_color` 无效时，其他字段（yaw/pitch/hp 等）是否正常接收。

## 修改位置
`src/hnurm_uart/src/uart_node.cpp` 第 242-249 行

## 修改内容
在颜色判断之前添加了打印日志，输出所有接收到的字段：
- self_color
- yaw, pitch, roll
- current_hp
- game_progress
- allow_fire_amount

## 恢复方法
如需恢复原代码，删除打印的 RCLCPP_INFO 行即可。
