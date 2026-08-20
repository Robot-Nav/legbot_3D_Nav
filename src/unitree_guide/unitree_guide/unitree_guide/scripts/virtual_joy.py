"""
    @brief:The analog handle is used to control the unitree robot
    @Editor:CJH
    @Date:2025/10/31
"""
import uinput
import time

# 定义虚拟手柄的按键和轴（可根据需求调整）
events = (
    uinput.BTN_A,    # 按键A
    uinput.BTN_B,    # 按键B
    uinput.ABS_X + (0, 255, 0, 0),  # X轴（范围0-255）
    uinput.ABS_Y + (0, 255, 0, 0),  # Y轴（范围0-255）
    uinput.ABS_Z + (0, 255, 0, 0),
)

# 初始化虚拟手柄，设备名称为"Virtual Gamepad"
device = uinput.Device(events, "Virtual Gamepad")
print("虚拟手柄已创建，设备节点：/dev/input/js0")

# 保持程序运行（否则设备会销毁），可按Ctrl+C退出
try:
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    print("虚拟手柄已销毁")
