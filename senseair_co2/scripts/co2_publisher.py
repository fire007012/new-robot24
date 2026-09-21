#!/usr/bin/env python3
# 这行是告诉系统：用 Python3 来运行这个文件（ROS 节点必须这样写）

import rospy          # 导入 ROS 的 Python 接口（没有它就无法发布话题）
import socket         # 导入 socket 库，用于通过网络（TCP）连接 TTL-NET 模块
import time           # 导入时间库（后面用来控制每4秒读一次）
from std_msgs.msg import Int32   # 从 ROS 标准消息库导入 Int32 类型（用来发布 CO₂ 数值）

# ================== 配置区 ==================
# 这里集中放所有可以修改的参数，方便你以后调整
MODULE_IP = '192.168.1.200'   # ← 你的 TTL-NET 模块的 IP 地址（在配置软件里看到的）
PORT = 6600                   # ← 模块配置的本地端口号（TCP SERVER 模式下用的端口）

# Modbus RTU 请求帧（固定写法，不用改）
# 这串字节是发给 S8 LP 传感器的“指令”：请把 CO₂ 值告诉我
# 含义：FE = 任意从机地址，04 = 读输入寄存器，0003 = CO₂ 所在的寄存器地址
REQUEST = bytes([0xFE, 0x04, 0x00, 0x03, 0x00, 0x01, 0xD5, 0xC5])


# ================== 读取 CO₂ 的函数 ==================
def read_co2():
    """
    这个函数负责通过网络去问传感器要 CO₂ 值
    返回值：成功时返回 CO₂ 数值（整数），失败时返回 None
    """
    try:
        # 1. 创建一个 TCP 网络连接（就像打电话给模块）
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3)                    # 设置超时时间为 3 秒，防止卡死
            s.connect((MODULE_IP, PORT))       # 连接到 TTL-NET 模块
            
            # 2. 发送读取指令给传感器
            s.sendall(REQUEST)
            
            # 3. 等待传感器回复（最多等 3 秒）
            response = s.recv(7)               # S8 LP 正常回复正好 7 个字节
            
            # 4. 检查回复是否完整
            if len(response) >= 7:
                # 解析 CO₂ 值（第 4、5 个字节是大端存储的数值）
                co2 = (response[3] << 8) | response[4]
                return co2                     # 返回 CO₂ 浓度（单位：ppm）
    
    except:
        # 如果网络断开、模块没响应等情况，就安静地返回 None
        # （ROS 会自动重试，不需要打印错误）
        pass
    
    return None   # 读取失败时返回 None


# ================== 主程序（ROS 节点真正开始运行的地方） ==================
if __name__ == '__main__':
    # 1. 初始化 ROS 节点
    rospy.init_node('senseair_co2_publisher', anonymous=True)
    # 节点名字叫 senseair_co2_publisher，anonymous=True 让 ROS 自动加后缀防止重名

    # 2. 创建一个发布者（Publisher）
    # 意思是：我要向整个 ROS 系统发布 CO₂ 数据，话题名字叫 /co2_ppm
    pub = rospy.Publisher('/co2_ppm', Int32, queue_size=10)

    # 3. 设置发布频率：每秒 0.25 次 = 每 4 秒发布一次（和传感器测量周期一致）
    rate = rospy.Rate(0.25)

    # 打印启动信息（方便你在终端看到）
    rospy.loginfo("SenseAir S8 LP CO₂ 发布节点已启动 → %s:%d", MODULE_IP, PORT)
    rospy.loginfo("发布话题: /co2_ppm")

    # 4. 主循环：只要 ROS 没关闭，就一直运行
    while not rospy.is_shutdown():
        # 读取一次 CO₂
        co2_value = read_co2()
        
        if co2_value is not None:          # 读取成功
            pub.publish(co2_value)         # 把数值发布到 /co2_ppm 话题
        else:
            rospy.logwarn("读取失败，正在重连...")   # 偶尔失败很正常，自动重试
        
        rate.sleep()   # 等待 4 秒（保持每 4 秒一次的节奏）
