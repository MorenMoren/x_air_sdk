#!/bin/bash

# 批量配置CAN接口的脚本
# 配置CAN0-CAN3接口，设置为CAN-FD模式

ok=0
skip=0

for i in {0..3}; do
    # 先关闭接口（如果已经启动）
    sudo ip link set can$i down 2>/dev/null

    # 配置CAN接口参数，失败说明设备不存在
    if ! sudo ip link set can$i type can bitrate 1000000 dbitrate 5000000 fd on 2>/dev/null; then
        echo "can$i: 设备不存在"
        ((skip++))
        continue
    fi

    # 增大发送队列长度，避免高频发送时阻塞 by: kds 2026-06-24
    #sudo ip link set can$i txqueuelen 1000

    # 启动接口
    if sudo ip link set can$i up 2>/dev/null; then
        echo "can$i: 配置成功"
        ((ok++))
    else
        echo "can$i: 启动失败"
        ((skip++))
    fi
done

if [ $ok -eq 0 ]; then
    echo "未检测到 CAN 硬件，请检查设备连接"
else
    echo "完成: $ok 个接口已配置，$skip 个跳过"
fi
