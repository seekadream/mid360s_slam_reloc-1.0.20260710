#!/bin/bash
# 雷达网络配置（开机自启）
# 1. 删除eth0上自动生成的192.168.0.0/24路由（与WiFi冲突）
/sbin/ip route del 192.168.0.0/24 dev eth0 2>/dev/null
# 2. 添加雷达精确路由（只走eth0）
/sbin/ip route replace 192.168.0.126/32 dev eth0 src 192.168.0.108 2>/dev/null
# 3. WiFi出去的包强制使用WiFi的源IP
/sbin/iptables -t nat -C POSTROUTING -o wlP4p65s0 -j MASQUERADE 2>/dev/null || \
/sbin/iptables -t nat -A POSTROUTING -o wlP4p65s0 -j MASQUERADE
exit 0
