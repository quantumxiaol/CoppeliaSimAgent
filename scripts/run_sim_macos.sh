#!/bin/bash

# 定义 App 的路径（根据你实际存放的位置修改）
APP_PATH="./CoppeliaSimSoftware/coppeliaSim.app"

if [ -d "$APP_PATH" ]; then
    echo "正在启动 CoppeliaSim..."
    open "$APP_PATH"
else
    echo "错误: 找不到路径 $APP_PATH"
fi