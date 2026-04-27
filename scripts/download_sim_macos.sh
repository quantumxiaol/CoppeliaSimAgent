#!/bin/bash

# 1. 定义变量
URL="https://downloads.coppeliarobotics.com/V4_10_0_rev0/CoppeliaSim_Edu_V4_10_0_rev0_macOS15_arm64.zip"
ZIP_NAME="temp_coppelia.zip"
TARGET_PARENT="CoppeliaSimSoftware"
FINAL_APP_NAME="coppeliaSim.app"

echo "🚀 正在下载 CoppeliaSim..."

# 2. 下载文件
curl -L -# "$URL" -o "$ZIP_NAME"

# 3. 准备目标文件夹
# 如果文件夹已存在，先删除（可选，为了确保全新安装）
rm -rf "$TARGET_PARENT"
mkdir -p "$TARGET_PARENT"

echo "📦 正在解压并整理路径..."

# 4. 解压到临时目录并提取 .app
# 创建一个临时目录用于解压
TEMP_DIR="temp_extract"
mkdir -p "$TEMP_DIR"
unzip -q "$ZIP_NAME" -d "$TEMP_DIR"

# 5. 寻找解压出的 .app 文件夹并移动到指定位置
# 使用 find 找到解压出来的 .app 路径（因为解压出的原名通常很长）
EXTRACTED_APP=$(find "$TEMP_DIR" -name "*.app" -type d -maxdepth 2 | head -n 1)

if [ -n "$EXTRACTED_APP" ]; then
    # 移动并重命名为目标路径
    mv "$EXTRACTED_APP" "$TARGET_PARENT/$FINAL_APP_NAME"
    echo "✅ 已成功整理为: $TARGET_PARENT/$FINAL_APP_NAME"
    
    # 6. 【关键步骤】移除 macOS 的下载隔离属性，否则脚本无法直接启动它
    echo "🛡️  正在解除 macOS 安全隔离限制..."
    sudo xattr -r -d com.apple.quarantine "$TARGET_PARENT/$FINAL_APP_NAME"
else
    echo "❌ 错误：在压缩包中未找到 .app 文件"
fi

# 7. 清理垃圾
rm -rf "$ZIP_NAME" "$TEMP_DIR"

echo "🎉 准备完成！CoppeliaSim 已经成功安装了。"