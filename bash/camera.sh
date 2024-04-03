#!/bin/bash

# 定义曝光调节函数
adjust_exposure() {
    # 调节曝光的 v4l2-ctl 命令，这里可以根据需要自行调整参数
    v4l2-ctl --set-ctrl exposure_absolute=$1
}

# 监听键盘输入函数
listen_keyboard() {
    local key
    # 无限循环监听键盘输入
    while true; do
        # 读取单个字符输入
        read -rsn1 key
        # 如果输入为 F2 键，则返回
        if [[ "$key" == $'\eOP' ]]; then
            return
        fi
    done
}

# 主函数
main() {
    echo "按下 F2 键以切换曝光，按下任意其他键退出..."

    # 这里定义曝光值初始为 10
    exposure_value=10

    # 在 10 和 150 之间切换曝光值，直到按下 F2 键
    while true; do
        echo "调节曝光值为 $exposure_value..."
        adjust_exposure $exposure_value
        sleep 5 # 等待一段时间，确保相机有足够时间调整曝光

        # 切换曝光值方向，到达边界时反向
        if [ $exposure_value -eq 10 ]; then
            exposure_value=150
        else
            exposure_value=10
        fi

        # 监听键盘输入
        listen_keyboard
        # 如果监听到 F2 键，则退出循环
        if [ $? -eq 0 ]; then
            break
        fi
    done

    echo "曝光调节完成。"
}

# 调用主函数
main