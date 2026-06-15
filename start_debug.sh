#!/bin/bash
# 一键启动GDB调试（自动分割终端，需要tmux）

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "=== STM32F407 编码器调试环境 ==="
echo ""

# 检查tmux
if ! command -v tmux &> /dev/null; then
    echo "未安装tmux，使用手动模式"
    echo ""
    echo "请按照以下步骤操作："
    echo ""
    echo "【终端1】启动OpenOCD服务器:"
    echo "  cd $SCRIPT_DIR"
    echo "  ./debug_session.sh"
    echo ""
    echo "【终端2】启动GDB客户端:"
    echo "  cd $SCRIPT_DIR/Prototype_04_407_VE_Matlab/type_04_407VE"
    echo "  gdb-multiarch -x debug_stm32.gdb build_gcc/type_04_407VE.elf"
    echo ""
    exit 0
fi

# 使用tmux自动分割
SESSION="stm32_debug"

# 如果会话已存在，先杀掉
tmux kill-session -t $SESSION 2>/dev/null

# 创建新会话
tmux new-session -d -s $SESSION -n "STM32调试"

# 上半部分：OpenOCD服务器
tmux send-keys -t $SESSION "cd $SCRIPT_DIR" C-m
tmux send-keys -t $SESSION "echo '=== OpenOCD GDB Server ==='" C-m
tmux send-keys -t $SESSION "./debug_session.sh" C-m

# 分割窗口
tmux split-window -v -t $SESSION

# 下半部分：GDB客户端（等待2秒让OpenOCD启动）
tmux send-keys -t $SESSION "cd $SCRIPT_DIR/Prototype_04_407_VE_Matlab/type_04_407VE" C-m
tmux send-keys -t $SESSION "echo '=== GDB Client (等待OpenOCD启动...) ==='" C-m
tmux send-keys -t $SESSION "sleep 2" C-m
tmux send-keys -t $SESSION "gdb-multiarch -x debug_stm32.gdb build_gcc/type_04_407VE.elf" C-m

# 调整窗口大小
tmux select-layout -t $SESSION even-vertical

# 附加到会话
tmux attach -t $SESSION
