Import('env')
# 导入操作系统模块，用于操作文件路径等
import os

# 定义输出目录路径，使用环境变量BUILD_DIR和操作系统路径分隔符拼接
OUTPUT_DIR = "$BUILD_DIR{}".format(os.path.sep)
# 定义应用程序二进制文件路径，使用环境变量BUILD_DIR和程序名PROGNAME拼接
APP_BIN = "$BUILD_DIR/${PROGNAME}.bin"

# 定义一个函数，用于复制和合并二进制文件
def copy_merge_bins(source, target, env):
    # 获取目标文件路径（固件源文件）
    firmware_src = str(target[0])
    # 获取额外的闪存映像文件列表，并将其与默认的偏移量和应用程序二进制文件路径合并
    flash_images = env.Flatten(env.get("FLASH_EXTRA_IMAGES", [])) + ["$ESP32_APP_OFFSET", APP_BIN]
    # 从固件源文件路径中提取文件名（不包含路径）
    name = firmware_src.split(os.path.sep)[2]
    # 获取开发板配置信息
    board = env.BoardConfig()
    # 获取开发板的闪存大小，默认为4MB
    flash_size = board.get("upload.flash_size", "4MB")
    # 获取开发板的闪存频率，默认为40MHz
    f_flash = board.get("build.f_flash", "40000000L")
    # 根据f_flash值设置闪存频率
    flash_freq = '40m'
    if (f_flash == '80000000L'):
        flash_freq = '80m'
    # 获取开发板的主控芯片型号，默认为esp32
    mcu = board.get("build.mcu", "esp32")
    # 构造目标固件文件路径，包含输出目录、主控芯片型号、文件名、闪存大小和偏移量
    current_directory = os.getcwd()
    firmware_dst = "{}/firmware/{}_{}_0x0.bin".format(current_directory, mcu, flash_size)
    # 如果目标固件文件已存在，则删除
    if os.path.isfile(firmware_dst):
        os.remove(firmware_dst)
    # 构造合并二进制文件的命令
    cmd = " ".join(
        ["$PYTHONEXE", "$OBJCOPY", '--chip', mcu, 'merge_bin', '--output', firmware_dst, '--flash_mode', 'dio',
         '--flash_size', flash_size, '--flash_freq', flash_freq] + flash_images)
    # 执行合并命令
    env.Execute(cmd)

# 添加一个后处理动作，在构建完成后执行copy_merge_bins函数
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", [copy_merge_bins])