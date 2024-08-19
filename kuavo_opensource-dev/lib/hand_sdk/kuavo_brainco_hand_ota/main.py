import subprocess
import os

def main():
    # 菜单选择
    print("请选择需要设置的功能：")
    print("1. OTA")
    print("2. 恢复手的配置")
    option = input("请输入选项 (1 或 2): ")

    if option not in ['1', '2']:
        print("无效选项，请重新运行脚本并选择 1 或 2。")
        return

    # 设备选择
    print("请选择设备名称：")
    print("1. /dev/stark_serial_L ：左手")
    print("2. /dev/stark_serial_R ：右手")
    device_option = input("请输入选项 (1 或 2): ")

    if device_option == '1':
        device_name = "/dev/stark_serial_L"
    elif device_option == '2':
        device_name = "/dev/stark_serial_R"
    else:
        print("无效选项，请重新运行脚本并选择 1 或 2。")
        return

    # 设置程序路径
    build_dir = os.path.join(os.getcwd(), "build")
    if option == '1':
        program = os.path.join(build_dir, "ota")
    elif option == '2':
        program = os.path.join(build_dir, "rs485")

    # 执行相应程序
    run_program(program, device_name)

def run_program(program, device_name):
    try:
        subprocess.run([program, device_name], check=True)
    except subprocess.CalledProcessError as e:
        print(f"程序执行失败: {e}")
        print("请检查硬件连接。")
    except FileNotFoundError:
        print(f"找不到程序: {program}")
        print("请先编译工程。")
    except Exception as e:
        print(f"发生错误: {e}")
        print("请检查是否配置库环境。")

if __name__ == "__main__":
    main()
