# 					云台控制


硬件：
达妙DM-J4310-2EC电机，达妙H7开发板，JY61P模块

功能：最终要实现云台自稳定

## 项目使用

使用vscode中编写，使用CMake生成构建系统，ninja生成构建项目
前提是需要安装 `CMake`、`ARM GCC 工具链`、`Ninja`等工具

```powershell
# 1. 回到根目录
cd E:\Github_warehouse\JumpCat_Gimbal\FreeRTOS_Project

# 2. 清理旧的 build
Remove-Item build -Recurse -Force -ErrorAction SilentlyContinue

# 3. 用 Debug 预设配置
cmake --preset Debug

# 4. 注意！进入正确的目录：build/Debug
cd build/Debug

# 5. 编译
ninja


# 或者一键编译烧录(快捷键)
Crtl + Shift + B
```





## 项目简介介绍

```tex
JumpCat_Gimbal/
│
├── .assets/                              # README资源文件
│
├── FreeRTOS_Project/                     # STM32 + FreeRTOS 工程
│   │
│   ├── .vscode/                          # VS Code 配置
│   ├── build/                            # 编译输出目录
│   ├── cmake/                            # CMake 模块
│   │
│   ├── App/                              # 应用程序层
│   ├── Core/                             # 核心驱动层 (HAL/LL)
│   ├── Drivers/                          # 官方驱动库
│   ├── Middlewares/                      # 中间件 (FreeRTOS, USB等)
│   ├── USB_DEVICE/                       # USB设备相关
│   ├── User_Config/                      # 用户配置文件
│   │
│   ├── User_File/                        # 用户应用代码 (核心工作区)
│   │   │
│   │   ├── 1_Middleware/                 # 中间件封装层
│   │   │   │
│   │   │   ├── mw_pid.cpp/h              # PID控制器封装
│   │   │   ├── mw_filter.cpp/h           # 数字滤波器
│   │   │   └── mw_math.cpp/h             # 数学工具函数
│   │   │
│   │   ├── 2_Device/                     # 设备驱动层
│   │   │   │
│   │   │   ├── dvc_jy61p.cpp/h           # JY61P姿态传感器驱动
│   │   │   ├── dvc_vofa.cpp/h            # VOFA上位机协议驱动
│   │   │   ├── dvc_motor_dm.cpp/h        # 达妙电机驱动 (CAN)
│   │   │   ├── dvc_ws2812.cpp/h          # WS2812 RGB灯驱动
│   │   │   
│   │   │
│   │   └── 4_Task/                       # FreeRTOS任务层 (主要工作目录)
│   │       │
│   │       ├── task_ws2812.cpp/h         # RGB指示灯任务 (优先级最低)
│   │       │
│   │       ├── task_jy61p.cpp/h          # JY61P姿态采集任务
│   │       │   ├── 读取IMU数据
│   │       │   ├── 解析四元数/欧拉角
│   │       │   └── 提供 Get_Current_Yaw/Pitch/Roll() 接口
│   │       │
│   │       ├── task_motor_dm.cpp/h       # 达妙电机通信任务
│   │       │   ├── CAN/串口通信
│   │       │   ├── MIT模式指令封装
│   │       │   └── 提供 Motor_DM_Set_Speed() 接口
│   │       │
│   │       ├── task_vofa.cpp/h           # VOFA上位机调试任务
│   │       │   ├── 50Hz发送IMU数据到上位机
│   │       │   ├── 解析命令: yaw=90#, pitch=30#, roll=15#
│   │       │   ├── 调用 Set_Target_Yaw/Pitch/Roll() 接口
│   │       │   └── 保留 q00/r00 等调试命令
│   │       │
│   │       ├── task_angle_control.cpp/h  # 角度控制任务 (核心)
│   │       │   ├── 接收VOFA传来的目标角度
│   │       │   ├── 读取JY61P当前角度
│   │       │   ├── PID计算: 角度误差 → 目标速度
│   │       │   ├── 处理偏航轴±180°环绕
│   │       │   └── 调用 Motor_DM_Set_Speed() 执行
│   │       │
│   │       ├── task_balance_test.cpp/h   # 平衡测试任务 (备选/废弃)
│   │       └── task_control_old.cpp/h    # 早期控制任务 (已废弃)
│   │
│   ├── CMakeLists.txt                    # CMake构建配置
│   ├── CMakePresets.json                 # CMake预设
│   ├── H7_test.ioc                       # STM32CubeMX配置
│   ├── openocd.cfg                       # OpenOCD调试配置
│   ├── startup_stm32h723xx.s             # 启动文件
│   ├── STM32H723VGTX_RAM.ld              # RAM链接脚本
│   ├── STM32H723XG_FLASH.ld              # FLASH链接脚本
│   └── .mxproject                        # CubeMX项目文件
│
├── VOFA+/                                # VOFA+ 调试工具配置
│   └── vofa+.cmds.json                   # VOFA+ 命令脚本 (一些快捷命令)
│	└── new tab.tab.json				# VOFA+ 的通道标签
│
├── README.md                             # 项目说明文档
└── .gitignore                            # Git忽略配置
```

## 云台思路

```tex
VOFA发送 yaw=90°
      ↓
set_angle_test_task 接收目标角度
      ↓
读取 JY61P 当前角度 (如当前30°)
      ↓
计算角度误差 = 90° - 30° = 60°
      ↓
PID控制器: 误差60° → 目标速度 (rad/s)
      ↓
motor_dm_task: MIT模式控制电机转动
      ↓
云台转动，JY61P实时反馈
      ↓
角度误差逐渐减小 → 速度逐渐降低
      ↓
到达90° → 误差为0 → 停止

```



| `yaw=数值#`       |            **例如 `yaw=90#` 让云台转到90度方向**             | **设定目标偏航角** |      |
| ----------------- | :----------------------------------------------------------: | ------------------ | ---- |
| **`pitch=数值#`** |             **例如 `pitch=30#` 让云台上仰30度**              | **设定目标俯仰角** |      |
| **`roll=数值#`**  |              **例如 `roll=15#` 让云台侧倾15度**              | **设定目标横滚角** |      |
| **`kp_x=数值#`**  | **例如 `kp_x=1.0#` 让 `水平 `电机角度误差转换成目标速度的kp参数设为1.0** | **设定kp值**       |      |
| **`kp_y=数值#`**  | **例如 `kp_y=2.8#` 让 `竖直 `电机角度误差转换成目标速度的kp参数设为2.8** | **设定kp值**       |      |
| **`kd_x=数值#`**  | **例如 `kd_x=3.5#` 让 `水平 `电机角度误差转换成目标速度的kd参数设为3.5** | **设定kd值**       |      |
| **`kd_y=数值#`**  | **例如 `kd_y=2.5#` 让 `竖直 `电机角度误差转换成目标速度的kd参数设为2.5** | **设定kd值**       |      |
| **`en=0.0#`**     | **例如 `en=0.0#` 让竖直和水平电机失能呢个，不使用VOFA调参**  | **设定kd值**       |      |

如果使用VOFA调参需要先将`en`设置为`大于0`的数，例如**`en=1.0#`**

现在就可以使用**VOFA在线调节参数**，不需要手动一遍一遍的修改再烧录了

**下图可以看到`目标角度`和`当前角度`跟踪的还是挺不错的**

![双轴设定角度](./.assets/双轴设定角度.png)

### 注意

该工程中需要确认这个`JY61P`的**安装方向**，这里使用的**roll角**来确定的俯仰

这里贴出 *目前使用*  的 JY61P 的 *正方向*

*<img src="./.assets/JY61P%E6%AD%A3%E6%96%B9%E5%90%91.png" alt="JY61P正方向" style="zoom: 50%;" />*

**运动示意图：**
![运动示意图](./.assets/%E8%BF%90%E5%8A%A8%E7%A4%BA%E6%84%8F%E5%9B%BE.png)
