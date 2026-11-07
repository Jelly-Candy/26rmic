# MaixCAM2 蓝光反射容忍版：SSH 运行说明

本包为完整源码及随项目提供的头文件，不含旧的可执行文件。设备上已编译的程序无需重新编译即可启动。

## 连接现有摄像头

在电脑终端执行（MaixVision 内已经打开的 SSH 终端可直接跳过此步）：

```sh
ssh root@192.168.1.200
```

使用设备自己的登录密码。

## 在当前设备运行已经编译好的版本

```sh
cd /root/MaixCAM2_virtual_vision_20261107
sh stop.sh
sh start.sh
```

当前这次会话启动的是后台程序，先执行 stop.sh 释放摄像头，再用 start.sh 前台运行。前台运行时按 Ctrl+C 停止。
如果 stop.sh 提示没有后台 PID，而你确认程序已停止，可直接执行 sh start.sh。
保持 MaixVision 连接，可以看图像预览。不要同时运行其他占用摄像头的程序。

## 从本源码包重新编译（仅在需要恢复或重建时）

先将压缩包上传到设备 /root/。电脑上传示例：

```sh
scp MaixCAM2_blue_spill_source_20261107.tar.gz root@192.168.1.200:/root/
```

SSH 登录设备后执行：

```sh
sh /root/MaixCAM2_virtual_vision_20261107/stop.sh
# 备份现有项目后再解压
stamp=$(date +%Y%m%d_%H%M%S)
tar -czf "/root/MaixCAM2_before_restore_${stamp}.tar.gz" -C /root MaixCAM2_virtual_vision_20261107
tar -xzf /root/MaixCAM2_blue_spill_source_20261107.tar.gz -C /root
cd /root/MaixCAM2_virtual_vision_20261107
make -B -j1 all test
sh start.sh
```

首次安装而没有原项目时，跳过 stop.sh 与备份命令。编译依赖当前 MaixCAM2 已配好的 SDK、OpenCV 和运行库；这不是 Windows 直接运行包。

## 后台运行、看日志与停止

```sh
cd /root/MaixCAM2_virtual_vision_20261107
mkdir -p logs
nohup sh start.sh > logs/manual_live.log 2>&1 &
echo $! > live.pid
```

```sh
tail -f /root/MaixCAM2_virtual_vision_20261107/logs/manual_live.log
```

```sh
sh /root/MaixCAM2_virtual_vision_20261107/stop.sh
```

只允许一个程序使用摄像头。如果提示已有程序运行，先在原前台窗口按 Ctrl+C，或停止已记录的后台程序；不要反复启动。

## 本版参数与验证范围

- 保留禁止跨第三根灯条配对。
- 灯条间暗色比例至少 85%，每行至少 70%。
- 容忍蓝光照亮的黑色面板，同时限制红/绿通道与内部亮度，内部须明显暗于灯条。
- 具体判定在 main/src/ColorContourDetector.cpp 的 blackInteriorBetween 中，普通与倾斜配对共用。
- camera.conf 保留当前曝光、旋转 180 度及预览设置。
- 五项回归测试、倾斜配对测试通过；原始照片回放识别中央目标 1 个，左右可疑配对拒绝。
- 已启动并看到实时预览，约 27–30 帧/秒。最后启动检查中当前视角检测数为 0；实时相邻目标误配验证尚未完成。
- verification 中附当前编译回放、启动日志。旧 README 中的历史版本描述仅供参考，本文件描述当前交付版本。
