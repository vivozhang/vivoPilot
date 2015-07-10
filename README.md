#树莓派飞控RasPilot#

###硬件安装和接线###

###写入系统镜像到树莓派SD卡###

###配置WiFi连接###

###安装和设置飞控程序###

###编译飞控代码###
从Copter-3.3开始，ardupilot的编译需要使用4.7版本以上的gcc。而树莓派系统上还是4.6版本的gcc，所以建议在Ubuntu下面交叉编译代码。
这里使用树莓派官方提供的交叉编译工具<br>
<br>
1 下载树莓派编译工具，例如这里放到/opt目录下<br>
　`sudo git clone --depth 1 https://github.com/raspberrypi/tools.git /opt/tools`<br>
<br>
2 设置环境变量<br>
　32位系统输入：`export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian/bin:$PATH`<br>
　64位系统输入：`export PATH=/opt/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin:$PATH`<br>
<br>
3 编译ardupilot<br>
* 从GitHub上获取源代码：<br>
　`git clone https://github.com/raspilot/ardupilot-raspilot.git`<br>
* 进入ArduCopter目录后编译：<br>
　`cd ardupilot-raspilot/ArduCopter`<br>
　`make raspilot`<br>
* 通过WiFi同步到树莓派：<br>
　`rsync -avz /tmp/ArduCopter.build/ArduCopter.elf pi@192.168.1.100:/home/pi/`<br>
　其中192.168.1.100改为树莓派的实际IP地址<br>
