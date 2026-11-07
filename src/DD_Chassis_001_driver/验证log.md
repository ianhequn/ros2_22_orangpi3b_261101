# 验证过程的日志

```bash
orangepi@orangepi3b:~/MyWork/Robot/ros2/ws_orangepi_01$ ros2 run DD_Chassis_001_driver serial_monitor.py --device /dev/ttyUSB0 --baud 1000000 --frame-end 0x7F800000
帧率≈153.8Hz  imu1= 142.820 imu2=  -7.280 imu3= 175.399  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈120.1Hz  imu1= 142.820 imu2=  -7.277 imu3= 175.394  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈119.9Hz  imu1= 142.758 imu2=  -6.923 imu3= 175.297  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈120.0Hz  imu1= 142.843 imu2=  -6.311 imu3= 175.078  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈120.1Hz  imu1= 142.843 imu2=  -6.311 imu3= 175.078  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈120.1Hz  imu1= 142.820 imu2=  -7.283 imu3= 175.403  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
帧率≈119.9Hz  imu1= 142.819 imu2=  -7.295 imu3= 175.422  speed0=   0.000 speed1=   0.000  err0=  0.000 out0=   0.000 err1=  0.000 out1=   0.000
^CTraceback (most recent call last):
  File "/home/orangepi/MyWork/Robot/ros2/ws_orangepi_01/install/DD_Chassis_001_driver/lib/DD_Chassis_001_driver/serial_monitor.py", line 133, in <module>
    main()
  File "/home/orangepi/MyWork/Robot/ros2/ws_orangepi_01/install/DD_Chassis_001_driver/lib/DD_Chassis_001_driver/serial_monitor.py", line 77, in main
    buf += port.read(256)
  File "/usr/lib/python3/dist-packages/serial/serialposix.py", line 565, in read
    ready, _, _ = select.select([self.fd, self.pipe_abort_read_r], [], [], timeout.time_left())
KeyboardInterrupt
[ros2run]: Interrupt
orangepi@orangepi3b:~/MyWork/Robot/ros2/ws_orangepi_01$ 
```

```bash
orangepi@orangepi3b:~/MyWork/Robot/ros2/ws_orangepi_01$ ros2 control list_hardware_interfaces
command interfaces
        wheel_l/velocity [available] [claimed]
        wheel_r/velocity [available] [claimed]
state interfaces
        wheel_l/position
        wheel_l/velocity
        wheel_r/position
        wheel_r/velocity
orangepi@orangepi3b:~/MyWork/Robot/ros2/ws_orangepi_01$ ros2 control list_hardware_components
Hardware Component 1
        name: dd_chassis_001
        type: system
        plugin name: dd_chassis_001_hardware/DDChassis001System
        state: id=3 label=active
        command interfaces
                wheel_l/velocity [available] [claimed]
                wheel_r/velocity [available] [claimed]
orangepi@orangepi3b:~/MyWork/Robot/ros2/ws_orangepi_01$ 
```
