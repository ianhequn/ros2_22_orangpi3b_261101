# easytier

## win

```powershell
# 已经注册服务
C:\OMEN_Portable\easytier-cli\easytier-windows-x86_64\easytier-core.exe -i 10.144.144.1
```

## orangepi

```bash
# 已经注册到systemd

sudo easytier-core -i 10.144.144.42 -p udp://192.168.223.1:11010

sudo /home/orangepi/MyWork/easytier/easytier-linux-aarch64/easytier-core -i 10.144.144.42 -p udp://192.168.223.1:11010
```

## ip分配

| host | ip |
| --- | --- |
| win | 10.144.144.1 |
| WSL_u24 | 10.144.144.24 |
| WSL_u22 | 10.144.144.22 |
| VM_u24 | 10.144.144.34 |
| orangepi_u22 | 10.144.144.42 |

