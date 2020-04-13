### Bluetooth Serial Communication

```bash
sudo apt install bluez
```


```
sudo bluetoothctl
[bluetooth]# agent on
[bluetooth]# scan on
[bluetooth]# scan off
[bluetooth]# pair 98:D3:32:11:06:B1  # change with your HC-05 address
[bluetooth]# trust 98:D3:32:11:06:B1
[bluetooth]# connect 98:D3:32:11:06:B1
```
