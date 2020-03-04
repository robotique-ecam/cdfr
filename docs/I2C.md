To enable all 6 I2C interfaces on Raspberry Pi 4, add this to boot config.txt

```
# Uncomment some or all of these to enable the optional hardware interfaces
#dtparam=spi=on
#dtparam=i2s=on
dtparam=i2c_arm=on
dtoverlay=i2c6
dtoverlay=i2c5
dtoverlay=i2c4
dtoverlay=i2c3
dtoverlay=i2c1
dtoverlay=i2c0
```

The GPIO Pinouts for each bus are

| I2C Bus | GPIO SDA | GPIO SCL |
|---------|----------|----------|
| 0       | 0        | 1        |
| 1       | 2        | 3        |
| 3       | 4        | 5        |
| 4       | 8        | 9        |
| 5       | 12       | 13       |
| 6       | 22       | 23       |
