### Build

```
colcon build --cmake-args ' -DCMAKE_CXX_FLAGS="-march=native" -DTIMER=ON'
```

#### Upgrading colcon on macOS / Win

```pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager```
