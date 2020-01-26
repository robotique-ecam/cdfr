![colcon build](https://github.com/3wnbr1/ros/workflows/colcon%20build/badge.svg?branch=master)

### Build

```
colcon build --cmake-args ' -DCMAKE_CXX_FLAGS="-march=native"' ' -DCMAKE_CXX_FLAGS="-O3"'
```

#### Upgrading colcon on macOS / Win

```
pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager
```
