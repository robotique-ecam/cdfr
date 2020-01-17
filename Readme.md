### Build

```
colcon build --cmake-args ' -DCMAKE_CXX_FLAGS="-march=native"' ' -DCMAKE_CXX_FLAGS="-O3"'
```

#### Upgrading colcon on macOS / Win

```
pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager
```
