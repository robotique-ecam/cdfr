### Build

```
colcon build --cmake-args ' -DCMAKE_CXX_FLAGS="-march=native"' ' -DCMAKE_CXX_FLAGS="-O3"'
```

#### Upgrading colcon on macOS / Win

```
pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager
```

```
set -gx CC (pwd)/tools/clang_complete/cc
set -gx CXX (pwd)/tools/clang_complete/g++
```

```
export CC=$(pwd)/tools/clang_complete/cc
export CXX=$(pwd)/tools/clang_complete/g++
```
