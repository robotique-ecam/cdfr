![colcon build](https://github.com/3wnbr1/ros/workflows/colcon%20build/badge.svg?branch=master)

### Build

#### Onboard build

```bash
cd ~/ros
. /opt/ros/eloquent/setup.bash
./setup.bash
. install/setup.bash
```

#### Dev / Simulation build

```bash
. /path/to/ros/install/setup.bash
cd path/to/ros
colcon build --symlink-install --cmake-args ' -DSIMULATION=ON'
```

#### Upgrading colcon on macOS / Win

```
pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager
```

#### Use clang_complete

fish users
```
set -gx CC (pwd)/tools/clang_complete/cc
set -gx CXX (pwd)/tools/clang_complete/g++
```

bash users
```
export CC=$(pwd)/tools/clang_complete/cc
export CXX=$(pwd)/tools/clang_complete/g++
```


### Code formating

fish
```
ament_clang_format --config ".clang-format" --reformat (colcon list -p)

ament_flake8 --linelength 160 .

ament_lint_cmake .

ament_xmllint .
```
