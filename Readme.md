![colcon build](https://github.com/3wnbr1/ros/workflows/colcon%20build/badge.svg?branch=master)

### Build

#### Onboard build

```bash
cd ~/ros
. /opt/ros/eloquent/setup.bash
./setup.bash robot_name
. install/setup.bash
```

#### Dev / Simulation build

```bash
. /path/to/ros/install/setup.bash
cd path/to/ros
./setup.bash simulation
. install/setup.bash
```

#### Upgrading colcon on macOS / Win

```bash
pip3 install --upgrade colcon-common-extensions --upgrade-strategy=eager
```

#### Use clang_complete

```bash
export CC=$(pwd)/tools/clang_complete/cc
export CXX=$(pwd)/tools/clang_complete/g++
```

Then run a normal colcon build


### Code formating

```bash
ament_clang_format --config ".clang-format" --reformat $(colcon list -p)

ament_flake8 --linelength 160 .

ament_lint_cmake .

ament_xmllint .
```
