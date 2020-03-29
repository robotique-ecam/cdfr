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
