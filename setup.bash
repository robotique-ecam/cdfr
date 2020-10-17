#!/bin/bash

function print_info {
    (>&2 printf "\x1b[36m[i] ${1}\x1b[0m\n")
}

function print_success {
    (>&2 printf "\x1b[32m[+] ${1}\x1b[0m\n")
}

function print_failure {
    (>&2 printf "\x1b[31m[!] ${1}\x1b[0m\n")
    exit 1
}

function generate_urdfs {
    xacro tools/xacro/asterix.xacro -o src/asterix/robot/asterix.urdf
    xacro tools/xacro/obelix.xacro -o src/obelix/robot/obelix.urdf
    if [[ $? -eq 0 ]]; then
        print_success "Generated URDF files"
        return 0
    else
        print_failure "Failled to generate URDFs"
        exit 1
    fi
}

function generate_yamls {
    python3 tools/compute_robots_kinematics.py
    emrichen -f obelix.vars --output-file src/obelix/param/obelix.yml src/obelix/param/obelix.in.yml
    emrichen -f asterix.vars --output-file src/asterix/param/asterix.yml src/asterix/param/asterix.in.yml
    emrichen --output-file src/modules/robot/param/robot.yml src/modules/robot/param/robot.in.yml
    if [[ $? -eq 0 ]]; then
        print_success "Generated YAMLs files from templates"
        return 0
    else
        print_failure "Failled to generate YAMLs"
        exit 1
    fi
}

if [ -n "$1" ]; then
    robot=$1
else
    read -p "Please enter the name of the robot to setup : " robot
fi


if [ $robot = "asterix" ] || [ $robot = "obelix" ]; then
    print_info "Setting up $robot"
    generate_yamls && generate_urdfs && colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --packages-skip assurancetourix strategix panoramix transformix transformix_msgs pharaon_msgs pharaon && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "assurancetourix" ]; then
    print_info "Setting up $robot"
    colcon build --symlink-install --cmake-args " -DMIPI_CAMERA=ON" --packages-select transformix_msgs transformix strategix_msgs strategix pharaon_msgs pharaon panoramix assurancetourix localisation && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "simulation" ]; then
    print_info "Setting up simulation environment"
    generate_yamls && generate_urdfs && colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --cmake-args=" -DSIMULATION=ON" && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "simulation-core" ]; then
    print_info "Setting up simulation environment"
    xacro ros/tools/xacro/asterix.xacro -o ros/src/asterix/robot/asterix.urdf
    xacro ros/tools/xacro/obelix.xacro -o ros/src/obelix/robot/obelix.urdf
    colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --cmake-args=" -DSIMULATION=ON" --packages-skip sensors drive assurancetourix && print_success "Built packages for $robot" || print_failure "Packages build failed"

elif [ $robot = "simulation-interfaces" ]; then
    print_info "Setting up simulation environment"
    generate_yamls
    colcon build --symlink-install --cmake-args=" -DCMAKE_BUILD_TYPE=Release" --cmake-args=" -DSIMULATION=ON" --packages-skip gradient_costmap_layer && print_success "Built packages for $robot" || print_failure "Packages build failed"
    print_info "Linking webots controllers"
    install_name_tool -change @rpath/lib/controller/libController.dylib $WEBOTS_HOME/lib/controller/libController.dylib install/drive/lib/drive/drive
    install_name_tool -change @rpath/lib/controller/libController.dylib $WEBOTS_HOME/lib/controller/libController.dylib install/assurancetourix/lib/assurancetourix/assurancetourix
    install_name_tool -change @rpath/lib/controller/libCppController.dylib $WEBOTS_HOME/lib/controller/libCppController.dylib install/drive/lib/drive/drive
    install_name_tool -change @rpath/lib/controller/libCppController.dylib $WEBOTS_HOME/lib/controller/libCppController.dylib install/assurancetourix/lib/assurancetourix/assurancetourix
else
    print_failure "No such component"

fi
