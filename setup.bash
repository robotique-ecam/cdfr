#!/bin/bash

function print_info {
    (>&2 printf "\x1b[36m[i] ${1}\x1b[0m\n")
}

function print_success {
    (>&2 printf "\x1b[32m[+] ${1}\x1b[0m\n")
    exit 0
}

function print_failure {
    (>&2 printf "\x1b[31m[!] ${1}\x1b[0m\n")
    exit 1
}

print_info "Welcome to Robot Setup Script"
read -p "Please enter the name of the robot to setup : " robot


print_info "Setting up robot : $robot"

xacro tools/xacro/$robot.xacro -o src/$robot/robot/$robot.urdf && print_success "Generated URDF for $robot" || print_failure "No such robot"
