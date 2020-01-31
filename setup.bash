#!/bin/bash

function print_info {
    (>&2 printf "\x1b[34m${1}\x1b[0m\n")
}

function print_success {
    (>&2 printf "\x1b[32m${1}\x1b[0m\n")
    exit 0
}

function print_failure {
    (>&2 printf "\x1b[31m${1}\x1b[0m\n")
    exit 1
}

print_info "Setting up robot"

xacro tools/xacro/asterix.xacro -o src/asterix/robot/asterix.urdf
print_success "Generated URDF for asterix"
