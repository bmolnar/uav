#!/bin/sh

DEVICE="/dev/ttyACM0"

HOST=${1}

ssh -t "${HOST}" "minicom -b 38400 -D ${DEVICE}"

