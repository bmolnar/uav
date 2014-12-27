#!/bin/sh

HEXFILE=$1
HOST=$2

RMTDIR="/home/themole/TheMole/Projects/github/projects/UAV/Arduino"

#
# Copy hex file to remote
#
scp ${HEXFILE} "${HOST}:/${RMTDIR}/.build/upload.hex"

#
# Upload remote hex file to device
#
ssh "${HOST}" "${RMTDIR}/scripts/upload.sh ${RMTDIR}/.build/upload.hex"

