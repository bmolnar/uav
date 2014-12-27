#!/bin/sh

DEVICE="/dev/ttyACM0"

while getopts "D:" flag "$@"; do
    case ${flag} in
	D) DEVICE=${OPTARG} ;;
    esac
done
shift $((${OPTIND} - 1))

hexfile=$1
if [ ! -f "${hexfile}" ]; then
    echo "Invalid file ${hexfile}"
    exit 1
fi

echo "Uploading hex file ${hexfile} to ${DEVICE}..."
avrdude -C/usr/share/arduino/hardware/tools/avrdude.conf -v -patmega328p -carduino -P${DEVICE} -b115200 -D -Uflash:w:${hexfile}:i
