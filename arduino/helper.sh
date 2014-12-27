#!/bin/sh

autogen ()
{
    set -x
    autoreconf --install --force -v || exit 1
    set +x
}

build ()
{
    set -x
    mkdir -p .build
    (cd .build; ../configure --host=avr)
    (cd .build; make)
    set +x
}

clean ()
{
    set -x
    rm -rf .build autom4te.cache config-aux aclocal.m4 configure Makefile.in
    set +x
}

usage ()
{
    echo "Usage:"
    echo "    helper.sh <command>"
    echo ""
    echo "  Commands:"
    echo "    autogen   Set up local build environment"
    echo "    build     Build project"
    echo "    clean     Clean local build artifacts"
    echo ""
}


case $1 in
    autogen)
	autogen
	;;
    build)
	build
	;;
    clean)
	clean
	;;
    *)
	usage
	;;
esac
