#!/usr/bin/env bash

if [ $# -eq 0 ] ; then
  /bin/echo "Usage: env.sh COMMANDS"
  exit 1
fi
CATKIN_SHELL=sh
_CATKIN_SETUP_DIR=$(cd "`dirname "$0"`" > /dev/null && pwd)
. "$_CATKIN_SETUP_DIR/setup.sh"
exec "$@"
