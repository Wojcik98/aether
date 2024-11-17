#!/usr/bin/env bash
set -e

. /ros_entrypoint.sh
. /opt/esp/entrypoint.sh > /dev/null
. /just-completions.bash

# Don't exit on error
set +e
exec "$@"
