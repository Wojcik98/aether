#!/usr/bin/env bash
set -e

. /ros_entrypoint.sh
. /just-completions.bash
. ~/.vcpkg/vcpkg-init

# Don't exit on error
set +e
exec "$@"
