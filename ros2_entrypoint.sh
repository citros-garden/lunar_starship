#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /workspaces/lunar_starship/install/setup.bash 

exec "$@"
