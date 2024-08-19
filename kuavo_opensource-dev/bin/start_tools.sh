#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

export LD_LIBRARY_PATH=$SCRIPT_DIR/../lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$SCRIPT_DIR/../lib/jodell_claw_driver/lib:$LD_LIBRARY_PATH

echo "Try to run command: $@"

$@
