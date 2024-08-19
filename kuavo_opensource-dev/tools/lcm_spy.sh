#!/bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
cd ${DIR}/../lcm_types/java
export CLASSPATH=${DIR}/../lcm_types/java/customize_types.jar
pwd
lcm-spy
