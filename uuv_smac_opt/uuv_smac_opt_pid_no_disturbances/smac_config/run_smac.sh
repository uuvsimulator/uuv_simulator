#!/usr/bin/env bash
# Copyright (c) 2016 The UUV Simulator Authors.
# All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Test whether the input argument is a valid positive integer
if [ -z "${1##[0-9]*}" ] && [ -n "$1" ] 2> /dev/null; then
  echo Number of simulation runs requested: $1
else
  echo Input argument must be a positive integer!
  exit 1
fi

# Path to the SMAC script
SMAC_SCRIPT="$HOME/Software/SMAC3/scripts/smac"

# SMAC configuration parameters
PARAM_FILE=parameter_config.pcs
RUN_OBJ=quality
DETERMINISTIC=1

SMAC_WRAPPER="$(rospack find uuv_evaluation)/scripts/smac_wrapper.py"

TODAY="$(date +%Y%m%d%H%M%S)"

printf "SMAC wrapper script = %s\n" "${SMAC_WRAPPER}"

SCENARIO_FILE="scenario_$TODAY.txt"

printf "Creating the scenario configuration file, file=%s\n" "${SCENARIO_FILE}"

# Fill the scenario configuration file
echo "algo = python ${SMAC_WRAPPER}" >> ${SCENARIO_FILE}
echo "paramfile = ${PARAM_FILE}" >> ${SCENARIO_FILE}
echo "run_obj = ${RUN_OBJ}" >> ${SCENARIO_FILE}
echo "runcount_limit = $1" >> ${SCENARIO_FILE}
echo "deterministic = ${DETERMINISTIC}" >> ${SCENARIO_FILE}

# Run SMAC for the scenario created, change the verbose flag to DEBUG, if necessary
printf "Running SMAC"
python3 ${SMAC_SCRIPT} --scenario ${SCENARIO_FILE} --verbose INFO