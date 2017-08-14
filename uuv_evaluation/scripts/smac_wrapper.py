#!/usr/bin/env python
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

import argparse
import roslib
import os
import yaml
from copy import deepcopy
from parameter_opt import SimulationRunner
from bag_evaluation import Evaluation

roslib.load_manifest('uuv_evaluation')


def parse_input(args, input_map):
    p = vars(args)
    params = dict()
    for tag in input_map:
        if type(input_map[tag]) == list:
            p_cont = list()
            for elem in input_map[tag]:
                if type(elem) == str:
                    p_cont.append(p[elem])
                else:
                    p_cont.append(elem)
        else:
            if type(input_map[tag]) == str:
                p_cont = p[input_map[tag]]
            else:
                p_cont = input_map[tag]
        params[tag] = p_cont

    return params

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='SMAC wrapper')
    # SMAC fixed positional parameters
    #    <instance_name> <instance_specific_information> <cutoff_time> <cutoff_length> <seed>
    parser.add_argument(
        'instance_name',
        help='The name of the problem instance we are executing against')
    parser.add_argument(
        'instance_specific_information',
        help='An arbitrary string associated with this instance as specified in the instance file.')
    parser.add_argument(
        'cutoff_time', type=float,
        help='The amount of time in seconds that the target algorithm is permitted to run.')
    parser.add_argument(
        'cutoff_length', type=float,
        help='A domain specific measure of when the algorithm should consider itself done')
    parser.add_argument(
        'seed', type=int,
        help='A positive integer that the algorithm should use to seed itself (for reproducibility).')

    # Load optimization configuration
    with open('opt_config.yml', 'r') as c_file:
        opt_config = yaml.load(c_file)

    assert 'cost_fcn' in opt_config, 'Cost function configuration available'
    assert 'input_map' in opt_config, 'Input parameter map is not available'
    assert 'parameters' in opt_config, 'Parameter labels is not available'

    # SMAC problem-specific parameters
    for param_tag in opt_config['parameters']:
        parser.add_argument('-' + param_tag, type=float)

    args = parser.parse_args()

    task = opt_config['task']
    results_dir = opt_config['output_dir']
    record_all = False

    params = parse_input(args, opt_config['input_map'])

    if 'store_all_results' in opt_config:
        record_all = opt_config['store_all_results']

    try:
        runner = SimulationRunner(params, task, results_dir, record_all)

        runner.run(params)

        sim_eval = Evaluation(runner.recording_filename, runner.current_sim_results_dir)
        output_path = deepcopy(runner.current_sim_results_dir)
        sim_eval.compute_kpis()

        if 'store_kpis_only' in opt_config:
            if opt_config['store_kpis_only']:
                sim_eval.save_kpis()
            else:
                sim_eval.save_evaluation()
        else:
            sim_eval.save_kpis()

        cost = 0.0
        for tag in opt_config['cost_fcn']:
            cost += sim_eval.get_kpi(tag) * opt_config['cost_fcn'][tag]

        status = 'SUCCESS'
        output = dict(status=status,
                      cost=float(cost))

        with open(os.path.join(runner.current_sim_results_dir, 'smac_result.yaml'), 'w') as smac_file:
            yaml.dump(output, smac_file, default_flow_style=False)

        with open(os.path.join(runner.current_sim_results_dir, 'cost_function.yaml'), 'w') as cf_file:
            yaml.dump(opt_config['cost_fcn'], cf_file, default_flow_style=False)

        del runner
        del sim_eval
    except Exception, e:
        print('Error occurred in this iteration, setting simulation status to CRASHED, message=' + str(e))
        status = 'CRASHED'
        cost = 1e7

    print('Result for SMAC: %s, 0, 0, %f, %s' % (status, cost, args.seed))
