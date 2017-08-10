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
import rospy
import json
import os
import yaml
import numpy as np
from parameter_opt import SimulationRunner
from bag_evaluation import Evaluation
from bag_evaluation.metrics import KPI
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

roslib.load_manifest('uuv_evaluation')


def parse_input(args, input_map):
    p = args
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
    parser = argparse.ArgumentParser(description='Evaluation of best SMAC output')
    parser.add_argument(
        '--input_dir',
        type=str,
        default='.')
    parser.add_argument(
        '--output_dir',
        type=str,
        default='best_results')
    parser.add_argument(
        '--simulate',
        type=int,
        default='1')

    # Parse input arguments
    args = parser.parse_args(rospy.myargv()[1:])

    smac_folder = None
    for item in os.listdir(args.input_dir):
        if 'smac' in item:
            if os.path.isdir(os.path.join(args.input_dir, item)):
                smac_folder = os.path.join(args.input_dir, item)

    if smac_folder is None:
        raise Exception('No SMAC output folder found')

    print 'SMAC RESULTS FOLDER=', smac_folder

    smac_file = os.path.join(smac_folder, 'traj_aclib2.json')

    if not os.path.isfile(smac_file):
        raise Exception('SMAC output file not found')

    print 'SMAC OUTPUT FILE=', smac_file

    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir)

    # Load optimization configuration
    with open(os.path.join(args.input_dir, 'opt_config.yml'), 'r') as c_file:
        opt_config = yaml.load(c_file)

    assert 'cost_fcn' in opt_config, 'Cost function configuration available'
    assert 'input_map' in opt_config, 'Input parameter map is not available'
    assert 'parameters' in opt_config, 'Parameter labels is not available'

    cost = list()
    evals = list()

    ########################################################################
    # Storing trajectories for later comparison
    ########################################################################
    desired = None
    traj = list()
    error_t = list()
    error_vec = list()
    error_yaw_vec = list()
    t_cur = None
    vec_cur = None
    t_force = None
    vec_force = None
    vec_torque = None
    kpi_results = list()

    ########################################################################
    # Plot error and trajectories
    ########################################################################
    idx = 0
    if args.simulate:
        with open(smac_file, 'r') as s_file:
            for line in s_file:
                smac_result = json.loads(line)
                sub_result_folder = os.path.join(os.getcwd(), args.output_dir, '%d' % idx)

                cost.append(smac_result['cost'])
                evals.append(smac_result['evaluations'])

                if not os.path.isdir(sub_result_folder):
                    os.makedirs(sub_result_folder)

                # Store the partial result in the folder
                with open(os.path.join(sub_result_folder, 'partial_result.json'), 'w') as p_res:
                    p_res.write(json.dumps(smac_result))

                parsed_params = dict()

                for item in smac_result['incumbent']:
                    tag, value = item.split('=')
                    parsed_params[tag] = float(value[1:-1])

                params = parse_input(parsed_params, opt_config['input_map'])

                task = os.path.join(args.input_dir, opt_config['task'])
                print idx, sub_result_folder

                runner = SimulationRunner(params, task, sub_result_folder, True, add_folder_timestamp=False)
                runner.run(params)

                sim_eval = Evaluation(runner.recording_filename, runner.current_sim_results_dir)
                sim_eval.compute_kpis()
                sim_eval.save_evaluation()

                if desired is None:
                    desired = sim_eval.get_trajectory_coord('desired')

                traj.append(sim_eval.get_trajectory_coord('actual'))

                error_t.append(sim_eval._error_set.get_time())
                error_vec.append(KPI.get_error(sim_eval._error_set.get_data('position')))
                error_yaw_vec.append(sim_eval._error_set.get_data('yaw'))

                if t_cur is None:
                    t_cur, vec_cur = sim_eval._bag.get_current_vel()

                if t_force is None:
                    t_force, vec_force, vec_torque = sim_eval._bag.get_wrench_dist()

                kpis = sim_eval.get_kpis()

                kpi_results.append(kpis)

                cur_cost = 0.0
                for tag in opt_config['cost_fcn']:
                    cur_cost += sim_eval.get_kpi(tag) * opt_config['cost_fcn'][tag]

                with open(os.path.join(runner.current_sim_results_dir, 'smac_result.yaml'), 'w') as smac_file:
                    yaml.dump(dict(cost=float(cur_cost)), smac_file, default_flow_style=False)

                with open(os.path.join(runner.current_sim_results_dir, 'cost_function.yaml'), 'w') as cf_file:
                    yaml.dump(opt_config['cost_fcn'], cf_file, default_flow_style=False)

                del sim_eval
                del runner

                idx += 1
    else:
        print 'Reprocessing the best results...'
        print 'Opening the results directory: ', args.output_dir

        for item in sorted(os.listdir(args.output_dir)):
            subdir = os.path.join(args.output_dir, item)
            partial_result_file = os.path.join(subdir, 'partial_result.json')

            if os.path.isfile(partial_result_file):
                with open(partial_result_file, 'r') as s_file:
                    for line in s_file:
                        smac_result = json.loads(line)
                        cost.append(smac_result['cost'])
                        evals.append(smac_result['evaluations'])

            if os.path.isdir(subdir):
                bag_path = subdir
                bag_filename = os.path.join(bag_path, 'recording.bag')
                if os.path.isfile(bag_filename):
                    print bag_filename
                    sim_eval = Evaluation(bag_filename, bag_path)
                    sim_eval.compute_kpis()
                    sim_eval.save_evaluation()

                    if desired is None:
                        desired = sim_eval.get_trajectory_coord('desired')

                    traj.append(sim_eval.get_trajectory_coord('actual'))

                    error_t.append(sim_eval._error_set.get_time())
                    error_vec.append(KPI.get_error(sim_eval._error_set.get_data('position')))
                    error_yaw_vec.append(sim_eval._error_set.get_data('yaw'))

                    if t_cur is None:
                        t_cur, vec_cur = sim_eval._bag.get_current_vel()

                    if t_force is None:
                        t_force, vec_force, vec_torque = sim_eval._bag.get_wrench_dist()

                    kpis = sim_eval.get_kpis()

                    kpi_results.append(kpis)

                    cur_cost = 0.0
                    for tag in opt_config['cost_fcn']:
                        cur_cost += sim_eval.get_kpi(tag) * opt_config['cost_fcn'][tag]

                    with open(os.path.join(bag_path, 'smac_result.yaml'), 'w') as smac_file:
                        yaml.dump(dict(cost=float(cur_cost)), smac_file, default_flow_style=False)

                    with open(os.path.join(bag_path, 'cost_function.yaml'), 'w') as cf_file:
                        yaml.dump(opt_config['cost_fcn'], cf_file, default_flow_style=False)

                    print 'COST=', cur_cost
                    del sim_eval
                    idx += 1

    try:
        ########################################################################
        # Plot cost function evolution
        ########################################################################

        cost[0] = cost[1]
        fig = plt.figure(figsize=(12, 5.5))
        ax = fig.add_subplot(111)

        ax.plot(evals, cost, '--b', linewidth=3, zorder=1)
        ax.scatter(evals, cost, s=60, color='red', zorder=2)

        ax.set_xlabel('Number of evaluations', fontsize=30)
        ax.set_ylabel('Cost function', fontsize=30)
        ax.tick_params(axis='both', labelsize=25)
        ax.grid(True)
        ax.set_xlim(np.min(evals), np.max(evals))
        plt.tight_layout()

        filename = os.path.join(args.output_dir, 'smac_evolution.pdf')
        plt.savefig(filename)
        plt.clf()

        ########################################################################
        # Compare first and last trajectories
        ########################################################################

        fig = plt.figure(figsize=(12, 8))
        ax = fig.gca(projection='3d')

        ax.plot(desired[0], desired[1], desired[2], 'b--', label='Desired path', linewidth=3)
        ax.plot(traj[1][0], traj[1][1], traj[1][2], 'g', label='Initial set', linewidth=3)
        ax.plot(traj[-1][0], traj[-1][1], traj[-1][2], 'r', label='Optimal set', linewidth=3)

        ax.set_xlabel('X [m]', fontsize=20)
        ax.set_ylabel('Y [m]', fontsize=20)
        ax.set_zlabel('Z [m]', fontsize=20)

        ax.tick_params(axis='x', labelsize=18)
        ax.tick_params(axis='y', labelsize=18)
        ax.tick_params(axis='z', labelsize=18)

        ax.xaxis.labelpad = 10
        ax.yaxis.labelpad = 10
        ax.zaxis.labelpad = 10

        ax.legend(loc='upper left', fancybox=True, framealpha=0.8, fontsize=18)
        ax.grid(True)
        plt.tight_layout()

        filename = os.path.join(args.output_dir, 'trajectories_comparison.pdf')
        plt.savefig(filename)
        plt.clf()

        ########################################################################
        # Compare first and last KPIs
        ########################################################################

        fig = plt.figure(figsize=(8, 12))

        kpi_labels = dict(rmse_position='RMS Error - Position',
                          rmse_yaw='RMS Error - Yaw',
                          rmse_linear_velocity='RMS Error - Linear velocity',
                          rmse_angular_velocity='RMS Error - Angular velocity',
                          mean_abs_thrust='Mean abs. thrust')

        i = 1
        for tag in kpi_labels:
            ax = fig.add_subplot(len(kpi_labels.keys()), 1, i)
            ax.plot(np.arange(idx), [kpi_results[k][tag] for k in np.arange(idx)], '--b', linewidth=2, zorder=1)
            ax.scatter(np.arange(idx), [kpi_results[k][tag] for k in np.arange(idx)], s=30, color='red', zorder=2)
            i += 1

            ax.set_xlabel('Set index', fontsize=20)
            ax.set_ylabel('Metric value', fontsize=20)
            ax.set_title(kpi_labels[tag], fontsize=22)
            ax.tick_params(axis='both', labelsize=15)
            ax.grid(True)
            ax.set_xlim(0, idx - 1)

        plt.tight_layout()
        filename = os.path.join(args.output_dir, 'kpis_comparison.pdf')
        plt.savefig(filename)
        plt.clf()

        ########################################################################
        # Compare position error
        ########################################################################

        fig = plt.figure(figsize=(12, 10))

        # Plot position error
        ax = fig.add_subplot(211)

        ax.plot(error_t[0], error_vec[0], color='#E2742B', linewidth=3, label='Initial set')
        ax.plot(error_t[-1], error_vec[-1], color='#6600CC', linewidth=3, label='Optimal set')

        error_max = np.max([np.max(error_vec[0]), np.max(error_vec[-1])])
        if len(t_cur) > 0:
            v = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_cur])
            if v.max() > 0:
                v[v > 0] = 1.05
                ax.fill_between(t_cur, 0, v * error_max, facecolor='blue', alpha=0.2, label='Current disturbance activated')

        if len(t_force) > 0:
            f = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_force])
            tau = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_torque])

            if f.max() > 0:
                f[f > 0] = 1.05
                ax.fill_between(t_force, 0, f * error_max, facecolor='red', alpha=0.2, label='Force disturbance activated')

            if tau.max() > 0:
                tau[tau > 0] = 1.05
                ax.fill_between(t_force, 0, tau * error_max, facecolor='green', alpha=0.2, label='Torque disturbance activated')

        ax.set_xlabel('Time [s]', fontsize=20)
        ax.set_ylabel('Position error [m]', fontsize=20)
        ax.legend(fancybox=True, framealpha=1, loc='upper left', fontsize=16)
        ax.grid(True)
        ax.tick_params(axis='both', labelsize=15)
        ax.set_xlim(np.min(error_t[0]), np.max(error_t[0]))
        ax.set_ylim(0, error_max * 1.05)

        # Plot heading error
        ax = fig.add_subplot(212)

        ax.plot(error_t[0], error_yaw_vec[0], color='#E2742B', linewidth=3, label='Initial set')
        ax.plot(error_t[-1], error_yaw_vec[-1], color='#6600CC', linewidth=3, label='Optimal set')

        error_max = np.max([np.max(error_yaw_vec[0]), np.max(error_yaw_vec[-1])])
        error_min = np.min([np.min(error_yaw_vec[0]), np.min(error_yaw_vec[-1])])
        if len(t_cur) > 0:
            v = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_cur])
            if v.max() > 0:
                v[v > 0] = 1.05
                ax.fill_between(t_cur, v * error_min, v * error_max, facecolor='blue', alpha=0.2, label='Current disturbance activated')

        if len(t_force) > 0:
            f = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_force])
            tau = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in vec_torque])

            if f.max() > 0:
                f[f > 0] = 1.05
                ax.fill_between(t_force, f * error_min, f * error_max, facecolor='red', alpha=0.2, label='Force disturbance activated')

            if tau.max() > 0:
                tau[tau > 0] = 1.05
                ax.fill_between(t_force, tau * error_min, tau * error_max, facecolor='green', alpha=0.2, label='Torque disturbance activated')

        ax.set_xlabel('Time [s]', fontsize=20)
        ax.set_ylabel('Heading error [rad]', fontsize=20)
        ax.legend(fancybox=True, framealpha=1, loc='upper left', fontsize=16)
        ax.grid(True)
        ax.tick_params(axis='both', labelsize=15)
        ax.set_xlim(np.min(error_t[0]), np.max(error_t[0]))
        ax.set_ylim(error_min * 1.05, error_max * 1.05)

        plt.tight_layout()

        filename = os.path.join(args.output_dir, 'error_comparison.pdf')
        plt.savefig(filename)
        plt.clf()
    except Exception, e:
        print 'Error while plotting comparative results, message=', str(e)