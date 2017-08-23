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
import rospy
import roslib
import os
import sys
import shutil
import numpy as np
import yaml
import logging
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from bag_evaluation import Evaluation

try:
    plt.rc('text', usetex=True)
    plt.rc('font', family='sans-serif')
except Exception, e:
    print 'Cannot use Latex configuration with matplotlib, message=', str(e)

roslib.load_manifest('uuv_evaluation')


"""
This script can process a batch of simulation results in the form of ROS bags
and generate comparative plots with the best and the worst results. The folder
structure must be set as follows

- root_folder_1
    - sim_1_folder
        - recording.bag
    - sim_2_folder
        - recording.bag
    - sim_3_folder
        - recording.bag
- root_folder_2
    - sim_1_folder
        - recording.bag
    - sim_2_folder
        - recording.bag
    - sim_3_folder
        - recording.bag
        
This script will generate comparative plots for the results within each root_folder and amongst all root_folders 
provided.
"""

PLOT_CONFIGS = dict(plot=dict(figsize=[12, 6],
                              label_fontsize=25,
                              title_fontsize=22,
                              tick_fontsize=20,
                              linewidth=3,
                              legend=dict(loc='upper left',
                                          fontsize=18)),
                    paths=dict(figsize=[12, 10],
                               label_fontsize=25,
                               title_fontsize=22,
                               tick_fontsize=20,
                               linewidth=3,
                               legend=dict(loc='upper left',
                                           fontsize=18)))


def gen_evaluation(output_dir, bag_filename, task_filename):
    """Create a new evaluation object for a ROS bag."""
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    sim_eval = Evaluation(bag_filename, output_dir)
    sim_eval.save_evaluation()

    if os.path.isfile(task_filename):
        shutil.copy(task_filename, output_dir)

    del sim_eval


def gen_evaluations(bags, output_dir):
    """Generate evaluation instances for each ROS bag file in the bags array."""
    sim_evals = list()
    for bag in bags:
        print '\tOPENING BAG: ', bag
        sim_evals.append(Evaluation(bag, output_dir))
    return sim_evals


def plot_disturbance_areas(fig, ax, sim_eval, min_y, max_y):
    """Add a colored area on the plot where current or wrench disturbances were active."""
    t, cur_vel = sim_eval.get_current_velocity()
    if len(t) > 0:
        vec = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in cur_vel])
        if vec.max() > 0:
            vec[vec > 0] = 1.05
            ax.fill_between(t,
                            vec * min_y,
                            vec * max_y,
                            facecolor='blue',
                            alpha=0.2,
                            label='Current disturbance activated')
            fig.canvas.draw()

    t, force, torque = sim_eval.get_wrench_dist()
    if len(t) > 0:
        vec = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in force])
        if vec.max() > 0:
            vec[vec > 0] = 1.05
            ax.fill_between(t,
                            vec * min_y,
                            vec * max_y,
                            facecolor='red',
                            alpha=0.2,
                            label='Force disturbance activated')
            fig.canvas.draw()

        vec = np.array([np.sqrt(v[0]**2 + v[1]**2 + v[2]**2) for v in torque])
        if vec.max() > 0:
            vec[vec > 0] = 1.05
            ax.fill_between(t,
                            vec * min_y,
                            vec * max_y,
                            facecolor='green',
                            alpha=0.2,
                            label='Torque disturbance activated')
            fig.canvas.draw()


def plot_paths(output_dir, bags, labels, title, filename):
    """Generate path plots for the ROS bags provided"""
    assert len(labels) == len(bags), 'Number of labels and bags is different'

    fig = plt.figure(figsize=(PLOT_CONFIGS['paths']['figsize'][0],
                              PLOT_CONFIGS['paths']['figsize'][1]))
    ax = fig.gca(projection='3d')

    target_path = False

    min_z = None
    max_z = None

    for i in range(len(bags)):
        sim_eval = Evaluation(bags[i], output_dir)

        if not target_path:
            traj = sim_eval.get_trajectory_coord('desired')
            ax.plot(traj[0], traj[1], traj[2], 'g--',
                    label='Reference path',
                    linewidth=PLOT_CONFIGS['paths']['linewidth'])
            fig.canvas.draw()
            target_path = True

        traj = sim_eval.get_trajectory_coord('actual')
        ax.plot(traj[0], traj[1], traj[2],
                label=labels[i],
                linewidth=PLOT_CONFIGS['paths']['linewidth'])

        if min_z is None:
            min_z = np.min(traj[2])
            max_z = np.max(traj[2])
        else:
            min_z = min(np.min(traj[2]), min_z)
            max_z = max(np.max(traj[2]), max_z)
        fig.canvas.draw()

    ax.set_xlabel('X [m]', fontsize=PLOT_CONFIGS['paths']['label_fontsize'])
    ax.set_ylabel('Y [m]', fontsize=PLOT_CONFIGS['paths']['label_fontsize'])
    ax.set_zlabel('Z [m]', fontsize=PLOT_CONFIGS['paths']['label_fontsize'])

    ax.tick_params(axis='x', labelsize=PLOT_CONFIGS['paths']['tick_fontsize'], pad=15)
    ax.tick_params(axis='y', labelsize=PLOT_CONFIGS['paths']['tick_fontsize'], pad=15)
    ax.tick_params(axis='z', labelsize=PLOT_CONFIGS['paths']['tick_fontsize'], pad=15)

    ax.xaxis.labelpad = 30
    ax.yaxis.labelpad = 30
    ax.zaxis.labelpad = 30

    ax.set_zlim(min_z - 1, max_z + 1)

    ax.set_title(title, fontsize=PLOT_CONFIGS['paths']['title_fontsize'])

    ax.legend(loc=PLOT_CONFIGS['paths']['legend']['loc'],
              fancybox=True,
              framealpha=0.8,
              fontsize=PLOT_CONFIGS['paths']['legend']['fontsize'])
    ax.grid(True)

    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, filename))
    plt.close(fig)


def plot_comparison_pose_error(output_dir, bags, labels, title, filename):
    """Generate comparative plots for the ROS bags in the bags array regarding the position and heading errors."""
    assert len(labels) == len(bags), 'Number of labels and bags is different'

    fig = plt.figure(figsize=(PLOT_CONFIGS['plot']['figsize'][0],
                              PLOT_CONFIGS['plot']['figsize'][1]))
    ax = fig.gca()

    min_t = None
    max_t = None

    min_pos = 0.0
    max_pos = 0.0

    for i in range(len(bags)):
        sim_eval = Evaluation(bags[i], output_dir)

        t = sim_eval.get_error_time()

        if min_t is None:
            min_t = np.min(t)
            max_t = np.max(t)
        else:
            min_t = np.min([np.min(t), min_t])
            max_t = np.min([np.max(t), max_t])

        error = sim_eval.get_error_from_data('position')

        min_pos = np.min([np.min(error), min_pos])
        max_pos = np.max([np.max(error), max_pos])

        ax.plot(sim_eval.get_error_time(),
                sim_eval.get_error_from_data('position'),
                linewidth=PLOT_CONFIGS['plot']['linewidth'],
                label=labels[i],
                zorder=len(bags) - i)

        fig.canvas.draw()
        del sim_eval

    sim_eval = Evaluation(bags[0], output_dir)
    plot_disturbance_areas(fig, ax, sim_eval, min_pos, max_pos)
    del sim_eval

    ax.set_xlabel('Time [s]', fontsize=PLOT_CONFIGS['plot']['label_fontsize'])
    ax.set_ylabel('Position error [m]',
                  fontsize=PLOT_CONFIGS['plot']['label_fontsize'])
    ax.legend(fancybox=True, framealpha=0.5, loc='upper left',
              fontsize=PLOT_CONFIGS['plot']['legend']['fontsize'])
    ax.grid(True)
    ax.tick_params(axis='both',
                   labelsize=PLOT_CONFIGS['plot']['tick_fontsize'])

    ax.set_xlim(min_t, max_t)
    ax.set_ylim(min_pos, max_pos)


    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'position_' + filename))
    plt.close(fig)

    # Plotting heading error

    fig = plt.figure(figsize=(PLOT_CONFIGS['plot']['figsize'][0],
                              PLOT_CONFIGS['plot']['figsize'][1]))
    ax = fig.gca()

    min_t = None
    max_t = None

    min_yaw = 0.0
    max_yaw = 0.0

    for i in range(len(bags)):
        sim_eval = Evaluation(bags[i], output_dir)

        t = sim_eval.get_error_time()

        if min_t is None:
            min_t = np.min(t)
            max_t = np.max(t)
        else:
            min_t = np.min([np.min(t), min_t])
            max_t = np.min([np.max(t), max_t])

        error = sim_eval.get_error_set_data('yaw')

        min_yaw = np.min([np.min(error), min_yaw])
        max_yaw = np.max([np.max(error), max_yaw])

        ax.plot(sim_eval.get_error_time(),
                sim_eval.get_error_set_data('yaw'),
                linewidth=PLOT_CONFIGS['plot']['linewidth'],
                label=labels[i],
                zorder=len(bags) - i)
        fig.canvas.draw()
        del sim_eval

    sim_eval = Evaluation(bags[0], output_dir)
    plot_disturbance_areas(fig, ax, sim_eval, min_yaw, max_yaw)
    del sim_eval

    ax.set_xlim(min_t, max_t)
    ax.set_ylim(min_pos, max_pos)

    ax.set_xlabel('Time [s]', fontsize=PLOT_CONFIGS['plot']['label_fontsize'])
    ax.set_ylabel('Heading error [rad]', fontsize=PLOT_CONFIGS['plot']['label_fontsize'])
    ax.legend(fancybox=True, framealpha=0.5, loc='upper left',
              fontsize=PLOT_CONFIGS['plot']['legend']['fontsize'])
    ax.grid(True)
    ax.tick_params(axis='both',
                   labelsize=PLOT_CONFIGS['plot']['tick_fontsize'])

    ax.set_xlim(min_t, max_t)
    ax.set_ylim(min_yaw, max_yaw)

    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'heading_' + filename))
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Run best and worst results')

    parser.add_argument('--input_dir', type=str, default='./results', nargs='+', help='Input directory with all the processed tasks')
    parser.add_argument('--input_dir_labels', type=str, default='', nargs='+', help='Labels for each input directories')
    parser.add_argument('--output_dir', type=str, default='.', help='Output folder for plots and analysis data')
    parser.add_argument('--config_file', type=str, help='Output configuration file')
    parser.add_argument('--best_results_dir', type=str, default='', help='Output directory for the best results')

    # Parse input arguments
    args = parser.parse_args(rospy.myargv()[1:])

    for d in args.input_dir:
        assert os.path.isdir(d), 'Invalid input directory, dir=' + str(d)
    assert os.path.isdir(args.output_dir), 'Invalid output directory'
    assert os.path.isfile(args.config_file), 'Invalid configuration file'
    assert len(args.input_dir) == len(args.input_dir_labels), 'Labels list not matching the input directories'

    # Setup logging
    logger = logging.getLogger('run_best_worst')
    out_hdlr = logging.StreamHandler(sys.stdout)
    out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
    out_hdlr.setLevel(logging.INFO)
    logger.addHandler(out_hdlr)
    logger.setLevel(logging.INFO)

    logger.info('Processing the following folders')
    for d in args.input_dir:
        print '\t - ', d

    with open(args.config_file, 'r') as c_file:
        config_file = yaml.load(c_file)
    logger.info('Configuration file loaded <%s>' % args.config_file)

    if 'cost_fcn' in config_file:
        cost_fcn = config_file['cost_fcn']
        logger.info('Custom cost function provided:')
        for tag in cost_fcn:
            logger.info('\t- %s: %.4f' % (tag, cost_fcn[tag]))
    else:
        cost_fcn = None

    output_dir = os.path.join(args.output_dir, 'best_worst_analysis')

    if not os.path.isdir(output_dir):
        logger.info('Creating output directory <%s>' % output_dir)
        os.makedirs(output_dir)
    else:
        logger.info('Output directory already exists <%s>' % output_dir)

    logger.info('Storing this analysis parameters in the output directory')
    with open(os.path.join(output_dir, 'analysis_configuration.yaml'), 'w') as p_file:
        yaml.safe_dump(vars(args), p_file, default_flow_style=False)

    tasks = dict()
    kpis = dict()
    tasks_cost_fcn = dict()

    logger.info('Reading the simulation results...')
    n_tasks = 0
    for d, label in zip(args.input_dir, args.input_dir_labels):
        logger.info('Processing directory = ' + d)
        logger.info('Directory label = ' + label)
        tasks[d] = list()
        kpis[d] = list()
        tasks_cost_fcn[d] = list()
        for item in sorted(os.listdir(d)):
            p = os.path.join(d, item)
            if os.path.isdir(p):
                if 'recording.bag' not in os.listdir(p):
                    continue
                cur_kpi = None
                try:
                    if 'computed_kpis.yaml' not in os.listdir(p):
                        logger.info('KPIs are not yet available')
                        logger.info('Computing KPIs')
                        sim_eval = Evaluation(os.path.join(p, 'recording.bag'), p)
                        sim_eval.save_kpis()
                        del sim_eval
                    for f in os.listdir(p):
                        if 'computed_kpis' in f:
                            kpi_filename = os.path.join(p, f)

                            with open(kpi_filename, 'r') as k_file:
                                cur_kpi = yaml.load(k_file)

                            if cost_fcn is not None:
                                tasks_cost_fcn[d].append(0.0)
                                for tag in cost_fcn:
                                    tasks_cost_fcn[d][-1] += cost_fcn[tag] * cur_kpi[tag]

                            for tag in config_file['kpis']:
                                if 'factor' in config_file['kpis'][tag]:
                                    cur_kpi[tag] *= config_file['kpis'][tag]['factor']
                except:
                    logger.info('Error loading task %s, skipping' % item)
                    cur_kpi = None
                    temp_params = None

                if cur_kpi is not None:
                    n_tasks += 1
                    kpis[d].append(cur_kpi)
                    tasks[d].append(p)

    logger.info('Finished loading tasks!')
    logger.info('Total number of tasks=%d' % n_tasks)

    # Store the evaluations for the best and worst using the KPIs in
    # the configuration file
    for tag in config_file['kpis']:
        kpi = dict()
        min_idx = dict()
        max_idx = dict()

        kpi_bags = list()
        kpi_labels = list()

        worst_kpi_bags = list()
        worst_kpi_labels = list()

        for d, i in zip(args.input_dir, range(len(args.input_dir))):
            kpi[d] = [kpis[d][k][tag] for k in range(len(kpis[d]))]
            min_idx[d] = np.argmin(kpi[d])
            max_idx[d] = np.argmax(kpi[d])

            root_path = os.path.join(output_dir, tag, '%d_%s' % (i, args.input_dir_labels[i]))
            best_root_path = os.path.join(args.best_results_dir, tag, '%d_%s' % (i, args.input_dir_labels[i]))

            logger.info('KPI = %s' % tag)
            logger.info('\tDirectory = ' + d)
            logger.info('\tLabel = ' + args.input_dir_labels[i])
            logger.info('\tStoring the KPIs and plots for the best candidate')
            logger.info('\t%s = %.4f' % (tag, kpis[d][min_idx[d]][tag]))

            gen_evaluation(os.path.join(root_path, 'best'),
                           os.path.join(tasks[d][min_idx[d]], 'recording.bag'),
                           os.path.join(tasks[d][min_idx[d]], 'task.yml'))

            if os.path.isdir(args.best_results_dir):
                if not os.path.isdir(best_root_path):
                    os.makedirs(best_root_path)
                shutil.copy(os.path.join(tasks[d][min_idx[d]], 'recording.bag'),
                            best_root_path)
                shutil.copy(os.path.join(tasks[d][min_idx[d]], 'task.yml'),
                            best_root_path)

                with open(os.path.join(best_root_path, 'metric.yaml'), 'w') as m_file:
                    metric = dict()
                    metric[tag] = kpis[d][min_idx[d]][tag]
                    yaml.dump(metric, m_file, default_flow_style=False)

                gen_evaluation(best_root_path,
                               os.path.join(best_root_path, 'recording.bag'),
                               os.path.join(tasks[d][min_idx[d]], 'task.yml'))

            logger.info('KPI = %s' % tag)
            logger.info('\tDirectory = ' + d)
            logger.info('\tLabel = ' + args.input_dir_labels[i])
            logger.info('\tStoring the KPIs and plots for the worst candidate')
            logger.info('\t%s = %.4f' % (tag, kpis[d][max_idx[d]][tag]))

            gen_evaluation(os.path.join(root_path, 'worst'),
                           os.path.join(tasks[d][max_idx[d]], 'recording.bag'),
                           os.path.join(tasks[d][max_idx[d]], 'task.yml'))

            bags = [os.path.join(tasks[d][min_idx[d]], 'recording.bag'),
                    os.path.join(tasks[d][max_idx[d]], 'recording.bag')]

            kpi_bags.append(os.path.join(tasks[d][min_idx[d]], 'recording.bag'))
            worst_kpi_bags.append(os.path.join(tasks[d][max_idx[d]], 'recording.bag'))

            labels = ['Best simulation scenario [Metric = %.4f]' % kpis[d][min_idx[d]][tag],
                      'Worst simulation scenario [Metric = %.4f]' % kpis[d][max_idx[d]][tag]]

            kpi_labels.append(args.input_dir_labels[i])
            worst_kpi_labels.append(args.input_dir_labels[i])

            plot_comparison_pose_error(output_dir=root_path,
                                       bags=bags,
                                       labels=labels,
                                       title='%s - %s [%s]' % (config_file['kpis'][tag]['title'], config_file['kpis'][tag]['var'], config_file['kpis'][tag]['unit']),
                                       filename='error_comparison_%s.pdf' % tag)

            plot_paths(output_dir=root_path,
                       bags=bags,
                       labels=labels,
                       title='',
                       filename='paths_comparison_%s.pdf' % tag)

        plot_comparison_pose_error(output_dir=os.path.join(output_dir, tag),
                                   bags=kpi_bags,
                                   labels=kpi_labels,
                                   title='%s - %s [%s]' % (config_file['kpis'][tag]['title'], config_file['kpis'][tag]['var'], config_file['kpis'][tag]['unit']),
                                   filename='error_comparison.pdf')

        plot_paths(output_dir=os.path.join(output_dir, tag),
                   bags=kpi_bags,
                   labels=kpi_labels,
                   title='',
                   filename='paths_comparison.pdf')

        plot_comparison_pose_error(output_dir=os.path.join(output_dir, tag),
                                   bags=worst_kpi_bags,
                                   labels=worst_kpi_labels,
                                   title='%s - %s [%s]' % (config_file['kpis'][tag]['title'], config_file['kpis'][tag]['var'], config_file['kpis'][tag]['unit']),
                                   filename='worst_case_error_comparison.pdf')

        plot_paths(output_dir=os.path.join(output_dir, tag),
                   bags=worst_kpi_bags,
                   labels=worst_kpi_labels,
                   title='',
                   filename='worst_case_paths_comparison.pdf')

    if cost_fcn is not None:
        cost_fcn_min_idx = dict()
        cost_fcn_max_idx = dict()

        cost_fcn_bags = list()
        cost_fcn_labels = list()

        worst_cost_fcn_bags = list()
        worst_cost_fcn_labels = list()

        tag = 'cost_fcn'
        for d, i in zip(args.input_dir, range(len(args.input_dir))):
            # If a custom cost function is available, compute the best and worst
            # candidates according to the cost function

            logger.info('Calculating the best and worst candidates according to the cost function')

            cost_fcn_min_idx[d] = np.argmin(tasks_cost_fcn[d])
            cost_fcn_max_idx[d] = np.argmax(tasks_cost_fcn[d])

            root_path = os.path.join(output_dir, tag, '%d_%s' % (i, args.input_dir_labels[i]))
            best_root_path = os.path.join(output_dir, tag, '%d_%s' % (i, args.input_dir_labels[i]))

            if not os.path.isdir(root_path):
                os.makedirs(root_path)

            logger.info('Cost function configuration copied to <%s>' % root_path)

            logger.info('\tStoring the KPIs and plots for the best candidate according to the cost function')
            logger.info('\t%s = %.4f' % (tag, tasks_cost_fcn[d][cost_fcn_min_idx[d]]))

            gen_evaluation(os.path.join(root_path, 'best'),
                           os.path.join(tasks[d][cost_fcn_min_idx[d]], 'recording.bag'),
                           os.path.join(tasks[d][cost_fcn_min_idx[d]], 'task.yml'))

            if not os.path.isdir(best_root_path):
                os.makedirs(best_root_path)
            shutil.copy(os.path.join(tasks[d][cost_fcn_min_idx[d]], 'recording.bag'),
                        best_root_path)

            output_cost_fcn = dict(fcn=cost_fcn)
            output_cost_fcn['value'] = tasks_cost_fcn[d][cost_fcn_min_idx[d]]
            with open(os.path.join(best_root_path, 'cost_fcn.yaml'), 'w') as c_file:
                yaml.dump(output_cost_fcn, c_file, default_flow_style=False)

            if 'parameters' in config_file:
                with open(os.path.join(tasks[d][cost_fcn_min_idx[d]], 'task.yml'), 'r') as t_file:
                    task = yaml.load(t_file)
                params = dict()
                for p in config_file['parameters']:
                    params[p] = task[p]
                with open(os.path.join(best_root_path, 'params.yaml'), 'w') as p_file:
                    yaml.dump(params, p_file, default_flow_style=False)

            gen_evaluation(best_root_path,
                           os.path.join(best_root_path, 'recording.bag'),
                           os.path.join(tasks[d][cost_fcn_min_idx[d]], 'task.yml'))


            logger.info('\tStoring the KPIs and plots for the worst candidate according to the cost function')
            logger.info('\t%s = %.4f' % (tag, tasks_cost_fcn[d][cost_fcn_max_idx[d]]))

            gen_evaluation(os.path.join(root_path, 'worst'),
                           os.path.join(tasks[d][cost_fcn_max_idx[d]], 'recording.bag'),
                           os.path.join(tasks[d][cost_fcn_max_idx[d]], 'task.yml'))

            bags = [os.path.join(tasks[d][cost_fcn_min_idx[d]], 'recording.bag'),
                    os.path.join(tasks[d][cost_fcn_max_idx[d]], 'recording.bag')]

            cost_fcn_bags.append(os.path.join(tasks[d][cost_fcn_min_idx[d]], 'recording.bag'))
            worst_cost_fcn_bags.append(os.path.join(tasks[d][cost_fcn_max_idx[d]], 'recording.bag'))

            labels = ['Best parameter set [Cost function = %.4f]' % tasks_cost_fcn[d][cost_fcn_min_idx[d]],
                      'Worst parameter set [Cost function = %.4f]' % tasks_cost_fcn[d][cost_fcn_max_idx[d]]]

            cost_fcn_labels.append('Best parameter set [%s]' % args.input_dir_labels[i])
            worst_cost_fcn_labels.append('Worst parameter set [%s]' % args.input_dir_labels[i])

            plot_comparison_pose_error(output_dir=root_path,
                                       bags=bags,
                                       labels=labels,
                                       title='Position and heading errors',
                                       filename='error_comparison_%s.pdf' % tag)

            plot_paths(output_dir=root_path,
                       bags=bags,
                       labels=labels,
                       title='',
                       filename='paths_comparison_%s.pdf' % tag)

        plot_comparison_pose_error(output_dir=os.path.join(output_dir, tag),
                                   bags=cost_fcn_bags,
                                   labels=cost_fcn_labels,
                                   title='Position and heading errors',
                                   filename='error_comparison.pdf')

        plot_paths(output_dir=os.path.join(output_dir, tag),
                   bags=cost_fcn_bags,
                   labels=cost_fcn_labels,
                   title='',
                   filename='paths_comparison.pdf')

        plot_comparison_pose_error(output_dir=os.path.join(output_dir, tag),
                                   bags=worst_cost_fcn_bags,
                                   labels=worst_cost_fcn_labels,
                                   title='Position and heading errors',
                                   filename='worst_case_error_comparison.pdf')

        plot_paths(output_dir=os.path.join(output_dir, tag),
                   bags=worst_cost_fcn_bags,
                   labels=worst_cost_fcn_labels,
                   title='',
                   filename='worst_case_paths_comparison.pdf')
