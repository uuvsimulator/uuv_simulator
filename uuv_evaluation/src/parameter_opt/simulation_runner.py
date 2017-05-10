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

import logging
import os
import sys
import yaml
import time
import shutil
import psutil
from time import gmtime, strftime

LOG_FORMATTER = logging.Formatter('%(asctime)s %(name)-10s: %(levelname)-8s %(message)s')


class SimulationRunner(object):
    """
    This class can run a simulation scenario by calling roslaunch with configurable parameters and create a folder
    to store the simulation's ROS bag and configuration files. 
    """

    def __init__(self, params, task_filename, results_folder='./results', record_all_results=False,
                 add_folder_timestamp=True):
        # Setting up the logging
        self._logger = logging.getLogger('run_simulation_wrapper')
        out_hdlr = logging.StreamHandler(sys.stdout)
        out_hdlr.setFormatter(logging.Formatter('%(asctime)s | %(levelname)s | %(module)s | %(message)s'))
        out_hdlr.setLevel(logging.INFO)
        self._logger.addHandler(out_hdlr)
        self._logger.setLevel(logging.INFO)

        assert type(params) is dict, 'Parameter structure must be a dict'
        self._params = params

        self._sim_counter = 0

        assert os.path.isfile(task_filename), 'Invalid task file'
        self._task_filename = task_filename

        with open(self._task_filename, 'r') as task_file:
            self._task_text = task_file.read()

        self._logger.info('Task file <%s>' % self._task_filename)

        # Create results folder, if not existent
        self._results_folder = results_folder

        if self._results_folder[0:2] == './':
            self._results_folder = self._results_folder.replace('./', os.getcwd() + '/')

        if not os.path.isdir(self._results_folder):
            os.makedirs(self._results_folder)

        self._logger.info('Results folder <%s>' % self._results_folder)

        self._record_all_results = record_all_results

        self._logger.info('Record all results=' + str(record_all_results))
        # Filename for the ROS bag
        self._recording_filename = None

        self._add_folder_timestamp = add_folder_timestamp
        # Output directory
        self._sim_results_dir = None

    def __del__(self):
        if not self._record_all_results:
            self.remove_recording_dir()
        # Removing logging handlers
        while self._logger.handlers:
            self._logger.handlers.pop()

    @property
    def recording_filename(self):
        return self._recording_filename

    @property
    def current_sim_results_dir(self):
        return self._sim_results_dir

    def remove_recording_dir(self):
        if self._recording_filename is not None and not self._record_all_results:
            rec_path = os.path.dirname(self._recording_filename)
            self._logger.info('Removing old directory, path=' + rec_path) 
            shutil.rmtree(rec_path)

    def run(self, params=dict()):
        if len(params.keys()):
            for tag in self._params:
                if tag not in params:
                    raise Exception('Parameter list has the wrong dimension')
                else:
                    if type(params[tag]) == list:
                        self._params[tag] = [float(x) for x in params[tag]]
                    else:
                        self._params[tag] = params[tag]

        self.remove_recording_dir()

        if self._add_folder_timestamp:
            self._sim_results_dir = os.path.join(
                self._results_folder, strftime("%Y-%m-%d %H:%M:%S", gmtime())).replace(' ', '_')
        else:
            self._sim_results_dir = self._results_folder

        if not os.path.isdir(self._sim_results_dir):
            os.makedirs(self._sim_results_dir)

        task_filename = os.path.join(self._sim_results_dir,
                                     'task.yml')

        if len(self._params.keys()):
            with open(os.path.join(self._sim_results_dir,
                                   'params_%d.yml' % self._sim_counter), 'w') as param_file:
                yaml.dump(self._params, param_file, default_flow_style=False)

        self._logger.info('Running the simulation through system call')
        try:
            with open(task_filename, 'w') as task_file:
                task_file.write(self._task_text)

            with open(task_filename, 'r') as task_file:
                task = yaml.load(task_file)
                self._logger.info('Running task: ' + task['id'])

                self._recording_filename = os.path.join(self._sim_results_dir, 'recording.bag')
                self._logger.info('ROS bag: ' + self._recording_filename)
                cmd = task['execute']['cmd'] + ' '
                for param in task['execute']['params']:
                    cmd = cmd + param + ':=' + str(task['execute']['params'][param]) + ' '

                cmd = cmd + 'bag_filename:=\"' + self._recording_filename + '\" '

                for param in self._params:
                    param_values = str(self._params[param])
                    param_values = param_values.replace('[', '')
                    param_values = param_values.replace(']', '')
                    param_values = param_values.replace(' ', '')

                    cmd = cmd + param + ':=' + param_values + ' '
                self._logger.info('Run system call: ' + cmd)

                proc = psutil.Popen(cmd, shell=True)
                success = proc.wait(timeout=1e5)

                if success == 0:
                    self._logger.info('Simulation run successfully')
                else:
                    self._logger.info('Simulation finished with error')
        except Exception, e:
            self._logger.error('Error while running the simulation, message=' + str(e))

        self._logger.info('Simulation finished <%s>' % os.path.join(self._sim_results_dir, 'recording.bag'))
        time.sleep(0.05)

        self._sim_counter += 1
