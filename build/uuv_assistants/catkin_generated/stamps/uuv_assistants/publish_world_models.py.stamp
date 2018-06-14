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

import rospy
import sys
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from gazebo_msgs.srv import GetModelState


class WorldPublisher:
    def __init__(self):
        self._model_paths = dict()

        try:
            # Handle for retrieving model properties
            rospy.wait_for_service('/gazebo/get_model_state', 100)
            self._get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        except rospy.ROSException:
            print('/gazebo/get_model_state service is unavailable')
            sys.exit()

        if rospy.has_param('~meshes'):
            meshes = rospy.get_param('~meshes')
            if type(meshes) != dict:
                raise rospy.ROSException('A list of mesh filenames is required')

            self.add_meshes(meshes)

        self._mesh_topic = rospy.Publisher('/world_models', MarkerArray, queue_size=1)

        rate = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            self.publish_meshes()
            rate.sleep()

    def add_meshes(self, models):
        for model in models:
            if model in self._model_paths:
                print('Model %s already exists' % model)
                continue

            new_model = dict()

            new_model['position'] = [0, 0, 0]
            new_model['orientation'] = [0, 0, 0, 1]
            new_model['scale'] = [1, 1, 1]

            if 'pose' in models[model]:
                if 'position' in models[model]['pose']:
                    if len(models[model]['pose']['position']) == 3:
                        new_model['position'] = models[model]['pose']['position']
                if 'orientation' in models[model]['pose']:
                    if len(models[model]['pose']['orientation']) == 3:
                        new_model['orientation'] = quaternion_from_euler(*models[model]['pose']['orientation'])

            if 'scale' in models[model]:
                if len(models[model]['scale']) == 3:
                    new_model['scale'] = models[model]['scale']

            if 'mesh' in models[model]:
                new_model['mesh'] = models[model]['mesh']

                if 'model' in models[model]:
                    model_name = models[model]['model']
                    prop = self._get_model_state(model_name, '')
                    if prop.success:
                        new_model['position'] = [prop.pose.position.x,
                                                 prop.pose.position.y,
                                                 prop.pose.position.z]
                        new_model['orientation'] = [prop.pose.orientation.x,
                                                    prop.pose.orientation.y,
                                                    prop.pose.orientation.z,
                                                    prop.pose.orientation.w]
                    else:
                        print('Model %s not found in the current Gazebo scenario' % model)
                else:
                    print('Information about the model %s for the mesh %s could not be '
                          'retrieved' % (model, models[model]['mesh']))
            elif 'plane' in models[model]:
                new_model['plane'] = [1, 1, 1]
                if 'plane' in models[model]:
                    if len(models[model]['plane']) == 3:
                        new_model['plane'] = models[model]['plane']
                    else:
                        print('Invalid scale vector for ' + model)
            else:
                continue

            self._model_paths[model] = new_model
            print('New model being published: %s' % model)
            print('\t Position: ' + str(self._model_paths[model]['position']))
            print('\t Orientation: ' + str(self._model_paths[model]['orientation']))
            print('\t Scale: ' + str(self._model_paths[model]['scale']))

    def publish_meshes(self):
        markers = MarkerArray()
        i = 0
        total_models = len(self._model_paths.keys())
        for model in self._model_paths:
            marker = Marker()

            if 'mesh' in self._model_paths[model]:
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self._model_paths[model]['mesh']
                marker.scale.x = self._model_paths[model]['scale'][0]
                marker.scale.y = self._model_paths[model]['scale'][1]
                marker.scale.z = self._model_paths[model]['scale'][2]
            elif 'plane' in self._model_paths[model]:
                marker.type = Marker.CUBE
                marker.scale.x = self._model_paths[model]['plane'][0]
                marker.scale.y = self._model_paths[model]['plane'][1]
                marker.scale.z = self._model_paths[model]['plane'][2]

            marker.header.frame_id = 'world'
            marker.header.stamp = rospy.get_rostime()
            marker.ns = ''
            marker.id = i
            marker.action = Marker.ADD
            marker.pose.position.x = self._model_paths[model]['position'][0]
            marker.pose.position.y = self._model_paths[model]['position'][1]
            marker.pose.position.z = self._model_paths[model]['position'][2]
            marker.pose.orientation.x = self._model_paths[model]['orientation'][0]
            marker.pose.orientation.y = self._model_paths[model]['orientation'][1]
            marker.pose.orientation.z = self._model_paths[model]['orientation'][2]
            marker.pose.orientation.w = self._model_paths[model]['orientation'][3]
            marker.color.a = 0.2
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0 - float(i) / total_models

            markers.markers.append(marker)
            i += 1

        self._mesh_topic.publish(markers)


if __name__ == '__main__':
    print('Start publishing vehicle footprints to RViz')
    rospy.init_node('publish_world_models')

    try:
        world_pub = WorldPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('caught exception')
    print('exiting')
