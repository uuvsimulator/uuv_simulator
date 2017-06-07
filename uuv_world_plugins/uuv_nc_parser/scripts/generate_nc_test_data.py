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

import os
import argparse
import numpy as np
from netCDF4 import Dataset
import roslib
roslib.load_manifest('uuv_nc_parser')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate test NC data')
    parser.add_argument('--dir', default='.', type=str)

    args = parser.parse_args()

    if not os.path.isdir(args.dir):
        raise Exception('Invalid output directory, dir=' + str(args.dir))

    # Open a new NC data
    nc_fid = Dataset(os.path.join(args.dir, 'test_data.nc'),
                     'w', format='NETCDF4')

    nc_fid.description = "UUV Simulator - Test NC Data"
    nc_fid.title = "UUV Simulator - Test NC Data"

    # Creating the necessary dimensions
    nc_fid.createDimension('time', 10)
    nc_fid.createDimension('xc', 20)
    nc_fid.createDimension('yc', 20)
    nc_fid.createDimension('zc', 10)

    nc_var = nc_fid.createVariable('time', np.float64, ('time',))
    nc_var.setncatts(dict(long_name=u'Time',
                          standard_name='time',
                          units=u'hours since 2017-1-1 00:00:0.0',
                          axis=u't'))
    # Assign the dimension data to the new NetCDF file.
    nc_fid.variables['time'][:] = np.linspace(0, 10, nc_fid.dimensions['time'].size)

    #########################################################################
    # Create coordinates
    #########################################################################
    # X coordinates
    nc_var = nc_fid.createVariable('xc', np.float64, ('xc'))
    nc_var.setncatts(dict(long_name=u'X coordinates in the test volume',
                          var_desc=u'X coordinates',
                          standard_name='X',
                          units=u'm',
                          axis=u'X [m]'))
    # Assign the dimension data to the new NetCDF file.
    nc_fid.variables['xc'][:] = np.linspace(0, 2000, nc_fid.dimensions['xc'].size)

    # Y coordinates
    nc_var = nc_fid.createVariable('yc', np.float64, ('yc'))
    nc_var.setncatts(dict(long_name=u'Y coordinates in the test volume',
                          var_desc=u'Y coordinates',
                          standard_name='Y',
                          units=u'm',
                          axis=u'Y [m]'))
    # Assign the dimension data to the new NetCDF file.
    nc_fid.variables['yc'][:] = np.linspace(0, 2000, nc_fid.dimensions['yc'].size)

    # Z coordinates
    nc_var = nc_fid.createVariable('zc', np.float64, ('zc'))
    nc_var.setncatts(dict(long_name=u'Z coordinates in the test volume',
                          var_desc=u'Z coordinates',
                          standard_name='Z',
                          units=u'm',
                          axis=u'Z [m]'))
    # Assign the dimension data to the new NetCDF file.
    nc_fid.variables['zc'][:] = np.linspace(0, 100, nc_fid.dimensions['zc'].size)

    #########################################################################
    # Create random data
    #########################################################################
    # Temperature
    nc_var = nc_fid.createVariable('temperature', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Temperature in the test volume',
                          var_desc=u'Temperature',
                          standard_name='Temperature',
                          units=u'K',
                          axis=u'Temperature [K]'))
    shape = (nc_fid.dimensions['time'].size,
             nc_fid.dimensions['zc'].size,
             nc_fid.dimensions['yc'].size,
             nc_fid.dimensions['xc'].size)
    nc_fid.variables['temperature'][:] = 280.15 + np.random.rand(*shape)

    # Salinity
    nc_var = nc_fid.createVariable('salinity', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Water salinity in the test volume',
                          var_desc=u'Salinity',
                          standard_name='Salinity',
                          units=u'ppt',
                          axis=u'Salinity [ppt]'))
    nc_fid.variables['salinity'][:] = 35.0 + np.random.rand(*shape)

    # Turbidity
    nc_var = nc_fid.createVariable('turbidity', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Water turbidity in the test volume',
                          var_desc=u'Turbidity',
                          standard_name='Turbidity',
                          units=u'ppt',
                          axis=u'Turbidity [NTU]'))
    nc_fid.variables['turbidity'][:] = 60.0 + np.random.rand(*shape)

    # H2S Concentration
    nc_var = nc_fid.createVariable('h2s', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'H2S concentration salinity in the test volume',
                          var_desc=u'H2S',
                          standard_name='H2S',
                          units=u'ppt',
                          axis=u'H2S concentration [ml/l]'))
    nc_fid.variables['h2s'][:] = 2.25 + np.random.rand(*shape)

    # Current velocity
    nc_var = nc_fid.createVariable('u_east', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Eastward current velocity',
                          var_desc=u'Eastward current velocity',
                          standard_name='Eastward current velocity',
                          units=u'm/s',
                          axis=u'Eastward current velocity [m/s]'))
    nc_fid.variables['u_east'][:] = np.zeros(shape)

    nc_var = nc_fid.createVariable('v_north', np.float64, ('time', 'zc', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Northward current velocity',
                          var_desc=u'Northward current velocity',
                          standard_name='Northward current velocity',
                          units=u'm/s',
                          axis=u'Northward current velocity [m/s]'))
    nc_fid.variables['v_north'][:] = np.random.rand(*shape)

    # Wind velocity
    nc_var = nc_fid.createVariable('w_east', np.float64, ('time', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Eastward wind',
                          var_desc=u'Eastward wind',
                          standard_name='Eastward wind',
                          units=u'm/s',
                          axis=u'Eastward wind [m/s]'))
    shape = (nc_fid.dimensions['time'].size,
             nc_fid.dimensions['yc'].size,
             nc_fid.dimensions['xc'].size)
    nc_fid.variables['w_east'][:] = np.zeros(shape)

    nc_var = nc_fid.createVariable('w_north', np.float64, ('time', 'yc', 'xc'))
    nc_var.setncatts(dict(long_name=u'Northward wind',
                          var_desc=u'Northward wind',
                          standard_name='Northward wind',
                          units=u'm/s',
                          axis=u'Northward wind [m/s]'))
    nc_fid.variables['w_north'][:] = np.zeros(shape)

    print nc_fid
    print 'variables='
    for key in nc_fid.variables:
        print key

    nc_fid.close()
