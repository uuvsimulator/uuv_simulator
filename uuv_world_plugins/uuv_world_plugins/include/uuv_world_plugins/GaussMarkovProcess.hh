// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file GaussMarkovProcess.hh
/// \brief Implementation of a Gauss-Markov process model

#ifndef __GAUSS_MARKOV_PROCESS_HH__
#define __GAUSS_MARKOV_PROCESS_HH__

#include <gazebo/gazebo.hh>
#include <cstdlib>
#include <random>

namespace gazebo
{
  /// \brief Implementation of a Gauss-Markov process to model the current
  /// velocity and direction according to [1]
  /// [1] Fossen, Thor I. Handbook of marine craft hydrodynamics and motion
  /// control. John Wiley & Sons, 2011.
  class GaussMarkovProcess
  {
    /// \brief Class constructor
    public: GaussMarkovProcess();

    /// \brief Resets the process parameters
    public: void Reset();

    /// \brief Sets all the necessary parameters for the computation
    /// \param _mean Mean value
    /// \param _min Minimum limit
    /// \param _max Maximum limit
    /// \param _mu Process constant
    /// \param _noise Amplitude for the Gaussian white noise
    /// \return True, if all parameters were valid
    public: bool SetModel(double _mean, double _min, double _max,
        double _mu = 0, double _noise = 0);

    /// \brief Set mean process value
    /// \param _mean New mean value
    /// \return True, if value inside the limit range
    public: bool SetMean(double _mean);

    /// \brief Process variable
    public: double var;

    /// \brief Mean process value
    public: double mean;

    /// \brief Minimum limit for the process variable
    public: double min;

    /// \brief Maximum limit for the process variable
    public: double max;

    /// \brief Process constant, if zero, process becomes a random walk
    public: double mu;

    /// \brief Gaussian white noise amplitude
    public: double noiseAmp;

    /// \brief Timestamp for the last update
    public: double lastUpdate;

    /// \brief Update function for a new time stamp
    /// \param _time Current time stamp
    public: double Update(double _time);

    /// \brief Print current model paramters
    public: void Print();
  };
}

#endif  // __GAUSS_MARKOV_PROCESS_HH__
