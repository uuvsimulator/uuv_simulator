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

/// \file ThrusterConversionFcn.hh
/// \brief Description of the conversion function fo a thruster.

#ifndef __UUV_GAZEBO_PLUGINS_CONVERSION_FUNCTION_HH__
#define __UUV_GAZEBO_PLUGINS_CONVERSION_FUNCTION_HH__

#include <map>
#include <string>
#include <vector>

#include <sdf/sdf.hh>

namespace gazebo
{
/// \brief Abstact base class for a thruster conversion function.
class ConversionFunction
{
  /// \brief Protected constructor: Use the factory instead.
  protected: ConversionFunction() {}

  /// \brief Destructor.
  public: virtual ~ConversionFunction() {}

  /// \brief Return (derived) type of conversion function.
  public: virtual std::string GetType() = 0;

  /// \brief Return paramater in vector form for the given tag
  public: virtual bool GetParam(std::string _tag,
    double& _output) = 0;

  /// \brief Return input and output vectors of the lookup table
  public: virtual std::map<double, double> GetTable()
    { return std::map<double, double>(); };

  /// \brief Convert thruster state (e.g. angular velocity) to thrust force.
  public: virtual double convert(double _cmd) = 0;
};

/// \brief Function pointer to create a certain conversion function.
typedef ConversionFunction* (*ConversionFunctionCreator)(sdf::ElementPtr);

/// \brief Factory singleton class that creates a ConversionFunction from sdf.
class ConversionFunctionFactory
{
  /// \brief Create a ConversionFunction object according to its sdf Description
  public: ConversionFunction* CreateConversionFunction(sdf::ElementPtr _sdf);

  /// \brief Return the singleton instance of this factory.
  public: static ConversionFunctionFactory& GetInstance();

  /// \brief Register a ConversionFunction class with its creator.
  public: bool RegisterCreator(const std::string& _identifier,
                               ConversionFunctionCreator _creator);

  /// \brief Constructor is private since this is a singleton.
  private: ConversionFunctionFactory() {}

  /// \brief Map of each registered identifiers to its corresponding creator
  private: std::map<std::string, ConversionFunctionCreator> creators_;
};

/// Use the following macro within a ThrusterDynamics declaration:
#define REGISTER_CONVERSIONFUNCTION(type) \
  static const bool registeredWithFactory

/// Use the following macro before a ThrusterDynamics's definition:
#define REGISTER_CONVERSIONFUNCTION_CREATOR(type, creator) \
  const bool type::registeredWithFactory = \
  ConversionFunctionFactory::GetInstance().RegisterCreator( \
  type::IDENTIFIER, creator);


/// \brief The most basic conversion function: Thrust = const.*w*abs(w)
/// This corresponds to what is attrributed to Yoerger et al. and called Model 1
/// in Bessa et al.: Dynamic Positioning of Underwater Robotic Vehicles with
/// Thruster Dynamics Compensation.
class ConversionFunctionBasic : public ConversionFunction
{
  /// \brief Create a ConversionFunction object according to its sdf Description
  public: static ConversionFunction* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of conversion function.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Convert thruster state (e.g. angular velocity) to thrust force.
  public: virtual double convert(double _cmd);

  /// \brief Register this conversion function with the factory.
  private: REGISTER_CONVERSIONFUNCTION(ConversionFunctionBasic);

  /// \brief The unique identifier of this conversion function.
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: ConversionFunctionBasic(double _rotorConstant);
  private: double rotorConstant;
};

/// \brief Asymmetric conversion function with dead-zone nonlinearity.
/// This corresponds to what is called Model 2 in Bessa et al.:
/// Dynamic Positioning of Underwater Robotic Vehicles with Thruster Dynamics
/// Compensation.
class ConversionFunctionBessa : public ConversionFunction
{
  /// \brief Create a ConversionFunction object according to its sdf Description
  public: static ConversionFunction* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of conversion function.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Convert thruster state (e.g. angular velocity) to thrust force.
  public: virtual double convert(double _cmd);

  /// \brief Register this conversion function with the factory.
  private: REGISTER_CONVERSIONFUNCTION(ConversionFunctionBessa);

  /// \brief The unique identifier of this conversion function.
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: ConversionFunctionBessa(double _rotorConstantL,
                                   double _rotorConstantR,
                                   double _deltaL, double _deltaR);

  /// \brief Rotor constant for omega < 0.
  private: double rotorConstantL;

  /// \brief Rotor constant for omega > 0.
  private: double rotorConstantR;

  /// \brief Dead-zone for omega < 0.
  private: double deltaL;

  /// \brief Dead-zone for omega > 0.
  private: double deltaR;
};

/// \brief Conversion using linear interpolation between given data points.
class ConversionFunctionLinearInterp: public ConversionFunction
{
  /// \brief Create a ConversionFunction object according to its sdf Description
  public: static ConversionFunction* create(sdf::ElementPtr _sdf);

  /// \brief Return (derived) type of conversion function.
  public: virtual std::string GetType() { return IDENTIFIER; }

  /// \brief Return paramater in scalar form for the given tag
  public: virtual bool GetParam(std::string _tag, double& _output);

  /// \brief Return input and output vectors of the lookup table
  public: virtual std::map<double, double> GetTable();

  /// \brief Convert thruster state (e.g. angular velocity) to thrust force.
  public: virtual double convert(double _cmd);

  /// \brief Register this conversion function with the factory.
  private: REGISTER_CONVERSIONFUNCTION(ConversionFunctionLinearInterp);

  /// \brief The unique identifier of this conversion function.
  private: static const std::string IDENTIFIER;

  /// \brief Constructor.
  private: ConversionFunctionLinearInterp(const std::vector<double> &_input,
                                          const std::vector<double> &_output);

  /// \brief Lookup table maps input values -> output values.
  private: std::map<double, double> lookupTable;
};
}

#endif
