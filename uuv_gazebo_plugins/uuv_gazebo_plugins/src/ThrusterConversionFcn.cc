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

#include <uuv_gazebo_plugins/ThrusterConversionFcn.hh>

#include <uuv_gazebo_plugins/Def.hh>

namespace gazebo
{
/////////////////////////////////////////////////
ConversionFunction* ConversionFunctionFactory::CreateConversionFunction(
    sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("type"))
  {
    std::cerr << "conversion does not have a type element" << std::endl;
    return NULL;
  }

  std::string identifier = _sdf->Get<std::string>("type");

  if (creators_.find(identifier) == creators_.end())
  {
    std::cerr << "Cannot creatae ConversionFunction with unknown identifier: "
              << identifier << std::endl;
    return NULL;
  }

  return creators_[identifier](_sdf);
}

/////////////////////////////////////////////////
ConversionFunctionFactory& ConversionFunctionFactory::GetInstance()
{
  static ConversionFunctionFactory instance;
  return instance;
}

/////////////////////////////////////////////////
bool ConversionFunctionFactory::RegisterCreator(const std::string& _identifier,
                               ConversionFunctionCreator _creator)
{
  if (creators_.find(_identifier) != creators_.end())
  {
    std::cerr << "Warning: Registering ConversionFunction with identifier: "
              << _identifier << " twice" << std::endl;
  }
  creators_[_identifier] = _creator;

  std::cout << "Registered ConversionFunction type "
            << _identifier << std::endl;
  return true;
}


/////////////////////////////////////////////////
const std::string ConversionFunctionBasic::IDENTIFIER = "Basic";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionBasic,
                                    &ConversionFunctionBasic::create)

/////////////////////////////////////////////////
ConversionFunction* ConversionFunctionBasic::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("rotorConstant"))
  {
    std::cerr << "ConversionFunctionBasic: expected element rotorConstant"
                 << std::endl;
    return NULL;
  }
  double gain = _sdf->Get<double>("rotorConstant");
  return new ConversionFunctionBasic(gain);
}

/////////////////////////////////////////////////
double ConversionFunctionBasic::convert(double _cmd)
{
  return this->rotorConstant*std::abs(_cmd)*_cmd;
}

/////////////////////////////////////////////////
ConversionFunctionBasic::ConversionFunctionBasic(double _rotorConstant)
  : rotorConstant(_rotorConstant)
{
}


/////////////////////////////////////////////////
const std::string ConversionFunctionBessa::IDENTIFIER = "Bessa";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionBessa,
                                    &ConversionFunctionBessa::create)

/////////////////////////////////////////////////
ConversionFunction* ConversionFunctionBessa::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("rotorConstantL"))
  {
    std::cerr << "ConversionFunctionBasic: expected element rotorConstantL"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("rotorConstantR"))
  {
    std::cerr << "ConversionFunctionBasic: expected element rotorConstantR"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("deltaL"))
  {
    std::cerr << "ConversionFunctionBasic: expected element deltaL"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("deltaR"))
  {
    std::cerr << "ConversionFunctionBasic: expected element deltaR"
                 << std::endl;
    return NULL;
  }

  return new ConversionFunctionBessa(_sdf->Get<double>("rotorConstantL"),
                                     _sdf->Get<double>("rotorConstantR"),
                                     _sdf->Get<double>("deltaL"),
                                     _sdf->Get<double>("deltaR"));
}

/////////////////////////////////////////////////
double ConversionFunctionBessa::convert(double _cmd)
{
  double basic = _cmd*std::abs(_cmd);

  if (basic <= this->deltaL)
  {
    return this->rotorConstantL*(basic - this->deltaL);
  }
  else if (basic >= this->deltaR)
  {
    return this->rotorConstantR*(basic - this->deltaR);
  }
  else
  {
    return 0;
  }
}

/////////////////////////////////////////////////
ConversionFunctionBessa::ConversionFunctionBessa(double _rotorConstantL,
                                                 double _rotorConstantR,
                                                 double _deltaL,
                                                 double _deltaR) :
  rotorConstantL(_rotorConstantL),
  rotorConstantR(_rotorConstantR),
  deltaL(_deltaL),
  deltaR(_deltaR)
{
  GZ_ASSERT(rotorConstantL >= 0.0,
            "ConversionFunctionBessa: rotorConstantL should be >= 0");
  GZ_ASSERT(rotorConstantR >= 0.0,
            "ConversionFunctionBessa: rotorConstantR should be >= 0");
  GZ_ASSERT(deltaL <= 0.0,
            "ConversionFunctionBessa: deltaL should be <= 0");
  GZ_ASSERT(deltaR >= 0.0,
            "ConversionFunctionBessa: deltaR should be >= 0");
}


/////////////////////////////////////////////////
const std::string ConversionFunctionLinearInterp::IDENTIFIER = "LinearInterp";
REGISTER_CONVERSIONFUNCTION_CREATOR(ConversionFunctionLinearInterp,
                                    &ConversionFunctionLinearInterp::create)

/////////////////////////////////////////////////
ConversionFunction* ConversionFunctionLinearInterp::create(sdf::ElementPtr _sdf)
{
  if (!_sdf->HasElement("inputValues"))
  {
    std::cerr << "ConversionFunctionLinearInterp: expected element inputValues"
                 << std::endl;
    return NULL;
  }

  if (!_sdf->HasElement("outputValues"))
  {
    std::cerr << "ConversionFunctionLinearInterp: expected element outputValues"
                 << std::endl;
    return NULL;
  }

  std::vector<double> in = Str2Vector(_sdf->Get<std::string>("inputValues"));
  std::vector<double> out = Str2Vector(_sdf->Get<std::string>("outputValues"));

  if (in.size() < 1)
  {
    std::cerr << "ConversionFunctionLinearInterp: "
              << "need at least one input/output pair"
              << std::endl;
    return NULL;
  }

  if (in.size() != out.size())
  {
    std::cerr << "ConversionFunctionLinearInterp: "
              << "number of input and output values should be the same"
              << std::endl;
    return NULL;
  }

  return new ConversionFunctionLinearInterp(in, out);
}

/////////////////////////////////////////////////
double ConversionFunctionLinearInterp::convert(double _cmd)
{
  GZ_ASSERT(!lookupTable.empty(), "lookup table is empty");

  // "first element whose key is NOT considered to go before _cmd"
  auto iter = lookupTable.lower_bound(_cmd);

  if (iter == lookupTable.end())
  {
    // "all keys are considered to go before"
    // last element is closest
    return lookupTable.rbegin()->second;
  }

  double i1 = iter->first;
  double o1 = iter->second;

  if (iter == lookupTable.begin())
    return o1;

  iter--;

  double i0 = iter->first;
  double o0 = iter->second;

  double w1 = _cmd - i0;
  double w0 = i1 - _cmd;

  return (o0*w0 + o1*w1)/(w0 + w1);
}

/////////////////////////////////////////////////
ConversionFunctionLinearInterp::ConversionFunctionLinearInterp(
    const std::vector<double> &_input,
    const std::vector<double> &_output)
{
  GZ_ASSERT(_input.size() == _output.size(), "input and output do not match");

  for (int i = 0; i < _input.size(); i++)
  {
    lookupTable[_input[i]] = _output[i];
  }
}
}
