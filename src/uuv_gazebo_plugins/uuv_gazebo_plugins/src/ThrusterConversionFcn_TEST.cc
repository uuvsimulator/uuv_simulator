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

#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include <uuv_gazebo_plugins/ThrusterConversionFcn.hh>

boost::shared_ptr<gazebo::ConversionFunction> ConversionFromString(
    const std::string& description)
{
  std::stringstream stream;
  stream << "<sdf version='" << SDF_VERSION << "'>" << std::endl
         << "<model name='test_model'>" << std::endl
         << "<plugin name='test_plugin' filename='test_file.so'>" << std::endl
         << description
         << "</plugin>" << std::endl
         << "</model>" << std::endl
         << "</sdf>" << std::endl;

  sdf::SDF sdfParsed;
  sdfParsed.SetFromString(stream.str());

  sdf::ElementPtr conversion = sdfParsed.Root()->GetElement("model")
      ->GetElement("plugin")->GetElement("conversion");

  boost::shared_ptr<gazebo::ConversionFunction> func;
  func.reset(gazebo::ConversionFunctionFactory::GetInstance().
             CreateConversionFunction(conversion));

  return func;
}

TEST(ThrusterConversionFcn, Basic)
{
  std::string description =
        "<conversion> \n"
        "  <type>Basic</type> \n"
        "  <rotorConstant>0.0049</rotorConstant> \n"
        "</conversion>";

  boost::shared_ptr<gazebo::ConversionFunction> func;
  func = ConversionFromString(description);

  EXPECT_TRUE(func != NULL);
  EXPECT_EQ(func->GetType(), "Basic");

  EXPECT_EQ(func->convert(0.0), 0.0);
  EXPECT_EQ(func->convert(50.), 50.0*50.0*0.0049);
  EXPECT_EQ(func->convert(-50.), -50.0*50.0*0.0049);
}

TEST(ThrusterConversionFcn, Bessa)
{
  double cl = 0.001;
  double cr = 0.002;
  double dl = -50;
  double dr = 25;

  double delta = 1e-6;

  std::stringstream stream;
  stream << "<conversion> \n"
         << "  <type>Bessa</type> \n"
         << "  <rotorConstantL>" << cl << "</rotorConstantL> \n"
         << "  <rotorConstantR>" << cr << "</rotorConstantR> \n"
         << "  <deltaL>" << dl << "</deltaL> \n"
         << "  <deltaR>" << dr << "</deltaR> \n"
         << "</conversion>";

  boost::shared_ptr<gazebo::ConversionFunction> func;
  func = ConversionFromString(stream.str());

  EXPECT_TRUE(func != NULL);
  EXPECT_EQ(func->GetType(), "Bessa");

  // Test dead-zone and its boundaries
  EXPECT_EQ(0.0, func->convert(0.0));
  EXPECT_EQ(0.0, func->convert(sqrt(dr) - delta));
  EXPECT_EQ(0.0, func->convert(-sqrt(-dl) + delta));

  // Values left and right of the dead-zone
  double cmdl = -50.0;
  double cmdr =  50.0;
  EXPECT_EQ(cl*(cmdl*std::abs(cmdl)-dl), func->convert(cmdl));
  EXPECT_EQ(cr*(cmdr*std::abs(cmdr)-dr), func->convert(cmdr));
}

TEST(ThrusterConversionFcn, LinearInterp)
{
  std::vector<double> input = {-5.0, 0, 2.0, 5.0};
  std::vector<double> output = {-100, -10, 20, 120};
  std::vector<double> alpha = {0.1, 0.5, 0.9};

  std::stringstream stream;
  stream << "<conversion> \n"
         << "  <type>LinearInterp</type> \n"
         << "  <inputValues>";
  for (double d : input)
    stream << d << " ";
  stream << "</inputValues> \n"
         << "<outputValues>";
  for (double d : output)
    stream << d << " ";
  stream  << "</outputValues> \n"
         << "</conversion>";

  boost::shared_ptr<gazebo::ConversionFunction> func;
  func = ConversionFromString(stream.str());

  EXPECT_TRUE(func != NULL);
  EXPECT_EQ(func->GetType(), "LinearInterp");

  // Make sure the result is exactly correct for the provided values.
  for (int i = 0; i < input.size(); i++)
  {
    EXPECT_EQ(output[i], func->convert(input[i]));
  }

  // Outside of defined range: return closest value
  EXPECT_EQ(output[0], func->convert(input[0] - 0.5));
  EXPECT_EQ(output.back(), func->convert(input.back() + 0.5));

  // In between: make sure linear interpolation is working properly
  for (int i = 0; i < input.size()-1; i++)
  {
    double in  = alpha[i]*input[i] + (1-alpha[i])*input[i+1];
    double out = alpha[i]*output[i] + (1-alpha[i])*output[i+1];
    EXPECT_NEAR(out, func->convert(in), 1e-7);
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
