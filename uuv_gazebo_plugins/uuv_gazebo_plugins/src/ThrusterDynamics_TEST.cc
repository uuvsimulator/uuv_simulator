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

#include <uuv_gazebo_plugins/Dynamics.hh>

boost::shared_ptr<gazebo::Dynamics> DynamicsFromString(
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

  sdf::ElementPtr dynSdf = sdfParsed.Root()->GetElement("model")
      ->GetElement("plugin")->GetElement("dynamics");

  boost::shared_ptr<gazebo::Dynamics> dyn;
  dyn.reset(gazebo::DynamicsFactory::GetInstance().
             CreateDynamics(dynSdf));

  return dyn;
}

TEST(ThrusterDynamics, ZeroOrder)
{
  std::string description =
        "<dynamics> \n"
        "  <type>ZeroOrder</type> \n"
        "</dynamics>";

  boost::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "ZeroOrder");

  EXPECT_EQ(10.0, dyn->update(10.0, 0.0));
  EXPECT_EQ(20.0, dyn->update(20.0, 0.2));
}

TEST(ThrusterDynamics, FirstOrder)
{
  std::string description =
        "<dynamics>\n"
        "  <type>FirstOrder</type>\n"
        "  <timeConstant>0.5</timeConstant>\n"
        "</dynamics>";

  boost::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "FirstOrder");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  EXPECT_NEAR(1-0.36787944, dyn->update(1.0, 0.5), 1e-5);
}

TEST(ThrusterDynamics, Yoerger)
{
  std::string description =
        "<dynamics> \n"
        "  <type>Yoerger</type>\n"
        "  <alpha>0.5</alpha>\n"
        "  <beta>0.5</beta>\n"
        "</dynamics>";

  boost::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "Yoerger");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  // TODO: Actually test dynamic behavior
}

TEST(ThrusterDynamics, Bessa)
{
  std::string description =
        "<dynamics> \n"
        "  <type>Bessa</type>\n"
        "  <Jmsp>0.5</Jmsp>\n"
        "  <Kv1>0.5</Kv1>\n"
        "  <Kv2>0.5</Kv2>\n"
        "  <Kt>0.5</Kt>\n"
        "  <Rm>0.5</Rm>\n"
        "</dynamics>";

  boost::shared_ptr<gazebo::Dynamics> dyn;
  dyn = DynamicsFromString(description);

  EXPECT_TRUE(dyn != NULL);
  EXPECT_EQ(dyn->GetType(), "Bessa");
  EXPECT_EQ(0.0, dyn->update(0.0, 0));
  // TODO: Actually test dynamic behavior
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
