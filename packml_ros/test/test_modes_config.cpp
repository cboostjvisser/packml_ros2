// Copyright (c) 2026 PackML ROS2 Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// ---
// Tests for parse_modes_config() YAML configuration parser.

#include <gtest/gtest.h>
#include <fstream>
#include <map>
#include <string>
#include <cstdio>

#include "packml_sm/modes_config.hpp"
#include "packml_sm/common.hpp"

class ModesConfigTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Create a temp YAML file for tests
    temp_file_ = "/tmp/packml_ros_test_modes_config.yaml";
  }

  void TearDown() override
  {
    std::remove(temp_file_.c_str());
  }

  void write_yaml(const std::string & content)
  {
    std::ofstream ofs(temp_file_);
    ofs << content;
    ofs.close();
  }

  std::string temp_file_;
};

TEST_F(ModesConfigTest, ValidConfigParsesCorrectly)
{
  write_yaml(R"(
modes:
  PRODUCTION: 1
  MANUAL: 2
  MAINTENANCE: 3

state_masks:
  MANUAL:
    HOLDING: false
    HELD: false
    UNHOLDING: false
    SUSPENDING: false
    SUSPENDED: false
    UNSUSPENDING: false
)");

  auto result = packml_sm::parse_modes_config(temp_file_);

  // Should have at least the MANUAL mode with mask
  ASSERT_NE(result.find(2), result.end());
  auto & manual_mask = result[2];

  // HOLDING should be disabled
  EXPECT_FALSE(manual_mask[packml_sm::State::HOLDING]);
  EXPECT_FALSE(manual_mask[packml_sm::State::HELD]);
  EXPECT_FALSE(manual_mask[packml_sm::State::SUSPENDING]);
  EXPECT_FALSE(manual_mask[packml_sm::State::SUSPENDED]);
  EXPECT_FALSE(manual_mask[packml_sm::State::UNSUSPENDING]);

  // Unlisted states should default to true
  EXPECT_TRUE(manual_mask[packml_sm::State::IDLE]);
  EXPECT_TRUE(manual_mask[packml_sm::State::EXECUTE]);
  EXPECT_TRUE(manual_mask[packml_sm::State::STOPPED]);
}

TEST_F(ModesConfigTest, DeclaredModesDoNotRequireStateMasks)
{
  // A mode declared with no mask is legitimate -- it means fully open -- so the declaration and
  // the mask table have to be readable independently of one another.
  write_yaml(R"(
modes:
  Invalid: 0
  Production: 1
  Clean: 5
  EmptyOut: 7
)");

  const auto declared = packml_sm::parse_declared_modes(temp_file_);

  const std::map<std::string, packml_sm::ModeType> expected{
    {"Invalid", 0}, {"Production", 1}, {"Clean", 5}, {"EmptyOut", 7}};
  EXPECT_EQ(declared, expected);
  EXPECT_TRUE(packml_sm::parse_modes_config(temp_file_).empty());
}

TEST_F(ModesConfigTest, MissingDeclaredModesFileReturnsEmptyMap)
{
  EXPECT_TRUE(packml_sm::parse_declared_modes("/tmp/nonexistent_file_xyz123.yaml").empty());
}

TEST_F(ModesConfigTest, AFileWithNoModesSectionDeclaresNothing)
{
  write_yaml(R"(
state_masks:
  PRODUCTION:
    HOLDING: false
)");

  EXPECT_TRUE(packml_sm::parse_declared_modes(temp_file_).empty());
}

TEST_F(ModesConfigTest, MissingFileReturnsEmptyMap)
{
  auto result = packml_sm::parse_modes_config("/tmp/nonexistent_file_xyz123.yaml");
  EXPECT_TRUE(result.empty());
}

TEST_F(ModesConfigTest, MissingStateMasksSectionReturnsEmptyMap)
{
  write_yaml(R"(
modes:
  PRODUCTION: 1
  MANUAL: 2
)");

  auto result = packml_sm::parse_modes_config(temp_file_);
  EXPECT_TRUE(result.empty());
}

TEST_F(ModesConfigTest, UnlistedStatesDefaultToTrue)
{
  write_yaml(R"(
modes:
  PRODUCTION: 1

state_masks:
  PRODUCTION:
    HOLDING: false
)");

  auto result = packml_sm::parse_modes_config(temp_file_);
  ASSERT_NE(result.find(1), result.end());
  auto & mask = result[1];

  EXPECT_FALSE(mask[packml_sm::State::HOLDING]);
  // All other states should be true
  EXPECT_TRUE(mask[packml_sm::State::IDLE]);
  EXPECT_TRUE(mask[packml_sm::State::EXECUTE]);
  EXPECT_TRUE(mask[packml_sm::State::STOPPED]);
  EXPECT_TRUE(mask[packml_sm::State::ABORTED]);
}

TEST_F(ModesConfigTest, UndefinedModeInStateMasksIsSkipped)
{
  write_yaml(R"(
modes:
  PRODUCTION: 1

state_masks:
  NONEXISTENT_MODE:
    HOLDING: false
)");

  auto result = packml_sm::parse_modes_config(temp_file_);
  // NONEXISTENT_MODE has no matching entry in modes, should be skipped
  EXPECT_TRUE(result.empty());
}

TEST_F(ModesConfigTest, MultipleModesParsedCorrectly)
{
  write_yaml(R"(
modes:
  PRODUCTION: 1
  MANUAL: 2

state_masks:
  PRODUCTION:
    HOLDING: true
    HELD: true
  MANUAL:
    HOLDING: false
    HELD: false
    UNHOLDING: false
)");

  auto result = packml_sm::parse_modes_config(temp_file_);

  ASSERT_EQ(result.size(), 2u);
  EXPECT_TRUE(result[1][packml_sm::State::HOLDING]);
  EXPECT_TRUE(result[1][packml_sm::State::HELD]);
  EXPECT_FALSE(result[2][packml_sm::State::HOLDING]);
  EXPECT_FALSE(result[2][packml_sm::State::HELD]);
}

// A mode that disables a wait state while leaving its acting state enabled is incoherent: the
// acting state would have nowhere to land. The parser repairs it rather than rejecting the file.
TEST_F(ModesConfigTest, IncoherentMaskIsRepairedOnLoad)
{
  write_yaml(R"(
modes:
  MANUAL: 2

state_masks:
  MANUAL:
    HELD: false
    EXECUTE: false
)");

  auto result = packml_sm::parse_modes_config(temp_file_);
  ASSERT_NE(result.find(2), result.end());
  EXPECT_TRUE(result[2][packml_sm::State::EXECUTE])
    << "EXECUTE is mandatory and must be restored on load";
  EXPECT_TRUE(result[2][packml_sm::State::HELD])
    << "HOLDING/UNHOLDING are still enabled, so HELD must be restored on load";
}
