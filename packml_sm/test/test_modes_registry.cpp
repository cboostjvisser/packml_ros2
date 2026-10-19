// Copyright (c) 2026 PackML ROS2 Contributors
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

#include <gtest/gtest.h>
#include <map>
#include <string>

#include "packml_sm/default_modes.hpp"
#include "packml_sm/modes_registry.hpp"

// The registry is process-wide and these tests share it with every other test in this binary, so
// they assert about values no other test registers rather than about the whole table. 40-49 is
// reserved for that here; packml_sm's own vocabulary stops at 3.
namespace
{
constexpr packml_sm::ModeType kScratchMode = 40;
constexpr packml_sm::ModeType kRenamedMode = 41;
constexpr packml_sm::ModeType kNeverDeclared = 49;
}  // namespace

TEST(ModesRegistry, LinkingTheBundledHeaderDeclaresPackmlSmsOwnModes)
{
  // packml_sm's library registers modes/default_modes.yaml, so a program that links it and
  // declares nothing of its own still validates against those four.
  EXPECT_TRUE(packml_sm::is_known_mode(packml_modes::Invalid));
  EXPECT_TRUE(packml_sm::is_known_mode(packml_modes::Production));
  EXPECT_TRUE(packml_sm::is_known_mode(packml_modes::Maintenance));
  EXPECT_TRUE(packml_sm::is_known_mode(packml_modes::Manual));
}

TEST(ModesRegistry, AnUndeclaredValueIsNotKnown)
{
  EXPECT_FALSE(packml_sm::is_known_mode(kNeverDeclared));
  EXPECT_FALSE(packml_sm::is_known_mode(99));
  EXPECT_FALSE(packml_sm::is_known_mode(-1));
}

TEST(ModesRegistry, RegisteringMakesAValueKnownAndNamesIt)
{
  ASSERT_FALSE(packml_sm::is_known_mode(kScratchMode));

  packml_sm::register_modes({{"EmptyOut", kScratchMode}});

  EXPECT_TRUE(packml_sm::is_known_mode(kScratchMode));
  EXPECT_EQ(packml_sm::to_string(kScratchMode), "EmptyOut");
}

TEST(ModesRegistry, ARuntimeTableRegistersTheSameWay)
{
  const std::map<std::string, packml_sm::ModeType> declared{{"Renamed", kRenamedMode}};
  packml_sm::register_modes(declared);

  EXPECT_TRUE(packml_sm::is_known_mode(kRenamedMode));
  EXPECT_EQ(packml_sm::to_string(kRenamedMode), "Renamed");
}

TEST(ModesRegistry, TheLastNameRegisteredForAValueWins)
{
  // How a deployment's modes_config_file overrides the name a linked modes header gave a value:
  // it registers after static initialisation, so it always registers last.
  packml_sm::register_modes({{"FirstName", kRenamedMode}});
  packml_sm::register_modes({{"SecondName", kRenamedMode}});

  EXPECT_EQ(packml_sm::to_string(kRenamedMode), "SecondName");
}

TEST(ModesRegistry, AnUndeclaredValueStringifiesAsItsNumber)
{
  EXPECT_EQ(packml_sm::to_string(kNeverDeclared), "49");
}

TEST(ModesRegistry, ToStringResolvesToTheNamedOverloadNotTheGenericTemplate)
{
  // common.hpp also has a generic to_string<T> that stringifies anything. The regression this
  // guards is silent: mode logging degrades from "Production" to "1" the moment a call site
  // cannot see the named overload.
  EXPECT_EQ(packml_sm::to_string(packml_modes::Production), "Production");
}

TEST(ModesRegistry, KnownModesReportsTheWholeVocabulary)
{
  const auto modes = packml_sm::known_modes();

  ASSERT_NE(modes.find(packml_modes::Production), modes.end());
  EXPECT_EQ(modes.at(packml_modes::Production), "Production");
  EXPECT_EQ(modes.find(kNeverDeclared), modes.end());
}
