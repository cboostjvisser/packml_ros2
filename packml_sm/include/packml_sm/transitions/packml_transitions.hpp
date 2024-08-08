// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2 compatibility)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <iostream>
#include <memory>

#include "QEvent"
#include "QAbstractTransition"
// #include "packml_sm/common.hpp"
// #include "packml_sm/states/state.hpp"
// #include "packml_sm/states_generator.hpp"

namespace packml_sm
{

/**
* @brief Class to define transitions that are not valid
*/
class PackmlTransition : public QAbstractTransition
{
public:
  /**
  * @brief Constructor of the class
  */
  PackmlTransition() {}

//   std::shared_ptr<std::vector<StatesGenerator::Mode>> modes;
//   std::string current_mode = "";

  /**
  * @brief Destructor of the class
  */
  virtual ~PackmlTransition() {}

protected:
  /**
  * @brief Function to check if the transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e)
  {
    // for (auto && item : *modes.get()) {
    //     if (item.name == current_mode) {
            auto statetarget = targetState();
            if (statetarget->property("Available").toBool())
            {
                std::cout << "Transition is available!" << std::endl;
                return true;
            }
            else {
                std::cout << "Transition to next state: is not available in this mode!" << std::endl;
                e->ignore();
                return false;
            }
    //     }
    // }
    // std::cout << "unkown mode: " << item.name <<
  };

  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}
};

}  // namespace packml_sm