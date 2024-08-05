
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

#include "QEvent"
#include "QAbstractTransition"
#include "packml_sm/states/state.hpp"

namespace packml_sm
{

/**
* @brief Class defining a transition that has been completed
*/
class StateCompleteTransition : public QAbstractTransition
{
public:
  /**
  * @brief Constructor of the class
  */
  StateCompleteTransition() {}


  /**
  * @brief Constructor of the class
  * @param from - original state
  * @param to - ending state
  */
  StateCompleteTransition(PackmlState & from, PackmlState & to);


  /**
  * @brief Destructor of the class
  */
  virtual ~StateCompleteTransition() {}

protected:
  /**
  * @brief Function to check if the transition is valid
  * @param e - triggering event
  */
  virtual bool eventTest(QEvent * e);


  /**
  * @brief Function to trigger an action when the transition is happening
  * @param e - triggering event
  */
  virtual void onTransition(QEvent * e) {std::cout << e << std::endl;}
};
} // namespace packml_sm