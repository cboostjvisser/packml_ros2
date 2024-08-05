// Copyright (c) 2016 Shaun Edwards
// Copyright (c) 2019 ROS-Industrial Consortium Asia Pacific (ROS 2
// compatibility)
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

#include "QEvent"
#include "QString"

namespace packml_sm {

static int PACKML_ERROR_EVENT_TYPE = QEvent::User + 3;

struct ErrorEvent : public QEvent {
  explicit ErrorEvent(const int &code_value)
      : QEvent(QEvent::Type(PACKML_ERROR_EVENT_TYPE)), code(code_value), name(),
        description() {}

  ErrorEvent(const int &code_value, const QString &name_value,
             const QString &description_value)
      : QEvent(QEvent::Type(PACKML_ERROR_EVENT_TYPE)), code(code_value),
        name(name_value), description(description_value) {}

  int code;
  QString name;
  QString description;
};
} // namespace packml_sm