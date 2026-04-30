// Software License Agreement (Apache License)
//
// Copyright (c) 2017 Austin Deric
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


#include <pluginlib/class_list_macros.hpp>
#include "packml_plugin/packml_plugin.hpp"

PackmlPlugin::PackmlPlugin(QWidget * parent)
: rviz_common::Panel(parent), layout(nullptr), widget_(nullptr)
{
  widget_ = new PackmlWidget();
  layout = new QVBoxLayout(this);
  layout->addWidget(widget_);
  setLayout(layout);

  widget_->setServiceNames(transition_service_.toStdString(), status_service_.toStdString());
  widget_->setStatusSource(status_source_.toStdString(), status_topic_.toStdString());
}

void PackmlPlugin::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);

  QString value;
  if (config.mapGetString("transition_service", &value)) {
    transition_service_ = value;
  }
  if (config.mapGetString("status_service", &value)) {
    status_service_ = value;
  }
  if (config.mapGetString("status_source", &value)) {
    status_source_ = value;
  }
  if (config.mapGetString("status_topic", &value)) {
    status_topic_ = value;
  }

  if (widget_) {
    widget_->setServiceNames(transition_service_.toStdString(), status_service_.toStdString());
    widget_->setStatusSource(status_source_.toStdString(), status_topic_.toStdString());
  }
}

void PackmlPlugin::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("transition_service", transition_service_);
  config.mapSetValue("status_service", status_service_);
  config.mapSetValue("status_source", status_source_);
  config.mapSetValue("status_topic", status_topic_);
}

PLUGINLIB_EXPORT_CLASS(PackmlPlugin, rviz_common::Panel)
