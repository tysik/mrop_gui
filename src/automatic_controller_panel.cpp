/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2015, Poznan University of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Author: Mateusz Przybyla
 */

#include "automatic_controller_panel.h"

namespace mtracker_gui
{

AutomaticControllerPanel::AutomaticControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("automatic_controller_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("automatic_controller_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
}

void AutomaticControllerPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      //
    }
    else {
      //
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void AutomaticControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void AutomaticControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::AutomaticControllerPanel, rviz::Panel)
