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

#include "controls_scaling_panel.h"

namespace mtracker_gui
{

ControlsScalingPanel::ControlsScalingPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("controls_scaling_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("controls_scaling_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  set_button_ = new QPushButton("Set");
  set_button_->setDisabled(true);

  max_wheel_rate_input_ = new QLineEdit("15.0");
  max_wheel_rate_input_->setAlignment(Qt::AlignRight);
  max_wheel_rate_input_->setDisabled(true);

  QHBoxLayout* h_layout = new QHBoxLayout;
  h_layout->addWidget(new QLabel("Max wheel rate:"));
  h_layout->addWidget(max_wheel_rate_input_);
  h_layout->addWidget(new QLabel("rad/s"));
  h_layout->addWidget(set_button_);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(h_layout);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
}

void ControlsScalingPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      set_button_->setEnabled(true);
      max_wheel_rate_input_->setEnabled(true);
    }
    else {
      set_button_->setEnabled(false);
      max_wheel_rate_input_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ControlsScalingPanel::updateParams() {
  mtracker::Params params;
  params.request.params.resize(1);

  try {
    params.request.params[0] = boost::lexical_cast<double>(max_wheel_rate_input_->text().toStdString());
  }
  catch(boost::bad_lexical_cast &) {
    max_wheel_rate_input_->setText(":-(");
  }

  params_cli_.call(params);
}

void ControlsScalingPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ControlsScalingPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ControlsScalingPanel, rviz::Panel)
