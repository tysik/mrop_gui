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

#include "data_recorder_panel.h"

namespace mtracker_gui
{

DataRecorderPanel::DataRecorderPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("data_recorder_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("data_recorder_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  pose_checkbox_ = new QCheckBox("Pose");
  pose_checkbox_->setChecked(true);

  ref_pose_checkbox_ = new QCheckBox("Reference pose");
  ref_pose_checkbox_->setChecked(true);

  controls_checkbox_ = new QCheckBox("Controls");
  controls_checkbox_->setChecked(true);

  scaled_controls_checkbox_ = new QCheckBox("Scaled controls");
  scaled_controls_checkbox_->setChecked(true);

  obstacles_checkbox_ = new QCheckBox("Obstacles");
  obstacles_checkbox_->setChecked(true);

  potential_checkbox_ = new QCheckBox("Potential");
  potential_checkbox_->setChecked(true);

  enableCheckBoxes(false);

  start_button_ = new QPushButton;
  start_button_->setMinimumSize(50, 50);
  start_button_->setMaximumSize(50, 50);
  start_button_->setEnabled(false);
  start_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/play.png"));
  start_button_->setIconSize(QSize(25, 25));

  stop_button_ = new QPushButton;
  stop_button_->setMinimumSize(50, 50);
  stop_button_->setMaximumSize(50, 50);
  stop_button_->setEnabled(false);
  stop_button_->setIcon(QIcon("/home/tysik/workspace/catkin_ws/src/mtracker_gui/resources/stop.png"));
  stop_button_->setIconSize(QSize(25, 25));

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QGridLayout* checkbox_layout = new QGridLayout;
  checkbox_layout->addItem(margin, 0, 0, 2, 1);
  checkbox_layout->addWidget(pose_checkbox_, 0, 1);
  checkbox_layout->addWidget(ref_pose_checkbox_, 0, 2);
  checkbox_layout->addWidget(controls_checkbox_, 0, 3);
  checkbox_layout->addWidget(scaled_controls_checkbox_, 1, 1);
  checkbox_layout->addWidget(obstacles_checkbox_, 1, 2);
  checkbox_layout->addWidget(potential_checkbox_, 1, 3);
  checkbox_layout->addItem(margin, 0, 4, 2, 1);

  QHBoxLayout* buttons_layout = new QHBoxLayout;
  buttons_layout->addItem(margin);
  buttons_layout->addWidget(stop_button_);
  buttons_layout->addWidget(start_button_);
  buttons_layout->addItem(margin);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(buttons_layout);
  layout->addLayout(checkbox_layout);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(pose_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));
  connect(ref_pose_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));
  connect(controls_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));
  connect(scaled_controls_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));
  connect(obstacles_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));
  connect(potential_checkbox_, SIGNAL(clicked()), this, SLOT(chooseRecordedItems()));

  connect(start_button_, SIGNAL(clicked()), this, SLOT(start()));
  connect(stop_button_, SIGNAL(clicked()), this, SLOT(stop()));
}

void DataRecorderPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      start_button_->setEnabled(true);
      stop_button_->setEnabled(false);
      enableCheckBoxes(true);
    }
    else {
      start_button_->setEnabled(false);
      stop_button_->setEnabled(false);
      enableCheckBoxes(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void DataRecorderPanel::chooseRecordedItems() {
  mtracker::Params params;
  params.request.params.assign(8, 0.0);
  params.request.params[0] = 0.0;   // Stop
  params.request.params[1] = 1.0;   // Change params
  params.request.params[2] = static_cast<double>(pose_checkbox_->isChecked());
  params.request.params[3] = static_cast<double>(ref_pose_checkbox_->isChecked());
  params.request.params[4] = static_cast<double>(controls_checkbox_->isChecked());
  params.request.params[5] = static_cast<double>(scaled_controls_checkbox_->isChecked());
  params.request.params[6] = static_cast<double>(obstacles_checkbox_->isChecked());
  params.request.params[7] = static_cast<double>(potential_checkbox_->isChecked());

  params_cli_.call(params);
}

void DataRecorderPanel::enableCheckBoxes(bool enabled) {
  pose_checkbox_->setEnabled(enabled);
  ref_pose_checkbox_->setEnabled(enabled);
  controls_checkbox_->setEnabled(enabled);
  scaled_controls_checkbox_->setEnabled(enabled);
  obstacles_checkbox_->setEnabled(enabled);
  potential_checkbox_->setEnabled(enabled);
}

void DataRecorderPanel::start() {
  mtracker::Params params;
  params.request.params.resize(8, 0.0);
  params.request.params[0] = 1.0;   // Send start

  if (params_cli_.call(params)) {
    start_button_->setEnabled(false);
    stop_button_->setEnabled(true);

    enableCheckBoxes(false);
  }
}

void DataRecorderPanel::stop() {
  mtracker::Params params;
  params.request.params.resize(9, 0.0);
  params.request.params[0] = 0.0;   // Send stop

  if (params_cli_.call(params)) {
    start_button_->setEnabled(true);
    stop_button_->setEnabled(false);

    enableCheckBoxes(true);
  }
}

void DataRecorderPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void DataRecorderPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::DataRecorderPanel, rviz::Panel)
