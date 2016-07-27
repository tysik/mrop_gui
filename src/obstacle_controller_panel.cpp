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

#include "obstacle_controller_panel.h"

namespace mtracker_gui
{

ObstacleControllerPanel::ObstacleControllerPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<mtracker::Trigger>("obstacle_controller_trigger_srv");
  params_cli_ = nh_.serviceClient<mtracker::Params>("obstacle_controller_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  kappa_edit_ = new QLineEdit("3.0");
  kappa_edit_->setEnabled(false);

  epsilon_edit_ = new QLineEdit("0.0001");
  epsilon_edit_->setEnabled(false);

  k_w_edit_ = new QLineEdit("0.1");
  k_w_edit_->setEnabled(false);

  b_edit_ = new QLineEdit("5.0");
  b_edit_->setEnabled(false);

  a_edit_ = new QLineEdit("1.0");
  a_edit_->setEnabled(false);

  set_button_ = new QPushButton("Set");
  set_button_->setEnabled(false);

  QSpacerItem* margin = new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Fixed);

  QHBoxLayout* fields1_layout = new QHBoxLayout;
  fields1_layout->addItem(margin);
  fields1_layout->addWidget(new QLabel("a:"));
  fields1_layout->addWidget(a_edit_);
  fields1_layout->addWidget(new QLabel("b_:"));
  fields1_layout->addWidget(b_edit_);
  fields1_layout->addWidget(new QLabel("k_w:"));
  fields1_layout->addWidget(k_w_edit_);
  fields1_layout->addItem(margin);

  QHBoxLayout* fields2_layout = new QHBoxLayout;
  fields2_layout->addItem(margin);
  fields2_layout->addWidget(new QLabel("kappa:"));
  fields2_layout->addWidget(kappa_edit_);
  fields2_layout->addWidget(new QLabel("epsilon:"));
  fields2_layout->addWidget(epsilon_edit_);
  fields2_layout->addItem(margin);

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(fields1_layout);
  layout->addLayout(fields2_layout);
  layout->addWidget(set_button_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
}

void ObstacleControllerPanel::updateParams() {
  mtracker::Params params;
  params.request.params.resize(5);

  try {params.request.params[0] = boost::lexical_cast<double>(kappa_edit_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ kappa_edit_->setText(":-("); return; }

  try {params.request.params[1] = boost::lexical_cast<double>(epsilon_edit_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ epsilon_edit_->setText(":-("); return; }

  try {params.request.params[2] = boost::lexical_cast<double>(k_w_edit_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ k_w_edit_->setText(":-("); return; }

  try {params.request.params[3] = boost::lexical_cast<double>(b_edit_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ b_edit_->setText(":-("); return; }

  try {params.request.params[4] = boost::lexical_cast<double>(a_edit_->text().toStdString()); }
  catch(boost::bad_lexical_cast &){ a_edit_->setText(":-("); return; }

  params_cli_.call(params);
}

void ObstacleControllerPanel::trigger(bool checked) {
  mtracker::Trigger trigger;
  trigger.request.activate = checked;

  if (trigger_cli_.call(trigger)) {
    if (checked) {
      kappa_edit_->setEnabled(true);
      epsilon_edit_->setEnabled(true);
      k_w_edit_->setEnabled(true);
      a_edit_->setEnabled(true);
      b_edit_->setEnabled(true);
      set_button_->setEnabled(true);
    }
    else {
      kappa_edit_->setEnabled(false);
      epsilon_edit_->setEnabled(false);
      k_w_edit_->setEnabled(false);
      a_edit_->setEnabled(false);
      b_edit_->setEnabled(false);
      set_button_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void ObstacleControllerPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void ObstacleControllerPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mtracker_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mtracker_gui::ObstacleControllerPanel, rviz::Panel)
