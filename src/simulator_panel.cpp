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

#include "simulator_panel.h"

namespace mrop_gui
{

SimulatorPanel::SimulatorPanel(QWidget* parent) : rviz::Panel(parent), nh_("") {
  trigger_cli_ = nh_.serviceClient<std_srvs::Trigger>("simulator_trigger_srv");
  params_cli_ = nh_.serviceClient<std_srvs::Empty>("simulator_params_srv");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(false);

  set_button_ = new QPushButton("Set");
  set_button_->setEnabled(false);

  t_f_input_ = new QLineEdit("0.1");
  t_f_input_->setEnabled(false);

  t_o_input_ = new QLineEdit("0.0");
  t_o_input_->setEnabled(false);

  QHBoxLayout* input_layout = new QHBoxLayout;
  input_layout->addWidget(new QLabel("T<sub>f</sub>:"));
  input_layout->addWidget(t_f_input_);
  input_layout->addWidget(new QLabel("s, "));
  input_layout->addWidget(new QLabel("T<sub>o</sub>:"));
  input_layout->addWidget(t_o_input_);
  input_layout->addWidget(new QLabel("s"));

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addWidget(activate_checkbox_);
  layout->addLayout(input_layout);
  layout->addWidget(set_button_);
  setLayout(layout);

  connect(activate_checkbox_, SIGNAL(clicked(bool)), this, SLOT(trigger(bool)));
  connect(set_button_, SIGNAL(clicked()), this, SLOT(updateParams()));
}

void SimulatorPanel::trigger(bool checked) {
  std_srvs::Trigger trigger;

  if (trigger_cli_.call(trigger)) {
    if (trigger.response.success) {
      t_f_input_->setEnabled(true);
      t_o_input_->setEnabled(true);
      set_button_->setEnabled(true);
    }
    else {
      t_f_input_->setEnabled(false);
      t_o_input_->setEnabled(false);
      set_button_->setEnabled(false);
    }
  }
  else {
    activate_checkbox_->setChecked(!checked);
  }
}

void SimulatorPanel::updateParams() {
//  mtracker::Params params;
//  params.request.params.resize(2);

//  try {params.request.params[0] = boost::lexical_cast<double>(t_f_input_->text().toStdString()); }
//  catch(boost::bad_lexical_cast &){ t_f_input_->setText(":-("); return; }

//  try {params.request.params[1] = boost::lexical_cast<double>(t_o_input_->text().toStdString()); }
//  catch(boost::bad_lexical_cast &){ t_o_input_->setText(":-("); return; }

//  params_cli_.call(params);
}

void SimulatorPanel::save(rviz::Config config) const {
  rviz::Panel::save(config);
}

void SimulatorPanel::load(const rviz::Config& config) {
  rviz::Panel::load(config);
}

} // end namespace mrop_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrop_gui::SimulatorPanel, rviz::Panel)
