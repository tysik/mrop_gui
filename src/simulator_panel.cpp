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

SimulatorPanel::SimulatorPanel(QWidget* parent) : rviz::Panel(parent), nh_(""), nh_local_("simulator") {
  getParams();

  params_cli_ = nh_local_.serviceClient<std_srvs::Empty>("params");

  activate_checkbox_ = new QCheckBox("On/Off");
  activate_checkbox_->setChecked(p_active_);

  set_button_ = new QPushButton("Set");
  set_button_->setEnabled(p_active_);

  t_f_input_ = new QLineEdit(QString::number(p_time_constant_));
  t_f_input_->setEnabled(p_active_);

  t_o_input_ = new QLineEdit(QString::number(p_time_delay_));
  t_o_input_->setEnabled(p_active_);

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
  connect(set_button_, SIGNAL(clicked()), this, SLOT(setParamsButton()));
}

void SimulatorPanel::trigger(bool checked) {
  p_active_ = checked;
  setParams();

  if (p_active_ && notifyParamsUpdate()) {
    t_f_input_->setEnabled(true);
    t_o_input_->setEnabled(true);
    set_button_->setEnabled(true);
  }
  else {
    t_f_input_->setEnabled(false);
    t_o_input_->setEnabled(false);
    set_button_->setEnabled(false);

    activate_checkbox_->setChecked(false);
    p_active_ = false;
    setParams();
  }
}


void SimulatorPanel::setParamsButton() {
  setParams();
  notifyParamsUpdate();
}

void SimulatorPanel::setParams() {
  verifyInputs();

  nh_local_.setParam("active", p_active_);
  nh_local_.setParam("time_constant", p_time_constant_);
  nh_local_.setParam("time_delay", p_time_delay_);

  nh_local_.setParam("initial_x", p_initial_x_);
  nh_local_.setParam("initial_y", p_initial_y_);
  nh_local_.setParam("initial_theta", p_initial_theta_);
}

void SimulatorPanel::getParams() {
  nh_local_.param<bool>("active", p_active_, false);

  nh_local_.param<double>("time_constant", p_time_constant_, 0.0);
  nh_local_.param<double>("time_delay", p_time_delay_, 0.0);

  nh_local_.param<double>("initial_x", p_initial_x_, 0.0);
  nh_local_.param<double>("initial_y", p_initial_y_, 0.0);
  nh_local_.param<double>("initial_theta", p_initial_theta_, 0.0);
}

bool SimulatorPanel::notifyParamsUpdate() {
  std_srvs::Empty empty;
  return params_cli_.call(empty);
}

bool SimulatorPanel::verifyInputs() {
  try { p_time_constant_ = boost::lexical_cast<double>(t_f_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_time_constant_ = 0.0; t_f_input_->setText("0.0"); return false; }

  try { p_time_delay_ = boost::lexical_cast<double>(t_o_input_->text().toStdString()); }
  catch(boost::bad_lexical_cast &) { p_time_delay_ = 0.0; t_o_input_->setText("0.0"); return false; }

  return true;
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
