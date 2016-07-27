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

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "pose_visual.h"
#include "path_display.h"

namespace mrop_gui
{

PathDisplay::PathDisplay() {
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT(updateColor()));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT(updateHistoryLength()));
  history_length_property_->setMin(1);
  history_length_property_->setMax(10000);
}

void PathDisplay::onInitialize() {
  MFDClass::onInitialize();
  updateHistoryLength();
}

PathDisplay::~PathDisplay() {}

void PathDisplay::reset() {
  MFDClass::reset();
  visuals_.clear();
}

void PathDisplay::updateColor() {
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++) {
    visuals_[i]->setColor(color.r, color.g, color.b, 1.0);
  }
}

void PathDisplay::updateHistoryLength() {
  visuals_.rset_capacity(history_length_property_->getInt());
}

void PathDisplay::processMessage(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;

  if (!context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  boost::shared_ptr<PoseVisual> visual;
  if (visuals_.full()) {
    visual = visuals_.front();
  }
  else {
    visual.reset(new PoseVisual(context_->getSceneManager(), scene_node_));
  }

  visual->setMessage(msg);
  visual->setFramePosition(position);
  visual->setFrameOrientation(orientation);

  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor(color.r, color.g, color.b, 1.0);

  visuals_.push_back(visual);

  for (uint i = 0; i < visuals_.size(); ++i) {
    visuals_[i]->setColor(color.r, color.g, color.b, (float)i / visuals_.size());
  }
}

} // end namespace mrop_gui

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrop_gui::PathDisplay, rviz::Display)
