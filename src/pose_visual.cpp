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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>

#include "pose_visual.h"

namespace mrop_gui
{

PoseVisual::PoseVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();

  dot_.reset(new rviz::Shape(rviz::Shape::Sphere, scene_manager_, frame_node_));
}

PoseVisual::~PoseVisual() {
  scene_manager_->destroySceneNode( frame_node_ );
}

void PoseVisual::setMessage(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  Ogre::Vector3 scale(0.05, 0.05, 0.05);
  dot_->setScale(scale);
}

void PoseVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

void PoseVisual::setFrameOrientation(const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

void PoseVisual::setColor(float r, float g, float b, float a) {
  dot_->setColor(r, g, b, a);
}

} // end namespace mrop_gui

