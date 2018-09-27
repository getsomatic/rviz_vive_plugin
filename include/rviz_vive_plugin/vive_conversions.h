//
// Created by Denys Kotelovych on 07.09.18.
//

#ifndef RVIZ_VIVE_PLUGIN_VIVE_CONVERSIONS_H
#define RVIZ_VIVE_PLUGIN_VIVE_CONVERSIONS_H

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreMatrix4.h>

#include "rviz_vive_plugin/vive.h"

namespace rviz_vive_plugin {

struct Pose {
    Ogre::Vector3 Position;
    Ogre::Quaternion Orientation;
};

struct Controller {
    struct Pose Pose;
    Ogre::Vector3 TrackpadPosition;
    bool TriggerPressed;
    bool TrackpadPressed;
    bool MenuPressed;
    bool GripPressed;
};

struct HMD {
    struct Pose Pose;
};

Ogre::Vector3 TranslatePositionToOgreCoordinateSystem(const Ogre::Vector3 &position);

Ogre::Quaternion TranslateOrientationToOgreCoordinateSystem(const Ogre::Quaternion &orientation);

Ogre::Matrix4 Convert(const vr::HmdMatrix34_t &matPose);

Ogre::Matrix4 Convert(const vr::HmdMatrix44_t &matPose);

Pose Convert(const vr::TrackedDevicePose_t &pose);

HMD Convert(const Vive::HMD &hmd);

Controller Convert(const Vive::Controller &controller);

}

#endif //RVIZ_VIVE_PLUGIN_CONVERT_VIVE_H
