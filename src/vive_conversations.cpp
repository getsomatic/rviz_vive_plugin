//
// Created by Denys Kotelovych on 07.09.18.
//

#include "rviz_vive_plugin/vive_conversations.h"

namespace rviz_vive_plugin {

Ogre::Vector3 TranslatePositionToOgreCoordinateSystem(const Ogre::Vector3 &position) {
    return Ogre::Vector3(position.x, -position.z, position.y);
}

Ogre::Quaternion TranslateOrientationToOgreCoordinateSystem(const Ogre::Quaternion &orientation) {
    return Ogre::Quaternion(0.707, 0.707, 0.0, 0.0) * orientation;
}

Ogre::Matrix4 Convert(const vr::HmdMatrix34_t &matPose) {
    return Ogre::Matrix4(
            matPose.m[0][0], matPose.m[0][1], matPose.m[0][2], matPose.m[0][3],
            matPose.m[1][0], matPose.m[1][1], matPose.m[1][2], matPose.m[1][3],
            matPose.m[2][0], matPose.m[2][1], matPose.m[2][2], matPose.m[2][3],
            0.0f, 0.0f, 0.0f, 1.0f);
}

Ogre::Matrix4 Convert(const vr::HmdMatrix44_t &mat) {
    return Ogre::Matrix4(
            mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],
            mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],
            mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],
            mat.m[3][0], mat.m[3][1], mat.m[3][2], mat.m[3][3]);
}

Pose Convert(const vr::TrackedDevicePose_t &pose) {
    const auto &mat = Convert(pose.mDeviceToAbsoluteTracking);
    Pose result;
    result.Position = TranslatePositionToOgreCoordinateSystem(mat.getTrans());
    result.Orientation = TranslateOrientationToOgreCoordinateSystem(mat.extractQuaternion());
    return result;
}

HMD Convert(const Vive::HMD &hmd) {
    HMD result;
    result.Pose = Convert(hmd.Pose);
    return result;
}

Controller Convert(const Vive::Controller &controller) {
    Controller result;
    result.Pose = Convert(controller.Pose);
    result.TriggerPressed = controller.TriggerPressed;
    result.TrackpadPressed = controller.TrackpadPressed;
    result.GripPressed = controller.GripPressed;
    result.MenuPressed = controller.MenuPressed;
    result.TrackpadPosition = Ogre::Vector3(controller.Trackpad.x, controller.Trackpad.y, 0.0f);
    return result;
}

}