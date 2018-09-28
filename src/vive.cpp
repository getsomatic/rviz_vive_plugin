//
// Created by Denys Kotelovych on 04.09.18.
//

#include <ros/ros.h>

#include "rviz_vive_plugin/vive.h"

namespace rviz_vive_plugin {

Vive::Vive() : initialized_(false) {
}

Vive::~Vive() {
    vr::VR_Shutdown();
    vrSystem_ = nullptr;
}

bool Vive::Init(const std::string &actionManifestPath) {
    ROS_DEBUG_STREAM_NAMED("vive", "Initializing Vive Controller");

    vr::EVRInitError eError = vr::VRInitError_None;
    vrSystem_ = vr::VR_Init(&eError, vr::VRApplication_Scene);
    if (eError != vr::VRInitError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "FAiled to init VR: " << eError);
        vrSystem_ = nullptr;
        return false;
    }

    driver_ = TrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_TrackingSystemName_String);
    display_ = TrackedDeviceString(vr::k_unTrackedDeviceIndex_Hmd, vr::Prop_SerialNumber_String);

    if (!vr::VRCompositor()) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to initialise OpenVR Compositor");
        return false;
    }

    vr::EVRInputError error;
    error = vr::VRInput()->SetActionManifestPath(actionManifestPath.c_str());
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action manifest");
        return false;
    }

    error = vr::VRInput()->GetActionSetHandle("/actions/demo", &actionSet_);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action set");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left_Menu_Pressed", &actions_.Left.MenuPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right_Menu_Pressed", &actions_.Right.MenuPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left_Grip", &actions_.Left.GripPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right_Grip", &actions_.Right.GripPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left_Trigger_Pressed", &actions_.Left.TriggerPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right_Trigger_Pressed", &actions_.Right.TriggerPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left_Trackpad", &actions_.Left.Trackpad);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right_Trackpad", &actions_.Right.Trackpad);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left", &actions_.Left.Pose);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right", &actions_.Right.Pose);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/out/Hand_Left_Vibration", &actions_.Left.Vibration);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/out/Hand_Right_Vibration", &actions_.Right.Vibration);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Left_Trackpad_Pressed", &actions_.Left.TrackpadPressed);
    if (error != vr::VRInputError_None) {
        ROS_DEBUG_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    error = vr::VRInput()->GetActionHandle(
            "/actions/demo/in/Hand_Right_Trackpad_Pressed", &actions_.Right.TrackpadPressed);
    if (error != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Failed to load action handle");
        return false;
    }

    vrSystem_->GetRecommendedRenderTargetSize(&renderWidth_, &renderHeight_);
    ROS_DEBUG_STREAM_NAMED("vive", "Recommended Render Target Size: " << renderWidth_ << "x" << renderHeight_);

    textureBounds_.uMin = 0;
    textureBounds_.uMax = 1;
    textureBounds_.vMin = 1;
    textureBounds_.vMax = 0;

    initialized_ = true;

    return true;
}

bool Vive::ReadAll() {
    vr::VRActiveActionSet_t actionSet = {};
    actionSet.ulActionSet = actionSet_;

    if (vr::VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1) != vr::VRInputError_None) {
        ROS_ERROR_STREAM_NAMED("vive", "Error updating actions state");
        return false;
    }

    return true;
}

bool Vive::ReadHMD(Vive::HMD &result) {
    vr::VRCompositor()->WaitGetPoses(trackedPoses_, vr::k_unMaxTrackedDeviceCount, nullptr, 0);

    result.Pose = trackedPoses_[vr::k_unTrackedDeviceIndex_Hmd];

    return result.Pose.bPoseIsValid;
}

bool Vive::ReadLeftController(Vive::Controller &result) {
    if (!LoadPoseActionData(actions_.Left.Pose, result.Pose)) {
        ROS_ERROR_STREAM_NAMED("vive", "Error loading left controller position");
        return false;
    }

    if (!LoadAnalogActionData(actions_.Left.Trackpad, result.Trackpad)) {
        ROS_ERROR_STREAM_NAMED("vive", "Error loading left controller trackpad");
        return false;
    }

    result.TriggerPressed = LoadDigitalActionData(actions_.Left.TriggerPressed);
    result.MenuPressed = LoadDigitalActionData(actions_.Left.MenuPressed);
    result.GripPressed = LoadDigitalActionData(actions_.Left.GripPressed);
    result.TrackpadPressed = LoadDigitalActionData(actions_.Left.TrackpadPressed);

    return true;
}

bool Vive::ReadRightController(Vive::Controller &result) {
    if (!LoadPoseActionData(actions_.Right.Pose, result.Pose)) {
        ROS_ERROR_STREAM_NAMED("vive", "Error loading right controller position");
        return false;
    }

    if (!LoadAnalogActionData(actions_.Right.Trackpad, result.Trackpad)) {
        ROS_ERROR_STREAM_NAMED("vive", "Error loading right controller trackpad");
        return false;
    }

    result.TriggerPressed = LoadDigitalActionData(actions_.Right.TriggerPressed);
    result.MenuPressed = LoadDigitalActionData(actions_.Right.MenuPressed);
    result.GripPressed = LoadDigitalActionData(actions_.Right.GripPressed);
    result.TrackpadPressed = LoadDigitalActionData(actions_.Right.TrackpadPressed);

    return true;
}

void Vive::RegisterGLTextures(void *leftId, void *rightId) {
    vrTextures_[0] = {leftId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma};
    vrTextures_[1] = {rightId, vr::TextureType_OpenGL, vr::ColorSpace_Gamma};
}

void Vive::SubmitGLTextures() {
    vr::VRCompositor()->Submit(vr::EVREye::Eye_Left, &vrTextures_[0], &textureBounds_);
    vr::VRCompositor()->Submit(vr::EVREye::Eye_Right, &vrTextures_[1], &textureBounds_);
}

vr::HmdMatrix34_t Vive::LeftEyeTransform() const {
    return vrSystem_->GetEyeToHeadTransform(vr::EVREye::Eye_Left);
}

vr::HmdMatrix34_t Vive::RightEyeTransform() const {
    return vrSystem_->GetEyeToHeadTransform(vr::EVREye::Eye_Right);
}

vr::HmdMatrix44_t Vive::LeftEyeProjectionMatrix(float nearClip, float farClip) const {
    return vrSystem_->GetProjectionMatrix(vr::EVREye::Eye_Left, nearClip, farClip);
}

vr::HmdMatrix44_t Vive::RightEyeProjectionMatrix(float nearClip, float farClip) const {
    return vrSystem_->GetProjectionMatrix(vr::EVREye::Eye_Right, nearClip, farClip);
}

void Vive::VibrateLeft() {
    vr::VRInput()->TriggerHapticVibrationAction(
            actions_.Left.Vibration, 0, 1.0f, 4.0f, 1.0f, vr::k_ulInvalidInputValueHandle);
}

void Vive::VibrateRight() {
    vr::VRInput()->TriggerHapticVibrationAction(
            actions_.Right.Vibration, 0, 1.0f, 4.0f, 1.0f, vr::k_ulInvalidInputValueHandle);
}

std::string Vive::TrackedDeviceString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop) {
    uint32_t bufferLength = vrSystem_->GetStringTrackedDeviceProperty(unDevice, prop, nullptr, 0);
    if (bufferLength == 0)
        return "";
    char *buffer = new char[bufferLength];
    vrSystem_->GetStringTrackedDeviceProperty(unDevice, prop, buffer, bufferLength);
    std::string result = buffer;
    delete[] buffer;
    return result;
}

bool Vive::LoadPoseActionData(const vr::VRActionHandle_t &action, vr::TrackedDevicePose_t &result) {
    vr::InputPoseActionData_t data{};
    auto error = vr::VRInput()->GetPoseActionData(
            action, vr::TrackingUniverseStanding, 0, &data, sizeof(data), vr::k_ulInvalidInputValueHandle);
    if (error != vr::VRInputError_None || !data.bActive || !data.pose.bPoseIsValid)
        return false;
    result = data.pose;
    return true;
}

bool Vive::LoadAnalogActionData(const vr::VRActionHandle_t &action, vr::InputAnalogActionData_t &result) {
    auto error = vr::VRInput()->GetAnalogActionData(action, &result, sizeof(result), vr::k_ulInvalidInputValueHandle);
    return error == vr::VRInputError_None && result.bActive;
}

bool Vive::LoadDigitalActionData(vr::VRActionHandle_t action) {
    vr::InputDigitalActionData_t data{};
    vr::VRInput()->GetDigitalActionData(action, &data, sizeof(data), vr::k_ulInvalidInputValueHandle);
    return data.bActive && data.bState;
}

}