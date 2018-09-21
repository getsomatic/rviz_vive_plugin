//
// Created by Denys Kotelovych on 04.09.18.
//

#ifndef RVIZ_VIVE_PLUGIN_VIVE_H
#define RVIZ_VIVE_PLUGIN_VIVE_H

#include <string>

#include <openvr.h>

namespace rviz_vive_plugin {

class Vive {
public:
    struct Controller {
        vr::TrackedDevicePose_t Pose;
        vr::InputAnalogActionData_t Trackpad;
        bool TriggerPressed;
        bool TrackpadPressed;
        bool MenuPressed;
        bool GripPressed;
    };

    struct HMD {
        vr::TrackedDevicePose_t Pose;
    };

public:
    Vive();

    ~Vive();

public:
    bool Init(const std::string &actionManifestPath);

    bool ReadHMD(HMD &hmd);

    bool ReadLeftController(Controller &controller);

    bool ReadRightController(Controller &controller);

    void RegisterGLTextures(void *leftId, void *rightId);

    void SubmitGLTextures();

    void VibrateLeft();

    void VibrateRight();

public:
    const std::string &Driver() const { return driver_; }

    const std::string &Display() const { return display_; }

    std::uint32_t RenderWidth() const { return renderWidth_; }

    std::uint32_t RenderHeight() const { return renderHeight_; }

    bool Initialized() const { return initialized_; }

    vr::HmdMatrix34_t LeftEyeTransform() const;

    vr::HmdMatrix34_t RightEyeTransform() const;

    vr::HmdMatrix44_t LeftEyeProjectionMatrix(float nearClip, float farClip) const;

    vr::HmdMatrix44_t RightEyeProjectionMatrix(float nearClip, float farClip) const;

private:
    std::string TrackedDeviceString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop);

    bool LoadPoseActionData(const vr::VRActionHandle_t &action, vr::TrackedDevicePose_t &result);

    bool LoadAnalogActionData(const vr::VRActionHandle_t &action, vr::InputAnalogActionData_t &result);

    bool LoadDigitalActionData(vr::VRActionHandle_t action);

private:
    vr::IVRSystem *vrSystem_;
    vr::Texture_t vrTextures_[2];
    std::string driver_, display_;
    vr::VRTextureBounds_t textureBounds_;
    vr::TrackedDevicePose_t trackedPoses_[vr::k_unMaxTrackedDeviceCount];
    uint32_t renderWidth_, renderHeight_;
    vr::VRActionSetHandle_t actionSet_;
    bool initialized_;

    struct {
        struct {
            vr::VRActionHandle_t MenuPressed;
            vr::VRActionHandle_t TriggerPressed;
            vr::VRActionHandle_t GripPressed;
            vr::VRActionHandle_t TrackpadPressed;
            vr::VRActionHandle_t Trackpad;
            vr::VRActionHandle_t Pose;
            vr::VRActionHandle_t Vibration;
        } Left, Right;
    } actions_;
};

}

#endif //RVIZ_VIVE_PLUGIN_VIVE_H
