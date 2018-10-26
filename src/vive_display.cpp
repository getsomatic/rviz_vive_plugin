//
// Created by Denys Kotelovych on 04.09.18.
//

#include <random>

#include <QApplication>
#include <QDesktopWidget>

#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreCompositorManager.h>
#include <OGRE/OgreCompositorInstance.h>
#include <OGRE/OgreCompositionTargetPass.h>
#include <OGRE/OgreCompositionPass.h>
#include <OGRE/OgreRenderTexture.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreEntity.h>
#include <OGRE/OgreSubMesh.h>
#include <OGRE/OgreSubEntity.h>
#include <OGRE/RenderSystems/GL/OgreGLTextureManager.h>
#include <OGRE/RenderSystems/GL/OgreGLRenderSystem.h>
#include <OGRE/RenderSystems/GL/OgreGLTexture.h>

#include <rviz/properties/tf_frame_property.h>
#include <rviz/properties/quaternion_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/vector_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/line.h>
#include <rviz/frame_manager.h>

#include <sensor_msgs/Joy.h>
#include <rviz_vive_plugin_msgs/Controller.h>

#include "rviz_vive_plugin/vive_display.h"
#include "rviz_vive_plugin/vive_conversions.h"
#include "rviz_vive_plugin/vive.h"

static inline tf::StampedTransform BuildTransform(const rviz_vive_plugin::Pose &pose,
                                                  const std::string &parentFrameId,
                                                  const std::string &childFrameId) {
    return tf::StampedTransform(
        tf::Transform(
            tf::Quaternion(pose.Orientation.x, pose.Orientation.y, pose.Orientation.z, pose.Orientation.w),
            tf::Vector3(pose.Position.x, pose.Position.y, pose.Position.z)
        ), ros::Time::now(), parentFrameId, childFrameId);
}

static inline rviz_vive_plugin_msgs::Controller BuildMessage(const rviz_vive_plugin::Controller &controller,
                                                             const std::string &frameId) {
    rviz_vive_plugin_msgs::Controller msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frameId;
    msg.trackpad_position.x = controller.TrackpadPosition.x;
    msg.trackpad_position.y = controller.TrackpadPosition.y;
    msg.trackpad_position.z = 0.0f;
    msg.trigger = controller.TriggerPressed; // TBD: trigger value
    msg.trackpad_touched = 0; // TBD: trackpad touched
    msg.trackpad_pressed = static_cast<unsigned char>(controller.TrackpadPressed);
    msg.menu_pressed = static_cast<unsigned char>(controller.MenuPressed);
    msg.grip_pressed = static_cast<unsigned char>(controller.GripPressed);
    return msg;
};

namespace rviz_vive_plugin {

ViveDisplay::ViveDisplay() {
    ogre_.HMD.Node = nullptr;
    ogre_.Left.Texture.setNull();
    ogre_.Right.Texture.setNull();
    ogre_.Left.TextureGLID = 0;
    ogre_.Right.TextureGLID = 0;
    ogre_.Left.Viewport = nullptr;
    ogre_.Right.Viewport = nullptr;
    leftVibrationDuration_ = 0.0;
    rightVibrationDuration_ = 0.0;
    leftVibrationTimestamp_ = ros::Time::now();
    rightVibrationTimestamp_ = ros::Time::now();
}

ViveDisplay::~ViveDisplay() {
    pubs_.LeftHand.shutdown();
    pubs_.RightHand.shutdown();
    subs_.VibrationLeft.shutdown();
    subs_.VibrationRight.shutdown();
    scene_manager_->destroyCamera(ogre_.Left.EyeCamera);
    scene_manager_->destroyCamera(ogre_.Right.EyeCamera);
    scene_manager_->destroyLight(ogre_.HMD.Light);
    scene_manager_->destroySceneNode(ogre_.HMD.Node);
    ogre_.RenderWindow->removeAllListeners();
    ogre_.RenderWindow->removeAllViewports();
    delete ogre_.RenderWidget;
};

void ViveDisplay::onInitialize() {
    pubs_.LeftHand = node_.advertise<rviz_vive_plugin_msgs::Controller>("/vive/left_controller/state", 1);
    pubs_.RightHand = node_.advertise<rviz_vive_plugin_msgs::Controller>("/vive/right_controller/state", 1);
    subs_.VibrationLeft = node_.subscribe(
        "/vive/left_controller/vibration/cmd", 1, &ViveDisplay::LeftVibrationMessageReceived, this);
    subs_.VibrationRight = node_.subscribe(
        "/vive/right_controller/vibration/cmd", 1, &ViveDisplay::RightVibrationMessageReceived, this);

    if (!vive_.Init(ros::package::getPath("rviz_vive_plugin") + "/media/actions.json")) {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to initialize Vive");
        return;
    }

    if (!InitOgre()) {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to initialize Ogre");
        return;
    }
}

bool ViveDisplay::InitOgre() {
    ogre_.HMD.Node = scene_manager_->getRootSceneNode()->createChildSceneNode();

    ogre_.Left.EyeCamera = scene_manager_->createCamera("left_camera");
    ogre_.Left.EyeCamera->setAutoAspectRatio(true);
    ogre_.HMD.Node->attachObject(ogre_.Left.EyeCamera);

    ogre_.Right.EyeCamera = scene_manager_->createCamera("right_camera");
    ogre_.Right.EyeCamera->setAutoAspectRatio(true);
    ogre_.HMD.Node->attachObject(ogre_.Right.EyeCamera);

    ogre_.Left.EyeCamera->setPosition(Convert(vive_.LeftEyeTransform()).getTrans());
    ogre_.Left.EyeCamera->setOrientation(Ogre::Quaternion::IDENTITY);

    ogre_.Right.EyeCamera->setPosition(Convert(vive_.RightEyeTransform()).getTrans());
    ogre_.Right.EyeCamera->setOrientation(Ogre::Quaternion::IDENTITY);

    auto textureManager = dynamic_cast<Ogre::GLTextureManager *>(Ogre::TextureManager::getSingletonPtr());

    ogre_.Left.Texture = textureManager->createManual(
        "left_texture",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        vive_.RenderWidth(),
        vive_.RenderHeight(),
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET,
        nullptr,
        false);

    ogre_.Right.Texture = textureManager->createManual(
        "right_texture",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        Ogre::TEX_TYPE_2D,
        vive_.RenderWidth(),
        vive_.RenderHeight(),
        0,
        Ogre::PF_R8G8B8,
        Ogre::TU_RENDERTARGET,
        nullptr,
        false);

    ogre_.Left.Texture->getCustomAttribute("GLID", &ogre_.Left.TextureGLID);
    ogre_.Right.Texture->getCustomAttribute("GLID", &ogre_.Right.TextureGLID);

    ogre_.Left.Viewport = ogre_.Left.Texture->getBuffer()->getRenderTarget()->addViewport(ogre_.Left.EyeCamera);
    ogre_.Right.Viewport = ogre_.Right.Texture->getBuffer()->getRenderTarget()->addViewport(ogre_.Right.EyeCamera);

    static const float nearClip = 0.01f;
    static const float farClip = 1000.0f;

    const auto &projLeft = Convert(vive_.LeftEyeProjectionMatrix(nearClip, farClip));
    const auto &projRight = Convert(vive_.RightEyeProjectionMatrix(nearClip, farClip));

    ogre_.Left.EyeCamera->setCustomProjectionMatrix(true, projLeft);
    ogre_.Right.EyeCamera->setCustomProjectionMatrix(true, projRight);

    vive_.RegisterGLTextures((void *) ogre_.Left.TextureGLID, (void *) ogre_.Right.TextureGLID);

    ogre_.RenderWidget = new rviz::RenderWidget(rviz::RenderSystem::get());
    ogre_.RenderWidget->setWindowTitle("Vive");
    ogre_.RenderWidget->setParent(context_->getWindowManager()->getParentWindow());
    ogre_.RenderWidget->setWindowFlags(
        Qt::Window | Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMaximizeButtonHint);
    ogre_.RenderWidget->setVisible(true);

    ogre_.RenderWindow = ogre_.RenderWidget->getRenderWindow();
    ogre_.RenderWindow->setVisible(true);
    ogre_.RenderWindow->setAutoUpdated(false);

    Ogre::Viewport *port;
    port = ogre_.RenderWindow->addViewport(ogre_.Left.EyeCamera, 0, 0.0, 0.0f, 0.5f, 1.0f);
    port->setClearEveryFrame(true);
    port->setBackgroundColour(Ogre::ColourValue::Black);

    port = ogre_.RenderWindow->addViewport(ogre_.Right.EyeCamera, 1, 0.5f, 0.0f, 0.5f, 1.0f);
    port->setClearEveryFrame(true);
    port->setBackgroundColour(Ogre::ColourValue::Black);

    ogre_.HMD.Light = scene_manager_->createLight("point_light");
    ogre_.HMD.Light->setType(Ogre::Light::LT_POINT);
    ogre_.HMD.Light->setDiffuseColour(1.0, 1.0, 1.0);
    ogre_.HMD.Light->setSpecularColour(1.0, 1.0, 1.0);
    ogre_.HMD.Node->attachObject(ogre_.HMD.Light);

    return true;
}

void ViveDisplay::update(float, float) {
    const auto now = ros::Time::now();
    const auto dt1 = now - leftVibrationTimestamp_;
    const auto dt2 = now - rightVibrationTimestamp_;

    if (dt1.toSec() < leftVibrationDuration_)
        vive_.VibrateLeft();

    if (dt2.toSec() < rightVibrationDuration_)
        vive_.VibrateRight();

    if (!vive_.ReadAll())
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to all Vive data");

    Vive::HMD viveHMD{};
    if (vive_.ReadHMD(viveHMD)) {
        const auto &hmd = Convert(viveHMD);
        const auto &transform = BuildTransform(hmd.Pose, "world", "vive_hmd");
        ogre_.HMD.Node->setPosition(hmd.Pose.Position);
        ogre_.HMD.Node->setOrientation(hmd.Pose.Orientation);
        tb_.sendTransform(transform);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read HMD from Vive");
    }

    Vive::Controller viveLeftController{};
    if (vive_.ReadLeftController(viveLeftController)) {
        const auto &controller = Convert(viveLeftController);
        const auto &transform = BuildTransform(controller.Pose, "world", "vive_left_controller");
        const auto &msg = BuildMessage(controller, "vive_left_controller");
        tb_.sendTransform(transform);
        pubs_.LeftHand.publish(msg);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read left controller data from Vive");
    }

    Vive::Controller viveRightController{};
    if (vive_.ReadRightController(viveRightController)) {
        const auto &controller = Convert(viveRightController);
        const auto &transform = BuildTransform(controller.Pose, "world", "vive_right_controller");
        const auto &msg = BuildMessage(controller, "vive_right_controller");
        tb_.sendTransform(transform);
        pubs_.RightHand.publish(msg);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read right controller data from Vive");
    }

    ogre_.RenderWindow->update(true);
    vive_.SubmitGLTextures();
}

void ViveDisplay::reset() {
}

void ViveDisplay::LeftVibrationMessageReceived(const std_msgs::Float32Ptr &msg) {
    leftVibrationTimestamp_ = ros::Time::now();
    leftVibrationDuration_ = msg->data;
}

void ViveDisplay::RightVibrationMessageReceived(const std_msgs::Float32Ptr &msg) {
    rightVibrationTimestamp_ = ros::Time::now();
    rightVibrationDuration_ = msg->data;
}

} // namespace rviz_vive_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_vive_plugin::ViveDisplay, rviz::Display)