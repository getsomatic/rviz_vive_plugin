//
// Created by Denys Kotelovych on 04.09.18.
//

#include <random>
#include <ros/package.h>
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
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/string_property.h>
#include <rviz/window_manager_interface.h>
#include <rviz/display_context.h>
#include <rviz/ogre_helpers/render_widget.h>
#include <rviz/ogre_helpers/render_system.h>
#include <rviz/frame_manager.h>
#include <rviz_vive_plugin_msgs/Controller.h>
#include <rviz_vive_plugin/vive_display.h>
#include <dirent.h>

#include "rviz_vive_plugin/vive_display.h"
#include "rviz_vive_plugin/vive_conversions.h"
#include "rviz_vive_plugin/vive.h"

#include <OgreMeshManager.h>
#include "boost/filesystem.hpp"
#include <iostream>

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
    offset_ = Ogre::Vector3::ZERO;
}

ViveDisplay::~ViveDisplay() {
    pubs_.LeftHand.shutdown();
    pubs_.RightHand.shutdown();
    pubs_.HTC.shutdown();
    subs_.VibrationLeft.shutdown();
    subs_.VibrationRight.shutdown();
    scene_manager_->destroyCamera(ogre_.Left.EyeCamera);
    scene_manager_->destroyCamera(ogre_.Right.EyeCamera);
    scene_manager_->destroySceneNode(ogre_.HMD.Node);
    ogre_.RenderWindow->removeAllListeners();
    ogre_.RenderWindow->removeAllViewports();
    delete ogre_.RenderWidget;
};

void ViveDisplay::onInitialize() {
    InitProperties();

    pubs_.LeftHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
            properties_.Left.StateTopic->getStdString(), 1);
    pubs_.RightHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
            properties_.Right.StateTopic->getStdString(), 1);
    pubs_.HTC = node_.advertise<geometry_msgs::Pose>("/vive/htc_position", 1);

    subs_.VibrationLeft = node_.subscribe(
            properties_.Left.VibrationTopic->getStdString(), 1, &ViveDisplay::LeftVibrationMessageReceived, this);
    subs_.VibrationRight = node_.subscribe(
            properties_.Right.VibrationTopic->getStdString(), 1, &ViveDisplay::RightVibrationMessageReceived, this);
    subs_.HMDOffset = node_.subscribe(
            properties_.OffsetTopic->getStdString(), 1, &ViveDisplay::HMDOffsetMessageReceived, this);

    if (!vive_.Init(ros::package::getPath("rviz_vive_plugin") + "/media/actions.json")) {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to initialize Vive");
        return;
    }

    InitOgre();
}

void ViveDisplay::InitProperties() {
    properties_.RootFrame = new rviz::TfFrameProperty(
            "Root frame", rviz::TfFrameProperty::FIXED_FRAME_STRING,
            "Root frame for each transform", this, 0, true);

    properties_.Left.Frame = new rviz::StringProperty(
            "Left controller frame", "vive_left_controller", "Left controller frame", this);

    properties_.Right.Frame = new rviz::StringProperty(
            "Right controller frame", "vive_right_controller", "Right controller frame", this);

    properties_.Left.StateTopic = new rviz::StringProperty(
            "Left controller state topic", "/vive/left_controller/state", "Left controller state topic", this,
            SLOT(LeftStateTopicPropertyChanged()));

    properties_.Right.StateTopic = new rviz::StringProperty(
            "Right controller state topic", "/vive/right_controller/state", "Right controller state topic", this,
            SLOT(RightStateTopicPropertyChanged()));

    properties_.Left.VibrationTopic = new rviz::RosTopicProperty(
            "Left controller vibration topic", "",
            QString::fromStdString(ros::message_traits::datatype<std_msgs::Float32>()),
            "Left controller vibration topic", this, SLOT(LeftVibrationTopicPropertyChanged()));

    properties_.Right.VibrationTopic = new rviz::RosTopicProperty(
            "Right controller vibration topic", "",
            QString::fromStdString(ros::message_traits::datatype<std_msgs::Float32>()),
            "Right controller vibration topic", this, SLOT(RightVibrationTopicPropertyChanged()));

    properties_.HMD.Frame = new rviz::StringProperty(
            "HMD frame", "vive_left_controller", "HMD frame", this);

    properties_.OffsetTopic = new rviz::RosTopicProperty(
            "HMD offset topic", "",
            QString::fromStdString(ros::message_traits::datatype<geometry_msgs::Point32>()),
            "HMD offset topic", this, SLOT(OffsetTopicPropertyChanged()));

    properties_.RootFrame->setFrameManager(context_->getFrameManager());
}

void ViveDisplay::InitOgre() {
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
        auto hmd = Convert(viveHMD);
        hmd.Pose.Position += offset_;

        ogre_.HMD.Node->setPosition(hmd.Pose.Position);
        ogre_.HMD.Node->setOrientation(hmd.Pose.Orientation);

        geometry_msgs::Pose msg;

        msg.position.x = hmd.Pose.Position.x;
        msg.position.y = hmd.Pose.Position.y;
        msg.position.z = hmd.Pose.Position.z;

        msg.orientation.x = hmd.Pose.Orientation.x;
        msg.orientation.y = hmd.Pose.Orientation.y;
        msg.orientation.z = hmd.Pose.Orientation.z;
        msg.orientation.w = hmd.Pose.Orientation.w;

        pubs_.HTC.publish(msg);

        const auto &transform = BuildTransform(
                hmd.Pose,
                properties_.RootFrame->getFrame().toStdString(),
                properties_.HMD.Frame->getStdString());
        tb_.sendTransform(transform);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read HMD from Vive");
    }

    Vive::Controller viveLeftController{};
    if (vive_.ReadLeftController(viveLeftController)) {
        auto controller = Convert(viveLeftController);
        controller.Pose.Position += offset_;

        const auto &msg = BuildMessage(controller, "vive_left_controller");
        pubs_.LeftHand.publish(msg);

        const auto &transform = BuildTransform(
                controller.Pose,
                properties_.RootFrame->getFrame().toStdString(),
                properties_.Left.Frame->getStdString());
        tb_.sendTransform(transform);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read left controller data from Vive");
    }

    Vive::Controller viveRightController{};
    if (vive_.ReadRightController(viveRightController)) {
        auto controller = Convert(viveRightController);
        controller.Pose.Position += offset_;

        const auto &msg = BuildMessage(controller, "vive_right_controller");
        pubs_.RightHand.publish(msg);

        const auto &transform = BuildTransform(
                controller.Pose,
                properties_.RootFrame->getFrame().toStdString(),
                properties_.Right.Frame->getStdString());
        tb_.sendTransform(transform);
    } else {
        ROS_ERROR_STREAM_NAMED("vive_display", "Failed to read right controller data from Vive");
    }

    ogre_.RenderWindow->update(true);
    vive_.SubmitGLTextures();
}

void ViveDisplay::LeftStateTopicPropertyChanged() {
    pubs_.LeftHand.shutdown();
    pubs_.LeftHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
            properties_.Left.StateTopic->getStdString(), 1);
}

void ViveDisplay::RightStateTopicPropertyChanged() {
    pubs_.RightHand.shutdown();
    pubs_.RightHand = node_.advertise<rviz_vive_plugin_msgs::Controller>(
            properties_.Right.StateTopic->getStdString(), 1);
}

void ViveDisplay::LeftVibrationTopicPropertyChanged() {
    subs_.VibrationLeft.shutdown();
    subs_.VibrationLeft = node_.subscribe(
            properties_.Left.VibrationTopic->getStdString(), 1, &ViveDisplay::LeftVibrationMessageReceived, this);
}

void ViveDisplay::RightVibrationTopicPropertyChanged() {
    subs_.VibrationRight.shutdown();
    subs_.VibrationRight = node_.subscribe(
            properties_.Right.VibrationTopic->getStdString(), 1, &ViveDisplay::RightVibrationMessageReceived, this);
}

void ViveDisplay::OffsetTopicPropertyChanged() {
    subs_.HMDOffset.shutdown();
    subs_.HMDOffset = node_.subscribe(
            properties_.OffsetTopic->getStdString(), 1, &ViveDisplay::HMDOffsetMessageReceived, this);
}

void ViveDisplay::LeftVibrationMessageReceived(const std_msgs::Float32Ptr &msg) {
    leftVibrationTimestamp_ = ros::Time::now();
    leftVibrationDuration_ = msg->data;
}

void ViveDisplay::RightVibrationMessageReceived(const std_msgs::Float32Ptr &msg) {
    rightVibrationTimestamp_ = ros::Time::now();
    rightVibrationDuration_ = msg->data;
}

void ViveDisplay::HMDOffsetMessageReceived(const geometry_msgs::Point32Ptr &msg) {
    offset_.x = msg->x;
    offset_.y = msg->y;
    offset_.z = msg->z;
}



} // namespace rviz_vive_plugin

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(rviz_vive_plugin::ViveDisplay, rviz::Display)