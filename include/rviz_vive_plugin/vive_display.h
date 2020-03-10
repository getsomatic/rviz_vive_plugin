//
// Created by Denys Kotelovych on 04.09.18.
//

#ifndef RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H
#define RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H

#include <rviz/display.h>

#ifndef Q_MOC_RUN

#include <OGRE/OgreTexture.h>
#include <GL/glew.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>

#include "rviz_vive_plugin/vive.h"
#include "rviz_vive_plugin/vive_conversions.h"
#include <rviz_vive_plugin_msgs/ControllerVibration.h>

#endif

namespace Ogre {
class SceneNode;

class Viewport;

class Camera;

class RenderWindow;
}

namespace rviz {

class RenderWidget;

class StringProperty;

class RosTopicProperty;

class TfFrameProperty;

}

namespace rviz_vive_plugin {

class ViveDisplay : public rviz::Display {
Q_OBJECT

public:
    ViveDisplay();

    virtual ~ViveDisplay();

    virtual void onInitialize() override;

    virtual void update(float wall_dt, float ros_dt) override;

private:
    void InitProperties();

    void InitOgre();

private Q_SLOTS:

    void LeftStateTopicPropertyChanged();

    void RightStateTopicPropertyChanged();

    void LeftVibrationTopicPropertyChanged();

    void RightVibrationTopicPropertyChanged();

    void OffsetTopicPropertyChanged();

private:
    void LeftVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg);

    void RightVibrationMessageReceived(const rviz_vive_plugin_msgs::ControllerVibrationPtr &msg);

    void HMDOffsetMessageReceived(const geometry_msgs::Point32Ptr &msg);

private:
    struct {
        struct {
            rviz::StringProperty *Frame;
            rviz::StringProperty *StateTopic;
            rviz::RosTopicProperty *VibrationTopic;
        } Left, Right;
        struct {
            rviz::StringProperty *Frame;
        } HMD;
        rviz::RosTopicProperty *OffsetTopic;
        rviz::TfFrameProperty *RootFrame;
    } properties_;

    struct {
        rviz::RenderWidget *RenderWidget;
        Ogre::RenderWindow *RenderWindow;

        struct {
            GLuint TextureGLID;
            Ogre::TexturePtr Texture;
            Ogre::Viewport *Viewport;
            Ogre::Camera *EyeCamera;
        } Left, Right;

        struct {
            Ogre::SceneNode *Node;
        } HMD;
    } ogre_;

    tf::TransformBroadcaster tb_;
    ros::NodeHandle node_;
    struct {
        ros::Publisher LeftHand;
        ros::Publisher RightHand;
        ros::Publisher HTC;
    } pubs_;
    struct {
        ros::Subscriber VibrationLeft;
        ros::Subscriber VibrationRight;
        ros::Subscriber HMDOffset;
    } subs_;
    ros::Time leftVibrationTimestamp_, rightVibrationTimestamp_;
    double leftVibrationDuration_, rightVibrationDuration_;
    Ogre::Vector3 offset_;

    Vive vive_;

};

} // namespace rviz_vive_plugin

#endif
