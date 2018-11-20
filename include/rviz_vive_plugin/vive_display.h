//
// Created by Denys Kotelovych on 04.09.18.
//

#ifndef RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H
#define RVIZ_VIVE_PLUGIN_VIVE_DISPLAY_H

#ifndef Q_MOC_RUN

#include <rviz/display.h>

#endif

#include <OGRE/OgreTexture.h>
#include <GL/glew.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include "rviz_vive_plugin/vive.h"
#include "rviz_vive_plugin/vive_conversions.h"

namespace Ogre {
class SceneNode;

class Viewport;

class Camera;

class RenderWindow;
}

namespace rviz {

class RenderWidget;

}

namespace rviz_vive_plugin {

class ViveDisplay : public rviz::Display {
Q_OBJECT

public:
    ViveDisplay();

    virtual ~ViveDisplay();

    virtual void onInitialize() override;

    virtual void update(float wall_dt, float ros_dt) override;

    virtual void reset() override;

private:
    bool InitOgre();

private:
    void LeftVibrationMessageReceived(const std_msgs::Float32Ptr &msg);

    void RightVibrationMessageReceived(const std_msgs::Float32Ptr &msg);

private:
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
    } pubs_;
    struct {
        ros::Subscriber VibrationLeft;
        ros::Subscriber VibrationRight;
    } subs_;
    ros::Time leftVibrationTimestamp_, rightVibrationTimestamp_;
    double leftVibrationDuration_, rightVibrationDuration_;

    Vive vive_;
};

} // namespace rviz_vive_plugin

#endif
