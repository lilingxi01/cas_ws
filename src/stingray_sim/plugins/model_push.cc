#include "geometry_msgs/Twist.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <cmath>
#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include <thread>

#ifndef PI
#define PI 3.14159265
#endif

namespace gazebo {
class ModelPush : public ModelPlugin {
  /// \brief A node use for ROS transport
private:
  std::unique_ptr<ros::NodeHandle> rosNode;

  /// \brief A ROS subscriber
  ros::Subscriber rosSub;

  /// \brief A ROS callbackqueue that helps process messages
  ros::CallbackQueue rosQueue;

  /// \brief A thread the keeps running the rosQueue
  std::thread rosQueueThread;

  double xSpeed = 0, ySpeed = 0, thetaSpeed = 0;
  // Pointer to the model
  physics::ModelPtr model;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  /// \brief ROS helper function that processes messages
  void QueueThread() {
    static const double timeout = 0.01;
    while (this->rosNode->ok()) {
      this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
  }

public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
    // Store the pointer to the model
    this->model = _parent;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&ModelPush::OnUpdate, this));

    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()) {
      int argc = 0;
      char **argv = NULL;
      ros::init(argc, argv, "gazebo_client",
                ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

    // Create a named topic, and subscribe to it.
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(
            "/" + this->model->GetName() + "/vel_cmd", 1,
            boost::bind(&ModelPush::OnRosMsg, this, _1), ros::VoidPtr(),
            &this->rosQueue);
    this->rosSub = this->rosNode->subscribe(so);

    // Spin up the queue helper thread.
    this->rosQueueThread =
        std::thread(std::bind(&ModelPush::QueueThread, this));
  }

  // Called by the world update start event
  void OnUpdate() {
    // Apply a small linear velocity to the model.
    float world_angle = this->model->WorldPose().Rot().Yaw();
    float x_setpoint = xSpeed * std::cos(world_angle) +
                       ySpeed * std::cos(world_angle + PI / 2.0);
    float y_setpoint = xSpeed * std::sin(world_angle) +
                       ySpeed * std::sin(world_angle + PI / 2.0);

    float z_setpoint = this->model->RelativeLinearVel().Z();
    this->model->SetLinearVel(
        ignition::math::Vector3d(x_setpoint, y_setpoint, z_setpoint));
    this->model->SetAngularVel(ignition::math::Vector3d(0, 0, thetaSpeed));
  }

  /// \brief Handle an incoming message from ROS
  /// \param[in] _msg A float value that is used to set the velocity
  /// of the Velodyne.
  void OnRosMsg(const geometry_msgs::TwistConstPtr &_msg) {
    this->xSpeed = _msg->linear.x;
    this->ySpeed = _msg->linear.y;
    this->thetaSpeed = _msg->angular.z;
  }
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ModelPush)
} // namespace gazebo
