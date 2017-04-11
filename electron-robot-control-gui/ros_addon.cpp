#include <motor_control_msg/Motor.h>
#include <nan.h>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"

namespace ros_addon {

    using v8::FunctionTemplate;
    using v8::Handle;
    using v8::Object;
    using v8::String;
    using Nan::GetFunction;
    using Nan::New;
    using Nan::Set;

    int count = 0;
    double velocity_left = 50.;
    double velocity_right = 33.0;
    ros::Publisher chatter_pub;
    bool rosInitialized = false;

    void rosInit() {
        int argc = 0;
        char argv[1];
        char *argvPtr = &argv[0];
        ros::init(argc, &argvPtr, "robot_motor_control");

        ros::NodeHandle n;

        chatter_pub = n.advertise<motor_control_msg::Motor>("motor", 1000);

        rosInitialized = true;
    }

    NAN_METHOD(MethodChangeVelocity) {
        double value = info[0]->NumberValue();
        velocity_left = value;
        std::cout << "velocity_left changed: " << velocity_left << std::endl;
    }

    NAN_METHOD(MethodChangeSteering) {
        double value = info[0]->NumberValue();
        velocity_right = value;
        std::cout << "velocity_right changed: " << velocity_right << std::endl;
    }

    NAN_METHOD(MethodROS) {
        // expect a number as the first argument
        uint32_t value = info[0]->Uint32Value();

        if (!rosInitialized) {
            rosInit();
        }

        // check if connected to roscore
        if (ros::master::check()) {

            // check, if subscribers are connected
            ros::Rate poll_rate(10);
            while (chatter_pub.getNumSubscribers() == 0) {
                ROS_WARN("No subscribers! Try again... %i", count);
                poll_rate.sleep();
                ++count;
                if (count > 20) {
                    count = 0;
                    ROS_ERROR("No subscribers!");
                    return;
                }
            }

            motor_control_msg::Motor msg;
            msg.command = value;
            msg.velocity_left = velocity_left;
            msg.velocity_right = velocity_right;

            ROS_INFO("Send command: %i", msg.command);

            chatter_pub.publish(msg);
            ros::spinOnce();

            count = 0;

        }

        info.GetReturnValue().Set(value);
    }

    NAN_MODULE_INIT(InitAll) {
        Set(target, New<String>("rosCommand").ToLocalChecked(),
            GetFunction(New<FunctionTemplate>(MethodROS)).ToLocalChecked());
        Set(target, New<String>("changeVelocity").ToLocalChecked(),
            GetFunction(New<FunctionTemplate>(MethodChangeVelocity)).ToLocalChecked());
        Set(target, New<String>("changeSteering").ToLocalChecked(),
            GetFunction(New<FunctionTemplate>(MethodChangeSteering)).ToLocalChecked());
    }

    NODE_MODULE(addon, InitAll);

}  // namespace ros_addon
