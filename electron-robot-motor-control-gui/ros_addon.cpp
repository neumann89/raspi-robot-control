#include <motor_control/Motor.h>
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
    double steering = 0.5;
    double velocity = 0.33;
    ros::Publisher chatter_pub;
    bool rosInitialized = false;

    void rosInit() {
        int argc = 0;
        char argv[1];
        char *argvPtr = &argv[0];
        ros::init(argc, &argvPtr, "robot_motor_control");

        ros::NodeHandle n;

        chatter_pub = n.advertise<motor_control::Motor>("motor", 1000);

        rosInitialized = true;
    }

    NAN_METHOD(MethodChangeVelocity) {
        double value = info[0]->NumberValue();
        velocity = value;
        std::cout << "velocity changed: " << velocity << std::endl;
    }

    NAN_METHOD(MethodChangeSteering) {
        double value = info[0]->NumberValue();
        steering = value;
        std::cout << "steering changed: " << steering << std::endl;
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

            motor_control::Motor msg;
            msg.command = value;
            msg.velocity = velocity;
            msg.steering = steering;

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
