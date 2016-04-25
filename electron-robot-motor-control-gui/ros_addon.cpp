// TODO: better use NAN for compability of different node versions
#include <node.h>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"

int count = 0;

namespace ros_addon {

    using v8::Exception;
    using v8::FunctionCallbackInfo;
    using v8::Isolate;
    using v8::Local;
    using v8::Object;
    using v8::String;
    using v8::Value;

    ros::Publisher chatter_pub;
    bool rosInitialized = false;

    void rosInit() {
        int argc = 0;
        char argv[1];
        char *argvPtr = &argv[0];
        ros::init(argc, &argvPtr, "robot_motor_control");

        ros::NodeHandle n;

        chatter_pub = n.advertise<std_msgs::UInt8>("motor", 1000);

        rosInitialized = true;
    }

    void MethodROS(const FunctionCallbackInfo <Value> &args) {
        Isolate *isolate = args.GetIsolate();

        // Check the argument type
        if (!args[0]->IsNumber()) {
            isolate->ThrowException(Exception::TypeError(
                    String::NewFromUtf8(isolate, "Wrong arguments")));
            return;
        }

        uint32_t value = args[0]->Uint32Value();

        if(!rosInitialized) {
            rosInit();
        }

        // check if connected to roscore
        if (ros::master::check()) {

            // check, if subscribers are connected
            ros::Rate poll_rate(100);
            while (chatter_pub.getNumSubscribers() == 0) {
                ROS_WARN("No subscribers! Try again... %i", count);
                poll_rate.sleep();
                ++count;
                if(count > 20) {
                    count = 0;
                    isolate->ThrowException(Exception::Error(
                            String::NewFromUtf8(isolate, "No subscribers")));
                    return;
                }
            }

            std_msgs::UInt8 msg;
            msg.data = value;

            ROS_INFO("Send command: %i", msg.data);

            chatter_pub.publish(msg);
            ros::spinOnce();

            count = 0;
            args.GetReturnValue().Set(String::NewFromUtf8(isolate, "ros command published"));

        } else {
            isolate->ThrowException(Exception::Error(
                    String::NewFromUtf8(isolate, "Master not running")));
            return;
        }

    }

    void init(Local <Object> exports) {
        NODE_SET_METHOD(exports, "rosCommand", MethodROS);

    }

    NODE_MODULE(addon, init);

}
