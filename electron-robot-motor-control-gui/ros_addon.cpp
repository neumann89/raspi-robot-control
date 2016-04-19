// TODO: better use NAN for compability of different node versions
#include <node.h>
#include <sstream>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt8.h"

int count = 0;

namespace ros_addon {

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

    void MethodROSForward(const FunctionCallbackInfo <Value> &args) {
        Isolate *isolate = args.GetIsolate();

        if(!rosInitialized) {
            rosInit();
        }

        // check if connected to roscore
        if (ros::master::check()) {

            // check, if subscribers are connected
            ros::Rate poll_rate(100);
            while (chatter_pub.getNumSubscribers() == 0) {
                std::cerr << "No subscribers! Try again... " << count << std::endl;
                poll_rate.sleep();
                ++count;
                if(count > 20) {
                    std::cerr << "No subscribers!" << std::endl;
                    args.GetReturnValue().Set(String::NewFromUtf8(isolate, "no subscribers"));
                    return;
                }
            }

            std_msgs::UInt8 msg;
            msg.data = 1;

            ROS_INFO("Send command: %i", msg.data);

            chatter_pub.publish(msg);
            ros::spinOnce();

            ++count;
            args.GetReturnValue().Set(String::NewFromUtf8(isolate, "forward published"));

        } else {
            std::cerr << "Master not running!" << std::endl;
            args.GetReturnValue().Set(String::NewFromUtf8(isolate, "master not running"));
        }

    }

    void init(Local <Object> exports) {
        NODE_SET_METHOD(exports, "rosForward", MethodROSForward);

        // will stop app to start, if init fails
        // initROS();
    }

    NODE_MODULE(addon, init);

}
