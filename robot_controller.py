#!/usr/bin/env python3

import rospy
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected to MQTT broker with result code " + str(rc))

def publish_command(command):
    client.publish("robot/control", command)
    rospy.loginfo(f"Published {command} to robot/control")

if __name__ == '__main__':
    rospy.init_node('robot_controller')

    mqtt_broker_ip = "your_MQTT_SERVER_IP"
    client = mqtt.Client()
    client.on_connect = on_connect

    client.connect(mqtt_broker_ip, 1883, 60)
    client.loop_start()

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        command = input("Enter command (FORWARD, BACKWARD, LEFT, RIGHT, STOP): ").strip().upper()
        publish_command(command)
        rate.sleep()
