#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Int32

import tkinter as tk

from tkinter import ttk
from ttkthemes import ThemedStyle
import signal

def publish_slider_value(event=None):
    slider_value = slider.get()
    rospy.loginfo(f"Publishing Slider Value: {int(slider_value)}")
    pub.publish(int(slider_value))
    update_slider_label(slider_value)
def update_slider_label(value):
    slider_label.config(text=f"Speed: {int(value)}")

def timer_callback(event):
    publish_slider_value()

def shutdown_handler(signum, frame):
    rospy.loginfo("Shutting down...")
    window.quit()  # Exit the tkinter main loop
    rospy.signal_shutdown("Ctrl+C was pressed")


rospy.init_node('slider_publisher')


window = tk.Tk()
window.title("Slider Publisher")



style = ThemedStyle(window)
style.set_theme("adapta")

slider_label = ttk.Label(window, text="Throttle Value: ")
slider_label.pack()

slider = ttk.Scale(window, from_=100, to=0, orient="vertical", length = 400)
slider.pack()

# Change slider color



publish_button = ttk.Button(window, text="Publish", command=publish_slider_value)
publish_button.pack()

timer = rospy.Timer(rospy.Duration(0.1), timer_callback)

pub = rospy.Publisher('/teleop_speed', Int32, queue_size=10)

signal.signal(signal.SIGINT, shutdown_handler)
if __name__ == "__main__":
    try:
        window.mainloop()
    except rospy.ROSInterruptException:
        pass