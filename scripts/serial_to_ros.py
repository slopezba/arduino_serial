#!/usr/bin/env python3
import rospy
import serial
import json
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse

PORT = "/dev/ttyACM0"
BAUD = 115200

def main():
    rospy.init_node("arduino_serial_node")

    pub = rospy.Publisher("/arduino/adc", Int32MultiArray, queue_size=10)

    ser = serial.Serial(PORT, BAUD, timeout=1)

    # -------- SERVICES --------

    def handle_output(req):
        pin = rospy.get_param("~output_pin", 8)
        value = 1 if req.data else 0

        cmd = {
            "cmd": "set_output",
            "pin": pin,
            "value": value
        }

        ser.write((json.dumps(cmd) + "\n").encode())
        return SetBoolResponse(True, f"Pin {pin} set to {value}")

    def handle_period(req):
        period = rospy.get_param("~period_ms", 50)

        cmd = {
            "cmd": "set_period",
            "period_ms": period
        }

        ser.write((json.dumps(cmd) + "\n").encode())
        return TriggerResponse(True, f"Period set to {period} ms")

    rospy.Service("/set_output", SetBool, handle_output)
    rospy.Service("/set_period", Trigger, handle_period)

    rospy.loginfo("Arduino serial node running")

    rate = rospy.Rate(200)

    while not rospy.is_shutdown():
        line = ser.readline().decode().strip()
        if not line:
            rate.sleep()
            continue

        try:
            data = json.loads(line)
            msg = Int32MultiArray()
            msg.data = [data["A0"], data["A1"], data["A2"], data["A3"]]
            pub.publish(msg)
        except Exception as e:
            rospy.logwarn(f"Bad data: {line}")

        rate.sleep()

if __name__ == "__main__":
    main()

