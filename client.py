import rospy
import json
from std_msgs.msg import String
from string_service_demo.srv import StringService

def callback(msg):
    try:
        data = json.loads(msg.data)
        rospy.loginfo("Received robot status: %s", data)
    except Exception as e:
        rospy.logerr("Failed to parse: %s", str(e))

def main():
    rospy.init_node('string_service_client')

    rospy.Subscriber("/robot_status", String, callback)
    rospy.wait_for_service("/send_string")
    try:
        proxy = rospy.ServiceProxy("/send_string", StringService)
        resp = proxy("Hello from client!")
        rospy.loginfo("Service Response: %s", resp.result)
    except Exception as e:
        rospy.logerr("Service call failed: %s", str(e))

    rospy.spin()

if __name__ == '__main__':
    main()
