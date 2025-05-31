import os
import rospy
import rosbag
import rospkg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64



from datetime import datetime

def make_bag_filename(topic_name):
    # e.g. topic_name="/odom" → basename="odom"
    #      topic_name="/my_ns/scan" → basename="my_ns_scan"
    basename = topic_name.lstrip("/").replace("/", "_")
    now_str = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
    filename = f"{basename}_{now_str}.bag"
    return filename

class TopicBagRecorder:
    def __init__(self, topics_dict):
        """
        topics_dict: { topic_name (str) : MessageClass }
        """
        self._bags = {}         # will hold rosbag.Bag instances, keyed by topic
        self._subs = []         # keep subscriptions alive
        self._topics = topics_dict

        # Create a package
        rospack = rospkg.RosPack()
        try:
            pkg_path = rospack.get_path("planner")
        except rospkg.ResourceNotFound:
            rospy.logerr(f"[bag_recorder] Package \'planner\' not found in ROS_PACKAGE_PATH.")
            rospy.signal_shutdown("Package not found")
            return
        
        self._bags_dir = os.path.join(pkg_path, "bags")
        os.makedirs(self._bags_dir, exist_ok=True)

        # for each topic, open a bag file and create a subscriber
        for topic_name, msg_type in self._topics.items():
            bag_path = os.path.join(self._bags_dir, make_bag_filename(topic_name))
            rospy.loginfo(f"[bag_recorder] Opening bag for topic '{topic_name}': {bag_path}")
            bag = rosbag.Bag(bag_path, mode="w")
            self._bags[topic_name] = bag

            # Create a subscriber for this topic. Use a λ to pass topic_name
            sub = rospy.Subscriber(
                topic_name,
                msg_type,
                callback=self._make_callback(topic_name),
                queue_size=5,
            )
            self._subs.append(sub)

        # Register shutdown hook to close all bags
        rospy.on_shutdown(self._close_all_bags)

    def _make_callback(self, topic_name):
        """
        Return a callback function that writes any incoming message
        on 'topic_name' into that topic’s bag file.
        """
        def _cb(msg):
            bag = self._bags.get(topic_name, None)
            if bag is None:
                rospy.logwarn(f"[bag_recorder] No bag found for topic '{topic_name}'")
                return
            # Write the raw message into the bag under the same topic name.
            # Use the timestamp from the message header if present; otherwise use rospy.Time.now().
            try:
                stamp = msg.header.stamp
            except AttributeError:
                stamp = rospy.Time.now()
            bag.write(topic_name, msg, t=stamp)
        return _cb

    def _close_all_bags(self):
        """Called on rospy shutdown; closes every open rosbag."""
        rospy.loginfo("[bag_recorder] Shutdown triggered. Closing all bag files...")
        for topic_name, bag in self._bags.items():
            rospy.loginfo(f"[bag_recorder] Closing bag for '{topic_name}'")
            bag.close()
        rospy.loginfo("[bag_recorder] All bags closed.")


if __name__ == "__main__":
    rospy.init_node("recorder")

    # Instantiate the recorder: it will open one .bag per topic and start subscribing.
    TOPICS = {
            "/odometry/filtered": Odometry,
            "/sensors/dvl/twist": TwistWithCovarianceStamped,
            "/sensors/imu/data": Imu,
            "/sensors/depth/z": Float64
        }
    recorder = TopicBagRecorder(TOPICS)

    rospy.loginfo("[bag_recorder] Recording started. Press Ctrl+C to stop and close bag files.")
    
    try:
        rospy.loginfo("Recorder Entering spin(). Press Ctrl+C to exit.")
        rospy.spin()
    except rospy.ROSInterruptException:
        # Thrown if Ctrl+C is pressed during spin() or if rospy.signal_shutdown() is called
        rospy.loginfo("Recorder  ROSInterruptException caught (node interrupted).")
    finally:
        # In case spin() returns or an exception is raised, ensure shutdown is signaled
        if not rospy.is_shutdown():
            rospy.loginfo("Recorder  Signaling shutdown from finally block.")
            rospy.signal_shutdown("Exiting node")

        # The cleanup callback (registered via on_shutdown) will be invoked automatically.
        rospy.loginfo("Recorder  Exiting main().")

    