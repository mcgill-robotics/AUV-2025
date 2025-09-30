#!/usr/bin/env python3
'''
Script to interactively test and adjust the thrust allocation matrix.
Goal: 
1. Minimize pitch occuring from surge command by adjusting the 'e' parameter in the allocation matrix in propulsion launch file
2. Update the e parameter to make it more accurately represent lever arm of thrusters causing pitch

Script created by Stuart Klenner, likely to be modified in near future as debugging definitely required
'''
import rospy
from auv_msgs.msg import ThrusterMicroseconds
from geometry_msgs.msg import Wrench 
import threading
import keyboard

# --- GLOBAL SAFETY FLAG ---
emergency_triggered = False

def emergency_reset_loop():
    """Continuously monitors for the 'esc' keypress to reset thrusters."""
    global emergency_triggered
    while not rospy.is_shutdown():
        if keyboard.is_pressed('esc'):
            rospy.logwarn("[SAFETY] Reset key pressed. Resetting thrusters.")
            # Publish neutral command directly to the thrusters
            pwm_pub.publish(reset_cmd)
            emergency_triggered = True
            rospy.sleep(0.5)

# --- SAFE SLEEP FUNCTION ---
def safe_sleep(duration):
    """Sleeps while continuously checking for the emergency flag."""
    global emergency_triggered
    interval = 0.1  # check frequency
    elapsed = 0.0
    while elapsed < duration and not emergency_triggered:
        rospy.sleep(interval)
        elapsed += interval



# --- MAIN TEST FUNCTION ---
def run_allocation_test(effort_pub, pwm_pub, reset_cmd):
    """Main loop to interactively test the thrust allocation matrix."""
    global emergency_triggered

    # Outer loop allows repeating the test
    while not rospy.is_shutdown():
        emergency_triggered = False
        pwm_pub.publish(reset_cmd) # Safety reset before asking for input

        user_input = input("\n[INPUT] Press ENTER to start new allocation test, or type 'q' to quit: ")
        if user_input.lower() == 'q':
            print("[INFO] Exiting test loop.")
            break

        # Get the desired force from the user
        try:
            # We must command a forward force (Surge) to test the Pitch coupling
            surge_force = float(input("\n[INPUT] Specify a FORWARD (Surge) force in Newtons (e.g., 5.0): "))
            
        except ValueError:
             print("[ERROR] Invalid input. Using default force of 5.0 N.")
             surge_force = 5.0
        
        # --- PHASE 1: COMMAND PURE SURGE ---
        print(f"[INFO] Commanding pure Surge force: {surge_force:.2f} N for 5 seconds...")

        # 1. Create the Wrench message
        test_wrench = Wrench()
        
        # 2. Set the Surge (X) Force component
        test_wrench.force.x = surge_force 
        
        # All other 5 components (Sway, Heave, Roll, Pitch, Yaw) are implicitly zero (0.0)

        # 3. Publish the Wrench command
        # We publish the command once every 0.1s to keep the thrust_mapper node active
        start_time = rospy.Time.now()
        duration = 20
        rate = rospy.Rate(10) # 10 Hz

        while ((rospy.Time.now() - start_time).to_sec() < duration 
               and not emergency_triggered
               and not rospy.is_shutdown()):
            effort_pub.publish(test_wrench)
            rate.sleep()

        if emergency_triggered:
            continue

        # --- PHASE 2: RESET AND OBSERVE ---
        pwm_pub.publish(reset_cmd)
        print("[INFO] Test complete. Thrusters reset to neutral.")
        print("[OBSERVE] Check the AUV's pitch tendency. If it pitches, the 'e' parameter needs adjustment.")


if __name__ == "__main__":
    # --- ROS SETUP ---
    rospy.init_node("allocation_test_node")
    # Publisher for safety reset (sends microsecond signals directly to hardware)
    pwm_pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)

    # Publisher for the test command (sends Wrench to Thrust Mapper)
    effort_pub = rospy.Publisher("/controls/effort", Wrench, queue_size=1) 

    # --- CONSTANTS ---
    reset_cmd = ThrusterMicroseconds(microseconds=[1500]*8) 

    rospy.sleep(1.0)  # warm-up for connections

    # --- START SAFETY THREAD ---
    safety_thread = threading.Thread(target=emergency_reset_loop, daemon=True)
    safety_thread.start()

    run_allocation_test(effort_pub,pwm_pub, reset_cmd)