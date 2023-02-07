def wrench_to_thrust(w):
	
	a = np.array(
		[[w.force.x],
		[w.force.y],
		[w.force.z],
		[w.torque.x],
		[w.torque.y],
		[w.torque.z]]] )
	b = np.matmul(T_inv, a)
	tc = ThrusterCommand([b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]])
	thust_pub.publish(tc)
	return

if __name__ == '__main__':
	rospy.init_node('thrust_mapper')
	sub = rospy.Subscriber('/effort', Wrench, wrench_to_thrust)
	rospy.spin()


