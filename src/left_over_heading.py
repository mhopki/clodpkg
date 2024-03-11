
	"""
	def cam_orient(self, current_pose, ox, oy, oz, kpx, kpy, kpz):
		#XY plane
	    dx = ox - current_pose.pose.pose.position.x
	    dy = oy - current_pose.pose.pose.position.y
	    desired_heading = math.atan2(-dy, -dx) - (math.pi*0.5)#-dy, -dx

	    #print("dx, dy, des_heading: ", dx, dy, desired_heading, math.atan2(-dy, -dx))

	    cur_or = current_pose.pose.pose.orientation.z 
	    if cur_or < 0:
	    	xx = 0
	    	#also alter the z direction
	    #print("cur_or: ", cur_or)

	    #print("hd_comp: ", desired_heading, (cur_or * math.pi), (math.radians(180)))

	    heading_error_gen = desired_heading - (cur_or * math.pi) - (math.radians(180))
	    heading_error = desired_heading - (cur_or * math.pi) - (math.radians(self.r_theta_cam[0]))
	    
	    #print("head_er_gem, head_er: ", heading_error_gen, heading_error)

	    #Correct orientation of robot to fit within -pi to pi
	    if (True):#10000
	    	if heading_error > math.pi:
	    		heading_error -= 2 * math.pi
	    	elif heading_error < -math.pi:
	    		heading_error += 2 * math.pi

	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    
	    heading_error = -heading_error
	    heading_error_gen = -heading_error_gen

	    #print("FIXED: head_er_gem, head_er: ", heading_error_gen, heading_error)
	    
	    cex = ((kpx * heading_error)/math.pi + 3.0)
	    if (1.0 - (heading_error_gen / math.pi) > 0 and 1.0 - (heading_error_gen / math.pi) <= 1.0):
	    	truecx = 3.0 - (heading_error_gen / math.pi)
	    	truecy = 2.0
	    else:
	    	truecx = 3.0 - (1.0 + (heading_error_gen / math.pi))
	    	truecy = 3.0
	    cey = ((kpy * 0)/math.pi + 3.0)


	    dz = oz - current_pose.pose.pose.position.z
	    truecy = 2.0 + 0.5*(dz/30)
	    if truecy < 2.0: truecy = 2.0"""

		"""
	    #XZ plane
	    dx = ox - current_pose.pose.pose.position.x
	    dz = oz - current_pose.pose.pose.position.z
	    desired_heading = math.atan2(-dx, -dz) - (math.pi*0.5)#-dy, -dx

	    print("dx, dz, des_heading: ", dx, dz, desired_heading, math.atan2(-dz, -dx))

	    cur_or = ((self.cam_pose[1] - 2.0)) #current_pose.pose.pose.orientation.z 
	    if cur_or < 0:
	    	xx = 0
	    	#also alter the z direction
	    print("cur_or: ", cur_or)

	    print("hd_comp: ", desired_heading, (cur_or * math.pi), (math.radians(180)))

	    heading_error_gen = desired_heading - (cur_or * math.pi) - (math.radians(180))
	    heading_error = desired_heading - (cur_or * math.pi) - (math.radians(self.r_theta_cam[0]))
	    
	    print("head_er_gem, head_er: ", heading_error_gen, heading_error)

	    #Correct orientation of robot to fit within -pi to pi
	    if (True):#10000
	    	if heading_error > math.pi:
	    		heading_error -= 2 * math.pi
	    	elif heading_error < -math.pi:
	    		heading_error += 2 * math.pi

	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    if (True):#10000
	    	if heading_error_gen > math.pi:
	    		heading_error_gen -= 2 * math.pi
	    	if heading_error_gen < -math.pi:
	    		heading_error_gen += 2 * math.pi
	    
	    heading_error = -heading_error
	    heading_error_gen = -heading_error_gen

	    print("FIXED: head_er_gem, head_er: ", heading_error_gen, heading_error)
	    
	    cex = ((kpx * heading_error)/math.pi + 3.0)
	    if (1.0 - (heading_error_gen / math.pi) > 0 and 1.0 - (heading_error_gen / math.pi) <= 1.0):
	    	truecy = 2.0 + (heading_error_gen / math.pi)
	    	if truecy < 2.0: truecy = 2.0
	    	#truecy = 2.0
	    else:
	    	truecy = 2.0 + (heading_error_gen / math.pi)#+ (1.0 + (heading_error_gen / math.pi))
	    	if truecy < 2.0: truecy = 2.0
	    	#truecy = 3.0
	    cey = ((kpy * 0)/math.pi + 3.0)
	    """

	    #return truecx, truecy