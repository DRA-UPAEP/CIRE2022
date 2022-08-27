#!/usr/bin/env python3
# license removed for brevity

#Edición sobre programa proporcionado
#Edición del equipo: DRA - U
#Intención del programa: Llevar al robot a la meta aleatoria y en caso de encontrar obstáculos, evitarlos
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped


from sensor_msgs.msg   import LaserScan
from geometry_msgs.msg import Twist
import tf

def gen_random_goal():
    rand_pos = np.asarray((-4.5,-3.5))+ 0.5*np.random.randn(2)
    rand_rot = tf.transformations.quaternion_from_euler(((np.random.rand()-0.5 )*2*np.pi) ,0,0 )
    
    pose_stamped=PoseStamped()
    pose_stamped.header.frame_id= 'map'
    pose_stamped.pose.position.x=rand_pos[0]
    pose_stamped.pose.position.y=rand_pos[1]
    ########################################
    pose_stamped.pose.orientation.w= rand_rot[0]
    pose_stamped.pose.orientation.x= rand_rot[1]
    pose_stamped.pose.orientation.y= rand_rot[2]
    pose_stamped.pose.orientation.z= rand_rot[3]
    
    print('r: ', rand_pos)    
    
    return pose_stamped
    print('Calculando una meta aleatoria al rededor del punto -4.5, -3.5 (zona 1)',rand_pos)

def callback_scan(msg):
    global obstacle_detected
    
    
    n=int((msg.angle_max - msg.angle_min)/msg.angle_increment/2)
    rango = int((msg.angle_max - msg.angle_min) / 4)
    
    #primer radián 
    for i in range(msg.angle_min, rango, msg.angle_increment):
        obstacle_detected=msg.ranges[i]<1.0
    	velx = 0.3*((rand_pos[0] - get_coordinates[0])/abs(rand_pos[0] - get_coordinates[0]))
    	#vely = (rand_pos[1] - get_coordinates[1])/abs(rand_pos[1] - get_coordinates[1])
    	msg_cmd_vel = Twist()
    	msg_cmd_vel.linear.x = 0.5 if obstacle_detected else velx
    	msg_cmd_vel.linear.y = -0.5 if obstacle_detected else ((rand_pos[1] - get_coordinates[1])/(rand_pos[0] - get_coordinates[0]))*velx #La velocidad en y se define en proporción a la de x
    	msg_cmd_vel.publish(msg_cmd_vel)
    	break if obstacle_detected
    #Segundo radián (-1 a 0 rad)
    if not obstacle_detected:
    	for i in range(rango, 2*rango, msg.angle_increment):
    	    obstacle_detected=msg.ranges[i]<1.0
        	velx = 0.3*((rand_pos[0] - get_coordinates[0])/abs(rand_pos[0] - get_coordinates[0]))
        	#vely = (rand_pos[1] - get_coordinates[1])/abs(rand_pos[1] - get_coordinates[1])
        	msg_cmd_vel = Twist()
        	msg_cmd_vel.linear.x = -0.5 if obstacle_detected else velx
        	msg_cmd_vel.linear.y = -0.5 if obstacle_detected else ((rand_pos[1] - get_coordinates[1])/(rand_pos[0] - get_coordinates[0]))*velx #La velocidad en y se define en proporción a la de x
        	msg_cmd_vel.publish(msg_cmd_vel)
        	break if obstacle_detected
        	
   #Tercer radián (0 a 1 rad)
   if not obstacle_detected:
    	for i in range(2*rango, 3*rango, msg.angle_increment):
    	    obstacle_detected=msg.ranges[i]<1.0
        	velx = 0.3*((rand_pos[0] - get_coordinates[0])/abs(rand_pos[0] - get_coordinates[0]))
        	#vely = (rand_pos[1] - get_coordinates[1])/abs(rand_pos[1] - get_coordinates[1])
        	msg_cmd_vel = Twist()
        	msg_cmd_vel.linear.x = -0.5 if obstacle_detected else velx
        	msg_cmd_vel.linear.y = 0.5 if obstacle_detected else ((rand_pos[1] - get_coordinates[1])/(rand_pos[0] - get_coordinates[0]))*velx #La velocidad en y se define en proporción a la de x
        	msg_cmd_vel.publish(msg_cmd_vel)
        	break if obstacle_detected
        	
    #Cuarto radián (1 a 2 rad)
    if not obstacle_detected:
    	for i in range(3*rango, msg.angle_max, msg.angle_increment):
    	    obstacle_detected=msg.ranges[i]<1.0
        	velx = 0.3*((rand_pos[0] - get_coordinates[0])/abs(rand_pos[0] - get_coordinates[0]))
        	#vely = (rand_pos[1] - get_coordinates[1])/abs(rand_pos[1] - get_coordinates[1])
        	msg_cmd_vel = Twist()
        	msg_cmd_vel.linear.x = 0.5 if obstacle_detected else velx
        	msg_cmd_vel.linear.y = 0.5 if obstacle_detected else ((rand_pos[1] - get_coordinates[1])/(rand_pos[0] - get_coordinates[0]))*velx #La velocidad en y se define en proporción a la de x
        	msg_cmd_vel.publish(msg_cmd_vel)
        	break if obstacle_detected
    
    rospy.Subscriber("/hsrb/base_scan", LaserScan, callback_scan)
    pub_cmd_vel = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
    loop = rospy.Rate(10)
    
    global obstacle_detected
    obstacle_detected = False
    while not rospy.is_shutdown():


def talker():
    goal_comp_pub = rospy.Publisher('/meta_competencia', PoseStamped, queue_size=10)
    rospy.init_node('meta_publisher_node')
    rate = rospy.Rate(1) # 1hz
    pose_stamped_goal=gen_random_goal()
    print('Meta publicando')

    while not rospy.is_shutdown():
        
        
        goal_comp_pub.publish(pose_stamped_goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
        call_back_scan(msg)
    except rospy.ROSInterruptException:
        pass
