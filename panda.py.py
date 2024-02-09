#!/usr/bin/env python3

import rospy
import copy
import moveit_commander
import geometry_msgs.msg
import sys
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point


class ArmController:
    def __init__(self):
        #----Inizializzazioni Base ----
        # Inizializzazione del nodo ROS
        rospy.init_node('arm_controller')
        # Inizializzazione di MoveIt
        moveit_commander.roscpp_initialize(sys.argv)        
        # Creazione di un oggetto RobotCommander per ottenere informazioni sul robot
        self.robot = moveit_commander.RobotCommander()
        # Creazione di un oggetto PlanningSceneInterface per interagire con lo scenario di pianificazione
        self.scene = moveit_commander.PlanningSceneInterface()

        # Creazione di un oggetto MoveGroupCommander per controllare il gruppo dei giunti relativi al braccio
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        # Creazione di un oggetto MoveGroupCommander per controllare il gruppo dei giunti relativi al endeffector
        self.move_group_eef = moveit_commander.MoveGroupCommander("panda_hand")
        
        #----Inizializzazione dei Publisher ----
        # Publisher per comunicare lo status del lavoro al control
        self.status_job_publisher = rospy.Publisher('status_job', Bool, queue_size=10)

        #----Inizializzazione dei Subscriber ----
        # Subscriber per monitorare lo status
        rospy.Subscriber('status_job', Bool, self.status_job_callback)
        # Subscriber per monitorare il target del braccio
        rospy.Subscriber('target_arm', Pose, self.target_arm_callback)
        # Subscriber per monitorare il target del endeffector
        rospy.Subscriber('endeffector_status', Point, self.target_eef_callback)
        
        #----Inizializzazione delle variabili ----

        # Variabili per salvare lo stato di PANDA
        self.status_job = None
        # Variabili per salvare lo stato corrente del target del braccio
        self.target_arm_pose = None        
        # Variabili per salvare lo stato corrente del target del endeffector
        self.target_eff_pose = None

    def status_job_callback(self, msg):
        self.status_job = msg.data

    def target_arm_callback(self, msg):
        # Callback per aggiornare lo stato del target del braccio
        self.target_arm_pose = msg
        # Quando arriva un nuovo messaggio sul topic target_arm, impostiamo lo status_job su False
        self.status_job_publisher.publish(False)

    def target_eef_callback(self, msg):
        # Callback per aggiornare lo stato del target del endeffector
        self.target_eff_pose = msg
        # Quando arriva un nuovo messaggio sul topic endeffector_status, impostiamo lo status_job su False
        self.status_job_publisher.publish(False)

    def move_controller(self):
        while not rospy.is_shutdown():
            # Controlla se il target del braccio è disponibile
            if self.target_arm_pose is not None:
                self.move_arm()
            # Controlla se il target dell'endeffector è disponibile
            elif self.target_eff_pose is not None:
                self.move_eef()

    def move_arm(self):
        # Verifica se la posa del target del braccio è valida
        while self.target_arm_pose is None:
            rospy.loginfo("In attesa del target del braccio...")
            rospy.sleep(1)  # Attendi 1 secondo prima di controllare nuovamente
            rospy.loginfo("Target del braccio ricevuto. Inizio movimento...")

        # Definizione della posa obiettivo per il movimento del braccio
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = self.target_arm_pose.position.x
        pose_goal.position.y = self.target_arm_pose.position.y
        pose_goal.position.z = self.target_arm_pose.position.z
        pose_goal.orientation.x = self.target_arm_pose.orientation.x
        rospy.loginfo(pose_goal)
        # Imposta la posa obiettivo per il gruppo di movimento
        self.move_group.set_pose_target(pose_goal)

        # Esegue il piano per il movimento del braccio
        plan = self.move_group.go(wait=True)
        
        # Resetta il target del braccio a None dopo il movimento
        self.target_arm_pose = None

        # Comunica che il movimento è finito pubblicando "True" sul topic status_job
        self.status_job_publisher.publish(True)

    def move_eef(self):
        # Verifica se il target dell'end effector è valido
        while self.target_eff_pose is None:
            rospy.loginfo("In attesa del target dell'end effector...")
            rospy.sleep(1)  # Attendi 1 secondo prima di controllare nuovamente
            rospy.loginfo("Target dell'end effector ricevuto. Inizio movimento...")

        # Definizione del movimento degli end effector
        joint_goal = self.move_group_eef.get_current_joint_values()
        joint_goal[0] = self.target_eff_pose.x
        joint_goal[1] = self.target_eff_pose.y
        rospy.loginfo(self.target_eff_pose.y)

        # Esegue il movimento degli end effector
        self.move_group_eef.go(joint_goal, wait=True)

        # Comunica che il movimento è finito pubblicando "True" sul topic status_job
        self.status_job_publisher.publish(True)


if __name__ == '__main__':
    try:
        # Inizializza il controller del braccio
        arm_controller = ArmController()

        # Avvia il movimento del braccio
        arm_controller.move_controller()
    except rospy.ROSInterruptException:
        pass