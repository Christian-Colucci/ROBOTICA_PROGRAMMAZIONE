#Ex 4

#punto 1 e 2 : 
1 Tab: roslaunch kinematics_service fk_solver.launch
2 Tab: roslaunch kinematics_Service fk_client_node

#punto 3: 
1 Tab: roslaunch kinematics_action ik_solver.launch
2 Tab: rosrun kinematics_action client_node

# punto 4:
eliminare i limiti di giunto

#punto 5:
non riesco a visualizzare le configurazioni in rviz anche dopo avere creato il nodo che pubblica il topic joint_states

