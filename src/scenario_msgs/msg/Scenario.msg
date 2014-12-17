#definition du message pour la communications des scenarios

#Header pour enregistrer la date du scénario
Header header

#nom/path du scénario
string name

#Liste des coures de beziers
scenario_msgs/BezierPath  bezier_paths

#destination du scenario 
geometry_msgs/Pose2D cible

###### deprecated
####path des videos pour chaque scenario
###string[] videos_paths
####point de référence sur les courbes
###scenario_msgs/Pose2DArray references

### new solution
scenario_msgs/VideoAndRef video_and_ref
