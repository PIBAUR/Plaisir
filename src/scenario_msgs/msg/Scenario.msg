#definition du message pour la communications des scenarios

#Header pour enregistrer la date du scenario
Header header

#nom/path du scenario
string name

#Liste des courbes de beziers
scenario_msgs/BezierPath  bezier_paths

#destination du scenario 
geometry_msgs/Pose2D target

###### deprecated
####path des videos pour chaque scenario
###string[] videos_paths
####point de reference sur les courbes
###scenario_msgs/Pose2DArray references

### new solution
scenario_msgs/VideoPlayer video_player
