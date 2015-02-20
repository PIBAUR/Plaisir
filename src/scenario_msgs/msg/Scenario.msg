#definition du message pour la communications des scenarios

#nom/path du scenario
string name

#type de scenario (choregraphic, travel, follow)
string type

#Liste des courbes de beziers
scenario_msgs/BezierPath bezier_paths

#destination du scenario 
geometry_msgs/Pose2D target

#list of medias to display
scenario_msgs/Media[] medias 
