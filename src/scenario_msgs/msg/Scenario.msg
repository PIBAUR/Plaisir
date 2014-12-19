#definition du message pour la communications des scenarios

#Header pour enregistrer la date du scenario
Header header

#nom/path du scenario
string name

#Liste des courbes de beziers
scenario_msgs/BezierPath bezier_paths

#destination du scenario 
geometry_msgs/Pose2D target

#video
scenario_msgs/VideoPlayer video_player
