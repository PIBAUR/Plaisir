  var viewer;
  var ros;
  
  function initRos() {
    // Connect to ROS.
    // Create the main viewer.
	/* deposté ds image.js
    viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 600,
      height : 600
    });
	*/

    ros = new ROSLIB.Ros({
      url : 'ws://192.168.1.98:9090'
    });
	
	  ros.on('connection', function() {
		console.log('Connected to websocket server.');
	  });
	
	  ros.on('error', function(error) {
		console.log('Error connecting to websocket server: ', error);
	  });
	
	  ros.on('close', function() {
		console.log('Connection to websocket server closed.');
	  });

    // Setup the map client.
    var gridClient = new ROS2D.OccupancyGridClient({
      ros : ros,
      rootObject : viewer.scene
    });
    // Scale the canvas to fit to the map
    gridClient.on('change', function(){
      //viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
	  //console.log(viewer.scene.y);
	 
	  gridClient.currentGrid.scaleX=4;
	  gridClient.currentGrid.scaleY=4;
	  gridClient.currentGrid.x=-1900;
	  gridClient.currentGrid.y=-1700;
	  for(var i=0;i<3;i++){
		  viewer.scene.removeChild(tabBmpCarte[i]);
		  viewer.scene.addChild(tabBmpCarte[i]);
	  }
    });
	/**/
	// affiche Image de la carte
	//-- souscrire au tf
	  var tfClient = new ROSLIB.TFClient({
		ros : ros,
		fixedFrame : 'world',
		angularThres : 0.01,
		transThres : 0.01
	  });
	
		// dédié au tf ---------------------
	  // receive from tf --------------------------------
	  // turtle1 : c'est le nom du tf
	  tfClient.subscribe('turtle1', function(tf) {
		console.log(tf);
		console.log(tf.translation);
	  });
	
  }
