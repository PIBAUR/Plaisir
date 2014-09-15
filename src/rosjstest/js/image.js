// variables
var tabNomsImgsCarte, tabCoords; 			// nom des fichiers images et coordonnées affichage
var tabImgCarte, tabBmpCarte;   // images et Bmp
var vIndiceImgCarte; // pour chargement progressif
//-------------------------------
// -- init des images carte
//-------------------------------
function initCarte() {
	 viewer = new ROS2D.Viewer({
      divID : 'map',
      width : 600,
      height : 600
    });
	
	vIndiceImgCarte	= 0;
	tabNomsImgsCarte=new Array('carte','poi-big-red','poi-big-yellow');
	
	tabCoords		= new Array(3);
	tabImgCarte		= new Array(3);
	tabBmpCarte		= new Array(3);
	tabCoords[0]= new Array(300,250);
	tabCoords[1]= new Array(400,250);
	tabCoords[2]= new Array(200,250);

	chargeImgCarte();
}
function chargeImgCarte() {
	tabImgCarte[vIndiceImgCarte] = new Image();
	tabImgCarte[vIndiceImgCarte].onload = handleImageLoadCarte;
	tabImgCarte[vIndiceImgCarte].src = "images/"+tabNomsImgsCarte[vIndiceImgCarte]+".png";
}
function handleImageLoadCarte() {
	 viewer.scene.y = 0; // pour compenser ce qui est fait dans viewer
	tabBmpCarte[vIndiceImgCarte] = new createjs.Bitmap(tabImgCarte[vIndiceImgCarte]);
	tabBmpCarte[vIndiceImgCarte].regX = tabImgCarte[vIndiceImgCarte].width >> 1;
	tabBmpCarte[vIndiceImgCarte].regY = tabImgCarte[vIndiceImgCarte].height >> 1;
	tabBmpCarte[vIndiceImgCarte].x = tabCoords[vIndiceImgCarte][0];
	tabBmpCarte[vIndiceImgCarte].y = tabCoords[vIndiceImgCarte][1];
	tabBmpCarte[vIndiceImgCarte].scaleX = tabBmpCarte[vIndiceImgCarte].scaleY = 1.0;
	tabBmpCarte[vIndiceImgCarte].id = vIndiceImgCarte;
	viewer.scene.addChild(tabBmpCarte[vIndiceImgCarte]);
 	
	//-- interactivité de carte ---------------
	tabBmpCarte[vIndiceImgCarte].onClick = function() {
		switch(this.id){
			case 1:
			  // Publishing a Topic
			  // ------------------
			
			  var cmdVel = new ROSLIB.Topic({
				ros : ros,
				name : '/pathTurtle1',
				messageType : 'std_msgs/Float32MultiArray'
			  });
			
			  var twist = new ROSLIB.Message({
				
				data : [8.0,1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0],
			  });
			  cmdVel.publish(twist);
			  
			  console.log("envoi");			
			break;
		}
	}
	
	if (vIndiceImgCarte<2){
		vIndiceImgCarte = vIndiceImgCarte+1;
		chargeImgCarte();
	}else{
		// on établit al comm
		initRos();
		
	}

}
