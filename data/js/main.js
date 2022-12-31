var host = '';
if (window.location.hostname == 'localhost') {
  host = 'http://192.168.50.174/';
}

var scene, camera, renderer, loader, ground;

var legs = new Array(6);  // array of legs

/*
var config;
var legState = {
  "restPos":[112.53,205.09,0.00],
  "_fwdPose":[154.56,273.16,0.00]
};
*/

var zAxis = new THREE.Vector3( 0, 0, 1 );

var bodyOffset = new THREE.Vector3(0, 0, 80);
var bodyRotation = new THREE.Euler(0,0,0, 'XYZ');  // pitch, roll, yaw


function initScene() {
  // Loader
		loader = new THREE.TextureLoader();

    // setup the renderer - use WebGL if possible
    if( Detector.webgl ){
        renderer = new THREE.WebGLRenderer({
            antialias		: true,	// to get smoother output
            alpha: true,
            preserveDrawingBuffer	: true	// to allow screenshot
        });
        renderer.setClearColor( 0xffffff );  // white background
        renderer.shadowMap.enabled = true;
        renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    }else{
        renderer	= new THREE.CanvasRenderer();
    }
    renderer.setSize( 600, 600 );
    document.getElementById('viewer').appendChild(renderer.domElement);

    // create a scene
    scene = new THREE.Scene();

    // camera
    camera	= new THREE.PerspectiveCamera(35, 600/600, 1, 10000 );
    camera.up.set(0,0,1);  // set Z+ axis to up
    camera.position.set(-1200, -800, 400);
    camera.lookAt(scene.position);
    scene.add(camera);

    // setup lighting
    var light	= new THREE.AmbientLight( 0xa0a0a0 );
    scene.add( light );

    light	= new THREE.SpotLight( 0xffffff, 0.4 );
    light.position.set(400,-400,600);
    light.lookAt(scene.position);
    light.castShadow = true;
    light.shadow.bias = 0.0001;
	  light.shadow.mapSize.width = 1024;
    light.shadow.mapSize.height = 1024;
    light.shadow.camera.near = 50;
    light.shadow.camera.far = 1300;
    light.shadow.camera.fov = 50;
    light.shadow.radius = 8;
    scene.add( light );

    light	= new THREE.SpotLight( 0xffffff, 0.2 );
    light.position.set(-500,-500,700);
    light.lookAt(scene.position);
    light.castShadow = true;
    light.shadow.bias = 0.0001;
	  light.shadow.mapSize.width = 1024;
    light.shadow.mapSize.height = 1024;
    light.shadow.camera.near = 50;
    light.shadow.camera.far = 1300;
    light.shadow.camera.fov = 50;
    light.shadow.radius = 8;
    scene.add( light );

    controls = new THREE.OrbitControls( camera, renderer.domElement );
	  controls.enableDamping = true;
	  controls.dampingFactor = 0.25;
	  controls.enableZoom = true;

    var axisHelper = new THREE.AxisHelper( 50 );
    scene.add( axisHelper );

    // Ground
    const groundGeo = new THREE.PlaneGeometry( 800, 800, 1,1 );
		//groundGeo.rotateX( - Math.PI / 2 );
    ground = new THREE.Mesh(
			groundGeo,
			new THREE.MeshStandardMaterial({
        color: 0xc0c0c0,
        side: THREE.DoubleSide
      })
		);
    ground.position.z = 0;
		ground.receiveShadow = true;
		//scene.add( ground );


    // register event handlers
    document.addEventListener( 'mousemove', onDocumentMouseMove, false );
    document.addEventListener( 'mousedown', onDocumentMouseDown, false );
    document.addEventListener( 'mouseup', onDocumentMouseUp, false );
    window.addEventListener( 'resize', onWindowResize, false );

    // bootstrap the animation loop
    animate();
}


function animate() {
  requestAnimationFrame( animate )
  //cube.rotation.x += 0.04;
  //cube.rotation.y += 0.04;
  renderer.render( scene, camera )
}

function onDocumentMouseMove( event ) {

    mouseX = ( event.clientX - 300 );
    mouseY = ( event.clientY - 300 );

    //if (pause && mouseDown) render();
}

function onDocumentMouseDown( event ) {
    mouseDown = true;
}

function onDocumentMouseUp( event ) {
    mouseDown = false;
}

function onWindowResize() {
    windowHalfX = window.innerWidth / 2;
    windowHalfY = window.innerHeight / 2;

    //camera.aspect = window.innerWidth / window.innerHeight;
    camera.aspect = 1;
    camera.updateProjectionMatrix();

    renderer.setSize( 600,600 );

    //if (pause) render();
}


function updateLegState(legIndex, legState) {


    var leg = legs[legIndex];

    leg.fwdPoseTarget.position.set(legState._fwdPose[0], legState._fwdPose[1], legState._fwdPose[2]);
    leg.restPosTarget.position.set(legState.restPos[0], legState.restPos[1], legState.restPos[2]);

    // convert joint angles to radians
    var ang = new Array(3);
    for (var i=0; i<3; i++) {
      ang[i] = legState.jointAngles[i] * Math.PI / 180;
    }

    leg.setTargetAngles( ang );
    leg.update();

}

function updateLegs() {
  $.getJSON( host + "legState" , function( data ) {
    legState = data;
    console.log(JSON.stringify(data));

    for (var i=0; i<6; i++) {
      updateLegState(i, legState.legs[i] );
    }

    setTimeout(updateLegs, 200);
  });
}


function initLeg(config) {
  var leg = new Leg(config.boneLengths);

  var ang = degToRad(config.coxaAngle);

  // move to position
  leg.object.position.set(config.coxaOffsets[0], config.coxaOffsets[1], config.coxaOffsets[2]);

  leg.object.rotation.z = ang;

  scene.add(leg.object);


  // _fwdPose target
  leg.fwdPoseTarget = new THREE.AxisHelper( 50 );
  leg.fwdPoseTarget.position.set(0,0,0);
  scene.add( leg.fwdPoseTarget );

  // restPos target
  leg.restPosTarget = new THREE.AxisHelper( 50 );
  leg.restPosTarget.position.set(0,0,0);
  scene.add( leg.restPosTarget );

  return leg;
}


var legInitCount = 0;

function initLegs() {

  $.getJSON( host + "config?leg=" + legInitCount, function( data ) {
    var config = data;
    console.log(data);

    legs[config.id] = initLeg(config);

    legInitCount++;

    if (legInitCount < 6) {
      initLegs();
    } else {
      updateLegs();
    }
  });

}


$(function() {
  console.log('boo');

  initScene();

  initLegs();

  // start animation loop
  animate();

});


// event handler for save button
$('#saveConfigBut').click(()=>{
  alert('save me');
});
