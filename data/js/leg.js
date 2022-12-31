
function radToDeg(v) {
  return v * 180 / Math.PI;
}
function degToRad(v) {
  return v * Math.PI / 180;
}

const legMinAngles = [degToRad(-90), degToRad(-110), degToRad(-75)];
const legMaxAngles = [degToRad(90), degToRad(110), degToRad(160)];


function sqr(x) {
    return x*x;
}

function constrain(x, xmin, xmax) {
    return Math.min(Math.max(x, xmin), xmax);
}


function makeAX12(hornMat, bodyMat) {
  // servo horn
  var geo = new THREE.CylinderGeometry(22/2, 22/2, 40);
  var horn = new THREE.Mesh(geo, hornMat);
  horn.castShadow = true;
  horn.receiveShadow = true;
  horn.rotation.x = degToRad(90);

  // body
  geo = new THREE.BoxGeometry(50,32,26);
  var b = new THREE.Mesh(geo, bodyMat);
  b.castShadow = true;
  b.receiveShadow = true;
  b.position.x = -(50/2 - 11.5)
  horn.add(b);

  return horn;
}


// Constructor
function Leg(boneLengths) {
    this.boneLengths = boneLengths;
    this.canReachTarget = true;
    this.extension = 0.5;  // range 0..1

    this.object = new THREE.Object3D();
    this.targetAng = [0,0,0];
    this.curAng = [0,0,0];
    this.minAng = legMinAngles;
    this.maxAng = legMaxAngles;
    this.current = new THREE.Vector3(boneLengths[0] + boneLengths[1] + boneLengths[2], 0, 0);

    this.jointMode = false;  // false = IK, true = joint driven

    // axis for root object

    var axisHelper = new THREE.AxisHelper( 30 );
    this.object.add( axisHelper );


    // materials
    var mat = new THREE.MeshPhongMaterial({
        color:0xffaa00
    });
    var mat2 = new THREE.MeshPhongMaterial({
        color:0xf03000
    });
    var mat3 = new THREE.MeshPhongMaterial({
        color:0x505050
    });

    // COXA servo
    this.object.add( makeAX12(mat2, mat3) );

    // COXA
    var geo = new THREE.BoxGeometry(this.boneLengths[0],10,10);
    // position geometry
    geo.translate(this.boneLengths[0]/2,0,0);
    this.coxa = new THREE.Mesh(geo, mat3);
    this.coxa.castShadow = true;
    this.coxa.receiveShadow = true;

    var axisHelper = new THREE.AxisHelper( 30 );
    this.coxa.add( axisHelper );

    //this.coxa.position.set(hipPos[0], hipPos[1], hipPos[2]);
    //this.coxa.rotation.z = hipAng;

    this.object.add( this.coxa );


    // FEMUR
    geo = new THREE.BoxGeometry(this.boneLengths[1],7,7);
    // position geometry
    geo.translate(this.boneLengths[1]/2,0,0);
    this.femur = new THREE.Mesh(geo, mat);
    this.femur.castShadow = true;
    this.femur.receiveShadow = true;
    this.femur.position.set(this.boneLengths[0], 0, 0);

    var femurServo = makeAX12(mat2, mat3);
    femurServo.rotation.z = degToRad(180);
    femurServo.rotation.x = degToRad(0);
    femurServo.rotation.y= degToRad(-13);
    this.femur.add( femurServo );

    var axisHelper = new THREE.AxisHelper( 30 );
    this.femur.add( axisHelper );

    this.coxa.add(this.femur);

    // TIBIA
    geo = new THREE.CylinderGeometry(4, 1, this.boneLengths[2]);
    // position geometry
    geo.translate(0,-this.boneLengths[2]/2,0);
    geo.rotateZ(Math.PI/2);
    //this.tibia = new THREE.Mesh(geo, mat);
    //this.tibia = new Physijs.CylinderMesh(
    this.tibia = new THREE.Mesh(
      geo,
      mat
    )
    this.tibia.castShadow = true;
    this.tibia.receiveShadow = true;
    this.tibia.position.set(this.boneLengths[1],0,0);

    var tibiaServo = makeAX12(mat2, mat3);
    tibiaServo.rotation.z = degToRad(180);
    tibiaServo.rotation.x = degToRad(0);
    tibiaServo.rotation.y= degToRad(-46);
    this.tibia.add( tibiaServo );

    var axisHelper = new THREE.AxisHelper( 30 );
    this.tibia.add( axisHelper );

    this.femur.add(this.tibia);


    this.object.leg = this;

    // set default target
    this.target = new THREE.Vector3(0,0,0);

    // TARGET indicator
    // COXA
    geo = new THREE.BoxGeometry(3,3,3);
    this.targetObj = new THREE.Mesh(geo, mat2);
    this.targetObj.position.copy(this.target);
    this.object.add(this.targetObj);

    this.controller = {};
    this.controller.restPos = new THREE.Vector3(0,0,0);
}

// Methods
Leg.prototype = {


    setJointMode: function (m) {
      this.jointMode = m;
    },

    setWorldTarget: function(v1) {
      // calc vector from hip to target
      v1.sub(this.object.position);

      // account for leg orientation (z rotation)
      v1.applyAxisAngle(zAxis, -this.object.rotation.z);

      this.setTarget(v1);
    },

    // v1 = vector from hip to target
    setTarget: function (v1) {
        if (v1) {
            this.target = v1.clone();
            this.targetObj.position.copy(this.target);
        }

    },

    // v1 = array of joint angles [coxa, femur, tibia]
    setTargetAngles: function (v1) {
      this.targetAng = v1;
    },

    update: function (td) {
        // see https://oscarliang.com/inverse-kinematics-and-trigonometry-basics/
        var leg = this;



        // update joint angles
        for (i=0; i<3; i++) {
            var err = (leg.targetAng[i] - leg.curAng[i]);

            leg.curAng[i] = constrain(
                leg.curAng[i] + err,
                leg.minAng[i],
                leg.maxAng[i]
            );
        }

        // update leg angles
        leg.coxa.rotation.z = leg.curAng[0];
        leg.femur.rotation.y = leg.curAng[1];
        leg.tibia.rotation.y = leg.curAng[2];


        //leg.current.set( x1 * Math.cos(leg.curAng[0]) , x1 * Math.sin(leg.curAng[0]) , y1);

        // calculate extension factor
        //leg.extension = Math.sqrt(sqr(fx + tx) + sqr(fy + ty)) / (leg.boneLengths[1] + leg.boneLengths[2]);
    }
};
