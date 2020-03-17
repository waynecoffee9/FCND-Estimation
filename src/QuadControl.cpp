#include "Common.h"
#include "QuadControl.h"
#include <iostream>
#include <algorithm>
#include <math.h>
#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float L2 = L / sqrt(2.f);
  /*
  cmd.desiredThrustsN[0] = CONSTRAIN((momentCmd.x/L2 + momentCmd.y/L2 - momentCmd.z/kappa + collThrustCmd)/4.f, minMotorThrust, maxMotorThrust); // front left
  cmd.desiredThrustsN[1] = CONSTRAIN((-momentCmd.x/L2 + momentCmd.y/L2 + momentCmd.z/kappa + collThrustCmd)/4.f, minMotorThrust, maxMotorThrust); // front right
  cmd.desiredThrustsN[2] = CONSTRAIN((momentCmd.x/L2 - momentCmd.y/L2 + momentCmd.z/kappa + collThrustCmd)/4.f, minMotorThrust, maxMotorThrust); // rear left
  cmd.desiredThrustsN[3] = CONSTRAIN((-momentCmd.x/L2 - momentCmd.y/L2 - momentCmd.z/kappa + collThrustCmd)/4.f, minMotorThrust, maxMotorThrust); // rear right
  */
  cmd.desiredThrustsN[0] = (momentCmd.x/L2 + momentCmd.y/L2 - momentCmd.z/kappa + collThrustCmd)/4.f; // front left
  cmd.desiredThrustsN[1] = (-momentCmd.x/L2 + momentCmd.y/L2 + momentCmd.z/kappa + collThrustCmd)/4.f; // front right
  cmd.desiredThrustsN[2] = (momentCmd.x/L2 - momentCmd.y/L2 + momentCmd.z/kappa + collThrustCmd)/4.f; // rear left
  cmd.desiredThrustsN[3] = (-momentCmd.x/L2 - momentCmd.y/L2 - momentCmd.z/kappa + collThrustCmd)/4.f; // rear right
  
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F momentCmd;
  V3F MOI = V3F(Ixx, Iyy, Izz);
  
  momentCmd = MOI * kpPQR * (pqrCmd - pqr);
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  double b_c_x_dot, b_c_y_dot, b_c_x, b_c_y, c_d;
  if(collThrustCmd > 0.0) {
	  c_d = -collThrustCmd / mass;
	  b_c_x = CONSTRAIN(accelCmd.x/c_d, -sin(maxTiltAngle), sin(maxTiltAngle));
	  b_c_y = CONSTRAIN(accelCmd.y/c_d, -sin(maxTiltAngle), sin(maxTiltAngle));
	  
	  b_c_x_dot = kpBank * (b_c_x - R(0,2));
	  b_c_y_dot = kpBank * (b_c_y - R(1,2));

	  pqrCmd.x = (R(1,0) * b_c_x_dot - R(0,0) * b_c_y_dot)/R(2,2);
	  pqrCmd.y = (R(1,1) * b_c_x_dot - R(0,1) * b_c_y_dot)/R(2,2);
	  pqrCmd.z = 0.0;
  }
  else {
	  pqrCmd.x = 0.0;
	  pqrCmd.y = 0.0;
  }
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  float err = posZCmd - posZ;
  float vel_cmd = kpPosZ * err + velZCmd;
  
  if(abs(integratedAltitudeError) < 0.5)
	integratedAltitudeError += (posZCmd - posZ) * dt;
  
  vel_cmd = CONSTRAIN(vel_cmd, -maxAscentRate, maxDescentRate);
  
  float err_dot = vel_cmd - velZ;
  
  float u_1_bar = kpVelZ * err_dot + KiPosZ * integratedAltitudeError + accelZCmd;
  
  thrust = (9.81f - u_1_bar) / R(2,2) * mass;
  
  thrust = CONSTRAIN(thrust, 4 * minMotorThrust, 4 * maxMotorThrust);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  V3F err = posCmd - pos;
  V3F vel_cmd = kpPosXY * err + velCmd;
  
  if(vel_cmd.mag() > maxSpeedXY)
	  vel_cmd = vel_cmd.norm() * maxSpeedXY;

  
  V3F err_dot = vel_cmd - vel;
  
  accelCmd += kpVelXY * err_dot;
  accelCmd.z = 0.0;
  if (accelCmd.mag() > maxAccelXY)
	  accelCmd = accelCmd.norm() * maxAccelXY;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  if(yawCmd > 0.f) {
	  yawCmd = fmodf(yawCmd, 2.f * F_PI);
  } else {
	  yawCmd = fmodf(yawCmd, -2.f * F_PI);
  }
  
  float err = yawCmd - yaw;
  if(err > F_PI) {
	  err -= 2 * F_PI;
  }
  else if(err < -F_PI) {
	  err += 2 * F_PI;
  }
  
  yawRateCmd = kpYaw * err;

  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
