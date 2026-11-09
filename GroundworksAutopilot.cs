using System;using System.Linq;using UnityEngine;using KSP.UI.Screens;

// Original game-flight guidance. Stock KSP SAS supplies hardware-limited attitude control.
public static class GroundworksGuidanceMath {
 public static bool Finite(double x){return !double.IsNaN(x)&&!double.IsInfinity(x);}
 public static double CircularSpeed(double mu,double radius){return mu>0&&radius>0?Math.Sqrt(mu/radius):0;}
 public static double TransferSpeed(double mu,double radius,double otherRadius){return mu>0&&radius>0&&otherRadius>0?Math.Sqrt(Math.Max(0,mu*(2/radius-2/(radius+otherRadius)))):0;}
 public static double StoppingDistance(double speed,double netAcceleration){return netAcceleration>0?speed*speed/(2*netAcceleration):double.PositiveInfinity;}
 public static double LandingSpeed(double height,double netAcceleration){height=Math.Max(0,height);return Math.Min(35,Math.Min(.25+height*.18,Math.Sqrt(.25*.25+2*Math.Max(.1,netAcceleration*.16)*height)));}
 public static double AttitudeCommand(double error,double rate,double authority){if(authority<=1e-8||!Finite(authority)||!Finite(error)||!Finite(rate))return 0;double desired=Math.Sign(error)*Math.Min(.16,Math.Sqrt(2*authority*Math.Abs(error))*.65);return Math.Max(-1,Math.Min(1,-(desired-rate)/Math.Max(.15,Math.Min(1,.2/Math.Sqrt(authority)))/authority));}
 public static double BrakingSpeed(double total,double vertical,double altitude,double atmosphereDepth){return atmosphereDepth>0&&altitude>Math.Min(4000,atmosphereDepth*.05)?Math.Max(0,-vertical):total;}
 public static double AlignedAscentThrottle(double requested,double angle,double density){return density<.1&&angle>12?Math.Min(requested,.04):requested;}
 public static double VacuumSupport(double gravity,double speed,double radius,double vertical){return Math.Max(0,gravity-speed*speed/Math.Max(1,radius)-vertical*.25);}
 public static double AirlessWarpClearance(double speed,double vertical,double net){return Math.Max(6000,StoppingDistance(speed,Math.Max(.1,net*.7))+Math.Abs(vertical)*15+500);}
 public static double LaunchPitch(double height){return 90*(1-Math.Pow(Math.Max(0,Math.Min(1,(height-350)/45000)),.55));}
}
public class GroundworksAutopilot {
 public string Task="",Message="MANUAL";public bool Owned{get{return Task.Length>0;}}public bool HasControl{get{return Owned;}}public bool Available{get{return vessel!=null;}}public bool LastAbortClean=true;
 Vessel vessel;string phase="";double target,headingInclination,launchAltitude,lastStageUT,emptySince=-1;bool staging,warp,allNodes,braking,nodeStarted;double descentPeri,initialNodeDv;Vector3d lastNodeDirection;ManeuverNode currentNode;float commanded;Vector3d desiredDirection;
 public double LastBottomExtent,LastTerrainAltitude,LastClearance;
 public double PointingError{get;private set;}
 public string Phase{get{return phase;}}public float CommandedThrottle{get{return commanded;}}
 public void Bind(Vessel v){Cancel(false);vessel=v;}
 bool Fail(string text){Message=text;return false;}
 bool Control(){return vessel!=null&&!vessel.packed&&vessel.IsControllable&&vessel.Autopilot.CanSetMode(VesselAutopilot.AutopilotMode.StabilityAssist);}
 public bool Launch(double altitude,double inclination,bool autostage,bool autoWarp){
  if(!Control())return Fail("Needs command, SAS and control");if(Owned)return Fail("Disengage first");if(!vessel.Landed||vessel.Splashed)return Fail("Launch from land");
  if(!GroundworksGuidanceMath.Finite(altitude)||!GroundworksGuidanceMath.Finite(inclination)||altitude<Math.Max(1000,vessel.mainBody.atmosphereDepth+5000)||altitude>vessel.mainBody.sphereOfInfluence-vessel.mainBody.Radius-1000||Math.Abs(inclination)>180)return Fail("Invalid orbit target");
  if(Math.Abs(inclination)<Math.Abs(vessel.latitude)-.15||Math.Abs(inclination)>180-Math.Abs(vessel.latitude)+.15)return Fail("Inclination unreachable from this latitude");
  var next=vessel.FindPartModulesImplementing<ModuleEngines>().Where(e=>e.part.inverseStage==vessel.currentStage-1).ToArray();if(next.Length==0)return Fail("Next stage must contain launch engines");
  if(vessel.patchedConicSolver==null)return Fail("Tracking / maneuver capability needed");
  target=altitude;headingInclination=inclination;staging=autostage;warp=autoWarp;launchAltitude=vessel.altitude;Task="Launch";phase="ASCENT";emptySince=-1;braking=false;lastStageUT=Planetarium.GetUniversalTime();InputLockManager.RemoveControlLock("GWComputerTarget");StageManager.ActivateNextStage();Message="VERTICAL ASCENT";return true;
 }
 public bool Land(bool targeted,bool autoWarp){
  if(!Control())return Fail("Needs command, SAS and control");if(Owned)return Fail("Disengage first");if(vessel.LandedOrSplashed)return Fail("Already landed");if(!vessel.mainBody.hasSolidSurface&&!vessel.mainBody.ocean)return Fail("Body has no supported landing surface");if(targeted)return Fail("Use land nearby; precision targeting is not implemented");
  if(!vessel.FindPartModulesImplementing<ModuleEngines>().Any(e=>e.EngineIgnited&&!e.throttleLocked))return Fail("Activate a throttleable landing engine");
  warp=autoWarp;Task="Landing";phase=vessel.orbit.PeA>-(vessel.mainBody.Radius*.08)?"DEORBIT":"DESCENT";descentPeri=-(vessel.mainBody.Radius*.08);braking=false;BottomClearance();Message=phase;return true;
 }
 public bool ExecuteNodes(bool all,bool autoWarp){if(!Control())return Fail("Needs command, SAS and control");if(Owned)return Fail("Disengage first");if(vessel.patchedConicSolver==null||vessel.patchedConicSolver.maneuverNodes.Count==0)return Fail("Create a maneuver node first");Task="Maneuvers";phase="NODE";allNodes=all;warp=autoWarp;currentNode=null;nodeStarted=false;Message="ALIGNING FOR NODE";return true;}
 public bool PlanCircularization(){if(Owned)return Fail("Disengage guidance before planning");if(vessel==null||vessel.patchedConicSolver==null||vessel.orbit.eccentricity>=1)return Fail("Bound orbit and maneuver solver required");double ut=Planetarium.GetUniversalTime()+Math.Max(10,vessel.orbit.timeToAp);double r=vessel.orbit.ApR;double speed=GroundworksGuidanceMath.CircularSpeed(vessel.mainBody.gravParameter,r);double actual=Math.Sqrt(Math.Max(0,vessel.mainBody.gravParameter*(2/r-1/vessel.orbit.semiMajorAxis)));var n=vessel.patchedConicSolver.AddManeuverNode(ut);n.OnGizmoUpdated(new Vector3d(0,0,speed-actual),ut);Message="CIRCULARIZATION NODE CREATED";return true;}
 public bool PlanPrograde(double dv,double delay){if(Owned)return Fail("Disengage guidance before planning");if(vessel==null||vessel.patchedConicSolver==null||!GroundworksGuidanceMath.Finite(dv)||Math.Abs(dv)>10000||!GroundworksGuidanceMath.Finite(delay)||delay<5)return Fail("Invalid maneuver inputs");double ut=Planetarium.GetUniversalTime()+delay;var n=vessel.patchedConicSolver.AddManeuverNode(ut);n.OnGizmoUpdated(new Vector3d(0,0,dv),ut);Message="PROGRADE NODE CREATED";return true;}
 public bool PlanApsis(double altitude,bool apoapsis){
  if(Owned)return Fail("Disengage guidance before planning");if(vessel==null||vessel.patchedConicSolver==null||vessel.LandedOrSplashed||vessel.orbit.eccentricity>=1||vessel.orbit.PeA<0)return Fail("A bound orbit above the surface is required");double min=Math.Max(1000,vessel.mainBody.atmosphereDepth+5000);if(!GroundworksGuidanceMath.Finite(altitude)||altitude<min||altitude>vessel.mainBody.sphereOfInfluence-vessel.mainBody.Radius-1000)return Fail("Target is outside the orbit-planner limits");
  double radius=vessel.mainBody.Radius+altitude;double burnRadius=apoapsis?vessel.orbit.PeR:vessel.orbit.ApR;if(apoapsis&&radius<burnRadius||!apoapsis&&radius>burnRadius)return Fail(apoapsis?"Apoapsis must not be below periapsis":"Periapsis must not be above apoapsis");double time=apoapsis?vessel.orbit.timeToPe:vessel.orbit.timeToAp;if(time<10)time+=vessel.orbit.period;if(!GroundworksGuidanceMath.Finite(time)||time<0)return Fail("Orbit timing unavailable");
  double oldOther=apoapsis?vessel.orbit.ApR:vessel.orbit.PeR;double dv=GroundworksGuidanceMath.TransferSpeed(vessel.mainBody.gravParameter,burnRadius,radius)-GroundworksGuidanceMath.TransferSpeed(vessel.mainBody.gravParameter,burnRadius,oldOther);double ut=Planetarium.GetUniversalTime()+time;var node=vessel.patchedConicSolver.AddManeuverNode(ut);node.OnGizmoUpdated(new Vector3d(0,0,dv),ut);Message=apoapsis?"APOAPSIS NODE CREATED":"PERIAPSIS NODE CREATED";return true;
 }
 public void Refresh(){if(!Owned||vessel==null)return;if(vessel.LandedOrSplashed&&Task=="Landing"){Touchdown();return;}if(warp&&vessel.packed){if(phase=="DESCENT"&&vessel.mainBody.atmosphereDepth<=0){double terrain=vessel.mainBody.TerrainAltitude(vessel.latitude,vessel.longitude,true);double height=vessel.altitude-terrain-Math.Max(20,LastBottomExtent);double radius=vessel.mainBody.Radius+vessel.altitude;double gravity=vessel.mainBody.gravParameter/(radius*radius);if(height<=GroundworksGuidanceMath.AirlessWarpClearance(vessel.srfSpeed,vessel.verticalSpeed,Acceleration()-gravity))TimeWarp.SetRate(0,true);return;}double remaining=phase=="COAST"?vessel.orbit.timeToAp:phase=="NODE"&&currentNode!=null?currentNode.UT-Planetarium.GetUniversalTime():0;if(phase=="DESCENT"&&vessel.altitude>vessel.mainBody.atmosphereDepth+3000)return;if(remaining<40||vessel.altitude<vessel.mainBody.atmosphereDepth+2000)TimeWarp.SetRate(0,true);}}
 void Point(Vector3d direction){if(direction.sqrMagnitude<1e-8)return;desiredDirection=direction.normalized;PointingError=Vector3d.Angle(vessel.ReferenceTransform.up,desiredDirection);}
 void Steer(FlightCtrlState state){
  if(desiredDirection.sqrMagnitude<1e-8)return;if(vessel.Autopilot.Enabled){vessel.ActionGroups.SetGroup(KSPActionGroup.SAS,false);vessel.Autopilot.Disable();}
  Transform reference=vessel.ReferenceTransform;Quaternion delta=Quaternion.FromToRotation(reference.up,(Vector3)desiredDirection);float angle;Vector3 axis;delta.ToAngleAxis(out angle,out axis);if(angle>180)angle-=360;Vector3 error=reference.InverseTransformDirection(axis)*angle*Mathf.Deg2Rad;error.y=0;
  Vector3 rate=vessel.rootPart.Rigidbody==null?Vector3.zero:reference.InverseTransformDirection(vessel.rootPart.Rigidbody.angularVelocity);Vector3 torque=Vector3.zero;foreach(var part in vessel.parts)foreach(PartModule module in part.Modules){var provider=module as ITorqueProvider;if(provider==null)continue;Vector3 positive,negative;provider.GetPotentialTorque(out positive,out negative);torque+=new Vector3(Mathf.Max(Mathf.Abs(positive.x),Mathf.Abs(negative.x)),Mathf.Max(Mathf.Abs(positive.y),Mathf.Abs(negative.y)),Mathf.Max(Mathf.Abs(positive.z),Mathf.Abs(negative.z)));}Vector3 moi=vessel.MOI;
  state.pitch=(float)GroundworksGuidanceMath.AttitudeCommand(error.x,rate.x,Math.Abs(torque.x)/Math.Max(.01,moi.x));state.roll=(float)GroundworksGuidanceMath.AttitudeCommand(0,rate.y,Math.Abs(torque.y)/Math.Max(.01,moi.y));state.yaw=(float)GroundworksGuidanceMath.AttitudeCommand(error.z,rate.z,Math.Abs(torque.z)/Math.Max(.01,moi.z));
 }
 double Angle(Vector3d direction){return Vector3d.Angle(vessel.ReferenceTransform.up,direction);}
 double AvailableThrust(){double value=0;float pressure=(float)(vessel.staticPressurekPa/101.325);foreach(var e in vessel.FindPartModulesImplementing<ModuleEngines>()){if(!e.EngineIgnited||e.flameout||e.throttleLocked)continue;double isp0=e.atmosphereCurve.Evaluate(0);value+=e.maxThrust*e.thrustPercentage*.01*(isp0>0?e.atmosphereCurve.Evaluate(pressure)/isp0:1);}return value;}
 double Acceleration(){return AvailableThrust()/Math.Max(.001,vessel.GetTotalMass());}
 Vector3d Up{get{return ((Vector3d)vessel.CoM-vessel.mainBody.position).normalized;}}
 Vector3d Horizontal(Vector3d velocity){var up=Up;return velocity-up*Vector3d.Dot(velocity,up);}
 void Throttle(FlightCtrlState state,double value){commanded=(float)Math.Max(0,Math.Min(1,GroundworksGuidanceMath.Finite(value)?value:0));state.mainThrottle=commanded;FlightInputHandler.state.mainThrottle=commanded;}
 public void Drive(FlightCtrlState state){if(!Owned||vessel==null)return;if(vessel.packed)return;if(TimeWarp.CurrentRate>4&&warp&&(phase=="COAST"||phase=="NODE"||phase=="DESCENT")){Throttle(state,0);return;}if(!Control()||TimeWarp.CurrentRate>4){Throttle(state,0);Cancel(false);Message="CONTROL LOST / MANUAL WARP";return;}
  double accel=Acceleration(),g=FlightGlobals.getGeeForceAtPosition(vessel.CoM).magnitude;
  if(Task=="Launch")DriveLaunch(state,accel,g);else if(Task=="Maneuvers")DriveNode(state,accel);else if(Task=="Landing")DriveLanding(state,accel,g);if(Owned)Steer(state);else state.pitch=state.yaw=state.roll=0;
 }
 bool TryStage(){if(!staging||vessel.currentStage<=0)return false;double now=Planetarium.GetUniversalTime();if(now-lastStageUT<1)return false;var active=vessel.FindPartModulesImplementing<ModuleEngines>().Where(e=>e.EngineIgnited).ToArray();if(active.Length>0&&active.Any(e=>!e.flameout)) {emptySince=-1;return false;}if(emptySince<0){emptySince=now;return false;}if(now-emptySince<.7)return false;
  var next=vessel.parts.Where(p=>p.inverseStage==vessel.currentStage-1).ToArray();if(!next.Any(p=>p.FindModulesImplementing<ModuleEngines>().Any())||next.Any(p=>p.FindModulesImplementing<ModuleParachute>().Any()))return false;
  StageManager.ActivateNextStage();lastStageUT=now;emptySince=-1;return true;
 }
 Vector3d LaunchDirection(){Vector3d up=Up,north=Horizontal(vessel.mainBody.GetSurfaceNVector(90,0)).normalized;Vector3d east=Horizontal(vessel.obt_velocity-vessel.srf_velocity).normalized;if(east.sqrMagnitude<.01)east=Vector3d.Cross(up,north).normalized;double az=Math.Asin(Math.Max(-1,Math.Min(1,Math.Cos(Math.Abs(headingInclination)*Math.PI/180)/Math.Cos(vessel.latitude*Math.PI/180))));if(headingInclination<0)az=Math.PI-az;Vector3d heading=east*Math.Sin(az)+north*Math.Cos(az);double pitch=GroundworksGuidanceMath.LaunchPitch(vessel.altitude-launchAltitude)*Math.PI/180;Vector3d desired=heading*Math.Cos(pitch)+up*Math.Sin(pitch);
  if(vessel.srfSpeed>120&&vessel.atmDensity>.01){Vector3 pro=((Vector3)vessel.srf_velocity).normalized;desired=Vector3.RotateTowards(pro,(Vector3)desired,6*Mathf.Deg2Rad,0);}
  return desired.normalized;
 }
 void DriveLaunch(FlightCtrlState state,double accel,double g){
  if(phase=="ASCENT"){
   Vector3d direction=LaunchDirection();Point(direction);double ap=vessel.orbit.ApA;
   if(ap>=target+400){phase="COAST";Throttle(state,0);Message="COAST TO CIRCULARIZATION";return;}
   if(accel<.1){TryStage();Throttle(state,1);Message="STAGING / ENGINE NEEDED";return;}
   double throttle=Math.Max(.03,Math.Min(1,(target+500-ap)/2500));if(vessel.altitude-launchAltitude<200)throttle=1;double aligned=GroundworksGuidanceMath.AlignedAscentThrottle(throttle,PointingError,vessel.atmDensity);Throttle(state,aligned);Message=aligned<throttle?"ALIGNING / LIMITED THRUST":"ASCENT / GRAVITY TURN";
  }else if(phase=="COAST"){
   Vector3d tangent=Horizontal(vessel.obt_velocity).normalized;Point(tangent);Throttle(state,0);
   if(vessel.orbit.ApA<target-1500&&vessel.verticalSpeed>0&&vessel.altitude<vessel.mainBody.atmosphereDepth){phase="ASCENT";return;}
   double dv=GroundworksGuidanceMath.CircularSpeed(vessel.mainBody.gravParameter,vessel.orbit.ApR)-Horizontal(vessel.obt_velocity).magnitude;double burn=Math.Max(0,dv)/Math.Max(.1,accel);
   if(vessel.orbit.timeToAp<=burn*.55+2){TimeWarp.SetRate(0,true);phase="CIRCULARIZE";Message="CIRCULARIZING";}
   else if(warp&&vessel.altitude>vessel.mainBody.atmosphereDepth+3000&&vessel.orbit.timeToAp>burn+50)TimeWarp.SetRate(1,true);
  }else{
   if(vessel.orbit.PeA>=target-350&&vessel.orbit.ApA>vessel.mainBody.atmosphereDepth+5000){Throttle(state,0);Finish("ORBIT REACHED");return;}
   if(accel<.1){TryStage();Throttle(state,1);Message="CIRCULARIZATION NEEDS ENGINE";return;}
   Vector3d up=Up,tangent=Horizontal(vessel.obt_velocity).normalized;double radius=vessel.mainBody.Radius+vessel.altitude;double dv=GroundworksGuidanceMath.CircularSpeed(vessel.mainBody.gravParameter,radius)-Horizontal(vessel.obt_velocity).magnitude;
   double vr=Vector3d.Dot(vessel.obt_velocity,up),radial=Math.Max(-8,Math.Min(8,(target-vessel.altitude)*.002-vr*.12));Vector3d correction=tangent*Math.Max(.05,dv*.7)+up*radial;Point(correction);Throttle(state,Angle(correction)<15?Math.Min(1,Math.Max(.002,correction.magnitude/accel)):0);Message="CIRCULARIZING";
  }
 }
 void DriveNode(FlightCtrlState state,double accel){
  if(vessel.patchedConicSolver==null||vessel.patchedConicSolver.maneuverNodes.Count==0){Throttle(state,0);Finish("MANEUVERS COMPLETE");return;}
  var first=vessel.patchedConicSolver.maneuverNodes[0];if(first!=currentNode){currentNode=first;initialNodeDv=first.GetBurnVector(vessel.orbit).magnitude;nodeStarted=false;lastNodeDirection=first.GetBurnVector(vessel.orbit).normalized;}
  Vector3d burn=first.GetBurnVector(vessel.orbit);double remaining=burn.magnitude;if(remaining<.25||(nodeStarted&&Vector3d.Dot(burn,lastNodeDirection)<=0)){Throttle(state,0);first.RemoveSelf();currentNode=null;if(!allNodes||vessel.patchedConicSolver.maneuverNodes.Count==0)Finish("MANEUVERS COMPLETE");return;}
  Point(burn);double duration=initialNodeDv/Math.Max(.1,accel),time=first.UT-Planetarium.GetUniversalTime();
  if(time>duration*.5+.1){Throttle(state,0);Message="ALIGNING / WAITING FOR NODE";if(warp&&vessel.altitude>vessel.mainBody.atmosphereDepth+3000&&time>duration+50)TimeWarp.SetRate(1,true);return;}
  TimeWarp.SetRate(0,true);if(accel<.1){Throttle(state,0);Finish("NODE NEEDS ACTIVE ENGINE");return;}nodeStarted=true;Throttle(state,Angle(burn)<4?Math.Min(1,remaining/(accel*1.2)):0);Message="BURNING MANEUVER";
 }
 double BottomClearance(){Vector3 up=(Vector3)Up;Vector3 root=vessel.rootPart.transform.position;double bottom=0;foreach(var p in vessel.parts)foreach(var c in p.GetComponentsInChildren<Collider>()){if(c.isTrigger||c.gameObject.layer==16||c.gameObject.layer==20||!c.enabled)continue;Bounds b=c.bounds;double extent=Math.Abs(up.x)*b.extents.x+Math.Abs(up.y)*b.extents.y+Math.Abs(up.z)*b.extents.z;bottom=Math.Max(bottom,extent-Vector3.Dot(b.center-root,up));}
  double terrain=vessel.mainBody.TerrainAltitude(vessel.latitude,vessel.longitude,true);if(vessel.mainBody.ocean)terrain=Math.Max(0,terrain);double rootHeight=vessel.mainBody.GetAltitude((Vector3d)root)-terrain;LastBottomExtent=bottom;LastTerrainAltitude=terrain;LastClearance=Math.Max(.01,rootHeight-bottom);return LastClearance;
 }
 void DriveLanding(FlightCtrlState state,double accel,double g){
  if(accel<.1){Throttle(state,0);Finish("LANDING ABORTED - NO ENGINE / FUEL");return;}
  if(vessel.LandedOrSplashed){Throttle(state,0);Touchdown();return;}
  if(phase=="DEORBIT"){
   Vector3d retro=-Horizontal(vessel.obt_velocity).normalized;Point(retro);if(vessel.orbit.PeA<=descentPeri){phase=vessel.mainBody.atmosphereDepth<=0?"VACUUM BRAKE":"DESCENT";Throttle(state,0);Message=phase;return;}if(accel<.1){Throttle(state,0);Finish("DEORBIT NEEDS ENGINE");return;}Throttle(state,Angle(retro)<5?Math.Min(1,Math.Max(.02,(vessel.orbit.PeA-descentPeri)/100000)):0);Message="DEORBIT BURN";return;
  }
  if(phase=="VACUUM BRAKE"){
   TimeWarp.SetRate(0,true);Vector3d vacuumHorizontal=Horizontal(vessel.srf_velocity);if(vacuumHorizontal.magnitude<=8){phase="DESCENT";Point(Up);Throttle(state,0);Message="AIRLESS DESCENT COAST";return;}
   if(accel<=g*1.02){Throttle(state,0);Finish("LANDING ABORTED - INSUFFICIENT THRUST");return;}
   double support=Math.Min(accel*.8,GroundworksGuidanceMath.VacuumSupport(g,vacuumHorizontal.magnitude,vessel.mainBody.Radius+vessel.altitude,Vector3d.Dot(vessel.srf_velocity,Up)));double brake=Math.Min(vacuumHorizontal.magnitude*.6,Math.Sqrt(Math.Max(0,accel*accel-support*support))*.9);Vector3d vacuumForce=Up*support-vacuumHorizontal.normalized*brake;Point(vacuumForce);double vacuumAlignment=Math.Max(.7,Vector3d.Dot(vessel.ReferenceTransform.up,vacuumForce.normalized));Throttle(state,Angle(vacuumForce)<12?vacuumForce.magnitude/(accel*vacuumAlignment):0);Message="AIRLESS ORBITAL BRAKING / "+vacuumHorizontal.magnitude.ToString("F1")+" m/s";return;
  }
  Vector3d up=Up,velocity=vessel.srf_velocity;double vs=Vector3d.Dot(velocity,up),height=vessel.radarAltitude;double net=accel-g;
  if(height<15000)height=BottomClearance();double stop=GroundworksGuidanceMath.StoppingDistance(GroundworksGuidanceMath.BrakingSpeed(velocity.magnitude,vs,height,vessel.mainBody.atmosphereDepth),Math.Max(.1,net*.7));
  if(!braking&&height>stop+Math.Max(40,Math.Max(0,-vs)*1.7)){
   Point(velocity.magnitude>10?-velocity:up);Throttle(state,0);Message="DESCENT COAST / "+height.ToString("F0")+" m";if(warp&&height>vessel.mainBody.atmosphereDepth+10000&&vs<0)TimeWarp.SetRate(1,true);return;
  }
  TimeWarp.SetRate(0,true);braking=true;vessel.ActionGroups.SetGroup(KSPActionGroup.Gear,true);if(accel<=g*1.02){Throttle(state,1);Message="INSUFFICIENT LANDING THRUST";return;}
  double horizontalSpeed=Horizontal(velocity).magnitude;bool horizontalHold=height<700&&horizontalSpeed>.5;double desired=horizontalHold?Math.Max(-3,Math.Min(3,(400-height)*.3)):-GroundworksGuidanceMath.LandingSpeed(height,net);double low=Math.Max(0,height-.5),high=height+.5;double slope=(GroundworksGuidanceMath.LandingSpeed(high,net)-GroundworksGuidanceMath.LandingSpeed(low,net))/(high-low);if(horizontalHold)slope=0;double upAcceleration=Math.Max(g*.6,g-slope*vs+(desired-vs)*1.5);Vector3d horizontal=Horizontal(velocity);Vector3d horizontalForce=-horizontal*Math.Min(.7,.25+20/Math.Max(20,height));double tiltLimit=height>1000?50:height>200?20:8;double maxHorizontal=upAcceleration*Math.Tan(tiltLimit*Math.PI/180);if(horizontalForce.magnitude>maxHorizontal)horizontalForce=horizontalForce.normalized*maxHorizontal;Vector3d force=up*upAcceleration+horizontalForce;if(force.sqrMagnitude<.001)force=up*.01;Point(force);double alignment=Math.Max(.3,Vector3d.Dot(vessel.ReferenceTransform.up,force.normalized));Throttle(state,force.magnitude/(accel*alignment));Message=(horizontalHold?"ARRESTING DRIFT / ":"POWERED DESCENT / ")+height.ToString("F1")+" m";
 }
 void Touchdown(){Finish("TOUCHDOWN / SAS HOLD");if(vessel!=null){vessel.ActionGroups.SetGroup(KSPActionGroup.SAS,true);vessel.Autopilot.Enable(VesselAutopilot.AutopilotMode.StabilityAssist);}}
 void Finish(string text){Cancel(false);Message=text;}
 public void Cancel(bool all){bool active=Owned;Task="";phase="";commanded=0;currentNode=null;LastAbortClean=true;if(active||all){TimeWarp.SetRate(0,true);FlightInputHandler.state.mainThrottle=0;if(vessel!=null){vessel.ctrlState.mainThrottle=0;vessel.ctrlState.pitch=vessel.ctrlState.roll=vessel.ctrlState.yaw=0;vessel.ActionGroups.SetGroup(KSPActionGroup.SAS,false);vessel.Autopilot.Disable();}}}
}


