#region mdk preserve
// Sam's Autopilot Manager
public static string VERSION = "2.6.7";
//
// Documentation: http://steamcommunity.com/sharedfiles/filedetails/?id=1653875433
// 
// Owner: Sam
// Contributors: SCBionicle
//
// Latest changes:
// - Fix block attribute capitalization.
// - Allows selecting primary connector for docking with tag MAIN.
// - Hacked around KEEN bug with surface GetText.
// - Added Orbital dock. "ADD ORBIT" will add a dock at Orbit imeadiatelly above ship.
// - Hydrogen tanks can now be considered CARGO too, just add the tag [SAM CARGO];
// - when using the ADD GPS command, dont add the current dock if docked;
// - better error log when no Remote Controls added;
// - tuned approach calculations to eliminate crashes against the dock;
// - halfed the docking distances;
// - made the obstacle finder cause the ship to fly further away from obstacles to reduce crashes against mountains;
// - fixed a bug that could cause ships to crash against the docks;
// - added support for sending remote commands to ships;
// - support for FORCE Tag in batteries and tanks to force charging/filling upon docking;
// - SAMv2 will now disable dampeners after docking in order to avoid thrusters from firing while docked (happens when docked to ships). This can be disabled with PB Tag NODAMPENERS;

// Change the tag used to identify blocks        
public static string TAG = "SAM";

// -------------------------------------------------------
// Update at your own peril.
// -------------------------------------------------------
private static float HORIZONT_CHECK_DISTANCE = 2000.0f;
private static float MAX_SPEED = 95.0f;
private static float APPROACH_DISTANCE = 10.0f;
private static float DOCK_DISTANCE = 5.0f;
private static float UNDOCK_DISTANCE = 10.0f;
private static float DOCKING_SPEED = 2.5f;
private static float APPROACH_SAFE_DISTANCE = 5.0f;
private static float TAXIING_SPEED = 10.0f;
private static float COLLISION_CORRECTION_ANGLE = (float)Math.PI / 7.5f;
private static string ADVERT_ID = "SAMv2";
private static string ADVERT_ID_VER = "SAMv2V";
// private static int ADVERT_VERSION = 2;
private static string STORAGE_VERSION = "deadbeef";

// -------------------------------------------------------
// Avoid touching anything below this. Things will break.
// -------------------------------------------------------
private static string MSG_ALIGNING = "aligning...";
private static string MSG_DOCKING = "docking...";
private static string MSG_UNDOCKING = "undocking...";
private static string MSG_CONVERGING = "converging...";
private static string MSG_APPROACHING = "approaching...";
private static string MSG_NAVIGATING = "navigating...";
private static string MSG_TAXIING = "taxiing...";
private static string MSG_NAVIGATION_TO = "Navigating to ";
private static string MSG_NAVIGATION_TO_WAYPOINT = "Navigating to coordinates";
private static string MSG_NAVIGATION_SUCCESSFUL = "Navigation successful!";
private static string MSG_NO_CONNECTORS_AVAILABLE = "No connectors available!";
private static string MSG_FAILED_TO_DOCK = "Failed to dock!";
private static string MSG_DOCKING_SUCCESSFUL = "Docking successful!";
private static string MSG_NO_REMOTE_CONTROL = "No Remote Control!";
private static string MSG_INVALID_GPS_TYPE = "Invalid GPS format!";
private static float HORIZONT_CHECK_ANGLE_LIMIT = (float)Math.PI / 32.0f;
private static float HORIZONT_CHECK_ANGLE_STEP = (float)Math.PI / 75.0f;
private static float HORIZONT_MAX_UP_ANGLE = (float)Math.PI;
private static float COLLISION_DISABLE_RADIUS_MULTIPLIER = 2.0f;
private static float IDLE_POWER = 0.0000001f;
private static double TICK_TIME = 0.16666f;
private static double GYRO_GAIN = 1.0;
private static double GYRO_MAX_ANGULAR_VELOCITY = Math.PI;
private static float GUIDANCE_MIN_AIM_DISTANCE = 0.5f;
private static float DISTANCE_TO_GROUND_IGNORE_PLANET = 1.2f * HORIZONT_CHECK_DISTANCE;
private static int DOCK_ATTEMPTS = 5;
private static int LOG_MAX_LINES = 60;
private static string CMD_TAG = TAG + "CMD";
private static string CMD_RES_TAG = TAG + "CMDRES";

#endregion

public static class Raytracer {
	public enum Result { NotRun, NoHit, Hit };
	public static Vector3D hitPosition;
	public static MyDetectedEntityInfo hit;
	public static Result Trace(ref Vector3D target, bool ignorePlanet) {
		foreach(IMyCameraBlock camera in GridBlocks.cameraBlocks) {
			if(!camera.CanScan(target)) {
				continue;
			}
			hit = camera.Raycast(target);
			if(hit.IsEmpty()) {
				return Result.NoHit;
			}
			if(hit.EntityId == GridBlocks.MasterProgrammableBlock.CubeGrid.EntityId) {
				continue;
			}
			switch(hit.Type) {
				case MyDetectedEntityType.Planet:
					if(ignorePlanet) {
						return Result.NoHit;
					}
					goto case MyDetectedEntityType.SmallGrid;
				case MyDetectedEntityType.Asteroid:
				case MyDetectedEntityType.LargeGrid:
				case MyDetectedEntityType.SmallGrid:
					hitPosition = hit.HitPosition.Value;
					return Result.Hit;
				default:
					return Result.NoHit;
			}
		}
		return Result.NotRun;
	}
}
public static class Situation {
	public static Vector3D position;
	public static Vector3D linearVelocity;
	public static double elevationVelocity;
	public static Vector3D naturalGravity;
	public static bool planetDetected;
	public static Vector3D planetCenter = new Vector3D();
	public static bool inGravity;
	public static double distanceToGround;
	public static double radius;
	public static float mass;
	public static Vector3D gravityUpVector;
	public static Vector3D gravityDownVector;
	public static Vector3D upVector;
	public static Vector3D forwardVector;
	public static Vector3D backwardVector;
	public static Vector3D downVector;
	public static Vector3D rightVector;
	public static Vector3D leftVector;
	public static MatrixD orientation;
	public static Vector3D gridForwardVect;
	public static Vector3D gridUpVect;
	public static Vector3D gridLeftVect;
	private static Dictionary<Base6Directions.Direction, double> maxThrust = new Dictionary<Base6Directions.Direction, double>() { { Base6Directions.Direction.Backward, 0 }, { Base6Directions.Direction.Down, 0 }, { Base6Directions.Direction.Forward, 0 }, { Base6Directions.Direction.Left, 0 }, { Base6Directions.Direction.Right, 0 }, { Base6Directions.Direction.Up, 0 }, };
	private static double forwardChange, upChange, leftChange;
	private static Vector3D maxT;
	public static double GetMaxThrust(Vector3D dir) {
		// return MAX_TRUST_UNDERESTIMATE_PERCENTAGE * maxThrust.MinBy(kvp => (float)kvp.Value).Value;
		forwardChange = Vector3D.Dot(dir, Situation.gridForwardVect);
		upChange = Vector3D.Dot(dir, Situation.gridUpVect);
		leftChange = Vector3D.Dot(dir, Situation.gridLeftVect);
		maxT = new Vector3D();
		maxT.X = forwardChange * maxThrust[(forwardChange > 0) ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward];
		maxT.Y = upChange * maxThrust[(upChange > 0) ? Base6Directions.Direction.Up : Base6Directions.Direction.Down];
		maxT.Z = leftChange * maxThrust[(leftChange > 0) ? Base6Directions.Direction.Left : Base6Directions.Direction.Right];
		return maxT.Length();
	}
	public static void RefreshParameters() {
		foreach(Base6Directions.Direction dir in maxThrust.Keys.ToList()) {
			maxThrust[dir] = 0;
		}
		foreach(IMyThrust thruster in GridBlocks.thrustBlocks) {
			if(!thruster.IsWorking) {
				continue;
			}
			maxThrust[thruster.Orientation.Forward] += thruster.MaxEffectiveThrust;
		}
		//var myList = maxThrust.ToList();
		//myList.Sort((pair1, pair2) => pair1.Value.CompareTo(pair2.Value));
		//for(int i=0; i<myList.Count-2; ++i) {
		//    maxThrust[myList[i].Key] = myList[i].Value / 2.0f;
		//}

		gridForwardVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Forward);
		gridUpVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Up);
		gridLeftVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Left);
		mass = RemoteControl.block.CalculateShipMass().PhysicalMass;
		position = RemoteControl.block.CenterOfMass;
		orientation = RemoteControl.block.WorldMatrix.GetOrientation();
		radius = RemoteControl.block.CubeGrid.WorldVolume.Radius;
		forwardVector = RemoteControl.block.WorldMatrix.Forward;
		backwardVector = RemoteControl.block.WorldMatrix.Backward;
		rightVector = RemoteControl.block.WorldMatrix.Right;
		leftVector = RemoteControl.block.WorldMatrix.Left;
		upVector = RemoteControl.block.WorldMatrix.Up;
		downVector = RemoteControl.block.WorldMatrix.Down;
		linearVelocity = RemoteControl.block.GetShipVelocities().LinearVelocity;
		elevationVelocity = Vector3D.Dot(linearVelocity, upVector);
		planetDetected = RemoteControl.block.TryGetPlanetPosition(out planetCenter);
		naturalGravity = RemoteControl.block.GetNaturalGravity();
		inGravity = naturalGravity.Length() >= 0.5;
		if(inGravity) {
			RemoteControl.block.TryGetPlanetElevation(MyPlanetElevation.Surface, out distanceToGround);
			gravityDownVector = Vector3D.Normalize(naturalGravity);
			gravityUpVector = -1 * gravityDownVector;
		} else {
			distanceToGround = DISTANCE_TO_GROUND_IGNORE_PLANET;
			gravityDownVector = downVector;
			gravityUpVector = upVector;
		}
	}
}
public static class Horizont {
	public static float angle = 0.0f;
	public static bool hit = false;
	private static bool ignorePlanet;
	private static Vector3D tracePosition;
	private static float angleDir = 1.0f;
	private static float up = 1.0f, down = 1.0f, mult = 1.0f;
	public static Vector3D? ScanHorizont(float distance, Vector3D forwardVector, Vector3D rightVector) {
		tracePosition = Situation.position + Math.Min(distance, HORIZONT_CHECK_DISTANCE) * Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
		ignorePlanet = Situation.distanceToGround >= DISTANCE_TO_GROUND_IGNORE_PLANET;
		if(hit) {
			switch(Raytracer.Trace(ref tracePosition, ignorePlanet)) {
				case Raytracer.Result.Hit:
					angle += HORIZONT_CHECK_ANGLE_STEP * up;
					up = Math.Min(10.0f, up + 1.0f);
					down = 1.0f;
					mult = 1.0f;
					angle = (float)Math.Min(HORIZONT_MAX_UP_ANGLE, angle);
					return Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
				case Raytracer.Result.NoHit:
					angle -= HORIZONT_CHECK_ANGLE_STEP * down;
					down = Math.Min(10.0f, down + 1.0f);
					up = 1.0f;
					mult = 1.0f;
					if(angle < -HORIZONT_CHECK_ANGLE_LIMIT) {
						hit = false;
						angle = 0.0f;
						up = down = mult = 1.0f;
						return Vector3D.Zero;
					}
					return Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
			}
		} else {
			switch(Raytracer.Trace(ref tracePosition, ignorePlanet)) {
				case Raytracer.Result.Hit:
					hit = true;
					up = down = mult = 1.0f;
					return Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
				case Raytracer.Result.NoHit:
					up = down = 1.0f;
					angle += angleDir * mult * HORIZONT_CHECK_ANGLE_STEP;
					mult = Math.Min(10.0f, mult + 1.0f);
					if(Math.Abs(angle) > HORIZONT_CHECK_ANGLE_LIMIT) {
						angle = Math.Min(HORIZONT_CHECK_ANGLE_LIMIT, Math.Max(angle, -HORIZONT_CHECK_ANGLE_LIMIT));
						angleDir *= -1.0f;
						mult = 1.0f;
					}
					return Vector3D.Zero;
			}
		}
		return null;
	}
}
public static class Guidance {
	private static Vector3D desiredPosition = new Vector3D();
	private static Vector3D desiredFront = new Vector3D();
	private static Vector3D desiredUp = new Vector3D();
	private static float desiredSpeed = MAX_SPEED;
	public static void Set(Waypoint w) {
		desiredPosition = w.stance.position;
		desiredFront = w.stance.forward;
		desiredUp = w.stance.up;
		desiredSpeed = w.maxSpeed;
	}
	public static void Release() {
		foreach(IMyGyro gyro in GridBlocks.gyroBlocks) {
			gyro.GyroOverride = false;
		}
		foreach(IMyThrust thruster in GridBlocks.thrustBlocks) {
			thruster.ThrustOverride = 0;
		}
	}
	public static void Tick() {
		Guidance.StanceTick();
		Guidance.GyroTick();
		Guidance.ThrusterTick();
	}
	public static bool Done() {
		return worldVector.Length() < 0.05 && pathLen <= 0.1f;
	}
	private static Vector3D pathNormal, path, aimTarget, upVector, aimVector;
	private static float pathLen;
	private static void StanceTick() {
		path = desiredPosition - Situation.position;
		pathLen = (float)path.Length();
		pathNormal = Vector3D.Normalize(path);
		if(desiredFront != Vector3D.Zero) {
			aimTarget = Situation.position + desiredFront * Situation.radius;
		} else {
			aimVector = (pathLen > GUIDANCE_MIN_AIM_DISTANCE) ? pathNormal : Situation.forwardVector;
			if(Situation.inGravity) {
				aimTarget = Situation.position + Vector3D.Normalize(Vector3D.ProjectOnPlane(ref aimVector, ref Situation.gravityUpVector)) * Situation.radius;
			} else {
				aimTarget = Situation.position + aimVector * Situation.radius;
			}
		}
		if(Situation.inGravity) {
			upVector = (desiredUp == Vector3D.Zero) ? Situation.gravityUpVector : desiredUp;
		} else {
			upVector = (desiredUp == Vector3D.Zero) ? Vector3D.Cross(aimVector, Situation.leftVector) : desiredUp;
		}
	}
	private static Quaternion invQuat;
	private static Vector3D direction, refVector, worldVector, localVector, realUpVect, realRightVect;
	private static double azimuth, elevation, roll;
	private static void GyroTick() {
		if(GridBlocks.gyroBlocks.Count == 0) {
			return;
		}
		direction = Vector3D.Normalize(aimTarget - Situation.position);
		invQuat = Quaternion.Inverse(Quaternion.CreateFromForwardUp(Situation.forwardVector, Situation.upVector));
		refVector = Vector3D.Transform(direction, invQuat);
		Vector3D.GetAzimuthAndElevation(refVector, out azimuth, out elevation);
		realUpVect = Vector3D.ProjectOnPlane(ref upVector, ref direction);
		realUpVect.Normalize();
		realRightVect = Vector3D.Cross(direction, realUpVect);
		realRightVect.Normalize();
		roll = Vector3D.Dot(Situation.upVector, realRightVect);
		worldVector = Vector3.Transform((new Vector3D(elevation, azimuth, roll)), Situation.orientation);
		foreach(IMyGyro gyro in GridBlocks.gyroBlocks) {
			localVector = Vector3.Transform(worldVector, Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));
			gyro.Pitch = (float)MathHelper.Clamp((-localVector.X * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
			gyro.Yaw = (float)MathHelper.Clamp(((-localVector.Y) * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
			gyro.Roll = (float)MathHelper.Clamp(((-localVector.Z) * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
			gyro.GyroOverride = true;
		}
	}
	private static float forwardChange, upChange, leftChange, applyPower, massFix;
	private static Vector3D force, directVel, directNormal, indirectVel;
	private static double ttt, maxFrc, maxVel, maxAcc, TIME_STEP = 2.5 * TICK_TIME, smooth;
	private static float massA = 2000000.0f;
	private static float massB = 5000.0f;
	private static float massM = (massA - 2.0f * massB) / (massA - massB);
	private static float massN = 1.0f / (massA - massB);
	private static void ThrusterTick() {
		if(pathLen == 0.0f) {
			return;
		}
		massFix = massM * Situation.mass + massN * Situation.mass * Situation.mass;

		force = Situation.mass * Situation.naturalGravity;
		directVel = Vector3D.ProjectOnVector(ref Situation.linearVelocity, ref pathNormal);
		directNormal = Vector3D.Normalize(directVel);
		if(!directNormal.IsValid()) {
			directNormal = Vector3D.Zero;
		}
		maxFrc = Situation.GetMaxThrust(pathNormal) - ((Vector3D.Dot(force, pathNormal) > 0) ? Vector3D.ProjectOnVector(ref force, ref pathNormal).Length() : 0.0);
		maxVel = Math.Sqrt(2.0 * pathLen * maxFrc / massFix);
		smooth = Math.Min(Math.Max((desiredSpeed + 1.0f - directVel.Length()) / 2.0f, 0.0f), 1.0f);
		maxAcc = 1.0f + (maxFrc / massFix) * smooth * smooth * (3.0f - 2.0f * smooth);
		ttt = Math.Max(TIME_STEP, Math.Abs(maxVel / maxAcc));
		force += massFix * -2.0 * (pathNormal * pathLen / ttt / ttt - directNormal * directVel.Length() / ttt);
		indirectVel = Vector3D.ProjectOnPlane(ref Situation.linearVelocity, ref pathNormal);
		force += massFix * indirectVel / TIME_STEP;
		forwardChange = (float)Vector3D.Dot(force, Situation.gridForwardVect);
		upChange = (float)Vector3D.Dot(force, Situation.gridUpVect);
		leftChange = (float)Vector3D.Dot(force, Situation.gridLeftVect);
		foreach(IMyThrust thruster in GridBlocks.thrustBlocks) {
			if(!thruster.IsWorking) {
				thruster.ThrustOverridePercentage = 0;
				continue;
			}
			switch(thruster.Orientation.Forward) {
				case Base6Directions.Direction.Forward:
					thruster.ThrustOverridePercentage = ((forwardChange < 0) ? IDLE_POWER : (Guidance.Drain(ref forwardChange, thruster.MaxEffectiveThrust)));
					break;
				case Base6Directions.Direction.Backward:
					thruster.ThrustOverridePercentage = ((forwardChange > 0) ? IDLE_POWER : (Guidance.Drain(ref forwardChange, thruster.MaxEffectiveThrust)));
					break;
				case Base6Directions.Direction.Up:
					thruster.ThrustOverridePercentage = ((upChange < 0) ? IDLE_POWER : (Guidance.Drain(ref upChange, thruster.MaxEffectiveThrust)));
					break;
				case Base6Directions.Direction.Down:
					thruster.ThrustOverridePercentage = ((upChange > 0) ? IDLE_POWER : (Guidance.Drain(ref upChange, thruster.MaxEffectiveThrust)));
					break;
				case Base6Directions.Direction.Left:
					thruster.ThrustOverridePercentage = ((leftChange < 0) ? IDLE_POWER : (Guidance.Drain(ref leftChange, thruster.MaxEffectiveThrust)));
					break;
				case Base6Directions.Direction.Right:
					thruster.ThrustOverridePercentage = ((leftChange > 0) ? IDLE_POWER : (Guidance.Drain(ref leftChange, thruster.MaxEffectiveThrust)));
					break;
			}
		}
	}
	private static float Drain(ref float remainingPower, float maxEffectiveThrust) {
		applyPower = Math.Min(Math.Abs(remainingPower), maxEffectiveThrust);
		remainingPower = (remainingPower > 0) ? (remainingPower - applyPower) : (remainingPower + applyPower);
		return Math.Max(applyPower / maxEffectiveThrust, IDLE_POWER);
	}
}
public static class Navigation {
	public static List<Waypoint> waypoints = new List<Waypoint> { };
	public static string Status() {
		if(waypoints.Count == 0) {
			return "";
		}
		return waypoints[0].GetTypeMsg();
	}
	private static Vector3D? horizontDirectionNormal;
	private static Vector3D endTarget, endTargetPath, endTargetNormal, newDirection, newTarget, endTargetRightVector;
	private static void CheckColision() {
		switch(waypoints[0].type) {
			case Waypoint.wpType.CONVERGING:
				if((waypoints[0].stance.position - Situation.position).Length() < COLLISION_DISABLE_RADIUS_MULTIPLIER * Situation.radius) {
					return;
				}
				break;
			case Waypoint.wpType.NAVIGATING:
				break;
			default:
				return;
		}
		endTarget = waypoints[waypoints[0].type == Waypoint.wpType.NAVIGATING ? 1 : 0].stance.position;
		endTargetPath = endTarget - Situation.position;
		endTargetNormal = Vector3D.Normalize(endTargetPath);
		endTargetRightVector = Vector3D.Normalize(Vector3D.Cross(endTargetNormal, Situation.gravityUpVector));
		horizontDirectionNormal = Horizont.ScanHorizont((float)endTargetPath.Length(), endTargetNormal, endTargetRightVector);
		if(!horizontDirectionNormal.HasValue) {
			return;
		}
		if(Vector3D.IsZero(horizontDirectionNormal.Value)) {
			if(waypoints[0].type == Waypoint.wpType.NAVIGATING) {
				waypoints.RemoveAt(0);
			}
			return;
		}
		newDirection = Vector3D.Transform(horizontDirectionNormal.Value, Quaternion.CreateFromAxisAngle(endTargetRightVector, COLLISION_CORRECTION_ANGLE));
		newTarget = Situation.position + Math.Min(HORIZONT_CHECK_DISTANCE, (Situation.position - waypoints.Last().stance.position).Length()) * newDirection;

		if(waypoints[0].type == Waypoint.wpType.NAVIGATING) {
			waypoints[0].stance.position = newTarget;
		} else {
			waypoints.Insert(0, new Waypoint(new Stance(newTarget, Vector3D.Zero, Vector3D.Zero), MAX_SPEED, Waypoint.wpType.NAVIGATING));
		}
	}
	public static void Tick() {
		if(waypoints.Count == 0) {
			return;
		}
		Situation.RefreshParameters();
		CheckColision();
		Guidance.Set(waypoints.ElementAt(0));
		Guidance.Tick();
		if(Guidance.Done()) {
			waypoints.RemoveAt(0);
			if(waypoints.Count != 0) {
				return;
			}
			Guidance.Release();
		}
	}
	public static void AddWaypoint(Waypoint w) {
		waypoints.Insert(0, w);
	}
	public static void AddWaypoint(Stance s, float m, Waypoint.wpType wt) {
		AddWaypoint(new Waypoint(s, m, wt));
	}
	public static void AddWaypoint(Vector3D p, Vector3D f, Vector3D u, float m, Waypoint.wpType wt) {
		AddWaypoint(new Stance(p, f, u), m, wt);
	}
	public static void Stop() {
		Guidance.Release();
		waypoints.Clear();
	}
	public static bool Done() {
		return waypoints.Count == 0;
	}
}

public static class Commander {
	public static bool active = false;
	public static Dock currentDock;
	public enum Mode { SINGLE, LIST, LOOP };
	public static Mode mode = Mode.SINGLE;

	public static long idleStart = long.MaxValue;
	public static long waitTime = TimeSpan.FromSeconds(10.0).Ticks;

	public static void PilotDone() {
		idleStart = DateTime.Now.Ticks;
	}
	public static void Activate() {
		active = true;
		currentDock = null;

	}
	public static void Activate(Dock dock) {
		active = true;
		currentDock = dock;
		Logistics.Charge(false);
		Logistics.Dampeners(true);
	}
	public static void Deactivate() {
		active = false;
		currentDock = null;
	}

	private static bool chargeDone;
	private static bool cargoDone;
	public static void Tick() {
		if(!active || Pilot.running) {
			return;
		}
		if(mode == Mode.SINGLE) {
			Deactivate();
			return;
		}

		if(currentDock != null && currentDock.job == Dock.JobType.NONE) {
			if(DateTime.Now.Ticks - idleStart < waitTime) {
				return;
			}
		} else if(currentDock != null) {
			if(ConnectorControl.Connected() == null) {
				return;
			}

			cargoDone = true;
			switch(currentDock.job) {
				case Dock.JobType.LOAD:
				case Dock.JobType.CHARGE_LOAD:
					if(!Logistics.CargoFull()) {
						cargoDone = false;
					}
					break;
				case Dock.JobType.UNLOAD:
				case Dock.JobType.CHARGE_UNLOAD:
					if(!Logistics.CargoEmpty()) {
						cargoDone = false;
					}
					break;
			}

			chargeDone = true;
			switch(currentDock.job) {
				case Dock.JobType.CHARGE:
				case Dock.JobType.CHARGE_LOAD:
				case Dock.JobType.CHARGE_UNLOAD:
					Logistics.Charge(true);
					if(!Logistics.ChargeFull()) {
						chargeDone = false;
						return;
					}
					Logistics.Charge(false);
					break;
			}

			if(!cargoDone || !chargeDone) {
				return;
			}
		}

		DockData.NAVScreenHandle(Pannels.ScreenAction.Next);
		if(mode == Mode.LIST && DockData.selectedDockNAV == 0) {
			Deactivate();
			Logger.Info("Dock list finished, stopping autopilot.");
			return;
		}
		if(currentDock == null) {
			Logger.Info("Just a waypoint, navigating to next dock.");
		} else {
			switch(currentDock.job) {
				case Dock.JobType.NONE:
					Logger.Info("Wait time expired, resuming navigation.");
					break;
				case Dock.JobType.LOAD:
					Logger.Info("Cargo loaded, resuming navigation.");
					break;
				case Dock.JobType.UNLOAD:
					Logger.Info("Cargo unloaded, resuming navigation.");
					break;
				case Dock.JobType.CHARGE:
					Logger.Info("Charged, resuming navigation.");
					break;
				case Dock.JobType.CHARGE_LOAD:
					Logger.Info("Charged and cargo loaded, resuming navigation.");
					break;
				case Dock.JobType.CHARGE_UNLOAD:
					Logger.Info("Cargo unloaded, resuming navigation.");
					break;
			}
		}
		Pilot.Start();
	}

	private static string retStr;
	private static bool match;
	private static string myName;
	private static List<KeyValuePair<NavCmd, Dock>> found = new List<KeyValuePair<NavCmd, Dock>> { };
	private static List<NavCmd> notFound = new List<NavCmd> { };
	private static string command;
	public static void ProcessCmd(Program p, string cmd) {
		Serializer.InitUnpack(cmd);
		retStr = ExecuteCmd(Serializer.UnpackShipCommand());
		if(retStr == "") {
			return;
		}
		p.IGC.SendBroadcastMessage<string>(CMD_RES_TAG, retStr);
	}
	public static string ExecuteCmd(ShipCommand shipCommand) {
		if(!Block.GetProperty(GridBlocks.MasterProgrammableBlock.EntityId, "Name", ref myName)) {
			myName = GridBlocks.MasterProgrammableBlock.CubeGrid.CustomName;
		}
		if(shipCommand.ShipName != myName) {
			return "";
		}
		if(shipCommand.Command < 0 || shipCommand.Command > Terminal.COMMANDS.Count - 1) {
			return "Received a command that does not exist: " + shipCommand;
		}
		Logger.Info("Remote command received.");
		command = Terminal.COMMANDS[shipCommand.Command];
		if(command.Contains("start")) {
			Pilot.Start();
			return "Start success.";
		} else if(command.Contains("stop")) {
			Pilot.Stop();
			return "Stop success.";
		}

		found.Clear();
		notFound.Clear();
		foreach(NavCmd nc in shipCommand.navCmds) {
			match = false;
			foreach(Dock d in DockData.docks) {
				if(nc.Connector == d.blockName) {
					if(nc.Grid == "" || nc.Grid == d.gridName) {
						found.Add(new KeyValuePair<NavCmd, Dock>(nc, d));
						match = true;
						break;
					}
				}
			}
			if(!match) {
				notFound.Add(nc);
			}
		}
		if(notFound.Count != 0) {
			return "Not found: " + notFound.Count.ToString();
		}

		Pilot.Stop();
		DockData.selectedDocks.Clear();
		DockData.selectedDockNAV = 0;
		foreach(KeyValuePair<NavCmd, Dock> kp in found) {
			kp.Value.job = kp.Key.Action;
			DockData.selectedDocks.Add(kp.Value);
		}
		DockData.BalanceDisplays();
		if(command.Contains("step")) {
			SetMode(Mode.SINGLE);
		} else if(command.Contains("run")) {
			SetMode(Mode.LIST);
		} else if(command.Contains("loop")) {
			SetMode(Mode.LOOP);
		}

		if(!command.Contains("conf")) {
			Pilot.Start();
		}
		return "Command executed";
	}

	private static string newCustomName;
	private static void SetMode(Mode newMode) {
		mode = newMode;
		newCustomName = GridBlocks.MasterProgrammableBlock.CustomName;
		if(newMode == Mode.LIST) {
			newCustomName = newCustomName.Replace(" LOOP", "");
			newCustomName = newCustomName.Replace("[" + TAG, "[" + TAG + " LIST");
			Block.UpdateProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LIST", "");
			Block.RemoveProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LOOP");
		} else if(newMode == Mode.LOOP) {
			newCustomName = newCustomName.Replace(" LIST", "");
			newCustomName = newCustomName.Replace("[" + TAG, "[" + TAG + " LOOP");
			Block.UpdateProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LOOP", "");
			Block.RemoveProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LIST");
		} else {
			newCustomName = newCustomName.Replace(" LIST", "");
			newCustomName = newCustomName.Replace(" LOOP", "");
			Block.RemoveProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LOOP");
			Block.RemoveProperty(GridBlocks.MasterProgrammableBlock.EntityId, "LIST");
		}
		GridBlocks.MasterProgrammableBlock.CustomName = newCustomName;
	}
}

public static class Logistics {
	private static string valStr;
	private static float valFloat;

	public static void Dampeners(bool enable) {
		RemoteControl.block.DampenersOverride = enable;
	}
	private static List<IMyGasTank> tempTanks = new List<IMyGasTank>();
	public static bool CargoFull() {
		tempTanks.Clear();
		foreach(IMyGasTank tankBlock in GridBlocks.tankBlocks) {
			if(!Block.HasProperty(tankBlock.EntityId, "CARGO")) {
				continue;
			}
			tempTanks.Add(tankBlock);
			tankBlock.Stockpile = true;
		}
		foreach(IMyGasTank tankBlock in tempTanks) {
			valFloat = 95.0f;
			if(Block.GetProperty(tankBlock.EntityId, "Full", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 95.0f;
				}
			}
			if(tankBlock.FilledRatio < valFloat / 100.0f) {
				return false;
			}
		}
		foreach(IMyCargoContainer cargoBlock in GridBlocks.cargoBlocks) {
			valFloat = 90f;
			if(Block.GetProperty(cargoBlock.EntityId, "Full", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 90f;
				}
			}
			IMyInventory inventory = cargoBlock.GetInventory();
			if(inventory.CurrentVolume.RawValue < (inventory.MaxVolume.RawValue * valFloat / 100.0f)) {
				return false;
			}
		}
		return true;
	}
	public static bool CargoEmpty() {
		tempTanks.Clear();
		foreach(IMyGasTank tankBlock in GridBlocks.tankBlocks) {
			if(!Block.HasProperty(tankBlock.EntityId, "CARGO")) {
				continue;
			}
			tempTanks.Add(tankBlock);
			tankBlock.Stockpile = false;
		}
		foreach(IMyGasTank tankBlock in tempTanks) {
			valFloat = 0.0f;
			if(Block.GetProperty(tankBlock.EntityId, "Empty", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 0.0f;
				}
			}
			if(tankBlock.FilledRatio > valFloat / 100.0f) {
				return false;
			}
		}
		foreach(IMyCargoContainer cargoBlock in GridBlocks.cargoBlocks) {
			valFloat = 0.0f;
			if(Block.GetProperty(cargoBlock.EntityId, "Empty", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 0.0f;
				}
			}
			IMyInventory inventory = cargoBlock.GetInventory();
			if(inventory.CurrentVolume.RawValue > (inventory.MaxVolume.RawValue * valFloat / 100.0f)) {
				return false;
			}
		}
		return true;
	}
	public static bool ChargeFull() {
		foreach(IMyGasTank tankBlock in GridBlocks.tankBlocks) {
			if(Block.HasProperty(tankBlock.EntityId, "CARGO")) {
				continue;
			}
			valFloat = 95.0f;
			tankBlock.Stockpile = true;
			if(Block.GetProperty(tankBlock.EntityId, "Full", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 95.0f;
				}
			}
			if(tankBlock.FilledRatio < valFloat / 100.0f) {
				return false;
			}
		}
		foreach(IMyBatteryBlock batteryBlock in GridBlocks.batteryBlocks) {
			valFloat = 95f;
			if(Block.GetProperty(batteryBlock.EntityId, "Full", ref valStr)) {
				if(!float.TryParse(valStr, out valFloat)) {
					valFloat = 95f;
				}
			}
			if(batteryBlock.CurrentStoredPower < (batteryBlock.MaxStoredPower * valFloat / 100.0f)) {
				return false;
			}
		}
		Logger.Info("Everything is charged!");
		return true;
	}
	private static bool forceCharge;
	public static void Charge(bool enable) {
		foreach(IMyBatteryBlock batteryBlock in GridBlocks.batteryBlocks) {
			forceCharge = Block.HasProperty(batteryBlock.EntityId, "FORCE");
			batteryBlock.ChargeMode = enable && forceCharge ? ChargeMode.Recharge : ChargeMode.Auto;
		}
		foreach(IMyGasTank tankBlock in GridBlocks.tankBlocks) {
			if(Block.HasProperty(tankBlock.EntityId, "CARGO")) {
				continue;
			}
			forceCharge = Block.HasProperty(tankBlock.EntityId, "FORCE");
			tankBlock.Stockpile = enable && forceCharge;
		}
	}
}

public static class Pilot {
	public static bool running = false;
	public static List<Dock> dock = new List<Dock>();
	public static void Tick() {
		if(!running) {
			return;
		}
		if(!RemoteControl.Present()) {
			if(!ErrorState.Get(ErrorState.Type.NoRemoteController)) {
				Logger.Err(MSG_NO_REMOTE_CONTROL);
			}
			ErrorState.Set(ErrorState.Type.NoRemoteController);
			Stop();
			return;
		}
		if(Navigation.Done()) {
			if(dock.Count != 0 && dock[0].gridEntityId != 0) {
				CalculateApproach();
				dock.Clear();
				return;
			}
			Logger.Info(MSG_NAVIGATION_SUCCESSFUL);
			Signal.Send(Signal.SignalType.NAVIGATION);
			Commander.PilotDone();
			ConnectorControl.AttemptConnect();
			running = false;
			return;
		}
		Navigation.Tick();
	}
	private static Quaternion qInitialInverse, qFinal, qDiff;
	private static Vector3D connectorToCenter, rotatedConnectorToCenter, newUp, newForward, up, referenceUp, direction, balancedDirection;
	private static IMyShipConnector connector;
	private static bool revConnector;
	private static float connectorDistance;
	private static void CalculateApproach() {
		connector = ConnectorControl.GetConnector(dock[0]);
		if(connector == null) {
			Logger.Warn(MSG_NO_CONNECTORS_AVAILABLE);
			return;
		}
		Situation.RefreshParameters();
		connectorToCenter = Situation.position - connector.GetPosition();
		if(Math.Abs(Vector3D.Dot(dock[0].stance.forward, Situation.gravityUpVector)) < 0.5f) {
			up = Situation.gravityUpVector;
			referenceUp = connector.WorldMatrix.GetDirectionVector(connector.WorldMatrix.GetClosestDirection(up));
			referenceUp = (referenceUp == connector.WorldMatrix.Forward || referenceUp == connector.WorldMatrix.Backward) ? connector.WorldMatrix.Up : referenceUp;
		} else {
			up = dock[0].stance.up;
			referenceUp = connector.WorldMatrix.Up;
		}
		revConnector = Block.HasProperty(connector.EntityId, "REV");
		qInitialInverse = Quaternion.Inverse(Quaternion.CreateFromForwardUp(revConnector ? connector.WorldMatrix.Backward : connector.WorldMatrix.Forward, referenceUp));
		qFinal = Quaternion.CreateFromForwardUp(-dock[0].stance.forward, up);
		qDiff = qFinal * qInitialInverse;
		rotatedConnectorToCenter = Vector3D.Transform(connectorToCenter, qDiff);
		newForward = Vector3D.Transform(RemoteControl.block.WorldMatrix.Forward, qDiff);
		newUp = Vector3D.Transform(RemoteControl.block.WorldMatrix.Up, qDiff);
		connectorDistance = (dock[0].cubeSize == VRage.Game.MyCubeSize.Large) ? 2.6f / 2.0f : 0.5f;
		connectorDistance += (GridBlocks.MasterProgrammableBlock.CubeGrid.GridSizeEnum == VRage.Game.MyCubeSize.Large) ? 2.6f / 2.0f : 0.5f;
		newPos = dock[0].stance.position + rotatedConnectorToCenter + (connectorDistance * dock[0].stance.forward);
		Navigation.AddWaypoint(newPos, newForward, newUp, DOCKING_SPEED, Waypoint.wpType.DOCKING);
		newPos = dock[0].stance.position + rotatedConnectorToCenter + ((DOCK_DISTANCE + connectorDistance) * dock[0].stance.forward);
		Navigation.AddWaypoint(newPos, newForward, newUp, TAXIING_SPEED, Waypoint.wpType.APPROACHING);
		dock[0].approachPath.Reverse();
		foreach(VectorPath vp in dock[0].approachPath) {
			newPos = vp.position + (vp.direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
			Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, TAXIING_SPEED, Waypoint.wpType.TAXIING);
		}
		dock[0].approachPath.Reverse();
	}
	private static Vector3D newPos, undockPos;
	private static Dock disconnectDock;
	private static void SetEndStance(Dock dock) {
		Situation.RefreshParameters();
		if(dock.blockEntityId == 0) {
			Navigation.AddWaypoint(dock.stance, MAX_SPEED, Waypoint.wpType.ALIGNING);
			Navigation.AddWaypoint(dock.stance.position, Vector3D.Zero, Vector3D.Zero, MAX_SPEED, Waypoint.wpType.CONVERGING);
			newPos = dock.stance.position;
		} else {
			if(dock.approachPath.Count == 0) {
				newPos = dock.stance.position + ((APPROACH_DISTANCE + Situation.radius) * dock.stance.forward);
			} else {
				newPos = dock.approachPath[0].position + ((APPROACH_DISTANCE + Situation.radius) * dock.approachPath[0].direction);
			}
			Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, MAX_SPEED, Waypoint.wpType.CONVERGING);
		}
		if(Situation.linearVelocity.Length() >= 2.0f) {
			return;
		}
		disconnectDock = ConnectorControl.DisconnectAndTaxiData();
		direction = Vector3D.Normalize(newPos - Situation.position);
		balancedDirection = Vector3D.ProjectOnPlane(ref direction, ref Situation.gravityUpVector);
		if(disconnectDock == null) {
			Navigation.AddWaypoint(Situation.position, balancedDirection, Situation.gravityUpVector, MAX_SPEED, Waypoint.wpType.ALIGNING);
			return;
		}
		if(disconnectDock.approachPath.Count > 0) {
			foreach(VectorPath vp in disconnectDock.approachPath) {
				newPos = vp.position + (vp.direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
				Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, TAXIING_SPEED, Waypoint.wpType.TAXIING);
			}
		}
		undockPos = disconnectDock.stance.forward;
		undockPos *= (Situation.radius + UNDOCK_DISTANCE);
		undockPos += Situation.position;
		Navigation.AddWaypoint(undockPos, balancedDirection, Situation.gravityUpVector, DOCKING_SPEED, Waypoint.wpType.ALIGNING);
		Navigation.AddWaypoint(undockPos, Situation.forwardVector, Situation.upVector, DOCKING_SPEED, Waypoint.wpType.UNDOCKING);
	}
	public static void Start() {
		Start(DockData.GetSelected());
	}
	public static void Start(Dock d) {
		if(d == null) {
			return;
		}
		if(!RemoteControl.PresentOrLog()) {
			return;
		}
		Stop();
		Logger.Info(MSG_NAVIGATION_TO + "[" + d.gridName + "] " + d.blockName);
		dock.Add(d);
		SetEndStance(d);
		running = true;
		Commander.Activate(d);
		Signal.Send(Signal.SignalType.START);
	}
	public static void Start(Waypoint w) {
		if(!RemoteControl.PresentOrLog()) {
			return;
		}
		if(w.stance.position == Vector3D.Zero) {
			Logger.Err(MSG_INVALID_GPS_TYPE);
			return;
		}
		Stop();
		Logger.Info(MSG_NAVIGATION_TO_WAYPOINT);
		Navigation.AddWaypoint(w);
		running = true;
		Commander.Activate();
	}
	public static void Stop() {
		Navigation.Stop();
		dock.Clear();
		running = false;
		Commander.Deactivate();
	}
	public static void Toggle() {
		if(running) {
			Stop();
			return;
		}
		Start();
	}
}
private IMyBroadcastListener listener;
private IMyBroadcastListener cmdListener;
private IMyBroadcastListener cmdResListener;
private bool clearStorage = false;
public Program() {
	try {
		if(this.Load()) {
			Logger.Info("Loaded previous session");
		}
	} catch(Exception e) {
		Logger.Warn("Unable to load previous session: " + e.Message);
		Storage = "";
	}
	Runtime.UpdateFrequency = UpdateFrequency.Update100 | UpdateFrequency.Update10 | UpdateFrequency.Once;
	listener = IGC.RegisterBroadcastListener(TAG);
	listener.SetMessageCallback(TAG);
	cmdListener = IGC.RegisterBroadcastListener(CMD_TAG);
	cmdListener.SetMessageCallback(CMD_TAG);
	cmdResListener = IGC.RegisterBroadcastListener(CMD_RES_TAG);
	cmdResListener.SetMessageCallback(CMD_RES_TAG);
}
public bool Load() {
	if(Storage.Length != 0) {
		Logger.Info("Loading session size: " + Storage.Length);
		if(StorageData.Load(Storage)) {
			return true;
		}
		Logger.Warn("Unable to Load previous session due to different version");
	}
	return false;
}
public void Save() {
	string str = clearStorage ? "" : StorageData.Save();
	Logger.Info("Saving session size: " + str.Length);
	Storage = str;
}
public static class StorageData {
	public static string Save() {
		Serializer.InitPack();
		Serializer.Pack(STORAGE_VERSION);
		Serializer.Pack(DockData.currentDockCount);
		Serializer.Pack(DockData.docks);
		Serializer.Pack(Pilot.dock);
		List<int> selected = new List<int>();
		foreach(Dock d in DockData.selectedDocks) {
			selected.Add(DockData.docks.IndexOf(d));
		}
		Serializer.Pack(selected);
		Serializer.Pack(Horizont.angle);
		Serializer.Pack(Horizont.hit);
		Serializer.Pack(DockData.selectedDockNAV);
		Serializer.Pack(DockData.selectedDockCONF);
		Serializer.Pack(DockData.selectedTopNAV);
		Serializer.Pack(DockData.selectedTopCONF);
		Serializer.Pack(Pilot.running);
		Serializer.Pack(Navigation.waypoints);
		Serializer.Pack(Commander.active);
		if(Commander.active) {
			Serializer.Pack(Commander.currentDock);
		}
		Serializer.Pack(Commander.mode);
		return Serializer.serialized;
	}
	public static bool Load(string str) {
		Serializer.InitUnpack(str);
		if(STORAGE_VERSION != Serializer.UnpackString()) {
			return false;
		}
		DockData.currentDockCount = Serializer.UnpackInt();
		DockData.docks = Serializer.UnpackListDock();
		Pilot.dock = Serializer.UnpackListDock();
		List<int> selected = Serializer.UnpackListInt();
		DockData.selectedDocks.Clear();
		foreach(int i in selected) {
			DockData.selectedDocks.Add(DockData.docks[i]);
		}
		DockData.dynamic.Clear();
		foreach(Dock d in DockData.docks) {
			if(d.gridEntityId == 0) {
				continue;
			}
			DockData.dynamic[d.blockEntityId] = d;
			if(Pilot.dock.Count != 0 && Pilot.dock[0].blockEntityId == d.blockEntityId) {
				Pilot.dock.Clear();
				Pilot.dock.Add(d);
			}
		}
		Horizont.angle = Serializer.UnpackFloat();
		Horizont.hit = Serializer.UnpackBool();
		DockData.selectedDockNAV = Serializer.UnpackInt();
		DockData.selectedDockCONF = Serializer.UnpackInt();
		DockData.selectedTopNAV = Serializer.UnpackInt();
		DockData.selectedTopCONF = Serializer.UnpackInt();
		Pilot.running = Serializer.UnpackBool();
		Navigation.waypoints = Serializer.UnpackListWaypoint();
		if(Serializer.deserialized.Count == 0) {
			return true;
		}
		Commander.active = Serializer.UnpackBool();
		if(Commander.active) {
			Commander.currentDock = Serializer.UnpackDock();
		}
		Commander.mode = Serializer.UnpackCommanderMode();
		return true;
	}
}
public void HandleCommand(ref string command) {
	string[] parts = command.Trim().Split(' ');
	parts.DefaultIfEmpty("");
	string arg0 = parts.ElementAtOrDefault(0).ToUpper();
	string arg1 = parts.ElementAtOrDefault(1);
	arg1 = arg1 ?? "";
	try {
		switch(arg0) {
			case "PREV":
				Pannels.ScreenHandle(Pannels.ScreenAction.Prev);
				break;
			case "NEXT":
				Pannels.ScreenHandle(Pannels.ScreenAction.Next);
				break;
			case "SELECT":
				Pannels.ScreenHandle(Pannels.ScreenAction.Select);
				break;
			case "ADD":
				switch(arg1.ToUpper()) {
					case "STANCE":
						Pannels.ScreenHandle(Pannels.ScreenAction.AddStance, String.Join(" ", parts.Skip(2).ToArray()));
						break;
					case "ORBIT":
						Pannels.ScreenHandle(Pannels.ScreenAction.AddOrbit);
						break;
					default:
						Pannels.ScreenHandle(Pannels.ScreenAction.Add, String.Join(" ", parts.Skip(1).ToArray()));
						break;
				}
				break;
			case "REMOVE":
				Pannels.ScreenHandle(Pannels.ScreenAction.Rem);
				break;
			case "SCREEN":
				Pannels.NextScreen();
				break;
			case "START":
				switch(arg1.ToUpper()) {
					case "PREV":
						Signal.Clear();
						DockData.NAVScreenHandle(Pannels.ScreenAction.Prev);
						Pilot.Start();
						break;
					case "NEXT":
						Signal.Clear();
						DockData.NAVScreenHandle(Pannels.ScreenAction.Next);
						Pilot.Start();
						break;
					case "":
						Signal.Clear();
						Pilot.Start();
						break;
					default:
						Signal.Clear();
						Pilot.Start(Waypoint.FromString(String.Join(" ", parts.Skip(1).ToArray())));
						break;
				}
				break;
			case "GO":
				switch(arg1.ToUpper()) {
					case "":
						Logger.Err("There is no where to GO.");
						break;
					default:
						Signal.Clear();
						Pilot.Start(DockData.GetDock(String.Join(" ", parts.Skip(1).ToArray())));
						break;
				}
				break;
			case "TOGGLE":
				Pilot.Toggle();
				Signal.Clear();
				break;
			case "STOP":
				Pilot.Stop();
				Signal.Clear();
				break;
			case "SAVE":
				Save();
				break;
			case "LOAD":
				Load();
				break;
			case "CLEARLOG":
				Logger.Clear();
				break;
			case "CLEARSTORAGE":
				clearStorage = true;
				break;
			case "TEST":
				Signal.Send(Signal.SignalType.DOCK);
				break;
			default:
				Logger.Err("Unknown command ->" + arg0 + "<-");
				break;
		}
	} catch(Exception e) {
		Logger.Err("Command exception -< " + command + " >-< " + e.Message + " >-");
	}
}
public static class MainHelper {
	public delegate void Updater(ref string msg);
	public static void TimedRunIf(ref UpdateType update, UpdateType what, Updater run, ref string argument) {
		if((update & what) == 0) {
			return;
		}
		TimeStats.Start(what.ToString());
		run(ref argument);
		update &= ~what;
		TimeStats.Stop(what.ToString());
	}
	public static void TimedRunDefault(ref UpdateType update, Updater run, ref string argument) {
		TimedRunIf(ref update, update, run, ref argument);
	}
	public static void WriteStats(Program p) {
		string str = String.Format("SAM v{0}. . .{1}\n", VERSION, Animation.Rotator());
		str += TimeStats.Results();
		str += String.Format("Load:{0:F3}%\n", 100.0 * (double)p.Runtime.CurrentInstructionCount / (double)p.Runtime.MaxInstructionCount);
		p.Echo(str);
	}
}
public void Main(string argument, UpdateType updateSource) {
	try {
		MainHelper.TimedRunIf(ref updateSource, UpdateType.Once, this.Once, ref argument);
		MainHelper.TimedRunIf(ref updateSource, UpdateType.Update100, this.Update100, ref argument);
		MainHelper.TimedRunIf(ref updateSource, UpdateType.IGC, this.UpdateIGC, ref argument);
		MainHelper.TimedRunIf(ref updateSource, UpdateType.Update10, this.Update10, ref argument);
		MainHelper.TimedRunDefault(ref updateSource, this.HandleCommand, ref argument);
		MainHelper.WriteStats(this);
	} catch(Exception e) {
		Logger.Err("Main exception: " + e.Message);
		Echo("Main exception: " + e.Message);
	}
}
public void Once(ref string unused) {
	try {
		this.ScanGrid();
	} catch(Exception e) {
		Logger.Err("Once ScanGrid exception: " + e.Message);
	}
}
private MyIGCMessage igcData;
public void UpdateIGC(ref string msg) {
	if(msg == TAG) {
		while(listener.HasPendingMessage) {
			igcData = listener.AcceptMessage();
			try {
				DockSystem.Listen((string)igcData.Data);
			} catch(Exception e) {
				Logger.Err("Antenna Docks.Listen exception: " + e.Message);
			}
		}
	} else if(msg == CMD_TAG) {
		while(cmdListener.HasPendingMessage) {
			igcData = cmdListener.AcceptMessage();
			try {
				Commander.ProcessCmd(this, (string)igcData.Data);
			} catch(Exception e) {
				Logger.Err("Antenna Commander.ProcessCmd exception: " + e.Message);

			}
		}
	} else if(msg == CMD_RES_TAG) {
		while(cmdResListener.HasPendingMessage) {
			igcData = cmdResListener.AcceptMessage();
			try {
				Terminal.ProcessResponse((string)igcData.Data);
			} catch(Exception e) {
				Logger.Err("Antenna Terminal.ProcessResponse exception: " + e.Message);

			}
		}
	}
}
public void Update10(ref string unused) {
	try {
		Pannels.Print();
	} catch(Exception e) {
		Logger.Err("Update10 Pannels.Print exception: " + e.Message);
	}
	try {
		Animation.Run();
	} catch(Exception e) {
		Logger.Err("Update10 Animation.Run exception: " + e.Message);
	}
	try {
		Pilot.Tick();
	} catch(Exception e) {
		Logger.Err("Update10 Pilot.Tick exception: " + e.Message);
	}
	if(Me.CustomName.Contains("DEBUG")) {
		this.DebugPrintLogging();
	}
}
public void Update100(ref string unused) {
	try {
		this.ScanGrid();
	} catch(Exception e) {
		Logger.Err("Update100 ScanGrid exception: " + e.Message);
	}
	try {
		DockSystem.Advertise(this);
	} catch(Exception e) {
		Logger.Err("Update100 Docks.Advertise exception: " + e.Message);
	}
	try {
		ConnectorControl.CheckConnect();
	} catch(Exception e) {
		Logger.Err("Update100 ConnectorControl.CheckConnect exception: " + e.Message);
	}
	try {
		this.SendSignals();
	} catch(Exception e) {
		Logger.Err("Update100 SendSignals exception: " + e.Message);
	}
	try {
		Commander.Tick();
	} catch(Exception e) {
		Logger.Err("Update100 Commander.Tick exception: " + e.Message);
	}
	try {
		Terminal.Tick(this);
	} catch(Exception e) {
		Logger.Err("Update100 Terminal.Tick exception: " + e.Message);
	}
}
public static class DockData {
	public static int currentDockCount = 0;
	public static int selectedDockNAV = 0, selectedTopNAV = 0;
	public static int selectedDockCONF = 0, selectedTopCONF = 0;
	public static List<Dock> selectedDocks = new List<Dock>();
	public static List<Dock> docks = new List<Dock>();
	public static Dictionary<long, Dock> dynamic = new Dictionary<long, Dock>();
	public static Dock GetDock(long entityId) {
		for(int i = 0; i < docks.Count; i++) {
			if(docks[i].blockEntityId == entityId) {
				return docks[i];
			}
		}
		return null;
	}
	public static Dock GetDock(string dockName) {
		for(int i = 0; i < docks.Count; i++) {
			if(docks[i].blockName == dockName) {
				return docks[i];
			}
		}
		throw new Exception("Connector ->" + dockName + "<- was not found.");
	}
	public static Dock GetSelected() {
		if(selectedDocks.Count == 0) {
			return null;
		}
		return selectedDocks[selectedDockNAV];
	}
	private static void SelectNAV() {
		if(selectedDockNAV < 0 || selectedDockNAV >= selectedDocks.Count) {
			return;
		}
		selectedDocks[selectedDockNAV].NextJob();
	}
	private static void SelectCONF() {
		if(selectedDockCONF < 0 || selectedDockCONF >= docks.Count) {
			return;
		}
		dock = docks[selectedDockCONF];
		if(selectedDocks.Contains(dock)) {
			selectedDocks.Remove(dock);
		} else {
			selectedDocks.Add(dock);
		}
		BalanceDisplays();
	}
	private static void AddOrbit() {
		if(RemoteControl.block == null) {
			Logger.Err("No Remote Control");
			return;
		}
		Vector3D gravity = RemoteControl.block.GetNaturalGravity();
		if(gravity == Vector3D.Zero) {
			Logger.Err("No Gravity detected");
			return;
		}
		Vector3D pos = RemoteControl.block.CenterOfMass;
		Vector3D forward = RemoteControl.block.WorldMatrix.Forward;
		Vector3D up = RemoteControl.block.WorldMatrix.Up;
		Vector3D newPos = (-45000.0 * Vector3D.Normalize(gravity)) + pos + (1000.0 * forward);
		string dockName = "Orbit";

		Dock d = Dock.NewDock(newPos, forward, up, dockName);
		Logger.Pos("Orbit", ref newPos);
		// Logger.Info("Added new orbital GPS: " + GPS.);

		docks.Add(d);
		docks.Sort();
		BalanceDisplays();
	}

	private static void AddDock(bool stance, string param) {
		if(RemoteControl.block == null) {
			Logger.Err("No Remote Control");
			return;
		}
		Vector3D pos = RemoteControl.block.CenterOfMass;
		Vector3D forward = stance ? RemoteControl.block.WorldMatrix.Forward : Vector3D.Zero;
		Vector3D up = stance ? RemoteControl.block.WorldMatrix.Up : Vector3D.Zero;
		string dockName = Helper.FormatedWaypoint(stance, currentDockCount);
		currentDockCount++;
		bool addGPS = false;

		IMyShipConnector connector = ConnectorControl.OtherConnected();
		if(connector != null) {
			up = connector.WorldMatrix.Up;
			pos = connector.GetPosition();
			forward = Vector3D.Normalize(connector.OtherConnector.GetPosition() - connector.GetPosition());
			dockName = connector.CustomName.Trim();
		}

		if(param != "") {
			string tempDockName;
			GPS gps = new GPS(param);
			if(gps.valid) {
				connector = null;
				pos = gps.pos;
				tempDockName = gps.name;
				addGPS = true;
			} else {
				tempDockName = param;
			}
			tempDockName = tempDockName.Trim();
			if(tempDockName == "") {
				Logger.Err("Invalid Dock name");
			} else {
				dockName = tempDockName;
			}
		}
		Dock d = Dock.NewDock(pos, forward, up, dockName);
		Logger.Info("Added new " + (connector != null ? "Connected " : "") + (addGPS ? "GPS " : "") + "Dock: " + dockName);

		if(connector != null) {
			d.blockEntityId = connector.EntityId;
			d.cubeSize = connector.CubeGrid.GridSizeEnum;
			d.gridEntityId = connector.CubeGrid.EntityId;
			d.gridName = connector.CubeGrid.CustomName;
			d.Touch();
		}

		docks.Add(d);
		docks.Sort();
		BalanceDisplays();
	}
	private static void RemDock() {
		if(selectedDockCONF < 0 || selectedDockCONF >= docks.Count) {
			return;
		}
		dock = docks[selectedDockCONF];
		if(dock.gridEntityId != 0 && dock.Fresh()) {
			return;
		}
		selectedDocks.Remove(dock);
		docks.Remove(dock);
		dynamic.Remove(dock.blockEntityId);
		BalanceDisplays();
	}
	public static void BalanceDisplays() {
		if(selectedDockNAV < 0) {
			selectedDockNAV = selectedDocks.Count - 1;
		}
		if(selectedDockNAV >= selectedDocks.Count) {
			selectedDockNAV = 0;
		}
		if(selectedDockNAV < selectedTopNAV) {
			selectedTopNAV = selectedDockNAV;
		}
		if(selectedDockNAV >= selectedTopNAV + MAX_ENTRIES_NAV) {
			selectedTopNAV = selectedDockNAV - MAX_ENTRIES_NAV + 1;
		}
		if(selectedDockCONF < 0) {
			selectedDockCONF = docks.Count - 1;
		}
		if(selectedDockCONF >= docks.Count) {
			selectedDockCONF = 0;
		}
		if(selectedDockCONF < selectedTopCONF) {
			selectedTopCONF = selectedDockCONF;
		}
		if(selectedDockCONF >= selectedTopCONF + MAX_ENTRIES_CONF) {
			selectedTopCONF = selectedDockCONF - MAX_ENTRIES_CONF + 1;
		}
	}
	public static void NAVScreenHandle(Pannels.ScreenAction sa) {
		NAVScreenHandle(sa, "");
	}

	public static void NAVScreenHandle(Pannels.ScreenAction sa, string param) {
		switch(sa) {
			case Pannels.ScreenAction.Prev:
				--selectedDockNAV;
				break;
			case Pannels.ScreenAction.Next:
				++selectedDockNAV;
				break;
			case Pannels.ScreenAction.Select:
				SelectNAV();
				break;
			case Pannels.ScreenAction.Add:
				AddDock(false, param);
				break;
			case Pannels.ScreenAction.AddStance:
				AddDock(true, param);
				break;
			case Pannels.ScreenAction.Rem:
				break;
		}
		BalanceDisplays();
	}
	public static void CONFScreenHandle(Pannels.ScreenAction sa) {
		CONFScreenHandle(sa, "");
	}
	public static void CONFScreenHandle(Pannels.ScreenAction sa, string param) {
		switch(sa) {
			case Pannels.ScreenAction.Prev:
				--selectedDockCONF;
				break;
			case Pannels.ScreenAction.Next:
				++selectedDockCONF;
				break;
			case Pannels.ScreenAction.Select:
				SelectCONF();
				break;
			case Pannels.ScreenAction.Add:
				AddDock(false, param);
				break;
			case Pannels.ScreenAction.AddOrbit:
				AddOrbit();
				break;
			case Pannels.ScreenAction.AddStance:
				AddDock(true, param);
				break;
			case Pannels.ScreenAction.Rem:
				RemDock();
				break;
		}
		BalanceDisplays();
	}
	private static Dock dock;
	private static string str, status, cmdStatus;
	private static int index;
	private static string NAV_HEADER_ACTIVE = " Navigation " + new String('=', 43 - 12) + "\n";
	private static string NAV_HEADER_NOT_ACTIVE = " Navigation " + new String('-', 43 - 12) + "\n";
	private static int MAX_ENTRIES_NAV = 11;
	public static string PrintBufferNAV(bool active) {
		str = Animation.Rotator() + (active ? NAV_HEADER_ACTIVE : NAV_HEADER_NOT_ACTIVE);
		status = Navigation.Status();
		cmdStatus = Commander.active ? "waiting..." : "disabled";
		str += "   " + ((status == "" && !Pilot.running) ? cmdStatus : status) + "\n";
		if(selectedDocks.Count() == 0) {
			return str + "\n - No docks selected.\n   Use Configuration\n   screen to select\n   them.";
		}
		str += (selectedTopNAV > 0) ? "     /\\/\\/\\\n" : "     ------\n";

		for(int i = 0; i < selectedDocks.Count; ++i) {
			if(i < selectedTopNAV || i >= selectedTopNAV + MAX_ENTRIES_NAV) {
				continue;
			}
			dock = selectedDocks[i];
			str += ((selectedDockNAV == i) ? " >" : "  ") + (dock.Fresh() ? "" : "? ");
			if(dock.job != Dock.JobType.NONE) {
				str += "{" + dock.JobName() + "}";
			}
			str += "[" + dock.gridName + "] " + dock.blockName + "\n";
		}
		str += (selectedTopNAV + MAX_ENTRIES_NAV < selectedDocks.Count) ? "     \\/\\/\\/\n" : "     ------\n";
		return str;
	}
	private static string DCS_HEADER_ACTIVE = " Configuration " + new String('=', 43 - 15) + "\n";
	private static string DCS_HEADER_NOT_ACTIVE = " Configuration " + new String('-', 43 - 15) + "\n";
	private static int MAX_ENTRIES_CONF = 12;
	public static string PrintBufferCONF(bool active) {
		str = Animation.Rotator() + (active ? DCS_HEADER_ACTIVE : DCS_HEADER_NOT_ACTIVE);
		if(docks.Count() == 0) {
			return str + "\n - No available docks\n   to configure.";
		}
		str += (selectedTopCONF > 0) ? "     /\\/\\/\\\n" : "     ------\n";
		for(int i = 0; i < docks.Count; i++) {
			if(i < selectedTopCONF || i >= selectedTopCONF + MAX_ENTRIES_CONF) {
				continue;
			}
			dock = docks[i];
			index = selectedDocks.IndexOf(dock) + 1;
			str += (index != 0) ? index.ToString().PadLeft(2, ' ') : "  ";
			str += ((selectedDockCONF == i) ? " >" : "  ");
			str += (dock.Fresh() ? " " : " ? ") + "[" + dock.gridName + "] " + dock.blockName + "\n";
		}
		str += (selectedTopCONF + MAX_ENTRIES_CONF < docks.Count) ? "     \\/\\/\\/\n" : "     ------\n";
		return str;
	}
}
public static class DockSystem {
	private static string connectorName;
	private static string panelName;
	private static List<VectorPath> approachPath = new List<VectorPath>();
	private static string Serialize() {
		Serializer.InitPack();
		Serializer.Pack(ADVERT_ID);
		if(!Block.GetProperty(GridBlocks.MasterProgrammableBlock.EntityId, "Name", ref connectorName)) {
			connectorName = GridBlocks.MasterProgrammableBlock.CubeGrid.CustomName;
		}
		Serializer.Pack(GridBlocks.MasterProgrammableBlock.CubeGrid.EntityId);
		Serializer.Pack(connectorName);
		Serializer.Pack(GridBlocks.MasterProgrammableBlock.CubeGrid.GridSizeEnum);
		Serializer.Pack(GridBlocks.shipConnectors.Count());
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			Serializer.Pack(connector.EntityId);
			if(!Block.GetProperty(connector.EntityId, "Name", ref connectorName)) {
				connectorName = connector.CustomName.Trim();
			}
			Serializer.Pack(connectorName);
			Serializer.Pack(connector.GetPosition());
			if(Block.HasProperty(connector.EntityId, "REV")) {
				Serializer.Pack(connector.WorldMatrix.Backward);
			} else {
				Serializer.Pack(connector.WorldMatrix.Forward);
			}
			Serializer.Pack(connector.WorldMatrix.Up);
			approachPath.Clear();
			foreach(IMyTextPanel panel in GridBlocks.textPanels) {
				if(!Block.GetProperty(panel.EntityId, "Name", ref panelName)) {
					continue;
				}
				if(panelName != connectorName) {
					continue;
				}
				approachPath.Add(new VectorPath(panel.GetPosition(), -panel.WorldMatrix.Forward));
			}
			Serializer.Pack(approachPath);
		}
		return Serializer.serialized;
	}
	public static void Advertise(Program p) {
		if(!Block.HasProperty(GridBlocks.MasterProgrammableBlock.EntityId, "ADVERTISE")) {
			return;
		}
		if(GridBlocks.shipConnectors.Count() == 0) {
			return;
		}
		Serialize();
		p.IGC.SendBroadcastMessage<string>(TAG, Serializer.serialized);
	}
	private static long gridEntityId;
	private static long blockEntityId;
	private static string gridName;
	private static VRage.Game.MyCubeSize gridSize;
	private static int count;
	private static Dock dock;
	private static bool exists;
	private static string advertId;
	private static bool newAdvert;
	public static void Listen(string message) {
		Serializer.InitUnpack(message);
		advertId = Serializer.UnpackString();
		if(advertId == ADVERT_ID_VER) {
			newAdvert = true;
		}
		if(!newAdvert && advertId != ADVERT_ID) {
			return;
		}
		if(newAdvert) {
			// New version number.
			Serializer.UnpackLong();
		}
		gridEntityId = Serializer.UnpackLong();
		gridName = Serializer.UnpackString();
		if(!newAdvert) {
			gridSize = Serializer.UnpackCubeSize();
		}
		count = Serializer.UnpackInt();
		for(int i = 0; i < count; ++i) {
			blockEntityId = Serializer.UnpackLong();
			if(DockData.dynamic.ContainsKey(blockEntityId)) {
				dock = DockData.dynamic[blockEntityId];
				exists = true;
			} else {
				dock = new Dock();
				exists = false;
			}
			dock.Touch();
			dock.blockName = Serializer.UnpackString();
			dock.stance = new Stance(Serializer.UnpackVector3D(), Serializer.UnpackVector3D(), Serializer.UnpackVector3D());
			dock.lastSeen = DateTime.Now.Ticks;
			dock.blockEntityId = blockEntityId;
			dock.gridEntityId = gridEntityId;
			dock.gridName = gridName;
			if(newAdvert) {
				dock.cubeSize = Serializer.UnpackCubeSize();
			} else {
				dock.cubeSize = gridSize;
			}
			dock.approachPath = Serializer.UnpackListVectorPath();
			dock.SortApproachVectorsByDistance(dock.stance.position);
			if(!exists) {
				DockData.dynamic[blockEntityId] = dock;
				DockData.docks.Add(dock);
				DockData.docks.Sort();
			}
		}
	}
}
public static class CustomData {
	public static System.Text.RegularExpressions.Regex customDataRegex = new System.Text.RegularExpressions.Regex("\\s*" + TAG + "\\.([a-zA-Z0-9]*)([:=]{1}([\\S]*))?", System.Text.RegularExpressions.RegexOptions.IgnoreCase);
	private static System.Text.RegularExpressions.Match match;
	private static char[] lineSeparator = new char[] { '\n' };
	private static char[] attributeSeparator = new char[] { ':', '=' };
	private static string[] lines;
	private static string tagUpper;
	private static string attributeCap;
	private static string value;
	private static string build;
	private static bool exclusiveFound;
	private static long entityId;
	private static bool matched;
	private static string trim;
	public static bool Sanitize(ref IMyTerminalBlock block, ref BlockProfile profile) {
		lines = block.CustomData.Split(lineSeparator);
		build = "";
		exclusiveFound = false;
		matched = false;
		entityId = block.EntityId;
		foreach(string line in lines) {
			trim = line.Trim();
			if(trim == "") {
				continue;
			}
			match = customDataRegex.Match(trim);
			matched = match.Success || matched;
			if(match.Groups.Count == 4) {
				if(match.Groups[1].Value != "") {
					if(match.Groups[3].Value != "") {
						attributeCap = Helper.Capitalize(match.Groups[1].Value);
						if(profile.attributes.Contains(attributeCap)) {
							value = match.Groups[3].Value;
							build += TAG + "." + attributeCap + "=" + value + "\n";
							Block.UpdateProperty(entityId, attributeCap, value);
							continue;
						}
					} else {
						tagUpper = match.Groups[1].Value.ToUpper();
						if(profile.exclusiveTags.Contains(tagUpper)) {
							if(exclusiveFound) {
								build += trim + "\n";
								continue;
							}
							exclusiveFound = true;
						} else if(!profile.tags.Contains(tagUpper)) {
							build += trim + "\n";
							continue;
						}
						Block.UpdateProperty(entityId, tagUpper, "");
						build += TAG + "." + tagUpper + "\n";
						continue;
					}
				}
				build += TAG + ".\n";
				continue;
			} else {
				build += trim + "\n";
			}
		}
		if(matched) {
			block.CustomData = build;
		}
		return matched;
	}
}
public static class CustomName {
	private static char[] attributeSeparator = new char[] { ':', '=' };
	private static System.Text.RegularExpressions.Regex tagSimpleRegex = new System.Text.RegularExpressions.Regex("\\[(" + TAG + "[\\s\\S]*)\\]", System.Text.RegularExpressions.RegexOptions.IgnoreCase);
	private static string tagRegStr = TAG + "\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*(\\S*)\\s*";
	private static System.Text.RegularExpressions.Regex tagRegex = new System.Text.RegularExpressions.Regex(tagRegStr, System.Text.RegularExpressions.RegexOptions.IgnoreCase);
	private static System.Text.RegularExpressions.Match simpleMatch;
	private static System.Text.RegularExpressions.Match match;
	private static string build;
	private static string subTag;
	private static string subTagUpper;
	private static bool foundExclusive;
	private static string[] attributePair;
	private static string attributeCap;
	private static long entityId;
	public static bool Sanitize(ref IMyTerminalBlock block, ref BlockProfile profile) {
		simpleMatch = tagSimpleRegex.Match(block.CustomName);
		if(!simpleMatch.Success) {
			return false;
		}
		match = tagRegex.Match(simpleMatch.Groups[1].Value);
		if(!match.Success) {
			return false;
		}
		entityId = block.EntityId;
		foundExclusive = false;
		build = "[" + TAG;
		for(int i = 1; i < match.Groups.Count; ++i) {
			subTag = match.Groups[i].Value;
			if(subTag == "") {
				break;
			}
			subTagUpper = subTag.ToUpper();
			if(profile.exclusiveTags.Contains(subTagUpper)) {
				if(foundExclusive) {
					continue;
				}
				foundExclusive = true;
				build += " " + subTagUpper;
				Block.UpdateProperty(entityId, subTagUpper, "");
				continue;
			}
			if(profile.tags.Contains(subTagUpper)) {
				build += " " + subTagUpper;
				Block.UpdateProperty(entityId, subTagUpper, "");
				continue;
			}
			attributePair = subTag.Split(attributeSeparator);
			if(attributePair.Count() > 1) {
				attributeCap = Helper.Capitalize(attributePair[0]);
				if(profile.attributes.Contains(attributeCap)) {
					Block.UpdateProperty(entityId, attributeCap, attributePair[1]);
					build += " " + attributeCap + "=" + attributePair[1];
					continue;
				}
				build += " " + attributeCap.ToLower() + "=" + attributePair[1];
				continue;
			}
			build += " " + subTag.ToLower();
		}
		build += "]";
		block.CustomName = block.CustomName.Replace(simpleMatch.Groups[0].Value, build);
		return true;
	}
}
public static class Profiles {
	private static string[] empty = new string[] { };
	private static string[] ignoreTags = new string[] { "IGNORE" };
	private static string[] namedAttributes = new string[] { "Name" };

	private static string[] pbTags = new string[] { "DEBUG", "ADVERTISE", "NODAMPENERS" };
	private static string[] pbExclusiveTags = new string[] { "LIST", "LOOP" };
	private static string[] pbAttributes = new string[] { "Name", "Speed", "Wait", "ApproachDistance", "DockDistance", "UndockDistance", "DockingSpeed", "TaxiingSpeed", "MaxSpeed" };

	private static string[] textPanelTags = new string[] { "OVR" };
	private static string[] textPanelExclusiveTags = new string[] { "LOG", "NAV", "CONF", "DATA" };

	private static string[] cockpitTags = new string[] { "OVR" };
	public static string[] cockpitAttributes = new string[] { "Panel0", "Panel1", "Panel2", "Panel3", "Panel4", "Panel5", "Panel6", "Panel7", "Panel8", "Panel9" };

	private static string[] connectorTags = new string[] { "REV", "MAIN" };
	private static string[] timerTags = new string[] { "DOCKED", "NAVIGATED" };

	public static string[] chargeAttributes = new string[] { "Full", "Empty" };
	private static string[] chargeTags = new string[] { "FORCE" };

	private static string[] chargeCargoTags = new string[] { "FORCE", "CARGO" };

	public static BlockProfile me = new BlockProfile(ref pbTags, ref pbExclusiveTags, ref pbAttributes);
	public static Dictionary<Type, BlockProfile> perType = new Dictionary<Type, BlockProfile> {
{ typeof(IMyProgrammableBlock), me },
{ typeof(IMyRemoteControl), new BlockProfile(ref empty, ref empty, ref empty) },
{ typeof(IMyCameraBlock), new BlockProfile(ref empty, ref empty, ref empty) },
{ typeof(IMyRadioAntenna), new BlockProfile(ref empty, ref empty, ref empty) },
{ typeof(IMyLaserAntenna), new BlockProfile(ref empty, ref empty, ref empty) },
{ typeof(IMyShipConnector), new BlockProfile(ref connectorTags, ref empty, ref namedAttributes) },
{ typeof(IMyTextPanel), new BlockProfile(ref textPanelTags, ref textPanelExclusiveTags, ref namedAttributes) },
{ typeof(IMyCockpit), new BlockProfile(ref cockpitTags, ref empty, ref cockpitAttributes) },
{ typeof(IMyTimerBlock), new BlockProfile(ref timerTags, ref empty, ref empty) },
{ typeof(IMyBatteryBlock), new BlockProfile(ref chargeTags, ref empty, ref chargeAttributes) },
{ typeof(IMyGasTank), new BlockProfile(ref chargeCargoTags, ref empty, ref chargeAttributes) },
{ typeof(IMyCargoContainer), new BlockProfile(ref empty, ref empty, ref chargeAttributes) },
{ typeof(IMyThrust), new BlockProfile(ref ignoreTags, ref empty, ref empty) },
};
}
public static class Block {
	public static bool ValidType(ref IMyTerminalBlock block, Type type) {
		return ValidProfile(ref block, Profiles.perType[type]);
	}
	public static bool ValidProfile(ref IMyTerminalBlock block, BlockProfile profile) {
		bool customNameValid = CustomName.Sanitize(ref block, ref profile);
		bool customDataValid = CustomData.Sanitize(ref block, ref profile);
		return customNameValid || customDataValid;
	}
	private static Dictionary<long, Dictionary<string, string>> properties = new Dictionary<long, Dictionary<string, string>>();
	public static void UpdateProperty(long entityId, string property, string value) {
		if(properties.ContainsKey(entityId)) {
			properties[entityId][property] = value;
		} else {
			properties[entityId] = new Dictionary<string, string> { { property, value } };
		}
	}
	public static void ClearProperties() {
		foreach(KeyValuePair<long, Dictionary<string, string>> entities in properties) {
			entities.Value.Clear();
		}
	}
	public static bool HasProperty(long entityId, string name) {
		if(!properties.ContainsKey(entityId)) {
			return false;
		}
		if(!properties[entityId].ContainsKey(name)) {
			return false;
		}
		return true;
	}
	public static bool GetProperty(long entityId, string name, ref string value) {
		if(!HasProperty(entityId, name)) {
			return false;
		}
		value = properties[entityId][name];
		return true;
	}
	public static void RemoveProperty(long entityId, string name) {
		if(!properties.ContainsKey(entityId)) {
			return;
		}
		properties[entityId].Remove(name);
	}
}
public static class GridBlocks {
	public static IMyProgrammableBlock MasterProgrammableBlock;
	public static Dictionary<string, PairCounter> blockCount = new Dictionary<string, PairCounter>();
	public static List<IMyTerminalBlock> terminalBlocks = new List<IMyTerminalBlock>();
	public static List<IMyRemoteControl> remoteControls = new List<IMyRemoteControl>();
	public static List<IMyCameraBlock> cameraBlocks = new List<IMyCameraBlock>();
	public static List<IMyRadioAntenna> radioAntennas = new List<IMyRadioAntenna>();
	public static List<IMyLaserAntenna> laserAntennas = new List<IMyLaserAntenna>();
	public static List<IMyProgrammableBlock> programmableBlocks = new List<IMyProgrammableBlock>();
	public static List<IMyShipConnector> shipConnectors = new List<IMyShipConnector>();
	public static List<IMyTextPanel> textPanels = new List<IMyTextPanel>();
	public static List<IMyGyro> gyroBlocks = new List<IMyGyro>();
	public static List<IMyThrust> thrustBlocks = new List<IMyThrust>();
	public static List<IMyTimerBlock> timerBlocks = new List<IMyTimerBlock>();
	public static List<IMyCockpit> cockpitBlocks = new List<IMyCockpit>();
	public static List<IMyBatteryBlock> batteryBlocks = new List<IMyBatteryBlock>();
	public static List<IMyCargoContainer> cargoBlocks = new List<IMyCargoContainer>();
	public static List<IMyGasTank> tankBlocks = new List<IMyGasTank>();
	public static IMyTerminalBlock terminalBlock;
	public static IMyRemoteControl remoteControl;
	public static IMyCameraBlock cameraBlock;
	public static IMyRadioAntenna radioAntenna;
	public static IMyLaserAntenna laserAntenna;
	public static IMyProgrammableBlock programmableBlock;
	public static IMyShipConnector shipConnector;
	public static IMyTextPanel textPanel;
	public static IMyGyro gyroBlock;
	public static IMyThrust thrustBlock;
	public static IMyTimerBlock timerBlock;
	public static IMyCockpit cockpitBlock;
	public static IMyBatteryBlock batteryBlock;
	public static IMyCargoContainer cargoBlock;
	public static IMyGasTank tankBlock;
	public static void Clear() {
		foreach(string key in blockCount.Keys) {
			blockCount[key].Recount();
		}
		terminalBlocks.Clear();
		remoteControls.Clear();
		cameraBlocks.Clear();
		radioAntennas.Clear();
		laserAntennas.Clear();
		programmableBlocks.Clear();
		shipConnectors.Clear();
		textPanels.Clear();
		gyroBlocks.Clear();
		thrustBlocks.Clear();
		timerBlocks.Clear();
		cockpitBlocks.Clear();
		batteryBlocks.Clear();
		cargoBlocks.Clear();
		tankBlocks.Clear();
	}
	public static void UpdateCount(string key) {
		if(blockCount.ContainsKey(key)) {
			blockCount[key].newC++;
		} else {
			blockCount[key] = new PairCounter();
		}
	}
	public static void LogDifferences() {
		foreach(string key in blockCount.Keys) {
			var diff = blockCount[key].Diff();
			if(diff > 0) {
				Logger.Info(String.Format("Found {0}x {1}", diff, key));
			} else if(diff < 0) {
				Logger.Info(String.Format("Lost {0}x {1}", -diff, key));
			}
		}
	}
	public static bool AddBlock(IMyTerminalBlock block) {
		if((remoteControl = block as IMyRemoteControl) != null) {
			if(!Block.ValidType(ref block, typeof(IMyRemoteControl))) {
				return false;
			}
			remoteControls.Add(remoteControl);
		} else if((cameraBlock = block as IMyCameraBlock) != null) {
			if(!Block.ValidType(ref block, typeof(IMyCameraBlock))) {
				return false;
			}
			cameraBlocks.Add(cameraBlock);
		} else if((radioAntenna = block as IMyRadioAntenna) != null) {
			radioAntennas.Add(radioAntenna);
		} else if((laserAntenna = block as IMyLaserAntenna) != null) {
			laserAntennas.Add(laserAntenna);
		} else if((programmableBlock = block as IMyProgrammableBlock) != null) {
			if(!Block.ValidType(ref block, typeof(IMyProgrammableBlock))) {
				return false;
			}
			programmableBlocks.Add(programmableBlock);
		} else if((shipConnector = block as IMyShipConnector) != null) {
			if(!Block.ValidType(ref block, typeof(IMyShipConnector))) {
				return false;
			}
			shipConnectors.Add(shipConnector);
		} else if((textPanel = block as IMyTextPanel) != null) {
			if(!Block.ValidType(ref block, typeof(IMyTextPanel))) {
				return false;
			}
			textPanels.Add(textPanel);
		} else if((gyroBlock = block as IMyGyro) != null) {
			gyroBlocks.Add(gyroBlock);
		} else if((thrustBlock = block as IMyThrust) != null) {
			if(Block.ValidType(ref block, typeof(IMyThrust))) {
				if(Block.HasProperty(block.EntityId, "IGNORE")) {
					return false;
				}
			}
			thrustBlocks.Add(thrustBlock);
		} else if((timerBlock = block as IMyTimerBlock) != null) {
			if(!Block.ValidType(ref block, typeof(IMyTimerBlock))) {
				return false;
			}
			timerBlocks.Add(timerBlock);
		} else if((cockpitBlock = block as IMyCockpit) != null) {
			if(!Block.ValidType(ref block, typeof(IMyCockpit))) {
				return false;
			}
			cockpitBlocks.Add(cockpitBlock);
		} else if((batteryBlock = block as IMyBatteryBlock) != null) {
			if(!Block.ValidType(ref block, typeof(IMyBatteryBlock))) {
				return false;
			}
			batteryBlocks.Add(batteryBlock);
		} else if((cargoBlock = block as IMyCargoContainer) != null) {
			if(!Block.ValidType(ref block, typeof(IMyCargoContainer))) {
				return false;
			}
			cargoBlocks.Add(cargoBlock);
		} else if((tankBlock = block as IMyGasTank) != null) {
			if(!Block.ValidType(ref block, typeof(IMyGasTank))) {
				return false;
			}
			tankBlocks.Add(tankBlock);
		} else {
			return false;
		}
		return true;
	}
	private static string valStr;
	private static float valFloat;
	private static double valDouble;
	public static void AddMe(IMyProgrammableBlock me) {

		MasterProgrammableBlock = me;
		terminalBlock = me as IMyTerminalBlock;
		if(!Block.ValidProfile(ref terminalBlock, Profiles.me)) {
			me.CustomName += " [" + TAG + "]";
			return;
		}
		if(Block.GetProperty(terminalBlock.EntityId, "Speed", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(MAX_SPEED != valFloat) {
					MAX_SPEED = valFloat;
					Logger.Info("Maximum speed changed to " + MAX_SPEED);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "MaxSpeed", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(MAX_SPEED != valFloat) {
					MAX_SPEED = valFloat;
					Logger.Info("Maximum speed changed to " + MAX_SPEED);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "DockingSpeed", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(DOCKING_SPEED != valFloat) {
					DOCKING_SPEED = valFloat;
					Logger.Info("Docking speed changed to " + DOCKING_SPEED);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "TaxiingSpeed", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(TAXIING_SPEED != valFloat) {
					TAXIING_SPEED = valFloat;
					Logger.Info("Taxiing speed changed to " + TAXIING_SPEED);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "ApproachDistance", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(APPROACH_DISTANCE != (float)valFloat) {
					APPROACH_DISTANCE = (float)valFloat;
					Logger.Info("Approach distance changed to " + APPROACH_DISTANCE);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "DockDistance", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(DOCK_DISTANCE != (float)valFloat) {
					DOCK_DISTANCE = (float)valFloat;
					Logger.Info("Docking distance changed to " + DOCK_DISTANCE);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "UndockDistance", ref valStr)) {
			if(float.TryParse(valStr, out valFloat)) {
				if(UNDOCK_DISTANCE != (float)valFloat) {
					UNDOCK_DISTANCE = (float)valFloat;
					Logger.Info("Undocking distance changed to " + UNDOCK_DISTANCE);
				}
			}
		}
		if(Block.GetProperty(terminalBlock.EntityId, "Wait", ref valStr)) {
			if(Double.TryParse(valStr, out valDouble)) {
				Commander.waitTime = TimeSpan.FromSeconds(valDouble).Ticks;
			}
		}
		if(Block.HasProperty(terminalBlock.EntityId, "LIST")) {
			Commander.mode = Commander.Mode.LIST;
		} else if(Block.HasProperty(terminalBlock.EntityId, "LOOP")) {
			Commander.mode = Commander.Mode.LOOP;
		} else {
			Commander.mode = Commander.Mode.SINGLE;
			Commander.active = false;
		}
	}
	private static int xB, yB;
	private static int CompareThrusters(IMyThrust x, IMyThrust y) {
		xB = yB = 0;
		if(x.DefinitionDisplayNameText.Contains("Hydrogen ")) {
			xB += 4;
		} else if(x.DefinitionDisplayNameText.Contains("Ion ")) {
			xB += 2;
		}
		if(x.DefinitionDisplayNameText.Contains("Large ")) {
			xB += 1;
		}
		if(y.DefinitionDisplayNameText.Contains("Hydrogen ")) {
			yB += 4;
		} else if(y.DefinitionDisplayNameText.Contains("Ion ")) {
			yB += 2;
		}
		if(y.DefinitionDisplayNameText.Contains("Large ")) {
			yB += 1;
		}
		return xB - yB;
	}
	public static void EvaluateRemoteControls() {
		if(remoteControls.Count() == 1) {
			RemoteControl.block = remoteControls[0];
			ErrorState.Reset(ErrorState.Type.NoRemoteController);
			ErrorState.Reset(ErrorState.Type.TooManyControllers);
			return;
		};
		RemoteControl.block = null;
		if(!ErrorState.Get(ErrorState.Type.TooManyControllers) && remoteControls.Count() > 1) {
			ErrorState.Set(ErrorState.Type.TooManyControllers);
			Logger.Err("Too many remote controllers");
		}
	}
	public static void EvaluateCameraBlocks() {
		foreach(IMyCameraBlock cameraBlock in cameraBlocks) {
			if(!cameraBlock.EnableRaycast) {
				cameraBlock.EnableRaycast = true;
			}
		}
	}
	public static void EvaluateThrusters() {
		thrustBlocks.Sort(CompareThrusters);
	}
}
public void ScanGrid() {
	Block.ClearProperties();
	GridBlocks.Clear();
	GridBlocks.AddMe(Me);
	this.GridTerminalSystem.GetBlocks(GridBlocks.terminalBlocks);
	foreach(IMyTerminalBlock block in GridBlocks.terminalBlocks) {
		if(!block.IsSameConstructAs(Me)) {
			continue;
		}
		if(block.EntityId == Me.EntityId) {
			continue;
		}
		if(GridBlocks.AddBlock(block)) {
			GridBlocks.UpdateCount(block.DefinitionDisplayNameText);
		}
	}
	GridBlocks.EvaluateThrusters();
	GridBlocks.EvaluateCameraBlocks();
	GridBlocks.EvaluateRemoteControls();
	GridBlocks.LogDifferences();
}
public static class Signal {
	public enum SignalType { DOCK, NAVIGATION, START, UNDOCK };
	public static Dictionary<SignalType, int> active = new Dictionary<SignalType, int>();
	private static HashSet<SignalType> expire = new HashSet<SignalType> { };
	private static int SIGNAL_TRY = 10;

	public static void Send(SignalType signal) {
		active[signal] = SIGNAL_TRY;
	}
	public static void Age() {
		foreach(var pair in active) {
			expire.Add(pair.Key);
		}
		foreach(var signal in expire) {
			if(--active[signal] < 1) {
				active.Remove(signal);
			}
		}
		expire.Clear();
	}
	public static void Clear() {
		active.Clear();
	}
}
public void SendSignals() {
	if(Signal.active.Count == 0) {
		return;
	}
	foreach(IMyTimerBlock timer in GridBlocks.timerBlocks) {
		if(Block.HasProperty(timer.EntityId, "DOCKED") && Signal.active.ContainsKey(Signal.SignalType.DOCK)) {
			Signal.active[Signal.SignalType.DOCK] = 0;
			Logger.Info("Timer triggered due to Docking accomplished");
			timer.StartCountdown();
		}
		if(Block.HasProperty(timer.EntityId, "UNDOCKED") && Signal.active.ContainsKey(Signal.SignalType.UNDOCK)) {
			Signal.active[Signal.SignalType.UNDOCK] = 0;
			Logger.Info("Timer triggered due to Undocking sequence");
			timer.StartCountdown();
		}
		if(Block.HasProperty(timer.EntityId, "NAVIGATED") && Signal.active.ContainsKey(Signal.SignalType.NAVIGATION)) {
			Signal.active[Signal.SignalType.NAVIGATION] = 0;
			Logger.Info("Timer triggered due to Navigation finished");
			timer.StartCountdown();
		}
		if(Block.HasProperty(timer.EntityId, "STARTED") && Signal.active.ContainsKey(Signal.SignalType.START)) {
			Signal.active[Signal.SignalType.START] = 0;
			Logger.Info("Timer triggered due to Navigation started");
			timer.StartCountdown();
		}
	}
	Signal.Age();
}
public static class RemoteControl {
	public static IMyRemoteControl block = null;
	public static bool Present() {
		return block != null;
	}
	public static bool PresentOrLog() {
		if(Present()) {
			return true;
		}
		Logger.Err(MSG_NO_REMOTE_CONTROL);
		return false;
	}
}
public static class ConnectorControl {
	private static int connectAttempts = 0;
	public static void AttemptConnect() {
		connectAttempts = DOCK_ATTEMPTS;
		doConnect();
	}
	public static void CheckConnect() {
		if(connectAttempts == 0) {
			return;
		}
		if(0 == --connectAttempts) {
			Logger.Info(MSG_FAILED_TO_DOCK);
		} else {
			doConnect();
		}
	}
	private static void doConnect() {
		if(!Connect()) {
			return;
		}
		connectAttempts = 0;
		Logger.Info(MSG_DOCKING_SUCCESSFUL);
		Signal.Send(Signal.SignalType.DOCK);
		if(!Block.HasProperty(GridBlocks.MasterProgrammableBlock.EntityId, "NODAMPENERS")) {
			Logistics.Dampeners(false);
		}
	}
	public static IMyShipConnector OtherConnected() {
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			if(connector.Status == MyShipConnectorStatus.Connected) {
				return connector.OtherConnector;
			}
		}
		return null;
	}
	public static IMyShipConnector Connected() {
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			if(connector.Status == MyShipConnectorStatus.Connected) {
				return connector;
			}
		}
		return null;
	}
	private static bool connected;
	private static bool Connect() {
		connected = false;
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			connector.Connect();
			if(connector.Status == MyShipConnectorStatus.Connected) {
				connected = true;
			}
		}
		return connected;
	}
	private static Vector3D retractVector;
	public static Vector3D Disconnect() {
		retractVector = Vector3D.Zero;
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			if(connector.Status == MyShipConnectorStatus.Connected) {
				connector.Disconnect();
				retractVector = -connector.WorldMatrix.Forward;
			}
		}
		return retractVector;
	}
	private static Dock retractDock;
	public static Dock DisconnectAndTaxiData() {
		retractDock = null;
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			if(connector.Status == MyShipConnectorStatus.Connected) {
				var dock = DockData.GetDock(connector.OtherConnector.EntityId);
				if(dock != null) {
					retractDock = dock;
				} else {
					retractDock = Dock.NewDock(connector.OtherConnector.GetPosition(), connector.OtherConnector.WorldMatrix.Forward, connector.OtherConnector.WorldMatrix.Up, "D");
				}
				Signal.Send(Signal.SignalType.UNDOCK);
				connector.Disconnect();
			}
		}
		return retractDock;
	}
	private static bool revConnector;
	public static IMyShipConnector GetConnector(Dock refDock) {
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			if(Block.HasProperty(connector.EntityId, "MAIN")) {
				return connector;
			}
		}
		if(Math.Abs(Vector3D.Dot(refDock.stance.forward, RemoteControl.block.WorldMatrix.Up)) < 0.5f) {
			foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
				revConnector = Block.HasProperty(connector.EntityId, "REV");
				if(Math.Abs(Vector3D.Dot(revConnector ? connector.WorldMatrix.Backward : connector.WorldMatrix.Forward, RemoteControl.block.WorldMatrix.Up)) < 0.5f) {
					return connector;
				}
			}
		} else {
			foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
				revConnector = Block.HasProperty(connector.EntityId, "REV");
				if(Vector3D.Dot(revConnector ? connector.WorldMatrix.Backward : connector.WorldMatrix.Forward, -refDock.stance.forward) > 0.5f) {
					return connector;
				}
			}
		}
		foreach(IMyShipConnector connector in GridBlocks.shipConnectors) {
			return connector;
		}
		return null;
	}
}
public static class Terminal {
	public static List<string> COMMANDS = new List<string> { "step", "run", "loop", "step conf", "run conf", "loop conf", "start", "stop" };
	private static string CMD = "SAMv2 cmd# ";
	private static System.Text.RegularExpressions.Regex cmdRegStr = new System.Text.RegularExpressions.Regex("^" + CMD + "\\s*([\\S ]+)\\s*$");
	private static System.Text.RegularExpressions.Regex navRegStr = new System.Text.RegularExpressions.Regex("^(\\{(\\S+)\\}){0,1}(\\[(\\S+)\\]){0,1}(\\S+)$");
	private static IMyTextSurface screen;
	private static IMyTextSurface keyboard;
	private static string line;
	private static string[] cmd;
	private static int command;
	private static List<string> cleanLines = new List<string> { };
	private static System.Text.RegularExpressions.Match cmdMatch;
	private static System.Text.RegularExpressions.Match navMatch;
	private static string DefaultScreen = "SAMv2 " + VERSION + " Terminal\n please write your command in the keyboard.";
	private static string screenText = DefaultScreen;
	private static System.Text.StringBuilder buffer = new System.Text.StringBuilder();

	public static void Reset() {
		screen.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
		screen.WriteText(screenText);
		screen.FontSize = 0.6f;
		screen.Font = "Monospace";
		screen.FontColor = Color.Green;
		screen.BackgroundColor = Color.Black;
		screen.TextPadding = 0.0f;

		keyboard.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
		keyboard.WriteText(CMD);
		keyboard.FontSize = 2.0f;
		keyboard.Font = "Monospace";
		keyboard.FontColor = Color.Green;
		keyboard.BackgroundColor = Color.Black;
		keyboard.TextPadding = 0.0f;
	}

	public static void Tick(Program p) {
		if(GridBlocks.MasterProgrammableBlock == null) {
			return;
		}
		screen = GridBlocks.MasterProgrammableBlock.GetSurface(0);
		keyboard = GridBlocks.MasterProgrammableBlock.GetSurface(1);
		if(screen == null || keyboard == null) {
			return;
		}
		if(keyboard.ContentType != VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE) {
			Reset();
			return;
		}
		buffer.Clear();
		keyboard.ReadText(buffer);
		cmd = buffer.ToString().Split('\n');
		if(cmd.Length == 0) {
			Reset();
			return;
		}
		if(cmd[0] == CMD) {
			return;
		}
		cleanLines.Clear();
		foreach(string l in cmd) {
			line = l.Trim();
			if(line != "") {
				cleanLines.Add(line);
			}
		}
		cmd = cleanLines.ToArray();
		cmdMatch = cmdRegStr.Match(cmd[0]);
		if(!cmdMatch.Success) {
			screenText = "Invalid command. Please try again.";
			Reset();
			return;
		}
		command = COMMANDS.FindIndex(a => a.ToLower() == cmdMatch.Groups[1].Value);
		if(command == -1) {
			screenText = "Invalid command: " + cmdMatch.Groups[1].Value + "\n\nAvailable commands are:\n " + string.Join("\n  ", COMMANDS);
			Reset();
			return;
		}
		if(cmd.Length == 1) {
			screenText = "Command must be followed by the ship name.\nExample:\n loop\n ShipName";
			Reset();
			return;
		}
		shipCommand.ShipName = cmd[1];
		if(ParseNav(cmd.Skip(2).ToArray())) {
			SendCmd(p, command);
		}
		Reset();
	}
	private static ShipCommand shipCommand = new ShipCommand();
	private static Dock.JobType jobType;
	private static void SendCmd(Program p, int cmd) {
		shipCommand.Command = cmd;
		screenText = Commander.ExecuteCmd(shipCommand);
		if(screenText != "") {
			return;
		}
		Serializer.InitPack();
		Serializer.Pack(shipCommand);
		p.IGC.SendBroadcastMessage<string>(CMD_TAG, Serializer.serialized);
		screenText = "Command sent.\n Will only be successful if acknowledged...";
	}
	public static void ProcessResponse(string msg) {
		screenText = msg;
		Reset();
	}
	public static bool ParseNav(string[] lines) {
		shipCommand.navCmds.Clear();
		foreach(string navStr in lines) {
			navMatch = navRegStr.Match(navStr);
			if(!navMatch.Success) {
				screenText = "Invalid navigation format:\n" + navStr + "\nUse:\n {Action}[Grid]DockName\nor:\n {Action}DockName\nor:\n DockName";
				return false;
			}
			jobType = Dock.JobTypeFromName(navMatch.Groups[2].Value);
			if(jobType == Dock.JobType.NONE && navMatch.Groups[2].Value != "") {
				screenText = "Invalid Action:\n" + navMatch.Groups[2].Value + "\n\nUse one of:\n Charge,Charge&Load,Charge&Unload,\n Load,Unload;";
				return false;
			}
			shipCommand.navCmds.Add(new NavCmd(jobType, navMatch.Groups[4].Value, navMatch.Groups[5].Value));
		}
		return true;
	}
}

public static class Pannels {
	private static List<string> types = new List<string> { "NAV", "CONF", "LOG", "DATA" };
	private static Dictionary<string, string> buffer = new Dictionary<string, string>();
	private static Queue<string> selected = new Queue<string>(new List<string> { "NAV", "CONF" });
	private static string printBuffer, screen;
	private static void ResetBuffers() {
		foreach(string s in types) {
			buffer[s] = "";
		}
	}
	private static void FillPrintBuffer(string type) {
		printBuffer = buffer[type];
		if(printBuffer != "") {
			return;
		}
		screen = selected.Peek();
		switch(type) {
			case "LOG":
				printBuffer = Logger.PrintBuffer(screen == "LOG");
				break;
			case "CONF":
				printBuffer = DockData.PrintBufferCONF(screen == "CONF");
				break;
			case "NAV":
				printBuffer = DockData.PrintBufferNAV(screen == "NAV");
				break;
		}
	}
	public static void Print() {
		if(GridBlocks.textPanels.Count() == 0 && GridBlocks.cockpitBlocks.Count() == 0) {
			return;
		}
		ResetBuffers();
		foreach(IMyTextPanel panel in GridBlocks.textPanels) {
			if(Block.HasProperty(panel.EntityId, "Name")) {
				break;
			}
			printBuffer = "";
			foreach(string type in types) {
				if(Block.HasProperty(panel.EntityId, type)) {
					FillPrintBuffer(type);
					break;
				}
			}
			if(printBuffer == "") {
				FillPrintBuffer(selected.Peek());
			}
			panel.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
			if(!Block.HasProperty(panel.EntityId, "OVR")) {
				panel.FontSize = 1.180f;
				panel.Font = "Monospace";
				panel.TextPadding = 0.0f;
			}
			panel.WriteText(printBuffer);
		}
		FillPrintBuffer(selected.Peek());
		foreach(IMyCockpit cockpit in GridBlocks.cockpitBlocks) {
			string screen = "";
			for(int panel = 0; panel < cockpit.SurfaceCount && panel < Profiles.cockpitAttributes.Length; ++panel) {
				if(!Block.GetProperty(cockpit.EntityId, Profiles.cockpitAttributes[panel], ref screen)) {
					continue;
				}
				IMyTextSurface surface = cockpit.GetSurface(panel);
				surface.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
				switch(screen.ToUpper()) {
					case "LOG":
						FillPrintBuffer("LOG");
						break;
					case "NAV":
						FillPrintBuffer("NAV");
						break;
					case "CONF":
						FillPrintBuffer("CONF");
						break;
					default:
						FillPrintBuffer(selected.Peek());
						break;
				}
				surface.WriteText(printBuffer);
				if(!Block.HasProperty(cockpit.EntityId, "OVR")) {
					surface.FontSize = 1.180f;
					surface.Font = "Monospace";
					surface.TextPadding = 0.0f;
				}
			}
		}
	}
	public static void NextScreen() {
		selected.Enqueue(selected.Dequeue());
	}
	public enum ScreenAction {
		Prev, Next, Select, Add, Rem, AddStance, AddOrbit
	};

	public static void ScreenHandle(ScreenAction sa) {
		ScreenHandle(sa, "");
	}

	public static void ScreenHandle(ScreenAction sa, string param) {
		switch(selected.Peek()) {
			case "NAV":
				DockData.NAVScreenHandle(sa, param);
				break;
			case "CONF":
				DockData.CONFScreenHandle(sa, param);
				break;
			case "LOG":
				break;
		}
	}
}
public static class Logger {
	private static List<string> logger = new List<string>();
	private static string str;
	private static string HEADER_ACTIVE = " SAM v" + Program.VERSION + " Logger " + new String('=', 43 - 14 - Program.VERSION.Length);
	private static string HEADER_NOT_ACTIVE = " SAM v" + Program.VERSION + " Logger " + new String('-', 43 - 14 - Program.VERSION.Length);
	public static void Log(string line) {
		logger.Insert(0, line);
		if(logger.Count() > LOG_MAX_LINES) {
			logger.RemoveAt(logger.Count() - 1);
		}
	}
	public static void Clear() {
		logger.Clear();
	}
	public static void Info(string line) {
		Log("I: " + line);
	}
	public static void Warn(string line) {
		Log("W: " + line);
	}
	public static void Err(string line) {
		Log("E: " + line);
	}
	public static void Pos(string where, ref Vector3D pos) {
		Log("GPS:" + where + ":" + pos.X.ToString("F2") + ":" + pos.Y.ToString("F2") + ":" + pos.Z.ToString("F2") + ":");
	}
	public static string PrintBuffer(bool active) {
		str = Animation.Rotator() + (active ? HEADER_ACTIVE : HEADER_NOT_ACTIVE);
		foreach(string line in logger) {
			str += "\n " + line;
		}
		return str;
	}
}
public void DebugPrintLogging() {
	List<IMyTextPanel> blocks = new List<IMyTextPanel>();
	GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(blocks);
	if(blocks.Count() == 0) {
		return;
	}
	Animation.DebugRun();
	var str = Logger.PrintBuffer(false);
	foreach(IMyTextPanel panel in blocks) {
		if(!panel.CustomName.Contains("LOG")) {
			continue;
		}
		panel.FontSize = 1.180f;
		panel.Font = "Monospace";
		panel.TextPadding = 0.0f;
		panel.WriteText(str);
	}
}
public static class Animation {
	private static string[] ROTATOR = new string[] { "|", "/", "-", "\\" };
	private static int rotatorCount = 0;
	private static int debugRotatorCount = 0;
	public static void Run() {
		if(++rotatorCount > ROTATOR.Length - 1) {
			rotatorCount = 0;
		}
	}
	public static string Rotator() {
		return ROTATOR[rotatorCount];
	}
	public static void DebugRun() {
		if(++debugRotatorCount > ROTATOR.Length - 1) {
			debugRotatorCount = 0;
		}
	}
	public static string DebugRotator() {
		return ROTATOR[debugRotatorCount];
	}
}
public static class TimeStats {
	public static Dictionary<string, DateTime> start = new Dictionary<string, DateTime>();
	public static Dictionary<string, TimeSpan> stats = new Dictionary<string, TimeSpan>();
	public static void Start(string key) {
		start[key] = DateTime.Now;
	}
	public static void Stop(string key) {
		stats[key] = DateTime.Now - start[key];
	}
	public static string Results() {
		string str = "";
		foreach(KeyValuePair<string, TimeSpan> stat in stats) {
			str += String.Format("{0}:{1:F4}ms\n", stat.Key, stat.Value.TotalMilliseconds);
		}
		return str;
	}
}
public static class Serializer {
	public static string[] separator = new string[] { "\n" };
	public static string serialized;
	public static Queue<string> deserialized;
	public static void InitPack() {
		serialized = "";
	}
	public static void Pack(string str) {
		serialized += str.Replace(separator[0], " ") + separator[0];
	}
	public static void Pack(int val) {
		serialized += val.ToString() + separator[0];
	}
	public static void Pack(long val) {
		serialized += val.ToString() + separator[0];
	}
	public static void Pack(float val) {
		serialized += val.ToString() + separator[0];
	}
	public static void Pack(double val) {
		serialized += val.ToString() + separator[0];
	}
	public static void Pack(bool val) {
		serialized += (val ? "1" : "0") + separator[0];
	}
	public static void Pack(VRage.Game.MyCubeSize val) {
		Pack((int)val);
	}
	public static void Pack(Dock.JobType val) {
		Pack((int)val);
	}
	public static void Pack(Vector3D val) {
		Pack(val.X);
		Pack(val.Y);
		Pack(val.Z);
	}
	public static void Pack(List<Vector3D> val) {
		Pack(val.Count);
		foreach(Vector3D v in val) {
			Pack(v);
		}
	}
	public static void Pack(List<int> val) {
		Pack(val.Count);
		foreach(int v in val) {
			Pack(v);
		}
	}
	public static void Pack(Stance val) {
		Pack(val.position);
		Pack(val.forward);
		Pack(val.up);
	}
	public static void Pack(Dock val) {
		Pack(val.stance);
		Pack(val.approachPath);
		Pack(val.cubeSize);
		Pack(val.gridEntityId);
		Pack(val.gridName);
		Pack(val.blockEntityId);
		Pack(val.blockName);
		Pack(val.lastSeen);
		Pack(val.job);
	}
	public static void Pack(List<Dock> val) {
		Pack(val.Count);
		foreach(Dock v in val) {
			Pack(v);
		}
	}
	public static void Pack(Waypoint val) {
		Pack(val.stance);
		Pack(val.maxSpeed);
		Pack((int)val.type);
	}
	public static void Pack(List<Waypoint> val) {
		Pack(val.Count);
		foreach(Waypoint v in val) {
			Pack(v);
		}
	}
	public static void Pack(VectorPath val) {
		Pack(val.position);
		Pack(val.direction);
	}
	public static void Pack(List<VectorPath> val) {
		Pack(val.Count);
		foreach(VectorPath v in val) {
			Pack(v);
		}
	}
	public static void Pack(NavCmd val) {
		Pack(val.Action);
		Pack(val.Grid);
		Pack(val.Connector);
	}
	public static void Pack(List<NavCmd> val) {
		Pack(val.Count);
		foreach(NavCmd v in val) {
			Pack(v);
		}
	}
	public static void Pack(ShipCommand val) {
		Pack(val.Command);
		Pack(val.ShipName);
		Pack(val.navCmds);
	}
	public static void Pack(Commander.Mode val) {
		Pack((int)val);
	}
	public static void InitUnpack(string str) {
		deserialized = new Queue<string>(str.Split(separator, StringSplitOptions.None));
	}
	public static string UnpackString() {
		return deserialized.Dequeue();
	}
	public static int UnpackInt() {
		return int.Parse(deserialized.Dequeue());
	}
	public static long UnpackLong() {
		return long.Parse(deserialized.Dequeue());
	}
	public static float UnpackFloat() {
		return float.Parse(deserialized.Dequeue());
	}
	public static double UnpackDouble() {
		return double.Parse(deserialized.Dequeue());
	}
	public static bool UnpackBool() {
		return deserialized.Dequeue() == "1";
	}
	public static VRage.Game.MyCubeSize UnpackCubeSize() {
		return (VRage.Game.MyCubeSize)UnpackInt();
	}
	public static Dock.JobType UnpackDockJobType() {
		return (Dock.JobType)UnpackInt();
	}
	public static Vector3D UnpackVector3D() {
		return new Vector3D(UnpackDouble(), UnpackDouble(), UnpackDouble());
	}
	public static List<Vector3D> UnpackListVector3D() {
		List<Vector3D> val = new List<Vector3D>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackVector3D());
		}
		return val;
	}
	public static List<int> UnpackListInt() {
		List<int> val = new List<int>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackInt());
		}
		return val;
	}
	public static Stance UnpackStance() {
		return new Stance(UnpackVector3D(), UnpackVector3D(), UnpackVector3D());
	}
	public static Dock UnpackDock() {
		Dock val = new Dock();
		val.stance = UnpackStance();
		val.approachPath = UnpackListVectorPath();
		val.cubeSize = UnpackCubeSize();
		val.gridEntityId = UnpackLong();
		val.gridName = UnpackString();
		val.blockEntityId = UnpackLong();
		val.blockName = UnpackString();
		val.lastSeen = UnpackLong();
		val.job = UnpackDockJobType();
		return val;
	}
	public static List<Dock> UnpackListDock() {
		List<Dock> val = new List<Dock>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackDock());
		}
		return val;
	}
	public static Waypoint UnpackWaypoint() {
		return new Waypoint(UnpackStance(), UnpackFloat(), (Waypoint.wpType)UnpackInt());
	}
	public static List<Waypoint> UnpackListWaypoint() {
		List<Waypoint> val = new List<Waypoint>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackWaypoint());
		}
		return val;
	}
	public static VectorPath UnpackVectorPath() {
		return new VectorPath(UnpackVector3D(), UnpackVector3D());
	}
	public static List<VectorPath> UnpackListVectorPath() {
		List<VectorPath> val = new List<VectorPath>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackVectorPath());
		}
		return val;
	}
	public static NavCmd UnpackNavCmd() {
		return new NavCmd((Dock.JobType)UnpackInt(), UnpackString(), UnpackString());
	}
	public static List<NavCmd> UnpackListNavCmd() {
		List<NavCmd> val = new List<NavCmd>();
		int count = UnpackInt();
		for(int i = 0; i < count; i++) {
			val.Add(UnpackNavCmd());
		}
		return val;
	}
	public static ShipCommand UnpackShipCommand() {
		ShipCommand sc = new ShipCommand();
		sc.Command = UnpackInt();
		sc.ShipName = UnpackString();
		sc.navCmds = UnpackListNavCmd();
		return sc;
	}
	public static Commander.Mode UnpackCommanderMode() {
		return (Commander.Mode)UnpackInt();
	}
}
public static class Helper {
	public static string FormatedWaypoint(bool stance, int pos) {
		return (stance ? "Ori " : "Pos ") + (++pos).ToString("D2");
	}
	public static string Capitalize(string s) {
		if(string.IsNullOrEmpty(s)) {
			return string.Empty;
		}
		return s.First().ToString().ToUpper() + s.Substring(1).ToLower();
	}
	public static Vector3D UnserializeVector(string str) {
		var parts = str.Split(':');
		Vector3D v = Vector3D.Zero;
		if(parts.Length != 6) {
			return v;
		}
		try {
			v = new Vector3D(double.Parse(parts[2]), double.Parse(parts[3]), double.Parse(parts[4]));
		} catch {
		}
		return v;
	}
}
public class GPS {
	public string name;
	public Vector3D pos;
	public bool valid = false;
	public GPS(string gps) {
		var parts = gps.Split(':');
		if(parts.Length != 6) {
			return;
		}
		try {
			name = parts[1];
			pos = new Vector3D(double.Parse(parts[2]), double.Parse(parts[3]), double.Parse(parts[4]));
		} catch {
			return;
		}
		valid = true;
	}
}
public static class ErrorState {
	public enum Type {
		TooManyControllers, NoRemoteController
	};
	private static Dictionary<Type, bool> errorState = new Dictionary<Type, bool> { };
	public static void Set(Type type) {
		errorState[type] = true;
	}
	public static void Reset(Type type) {
		errorState[type] = false;
	}
	public static bool Get(Type type) {
		if(!errorState.ContainsKey(type)) { return false; }
		return errorState[type];
	}
}
public class Stance {
	public Vector3D position;
	public Vector3D forward;
	public Vector3D up;
	public Stance(Vector3D p, Vector3D f, Vector3D u) {
		this.position = p;
		this.forward = f;
		this.up = u;
	}
}
public class VectorPath {
	public Vector3D position;
	public Vector3D direction;
	public VectorPath(Vector3D p, Vector3D d) {
		this.position = p;
		this.direction = d;
	}
}
public class Waypoint {
	public Stance stance;
	public float maxSpeed;
	public enum wpType {
		ALIGNING, DOCKING, UNDOCKING, CONVERGING, APPROACHING, NAVIGATING, TESTING, TAXIING
	};
	public wpType type;
	public Waypoint(Stance s, float m, wpType wt) {
		stance = s;
		maxSpeed = m;
		type = wt;
	}
	public string GetTypeMsg() {
		switch(this.type) {
			case wpType.ALIGNING:
				return MSG_ALIGNING;
			case wpType.DOCKING:
				return MSG_DOCKING;
			case wpType.UNDOCKING:
				return MSG_UNDOCKING;
			case wpType.CONVERGING:
				return MSG_CONVERGING;
			case wpType.APPROACHING:
				return MSG_APPROACHING;
			case wpType.NAVIGATING:
				return MSG_NAVIGATING;
			case wpType.TAXIING:
				return MSG_TAXIING;
		}
		return "Testing...";
	}
	public static Waypoint FromString(string coordinates) {
		return new Waypoint(new Stance(Helper.UnserializeVector(coordinates), Vector3D.Zero, Vector3D.Zero), MAX_SPEED, wpType.CONVERGING);
	}
}
public class PairCounter {
	public int oldC;
	public int newC;
	public PairCounter() {
		this.oldC = 0;
		this.newC = 1;
	}
	public void Recount() {
		this.oldC = this.newC;
		this.newC = 0;
	}
	public int Diff() {
		return newC - oldC;
	}
}
public class BlockProfile {
	public string[] tags;
	public string[] exclusiveTags;
	public string[] attributes;
	public BlockProfile(ref string[] tags, ref string[] exclusiveTags, ref string[] attributes) {
		this.tags = tags;
		this.exclusiveTags = exclusiveTags;
		this.attributes = attributes;
	}
}
public class Dock:IComparable<Dock> {
	private static long STALE_TIME = TimeSpan.FromSeconds(60.0).Ticks;
	public static Dock NewDock(Vector3D p, Vector3D f, Vector3D u, string name) {
		Dock d = new Dock();
		d.stance = new Stance(p, f, u);
		d.gridName = "Manual";
		d.blockName = name;
		return d;
	}
	public Stance stance;
	public List<VectorPath> approachPath = new List<VectorPath>();
	public VRage.Game.MyCubeSize cubeSize;
	public long gridEntityId = 0;
	public string gridName = "";
	public long blockEntityId = 0;
	public string blockName = "";
	public long lastSeen = 0;

	public enum JobType { NONE, CHARGE, LOAD, UNLOAD, CHARGE_LOAD, CHARGE_UNLOAD };
	public JobType job = JobType.NONE;

	public void NextJob() {
		int i = (int)job;
		if(++i == Enum.GetNames(typeof(JobType)).Length) {
			i = 0;
		}
		job = (JobType)i;
	}
	public string JobName() {
		switch(job) {
			case JobType.NONE:
				return "None";
			case JobType.CHARGE:
				return "Charge";
			case JobType.LOAD:
				return "Load";
			case JobType.UNLOAD:
				return "Unload";
			case JobType.CHARGE_LOAD:
				return "Charge&Load";
			case JobType.CHARGE_UNLOAD:
				return "Charge&Unload";
		}
		return "";
	}
	public static JobType JobTypeFromName(string name) {
		switch(name.ToLower()) {
			case "charge":
				return JobType.CHARGE;
			case "load":
				return JobType.LOAD;
			case "unload":
				return JobType.UNLOAD;
			case "charge&load":
				return JobType.CHARGE_LOAD;
			case "charge&unload":
				return JobType.CHARGE_UNLOAD;
		}
		return JobType.NONE;
	}

	public int CompareTo(Dock other) {
		if(this.gridEntityId != other.gridEntityId) {
			return (other.gridEntityId < this.gridEntityId) ? 1 : -1;
		}
		if(this.blockEntityId != other.blockEntityId) {
			return (other.blockEntityId < this.blockEntityId) ? 1 : -1;
		}
		return this.blockName.CompareTo(other.blockName);
	}
	public void SortApproachVectorsByDistance(Vector3D from) {
		approachPath.Sort(delegate (VectorPath a, VectorPath b) {
			return (int)(Vector3D.Distance(from, b.position) - Vector3D.Distance(from, a.position));
		});
	}
	public void Touch() {
		this.lastSeen = DateTime.Now.Ticks;
	}
	public bool Fresh() {
		if(lastSeen == 0) {
			return true;
		}
		return (DateTime.Now.Ticks - lastSeen) < STALE_TIME;
	}
}
public class NavCmd {
	public Dock.JobType Action;
	public string Grid;
	public string Connector;
	public NavCmd(Dock.JobType action, string grid, string connector) {
		this.Action = action;
		this.Grid = grid;
		this.Connector = connector;
	}
}
public class ShipCommand {
	public int Command;
	public string ShipName;
	public List<NavCmd> navCmds = new List<NavCmd> { };
}
