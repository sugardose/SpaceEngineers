/*
 * * vMod 9.4.2:
 * * - Now outputs to cockpits even if there's no LCD panels assigned.
 * * - No longer touches custom data that doesn't have SAM in it.
 * * vMod 9.4.3:
 * * - Moved the connector check to be after Navigation.tick() in order to fix a thruster override bug (if still occurs, pass cmd "stop" to work around this)
 * * - (Dev note: thruster/gyro override release when script forcefully connects with connector)
 * * vMod 9.4.4:
 * * - Returned pathLen back to [pathLen <= 0.1f] under Guidance for better docking accuracy
 * * - Changed aligning algorithm in Navigation to attempt better aligning when aligning directly
 * * vMod 9.4.5:
 * * - Scrapped the new aligning algorithm
 * * vMod 9.4.6:
 * * - Now scans blocks on the same grid as the programmable block.
 * * vMod 9.4.7:
 * * - Possible optimize of block scanning.
 * * vMod 9.4.8:
 * * - Made adjustments to leveling the ship when entering and exiting the planet
 * * vMod 9.5.0:
 * * - Added the ability to save docks of grids you don't own when docked to them (target grid must not move for this to work correctly)
 * * vMod 9.5.1:
 * * - Added the optional ability to name stations/grids that are added with the "add dock" command
 * * vMod 9.5.2:
 * * - Removed all unused references (warnings should disappear)
 * * vMod 9.5.3:
 * * - Modified waypoint parsing default speed (the difference shouldn't be noticable)
 * * vMod 9.5.4:
 * * - Fixed alignment issues
 * * vMod 9.5.5:
 * * - Fixed additional alignment issues
 * * vMod 9.5.6:
 * * - Fix for entering and exiting planets with your nose up and down (you no longer will flip upside down unexpectedly)
 * * vMod 9.6.0:
 * * - Added "APPROACHING" tag for timers (great for retracting wings when you're about to dock)
 * * vMod 9.6.1:
 * * - Fix for null connector exception when trying to dock when no connector was tagged
 * * vMod 10.0.0:
 * * - Added new feature, "Auto cruise". You can now go to a specified cruising altitude on planets to get from point A to B faster.
 * *		No more crashing into terrain when you destination is on the opposite side of the planet.
 * * - Updated GPS to use new format that Keen has changed. GPS will now work without workarounds.
 * * - Fixed misaligned dock jank that happened in space.
 * * vMod 10.1.0:
 * * - Fixed approach timer not firing
 * * - Added the ability for connectors to be on subgrids as long as it's the same construct
 * * - Optimized arrival slowdown to be more accurate
 * * - Added separate nose up and nose down elevations
 * * - Inertial dampeners will be deactivated when docking with a nonstatic grid
 * 
 * Commands (Arguments in Programmable Block)
 * -----------
 * - Next = next item in list
 * - Prev = prev item in list
 * - Clearstorage = reset all save data in this program (recompile needed after this)
 * - Clearlog = clear the log
 * - Start = start auto pilot to selected destination in nav screen
 * - Start [GPS] = start auto pilot setting a course to pasted GPS coordinate
 * - Start next = start auto pilot going to the next destination on the nav screen
 * - Start prev = start auto pilot going to the previous destination on the nav screen
 * - Stop = stop autopilot
 * - Screen = switch between conf screen and nav screen
 * - Select = add or remove waypoint/dock from nav screen (only works in conf screen)
 * - Add = add current position in conf screen
 * - Add stance = add current position and direction to conf screen
 * - Add dock <optional: station name> = save currently connected dock to list of docks you can navigate to (can actually dock to the target as long as it doesn't move)
 * 	(IMPORTANT: Make sure you're docked with only one SAM assigned connector)
 * 	(IMPORTANT: Does not work when target dock uses reverse connector)
 * - Add [GPS] = add pasted GPS coordinate to conf screen for later navigation (name of coordinate will be saved)
 * - Go [dock name] = Go to specified dock name (must be broadcasted by SAM at destination and must be known by ship first)
 * - Remove = Delete selected old dock/waypoint/GPS coordinate from nav/conf screen
 * - Undock = initiate undock sequence
 * - Save = save all data to storage (saving data is done automatically when game autosaves)
 * - Load = load all data from storage
 * - Toggle = start/stop autopilot
 * - Scan = scan the grid for changes
 */

//Modified by SCBionicle
// Sam's Autopilot Manager
public static string VERSION = "2 vMod 10.1.0";

//
// Documentation: http://steamcommunity.com/sharedfiles/filedetails/?id=1653875433
//
// Owner: Sam
//
// Modified By: SCBionicle
//
// Modified Script Workshop Page: https://steamcommunity.com/sharedfiles/filedetails/?id=1941423134
//
// Latest changes:
// - Added docking paths;

// - Added PB Attribute Wait;

// - Timer routine optimized;

// - Fixed deprecated warning for text pannels;

// - Go command fixed;

// - Now possible to override LCD text size and type with the OVR TAG;

// - Using IGC for Antenna communication;

// - Added log entry for Timer events;

// - Other small bugs fix and cleanup done;

// - Fixed invalid commands;


// Change the tag used to identify blocks
public static string TAG = "SAM";


// -------------------------------------------------------
// Update at your own peril.
// -------------------------------------------------------

private static float HORIZONT_CHECK_DISTANCE = 2000.0f;

private static float MAX_SPEED = 80.0f; //Speed when navigating (this is done during collision avoidance, not actually top speed)
                                        //^^ Tag: Speed

private static float APPROACH_DISTANCE = 20.0f; //How close to approach the runway/landing pad

private static float DOCK_DISTANCE = 10.0f; //How close to get when initiating final docking procedure

private static float DOCK_SPEED = 1f; //Ship speed when docking or undocking

private static float UNDOCK_DISTANCE = 20.0f; //How far to lift off when undocking

private static float APPROACH_SPEED = 2.5f; //how fast the ship moves to get to the docking height

private static float APPROACH_SAFE_DISTANCE = 5.0f; //how far away the ship hovers from the taxi points (Tag: Taxi_distance)

private static float CONVERGING_SPEED = 100f; //How fast the ship should go when no objects are in the ship's path

private static float TAXIING_SPEED = 2.0f; //Speed of which the ship goes while taxiing to its connector (low speed recommended)

private static float ARRIVAL_DISTANCE = 500.0f; //Distance to which the ship must slow down when arriving to its destination
                                                //^^ This should be adjusted to increase if your ship can't brake as fast

private static float ARRIVAL_SPEED = 10.0f; //Speed of which the which the ship should go when closing in to destination (low speed recommended)

private static float COLLISION_CORRECTION_ANGLE = (float)Math.PI / 10.0f;

private static float BRAKE_THRUST_TRIGGER_DIFFERENCE = 3.0f; //the threshold on which the ship hard brakes when going too fast

//VV If enabled, this will be the ground-to-air elevation at which the nose will point toward the target when leaving the atmosphere
private static float ESCAPE_NOSE_UP_ELEVATION = 2000.0f;

private static string ADVERT_ID = "SAMv2";

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

private static string MSG_CRUISING_AT = "cruising at {0:N} m, climbing at {1:N0}°...";

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

private static float COLLISION_DISABLE_RADIUS_MULTIPLIER = 3.0f;

private static float IDLE_POWER = 0.0000001f;

private static double TICK_TIME = 0.16666f;

private static double GYRO_GAIN = 1.0;

private static double GYRO_MAX_ANGULAR_VELOCITY = Math.PI;

private static float GUIDANCE_MIN_AIM_DISTANCE = 0.5f;

private static float DISTANCE_TO_GROUND_IGNORE_PLANET = 1.2f * HORIZONT_CHECK_DISTANCE;

private static float MAX_TRUST_UNDERESTIMATE_PERCENTAGE = 0.90f;

private static int DOCK_ATTEMPTS = 5;

private static int LOG_MAX_LINES = 30;

private const string TAXI_SPEED_TAG = "Taxi_speed";

private const string APPROACH_DISTANCE_TAG = "Approach_distance";

private const string DOCK_DISTANCE_TAG = "Dock_distance";

private const string DOCK_SPEED_TAG = "Dock_speed";

private const string UNDOCK_DISTANCE_TAG = "Undock_distance";

private const string APPROACH_SPEED_TAG = "Approach_speed";

private const string APPROACH_SAFE_DISTANCE_TAG = "Taxi_distance";

private const string CONVERGING_SPEED_TAG = "Converging_speed";

private const string ARRIVAL_DISTANCE_TAG = "Arrival_distance";

private const string ARRIVAL_SPEED_TAG = "Arrival_speed";

private const string ESCAPE_NOSE_UP_TAG = "ESCAPENOSEUP";

private const string ESCAPE_NOSE_UP_ELEVATION_TAG = "Nose_up_elevation";

private const string DESCEND_NOSE_DOWN_ELEVATION_TAG = "Nose_down_elevation";

private const string SLOW_ON_APPROACH_TAG = "SLOWONAPPROACH"; //slows the ship to taxiing speed when closing in onto the runway or docking connector

private const string ALLOW_DIRECT_ALIGNMENT_TAG = "ALLOWDIRECTALIGNMENT"; //In space, should the ship point directly at the destination on navigation started before taking off?

private const string CONNECTOR_REVERSE_TAG = "REVERSE"; //Some modded connectors are placed backwards to work for some reason. It confuses this script.
                                                        //^^ If the connector goes on backwards, add this tag to the connector to work around that. Use the BuildInfo mod to make sure that this is the case.
private const string AUTO_CRUISE_ATTRIBUTE = "Auto_cruise";



//private const string TAXIING_SPEED_TAG = "Taxiing_Speed";



public static class Raytracer
{
                public enum Result { NotRun, NoHit, Hit };

    public static Vector3D hitPosition;

    public static MyDetectedEntityInfo hit;

    public static Result Trace(ref Vector3D target, bool ignorePlanet)
    {
        foreach (IMyCameraBlock camera in GridBlocks.cameraBlocks)
        {
            if (!camera.CanScan(target))
            {
                continue;
            }
            hit = camera.Raycast(target);

            if (hit.IsEmpty())
            {
                return Result.NoHit;
            }

            //******************** Remove if below if script breaks *********************
            if (hit.EntityId == GridBlocks.MasterProgrammableBlock.CubeGrid.EntityId)
            {
                continue;
            }


            switch (hit.Type)
            {
                case MyDetectedEntityType.Planet:
                    if (ignorePlanet)
                    {
                        return Result.NoHit;
                    }
                    goto case MyDetectedEntityType.SmallGrid;

                case MyDetectedEntityType.Asteroid:
                case MyDetectedEntityType.LargeGrid:
                case MyDetectedEntityType.SmallGrid:
                    hitPosition = hit.HitPosition.Value;

                    return Result.Hit;

                default: return Result.NoHit;

            }
        }
        return Result.NotRun;

    }
}
public static class Situation
{
    //public static Dock wantedDock;

    //public static Vector3D wantedPosition;

    public static Vector3D position;

    public static Vector3D linearVelocity;

    public static double elevationVelocity;

    public static Vector3D naturalGravity;

    public static bool planetDetected;

    public static Vector3D planetCenter = new Vector3D();

    public static bool inGravity; //false when sufficiantly elevated above ground when "allowEscapeNoseUp" is turned on

    public static bool turnNoseUp; //true when in gravity but nose would be up

    public static float noseDownElevation = ESCAPE_NOSE_UP_ELEVATION;

    public static bool allowEscapeNoseUp;

    public static bool slowOnApproach;

    //public static double distanceToWantedPosition;

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

    public static double seaElevationVelocity = 0;

    public static bool alignDirectly = false;

    private static Dictionary<Base6Directions.Direction, double> maxThrust = new Dictionary<Base6Directions.Direction, double>()
    {
        { Base6Directions.Direction.Backward, 0 },
        { Base6Directions.Direction.Down, 0 },
        { Base6Directions.Direction.Forward, 0 },
        { Base6Directions.Direction.Left, 0 },
        { Base6Directions.Direction.Right, 0 },
        { Base6Directions.Direction.Up, 0 },
    };

    private static double forwardChange, upChange, leftChange;

    private static Vector3D maxT;

    public static double autoCruiseAltitude = double.PositiveInfinity;

    public static double GetMaxThrust(Vector3D dir)
    {
        forwardChange = Vector3D.Dot(dir, Situation.gridForwardVect);

        upChange = Vector3D.Dot(dir, Situation.gridUpVect);

        leftChange = Vector3D.Dot(dir, Situation.gridLeftVect);

        maxT = new Vector3D();

        maxT.X = forwardChange * maxThrust[(forwardChange > 0) ? Base6Directions.Direction.Forward : Base6Directions.Direction.Backward];

        maxT.Y = upChange * maxThrust[(upChange > 0) ? Base6Directions.Direction.Up : Base6Directions.Direction.Down];

        maxT.Z = leftChange * maxThrust[(leftChange > 0) ? Base6Directions.Direction.Left : Base6Directions.Direction.Right];

        return MAX_TRUST_UNDERESTIMATE_PERCENTAGE * maxT.Length();

    }
    public static void RefreshParameters()
    {
        foreach (Base6Directions.Direction dir in maxThrust.Keys.ToList())
        {
            maxThrust[dir] = 0;
        }
        foreach (IMyThrust thruster in GridBlocks.thrustBlocks)
        {
            if (!thruster.IsWorking)
            {
                continue;
            }
            maxThrust[thruster.Orientation.Forward] += thruster.MaxEffectiveThrust;
        }
        gridForwardVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Forward);
        gridUpVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Up);
        gridLeftVect = RemoteControl.block.CubeGrid.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Left);
        mass = RemoteControl.block.CalculateShipMass().PhysicalMass; //********************* was physical mass, revert if breaks **********************
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
        //distanceToWantedPosition = Vector3D.Distance(position, wantedPosition);
        planetDetected = RemoteControl.block.TryGetPlanetPosition(out planetCenter);
        naturalGravity = RemoteControl.block.GetNaturalGravity();
        inGravity = naturalGravity.Length() >= 0.5;
        if (inGravity)
        {
            seaElevationVelocity = Vector3D.Dot(linearVelocity, -Vector3D.Normalize(naturalGravity));
        }
        if (allowEscapeNoseUp && inGravity)
        {
            double groundElevation = 0;
            RemoteControl.block.TryGetPlanetElevation(MyPlanetElevation.Surface, out groundElevation);
            if (seaElevationVelocity > 0)
            {
                if (groundElevation > ESCAPE_NOSE_UP_ELEVATION)
                {
                    inGravity = false;
                    turnNoseUp = true;
                }
                else
                {
                    turnNoseUp = false;
                }
            }
            else if(seaElevationVelocity < 0)
            {
                if (groundElevation > noseDownElevation)
                {
                    inGravity = false;
                    turnNoseUp = true;
                }
                else
                {
                    turnNoseUp = false;
                }
            }
        }
        else
        {
            turnNoseUp = false;
        }

        if (inGravity)
        {
            RemoteControl.block.TryGetPlanetElevation(MyPlanetElevation.Surface, out distanceToGround);
            gravityDownVector = Vector3D.Normalize(naturalGravity);
            gravityUpVector = -1 * gravityDownVector;
        }
        else //if(!turnNoseUp) //************ Was just an else, revert back if script breaks
        {
            distanceToGround = DISTANCE_TO_GROUND_IGNORE_PLANET;
            gravityDownVector = downVector;
            gravityUpVector = upVector;
        }
        //else
        //{
        //    RemoteControl.block.TryGetPlanetElevation(MyPlanetElevation.Surface, out distanceToGround);
        //    gravityDownVector = downVector;
        //    gravityUpVector = upVector;
        //}
    }
}
public static class Horizont
{
    public static float angle = 0.0f;
    public static bool hit = false;
    private static bool ignorePlanet;
    private static Vector3D tracePosition;
    private static float angleDir = 1.0f;
    private static float up = 1.0f, down = 1.0f, mult = 1.0f;
    public static Vector3D? ScanHorizont(float distance, Vector3D forwardVector, Vector3D rightVector)
    {
        tracePosition = Situation.position + Math.Min(distance, HORIZONT_CHECK_DISTANCE) * Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
        ignorePlanet = Situation.distanceToGround >= DISTANCE_TO_GROUND_IGNORE_PLANET;
        if (hit)
        {
            switch (Raytracer.Trace(ref tracePosition, ignorePlanet))
            {
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
                    if (angle < -HORIZONT_CHECK_ANGLE_LIMIT)
                    {
                        hit = false;
                        angle = 0.0f;
                        up = down = mult = 1.0f;
                        return Vector3D.Zero;
                    }
                    return Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
            }
        }
        else
        {
            switch (Raytracer.Trace(ref tracePosition, ignorePlanet))
            {
                case Raytracer.Result.Hit:
                    hit = true;
                    up = down = mult = 1.0f;
                    return Vector3D.Transform(forwardVector, Quaternion.CreateFromAxisAngle(rightVector, angle));
                case Raytracer.Result.NoHit:
                    up = down = 1.0f;
                    angle += angleDir * mult * HORIZONT_CHECK_ANGLE_STEP;
                    mult = Math.Min(10.0f, mult + 1.0f);
                    if (Math.Abs(angle) > HORIZONT_CHECK_ANGLE_LIMIT)
                    {
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
public static class Guidance
{
    private static Vector3D desiredPosition = new Vector3D();
    private static Vector3D desiredFront = new Vector3D();
    private static Vector3D desiredUp = new Vector3D();
    private static float desiredSpeed = MAX_SPEED;
    private static Waypoint waypoint = null;

    //private static double planetDirDifference = 0;

    private const double planetUpDiffThreshold = 0.5;

    public static void Set(Waypoint w)
    {
        desiredPosition = w.stance.position;
        desiredFront = w.stance.forward;
        desiredUp = w.stance.up;
        desiredSpeed = w.maxSpeed;
        waypoint = w;
    }
    public static void Release()
    {
        foreach (IMyGyro gyro in GridBlocks.gyroBlocks)
        {
            gyro.GyroOverride = false;
        }
        foreach (IMyThrust thruster in GridBlocks.thrustBlocks)
        {
            thruster.ThrustOverride = 0;
        }
    }

    public static void ThrusterRelease()
    {
        foreach (IMyThrust thruster in GridBlocks.thrustBlocks)
        {
            thruster.ThrustOverride = 0;
        }
    }

    public static void ThrusterPlanetarySlowdown()
    {
        if (Situation.inGravity && Navigation.waypoints.Count != 0
            && Navigation.waypoints[0].type == Waypoint.wpType.NAVIGATING)
        {
            //Vector3D.Up();
            foreach (IMyThrust t in GridBlocks.thrustBlocks)
            {
                if (t.Orientation.Forward != Base6Directions.Direction.Down &&
                    t.Orientation.Forward != Base6Directions.Direction.Up)
                {
                    t.ThrustOverride = 0;
                }
            }
        }
        else
        {
            ThrusterRelease();
        }
    }

    public static void Tick()
    {
        Guidance.StanceTick();
        Guidance.GyroTick();
        Guidance.ThrusterTick();

    }
    public static bool Done()
    {
        return worldVector.Length() < 0.05 && pathLen <= 0.1f; // *********************** was at pathLen <= 0.1f

    }
    private static Vector3D pathNormal, path, aimTarget, upVector, aimVector;
    private static float pathLen;
    private static void StanceTick()
    {
        path = desiredPosition - Situation.position;
        pathLen = (float)path.Length();
        pathNormal = Vector3D.Normalize(path);
        if (desiredFront != Vector3D.Zero)
        {
            aimTarget = Situation.position + desiredFront * Situation.radius;
        }
        else
        {
            aimVector = (pathLen > GUIDANCE_MIN_AIM_DISTANCE) ? pathNormal : Situation.forwardVector;
            if (Situation.inGravity)
            {
                aimTarget = Situation.position + Vector3D.Normalize(Vector3D.ProjectOnPlane(ref aimVector, ref Situation.gravityUpVector)) * Situation.radius;
            }
            else
            {
                //if (Math.Abs(planetDirDifference) < planetUpDiffThreshold)
                //    aimTarget = Situation.position + Vector3D.Normalize(Vector3D.ProjectOnPlane(ref aimVector, ref Situation.gravityUpVector)) * Situation.radius;
                //else
                aimTarget = Situation.position + aimVector * Situation.radius; //if breaks, remove this else restore only this line
            }
        }
        Vector3D targetDirection = Vector3D.Normalize(aimTarget - Situation.position);
        if (Situation.inGravity)
        {
            upVector = (desiredUp == Vector3D.Zero) ? Situation.gravityUpVector : desiredUp;
        }
        else if (!Situation.turnNoseUp) //was just an else (change this back to else if script breaks)
        {
            upVector = (desiredUp == Vector3D.Zero) ? Vector3D.Cross(aimVector, Situation.leftVector) : desiredUp;
            //planetDirDifference = 0; //reset the diff so that it won't tilt the ship at the wrong time (when not in gravity well)
        }
        else
        {
            Vector3D planetUpVector = Vector3D.Normalize(RemoteControl.block.GetNaturalGravity() * -1);

            Vector3D desiredUpVector = Situation.upVector;

            if(waypoint.type == Waypoint.wpType.CONVERGING || waypoint.type == Waypoint.wpType.CRUISING)
            {
                Vector3D newUpVector = Vector3D.ProjectOnPlane(ref planetUpVector, ref targetDirection);
                newUpVector = Vector3D.Normalize(newUpVector);
                desiredUpVector = newUpVector;
            }

            upVector = (desiredUp == Vector3D.Zero) ?
                //Vector3D.CalculatePerpendicularVector(Vector3D.Normalize(Vector3D.Cross(aimVector, planetUpVector))) //was just working (abandoned due to strange orientation of dedicated servers
                //Vector3D.Cross(aimVector, Vector3D.CalculatePerpendicularVector(Vector3D.Normalize(planetUpVector)))
                desiredUpVector
                : desiredUp; //was "desiredUp"
        }

    }
    private static Quaternion invQuat;
    private static Vector3D direction, refVector, worldVector, localVector, realUpVect, realRightVect;
    private static double azimuth, elevation, roll;
    private static void GyroTick()
    {
        if (GridBlocks.gyroBlocks.Count == 0)
        {
            return;

        }

        direction = Vector3D.Normalize(aimTarget - Situation.position);
        //invQuat = Quaternion.Inverse(Quaternion.CreateFromForwardUp(Situation.forwardVector, Situation.upVector));
        //*************** REMOVE BELOW IF IF SCRIPT BREAKS *************************
        //Vector3D gDownVector = RemoteControl.block.GetNaturalGravity();
        //gDownVector.Normalize();
        //if (Situation.turnNoseUp && !Situation.inGravity)
        //{

        //    invQuat = Quaternion.Inverse(Quaternion.CreateFromForwardUp(Situation.forwardVector, gDownVector*-1));
        //}
        //else
        //{
        invQuat = Quaternion.Inverse(Quaternion.CreateFromForwardUp(Situation.forwardVector, Situation.upVector)); //original line
                                                                                                                   //}

        refVector = Vector3D.Transform(direction, invQuat);
        Vector3D.GetAzimuthAndElevation(refVector, out azimuth, out elevation);
        realUpVect = Vector3D.ProjectOnPlane(ref upVector, ref direction);
        realUpVect.Normalize();
        realRightVect = Vector3D.Cross(direction, realUpVect);
        realRightVect.Normalize();
        //************************ REMOVE BELOW IF IF SCRIPT BREAKS ************************
        //if (Situation.turnNoseUp && !Situation.inGravity)
        //    roll = Vector3D.Dot(gDownVector, realRightVect);
        //else
        roll = Vector3D.Dot(Situation.upVector, realRightVect); //original line
        worldVector = Vector3.Transform((new Vector3D(elevation, azimuth, roll)), Situation.orientation);
        foreach (IMyGyro gyro in GridBlocks.gyroBlocks)
        {
            localVector = Vector3.Transform(worldVector, Matrix.Transpose(gyro.WorldMatrix.GetOrientation()));
            gyro.Pitch = (float)MathHelper.Clamp((-localVector.X * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
            gyro.Yaw = (float)MathHelper.Clamp(((-localVector.Y) * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
            gyro.Roll = (float)MathHelper.Clamp(((-localVector.Z) * GYRO_GAIN), -GYRO_MAX_ANGULAR_VELOCITY, GYRO_MAX_ANGULAR_VELOCITY);
            gyro.GyroOverride = true;
        }
    }
    private static float forwardChange, upChange, leftChange, applyPower;
    private static Vector3D force, directVel, directNormal, indirectVel;
    private static double ttt, maxFrc, maxVel, maxAcc, TIME_STEP = 2.5 * TICK_TIME, smooth;
    private static void ThrusterTick()
    {
        if (pathLen == 0.0f)
        {
            return;

        }
        //Logger.Log($"Path length: {pathLen:N}");
        //*********Remove this "if" if slow down doesn't work**************
        //if (desiredSpeed < Situation.linearVelocity.Length() - BRAKE_THRUST_TRIGGER_DIFFERENCE)
        //{
        //ThrusterRelease();
        //return;
        //}

        force = Situation.mass * Situation.naturalGravity;
        directVel = Vector3D.ProjectOnVector(ref Situation.linearVelocity, ref pathNormal);
        directNormal = Vector3D.Normalize(directVel);
        if (!directNormal.IsValid())
        {
            directNormal = Vector3D.Zero;

        }

        maxFrc = Situation.GetMaxThrust(pathNormal) - ((Vector3D.Dot(force, pathNormal) > 0) ? Vector3D.ProjectOnVector(ref force, ref pathNormal).Length() : 0.0);
        maxVel = Math.Sqrt(2.0 * pathLen * maxFrc / Situation.mass);
        smooth = Math.Min(Math.Max((desiredSpeed + 1.0f - directVel.Length()) / 2.0f, 0.0f), 1.0f);
        maxAcc = 1.0f + (maxFrc / Situation.mass) * smooth * smooth * (3.0f - 2.0f * smooth);
        ttt = Math.Max(TIME_STEP, Math.Abs(maxVel / maxAcc));
        force += Situation.mass * -2.0 * (pathNormal * pathLen / ttt / ttt - directNormal * directVel.Length() / ttt); //lone number was -2.0 (revert if breaks)
        indirectVel = Vector3D.ProjectOnPlane(ref Situation.linearVelocity, ref pathNormal);
        force += Situation.mass * indirectVel / TIME_STEP;
        forwardChange = (float)Vector3D.Dot(force, Situation.gridForwardVect);
        upChange = (float)Vector3D.Dot(force, Situation.gridUpVect);
        leftChange = (float)Vector3D.Dot(force, Situation.gridLeftVect);
        foreach (IMyThrust thruster in GridBlocks.thrustBlocks)
        {
            if (!thruster.IsWorking)
            {
                thruster.ThrustOverridePercentage = 0;
                continue;
            }
            switch (thruster.Orientation.Forward)
            {
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
        /**************** REMOVE BELOW IF THE SHIP BREAKS *************************/
        if (desiredSpeed < Situation.linearVelocity.Length() - BRAKE_THRUST_TRIGGER_DIFFERENCE)
        {
            ThrusterPlanetarySlowdown();
        }
    }
    private static float Drain(ref float remainingPower, float maxEffectiveThrust)
    {
        applyPower = Math.Min(Math.Abs(remainingPower), maxEffectiveThrust);
        remainingPower = (remainingPower > 0) ? (remainingPower - applyPower) : (remainingPower + applyPower);
        return Math.Max(applyPower / maxEffectiveThrust, IDLE_POWER);

    }
}
public static class Navigation
{
    public static List<Waypoint> waypoints = new List<Waypoint> { };

    public static bool IsClose = false;

    public static Waypoint ArrivalWaypoint;
    public static string Status()
    {
        if (waypoints.Count == 0)
        {
            return "";

        }

        return waypoints[0].GetTypeMsg();

    }
    private static Vector3D? horizontDirectionNormal;
    private static Vector3D endTarget, endTargetPath, endTargetNormal, newDirection, newTarget, endTargetRightVector;
    private static void CheckColision()
    {
        switch (waypoints[0].type)
        {
            case Waypoint.wpType.CONVERGING:
                if ((waypoints[0].stance.position - Situation.position).Length() < COLLISION_DISABLE_RADIUS_MULTIPLIER * Situation.radius)
                {
                    if (Situation.slowOnApproach)
                    {
                        waypoints[0].maxSpeed = TAXIING_SPEED;
                    }
                    return;
                }
                break;

            case Waypoint.wpType.NAVIGATING: break;
            case Waypoint.wpType.CRUISING: break;

            default: return;

        }
        endTarget = waypoints[waypoints[0].type == Waypoint.wpType.NAVIGATING ? 1 : 0].stance.position;
        endTargetPath = endTarget - Situation.position;
        endTargetNormal = Vector3D.Normalize(endTargetPath);
        endTargetRightVector = Vector3D.Normalize(Vector3D.Cross(endTargetNormal, Situation.gravityUpVector));
        horizontDirectionNormal = Horizont.ScanHorizont((float)endTargetPath.Length(), endTargetNormal, endTargetRightVector);
        if (!horizontDirectionNormal.HasValue)
        {
            return;

        }

        if (Vector3D.IsZero(horizontDirectionNormal.Value))
        {
            if (waypoints[0].type == Waypoint.wpType.NAVIGATING)
            {
                waypoints.RemoveAt(0);

            }

            return;

        }
        newDirection = Vector3D.Transform(horizontDirectionNormal.Value, Quaternion.CreateFromAxisAngle(endTargetRightVector, COLLISION_CORRECTION_ANGLE));
        newTarget = Situation.position + HORIZONT_CHECK_DISTANCE * newDirection;
        if (waypoints[0].type == Waypoint.wpType.NAVIGATING)
        {
            waypoints[0].stance.position = newTarget;

        }
        else
        {
            waypoints.Insert(0, new Waypoint(new Stance(newTarget, Vector3D.Zero, Vector3D.Zero), MAX_SPEED, Waypoint.wpType.NAVIGATING));

        }
    }

    public static Waypoint NextWaypointOfType(Waypoint.wpType type)
    {
        foreach(Waypoint w in waypoints)
        {
            if (w.type == type)
                return w;
        }
        return null;
    }

    private static double altitudeGravityStart = 0;
    public static float ClimbAngle = 0;
    private static void ProcessAutoCruise()
    {
        Vector3D gravityUp;
        double seaLevelAltitude = double.MinValue;
        bool inGravity = RemoteControl.block?.TryGetPlanetElevation(MyPlanetElevation.Sealevel, out seaLevelAltitude) ?? false; //ways to bypass null pointers
        gravityUp = -RemoteControl.block?.GetNaturalGravity() ?? Vector3D.Zero;
        Vector3D gravityUpNorm = Vector3D.Normalize(gravityUp); //normalized vector of upwards gravity

        altitudeGravityStart = inGravity ? Math.Max(altitudeGravityStart, seaLevelAltitude) : 0;
        const float maxDescentAngle = (float) -Math.PI / 2;
        const float maxAscentAngle = (float) Math.PI / 4;

        if(!double.IsNaN(Situation.autoCruiseAltitude) && inGravity) //Is autocruise enabled and are you in a gravity well?
        {
            //Vector3D ?desiredDock = Pilot.dock.Count>0 ? Pilot.dock[0]?.stance.position : null;
            Vector3D? desiredDock = NextWaypointOfType(Waypoint.wpType.CONVERGING)?.stance?.position;
            if (desiredDock == null)
            {
                StopAutoCruise();
                return;
            }
            Vector3D desiredDestination = desiredDock ?? Vector3D.NegativeInfinity; //You can trust this to be valid coordinate (needed a default value to satisfy the compiler)
            Vector3D dockDirNotNormed = desiredDestination - Situation.position;
            bool closeEnough = Vector3D.Distance(desiredDestination, Situation.position) < Situation.autoCruiseAltitude * 2; //Close enough to start descending?
            Vector3D dockDir = Vector3D.Normalize(dockDirNotNormed); //Direction of the destination compared to the vessel in question
            bool toAbove = Vector3D.Dot(dockDir, gravityUpNorm) > 0.1; //Is the destination above the ship
            bool directlyBelow;
            if (Vector3D.Dot(dockDir, gravityUpNorm) < -0.95f && Vector3D.Distance(desiredDestination, Situation.position) < Vector3D.Distance(Situation.planetCenter, Situation.position))
                directlyBelow = true;
            else
                directlyBelow = false;

            if(!closeEnough && !toAbove && !directlyBelow && (waypoints[0].type & (Waypoint.wpType.CONVERGING | Waypoint.wpType.CRUISING | Waypoint.wpType.NAVIGATING)) != 0){ //all conditions must be true to cruise
                Vector3D DesiredCruiseToPoint; //This is where SAM will try to got when cruising. Be sure to have this var set by the time your code exits
                Vector3D dockDirGravityProj = Vector3D.ProjectOnPlane(ref dockDir, ref gravityUpNorm);
                Vector3D dockDirRightPerpendicular = Vector3D.Cross(Vector3D.Normalize(dockDirGravityProj), gravityUpNorm);
                //Climb angle calculations here
                //float climbAngle;
                if (seaLevelAltitude+100 <= Situation.autoCruiseAltitude)
                {                       //(Max angle rads) / ...
                    ClimbAngle = (float)(((Math.PI / 4) / (Math.PI/2)) * Math.Acos(2 * (seaLevelAltitude / Situation.autoCruiseAltitude) - 1));
                }
                else if(seaLevelAltitude-100 >= Situation.autoCruiseAltitude)
                {                        //(Max angle rads) / ...
                    ClimbAngle = (float)(-((Math.PI / 2) / (Math.PI/2)) * Math.Acos(2 *
                        ((altitudeGravityStart - seaLevelAltitude) / (altitudeGravityStart - Situation.autoCruiseAltitude))-1));
                }
                else
                {
                    ClimbAngle = 0;
                }
                //Logger.Info($"Climb angle: {MathHelper.ToDegrees(ClimbAngle):N2} -> {MathHelper.ToDegrees(MathHelperD.Clamp(ClimbAngle, maxDescentAngle, maxAscentAngle)):N2}");
                ClimbAngle = float.IsNaN(ClimbAngle) ? 0 : ClimbAngle;
                ClimbAngle = (float)MathHelperD.Clamp(ClimbAngle, maxDescentAngle, maxAscentAngle);
                Vector3D intendedDirection = Vector3D.Transform(dockDirGravityProj, Quaternion.CreateFromAxisAngle(dockDirRightPerpendicular, ClimbAngle)); //not normed or at desired magnitude
                Vector3D intendedDirectionNorm = Vector3D.Normalize(intendedDirection);
                Vector3D intendedDistanceAsVector = dockDirNotNormed;
                Vector3D finalDirection = Vector3D.ProjectOnVector(ref intendedDistanceAsVector, ref intendedDirectionNorm);
                DesiredCruiseToPoint = Situation.position + finalDirection;


                SetCruisePos(DesiredCruiseToPoint); //Inserts cruising waypoint and edits existing ones
                return;
            }
        }
        StopAutoCruise();
    }

    private static void StopAutoCruise()
    {
        if (waypoints[0]?.type == Waypoint.wpType.CRUISING)
        {
            waypoints.RemoveAt(0);
        }
    }

    /// <summary>
            /// Inserts a cruising waypoint or edits the existing one in a way that it will not interrupt normal operation
            /// </summary>
            /// <param name="pos">The position for SAM to move to to maintain a cruise altitude</param>
    private static void SetCruisePos(Vector3D pos)
    {
        Waypoint wp = new Waypoint(new Stance(pos, Vector3D.Zero, Vector3D.Zero), CONVERGING_SPEED, Waypoint.wpType.CRUISING);

        switch (waypoints[0].type)
        {
            case Waypoint.wpType.CRUISING:
                waypoints[0] = wp;
                return;
            case Waypoint.wpType.CONVERGING:
                waypoints.Insert(0, wp);
                return;
            case Waypoint.wpType.NAVIGATING:
                if (waypoints[1].type == Waypoint.wpType.CRUISING)
                {
                    waypoints[1] = wp;
                }
                else if(waypoints[1].type == Waypoint.wpType.CONVERGING)
                {
                    waypoints.Insert(1, wp);
                }
                return;
            default:
                return;
        }
    }
    public static void ResetArrival()
    {
        IsClose = false;
        ArrivalWaypoint = null;
    }

    public static void ProcessCloseness()
    {
        //*************** Remove outer if if this breaks ********************
        if (!(Situation.slowOnApproach && (waypoints[0].stance.position - Situation.position).Length() < COLLISION_DISABLE_RADIUS_MULTIPLIER * Situation.radius))
        {
            if (Pilot.running && CheckClose())
            {
                foreach (Waypoint w in waypoints)
                {
                    if (w.type == Waypoint.wpType.CONVERGING || w.type == Waypoint.wpType.NAVIGATING)
                    {
                        w.maxSpeed = ARRIVAL_SPEED;
                    }
                }
            }
        }
    }

    public static bool CheckClose()
    {
        if (ArrivalWaypoint == null)
        {
            ArrivalWaypoint = waypoints.Last(delegate (Waypoint w) { return w.type == Waypoint.wpType.CONVERGING; });
            /*foreach (Waypoint w in waypoints)
                    {
                        if (w.type == Waypoint.wpType.CONVERGING || w.type == Waypoint.wpType.NAVIGATING)
                        {
                            ArrivalWaypoint = w;

                            break;
                        }
                    }*/
            return false;
        }
        if (IsClose)
            return true;
        else
        {
            Vector3D destination = ArrivalWaypoint.stance.position;
            if ((destination - Situation.position).Length() <= ARRIVAL_DISTANCE)
            {
                Logger.Info("Slowing due to arriving at destination.");
                Signal.Send(Signal.SignalType.APPROACH);
                IsClose = true;
                return true;
            }
        }
        return false;
    }

    public static void Tick()
    {
        if (waypoints.Count == 0)
        {
            ResetArrival();
            return;

        }

        Situation.RefreshParameters();
        if (!Navigation.Done())
        {
            ProcessCloseness();
        }
        ProcessAutoCruise();
        CheckColision();
        Guidance.Set(waypoints.ElementAt(0));
        Guidance.Tick();
        if (Guidance.Done())
        {
            waypoints.RemoveAt(0);
            if (waypoints.Count != 0)
            {
                return;
            }
            Guidance.Release();
        }
    }


    public static void AddWaypoint(Waypoint w)
    {
        waypoints.Insert(0, w);

    }
    public static void AddWaypoint(Stance s, float m, Waypoint.wpType wt)
    {
        AddWaypoint(new Waypoint(s, m, wt));

    }
    public static void AddWaypoint(Vector3D p, Vector3D f, Vector3D u, float m, Waypoint.wpType wt)
    {
        AddWaypoint(new Stance(p, f, u), m, wt);

    }
    public static void Stop()
    {
        Guidance.Release();
        waypoints.Clear();

    }
    public static bool Done()
    {
        return waypoints.Count == 0;

    }
}
public static class Pilot
{
    public static bool running = false;
    public static List<Dock> dock = new List<Dock>();
    public static void Tick()
    {
        if (!running)
        {
            return;

        }

        if (!RemoteControl.Present())
        {
            if (!ErrorState.Get(ErrorState.Type.NoRemoteController))
            {
                Logger.Err(MSG_NO_REMOTE_CONTROL);

            }

            ErrorState.Set(ErrorState.Type.NoRemoteController);
            Stop();
            return;

        }
        else
        {
            RemoteControl.block.DampenersOverride = true;
        }
        if (Navigation.Done())
        {
            if (dock.Count != 0 && dock[0].gridEntityId != 0)
            {
                CalculateApproach();
                dock.Clear();
                return;
            }
            Logger.Info(MSG_NAVIGATION_SUCCESSFUL);
            Signal.Send(Signal.SignalType.NAVIGATION);
            ConnectorControl.AttemptConnect();
            running = false;
            return;
        }

        Navigation.Tick();

        //****************** If script breaks, remove if below *********************** (was before Navigation.Tick())
        if (Navigation.waypoints.Count > 0 && Navigation.waypoints[0].type == Waypoint.wpType.DOCKING
            && connector != null)
        {
            //if (connector.Status == MyShipConnectorStatus.Connectable)
            //{
            //    Navigation.waypoints.Clear();
            //}
            foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
            {
                if (connector.Status == MyShipConnectorStatus.Connectable)
                {
                    Navigation.waypoints.Clear();
                    Guidance.Release();
                }
            }
        }


    }
    private static Quaternion qInitialInverse, qFinal, qDiff;
    private static Vector3D connectorToCenter, rotatedConnectorToCenter, newUp, newForward, up, referenceUp, direction, balancedDirection;
    private static IMyShipConnector connector;
    private static float connectorDistance;
    private static void CalculateApproach()
    {
        connector = ConnectorControl.GetConnector(dock[0]);

        if (connector == null)
        {
            Logger.Warn(MSG_NO_CONNECTORS_AVAILABLE);
            return;
        }
        bool reversedConnector = Block.HasProperty(connector.EntityId, CONNECTOR_REVERSE_TAG);
        Situation.RefreshParameters();
        connectorToCenter = Situation.position - connector.GetPosition();
        if ((Situation.inGravity || Situation.turnNoseUp) && Math.Abs(Vector3D.Dot(dock[0].stance.forward, Situation.gravityUpVector)) < 0.5f)
        {
            up = Situation.gravityUpVector;
            referenceUp = connector.WorldMatrix.GetDirectionVector(connector.WorldMatrix.GetClosestDirection(up));
            referenceUp = (referenceUp == connector.WorldMatrix.Forward || referenceUp == connector.WorldMatrix.Backward) ? connector.WorldMatrix.Up : referenceUp;
        }
        else
        {
            up = dock[0].stance.up;
            referenceUp = connector.WorldMatrix.Up;
        }
        //************** qInitialInverse had just (connector.WorldMatrix.Forward) without the if flow statement  *****************
        qInitialInverse = Quaternion.Inverse(Quaternion.CreateFromForwardUp(!reversedConnector ? connector.WorldMatrix.Forward :
            connector.WorldMatrix.Backward, referenceUp));
        qFinal = Quaternion.CreateFromForwardUp(-dock[0].stance.forward, up);
        qDiff = qFinal * qInitialInverse;
        rotatedConnectorToCenter = Vector3D.Transform(connectorToCenter, qDiff);
        newForward = Vector3D.Transform(RemoteControl.block.WorldMatrix.Forward, qDiff);
        newUp = Vector3D.Transform(RemoteControl.block.WorldMatrix.Up, qDiff);
        connectorDistance = (dock[0].cubeSize == VRage.Game.MyCubeSize.Large) ? 2.6f / 2.0f : 0.5f;
        connectorDistance += (GridBlocks.MasterProgrammableBlock.CubeGrid.GridSizeEnum == VRage.Game.MyCubeSize.Large) ? 2.6f / 2.0f : 0.5f;
        newPos = dock[0].stance.position + rotatedConnectorToCenter + (connectorDistance * dock[0].stance.forward);
        Navigation.AddWaypoint(newPos, newForward, newUp, DOCK_SPEED, Waypoint.wpType.DOCKING);
        newPos = dock[0].stance.position + rotatedConnectorToCenter + ((DOCK_DISTANCE + connectorDistance) * dock[0].stance.forward);
        Navigation.AddWaypoint(newPos, newForward, newUp, APPROACH_SPEED, Waypoint.wpType.APPROACHING);
        dock[0].approachPath.Reverse();
        foreach (VectorPath vp in dock[0].approachPath)
        {
            newPos = vp.position + (vp.direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
            Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, TAXIING_SPEED, Waypoint.wpType.TAXIING);
        }
        dock[0].approachPath.Reverse();

    }
    private static Vector3D newPos, undockPos;
    private static Dock disconnectDock;
    private static void SetEndStance(Dock dock)
    {
        Navigation.ResetArrival();
        Situation.RefreshParameters();
        if (dock.blockEntityId == 0)
        {

            Navigation.AddWaypoint(dock.stance, APPROACH_SPEED, Waypoint.wpType.ALIGNING);
            Navigation.AddWaypoint(dock.stance.position, Vector3D.Zero, Vector3D.Zero, CONVERGING_SPEED, Waypoint.wpType.CONVERGING);
            newPos = dock.stance.position;
        }
        else
        {
            if (dock.approachPath.Count == 0)
            {
                newPos = dock.stance.position + ((APPROACH_DISTANCE + Situation.radius) * dock.stance.forward);
            }
            else
            {
                newPos = dock.approachPath[0].position + ((APPROACH_DISTANCE + Situation.radius) * dock.approachPath[0].direction);
            }
            Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, CONVERGING_SPEED, Waypoint.wpType.CONVERGING);
        }
        if (Situation.linearVelocity.Length() >= 2.0f)
        {
            return;

        }

        disconnectDock = ConnectorControl.DisconnectAndTaxiData();
        direction = Vector3D.Normalize(newPos - Situation.position);
        balancedDirection = Vector3D.ProjectOnPlane(ref direction, ref Situation.gravityUpVector);
        if (disconnectDock == null)
        {
            //Navigation.AddWaypoint(Situation.position, balancedDirection, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING); //original alignment
            if (!Situation.inGravity && Situation.alignDirectly)
            {
                Navigation.AddWaypoint(Situation.position, direction, Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(direction)),
                    APPROACH_SPEED, Waypoint.wpType.ALIGNING);
            }
            else
            {
                Navigation.AddWaypoint(Situation.position, balancedDirection, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING);
            }
            return;
        }
        //Signal.Send(Signal.SignalType.UNDOCK);
        //Logger.Info("Sending undock signal...");
        undockPos = disconnectDock.stance.forward;
        undockPos *= (Situation.radius + UNDOCK_DISTANCE);
        undockPos += Situation.position;
        if (disconnectDock.approachPath.Count > 0)
        {
            //*************************** Remove Below line if breaks *****************************//
            Vector3D taxiBeginUnadjustedPos = disconnectDock.approachPath[disconnectDock.approachPath.Count - 1].position;
            Vector3D taxiBeginPos = taxiBeginUnadjustedPos + (disconnectDock.approachPath[disconnectDock.approachPath.Count - 1].direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
            Vector3D taxiEndPos = disconnectDock.approachPath[0].position + (disconnectDock.approachPath[0].direction * (APPROACH_SAFE_DISTANCE + Situation.radius)); //end taxi way pos
            //"Vector3D newPos" is still the destination
            //*************************** Remove Below 2 lines if breaks *****************************//
            Vector3D direction2 = Vector3D.Normalize(newPos - taxiBeginPos);
            Vector3D balancedDirection2 = Vector3D.ProjectOnPlane(ref direction2, ref Situation.gravityUpVector);
            //*************************** Remove Below line if breaks *****************************//
            if(!Situation.inGravity && Situation.alignDirectly)
                Navigation.AddWaypoint(taxiEndPos, direction2, Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(direction2)),
                    APPROACH_SPEED, Waypoint.wpType.ALIGNING);
            else
                Navigation.AddWaypoint(taxiEndPos, balancedDirection2, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING);
            foreach (VectorPath vp in disconnectDock.approachPath)
            {
                newPos = vp.position + (vp.direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
                Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, TAXIING_SPEED, Waypoint.wpType.TAXIING);
            }
        }

        direction = Vector3D.Normalize(Navigation.waypoints[0].stance.position - undockPos);
        balancedDirection = Vector3D.Normalize(Vector3D.ProjectOnPlane(ref direction, ref Situation.gravityUpVector));
        //Navigation.AddWaypoint(undockPos, balancedDirection, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING); //original alignment
        if (!Situation.inGravity && Situation.alignDirectly)
        {
            Navigation.AddWaypoint(undockPos, direction, Vector3D.Normalize(Vector3D.CalculatePerpendicularVector(direction)),
                APPROACH_SPEED, Waypoint.wpType.ALIGNING);
        }
        else
        {
            Navigation.AddWaypoint(undockPos, balancedDirection, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING);
        }
        Navigation.AddWaypoint(undockPos, Situation.forwardVector, Situation.upVector, DOCK_SPEED, Waypoint.wpType.UNDOCKING);

    }

    public static void Undock()
    {
        Situation.RefreshParameters();

        if (Situation.linearVelocity.Length() >= 2.0f)
        {
            return;

        }

        disconnectDock = ConnectorControl.DisconnectAndTaxiData();
        direction = Vector3D.Normalize(newPos - Situation.position);
        balancedDirection = Vector3D.ProjectOnPlane(ref direction, ref Situation.gravityUpVector);
        if (disconnectDock == null)
        {
            Navigation.AddWaypoint(Situation.position, balancedDirection, Situation.gravityUpVector, MAX_SPEED, Waypoint.wpType.ALIGNING);
            return;
        }
        //Signal.Send(Signal.SignalType.UNDOCK);
        //Logger.Info("Sending undock signal.");
        if (disconnectDock.approachPath.Count > 0)
        {
            foreach (VectorPath vp in disconnectDock.approachPath)
            {
                newPos = vp.position + (vp.direction * (APPROACH_SAFE_DISTANCE + Situation.radius));
                Navigation.AddWaypoint(newPos, Vector3D.Zero, Vector3D.Zero, TAXIING_SPEED, Waypoint.wpType.TAXIING);
            }
        }
        undockPos = disconnectDock.stance.forward;
        undockPos *= (Situation.radius + UNDOCK_DISTANCE);
        undockPos += Situation.position;
        Navigation.AddWaypoint(undockPos, balancedDirection, Situation.gravityUpVector, APPROACH_SPEED, Waypoint.wpType.ALIGNING);
        Navigation.AddWaypoint(undockPos, Situation.forwardVector, Situation.upVector, DOCK_SPEED, Waypoint.wpType.UNDOCKING);

    }
    public static void Start()
    {
        Start(DockData.GetSelected());

    }
    public static void Start(Dock d)
    {
        if (d == null)
        {
            return;

        }

        if (!RemoteControl.PresentOrLog())
        {
            return;

        }

        Stop();
        Logger.Info(MSG_NAVIGATION_TO + "[" + d.gridName + "] " + d.blockName);
        dock.Add(d);
        Signal.Send(Signal.SignalType.START);
        SetEndStance(d);
        running = true;

    }
    public static void Start(Waypoint w)
    {
        if (!RemoteControl.PresentOrLog())
        {
            return;

        }

        if (w.stance.position == Vector3D.Zero)
        {
            Logger.Err(MSG_INVALID_GPS_TYPE);
            return;
        }
        Stop();


        /// EXPERIMENTAL CODE
        Dock d = Dock.NewDock(w.stance.position, w.stance.forward, w.stance.up, "GPS Cord");
        Logger.Info(MSG_NAVIGATION_TO + "[" + d.gridName + "] " + d.blockName);
        Logger.Info(MSG_NAVIGATION_TO_WAYPOINT);
        dock.Add(d);
        Signal.Send(Signal.SignalType.START);
        SetEndStance(d);
        /// EXPERIMENTAL CODE End


        ////**************** Remove below if it breaks the script *****************
        //newPos = w.stance.position;

        //Logger.Info(MSG_NAVIGATION_TO_WAYPOINT);
        //Navigation.AddWaypoint(w);
        ////*********************If below line doesn't work, remove it.*********************
        //Navigation.AddWaypoint(w.stance.position, Vector3D.Zero, Vector3D.Zero, CONVERGING_SPEED, Waypoint.wpType.CONVERGING);
        ////*************** If below doesn't work, delete me *************
        //Undock();

        running = true;

    }

    public static void StartUndock()
    {
        if (!RemoteControl.PresentOrLog())
        {
            return;

        }

        Undock();
        running = true;
    }

    public static void Stop()
    {
        Navigation.Stop();
        dock.Clear();
        running = false;

    }
    public static void Toggle()
    {
        if (running)
        {
            Stop();
            return;
        }
        Start();

    }
}
private IMyBroadcastListener listener;
private bool clearStorage = false;
public Program()
{
    try
    {
        if (this.Load())
        {
            Logger.Info("Loaded previous session");
        }
    }
    catch (Exception e)
    {
        Logger.Warn("Unable to load previous session: " + e.Message);
        Storage = "";
    }
    Runtime.UpdateFrequency = UpdateFrequency.Update100 | UpdateFrequency.Update10 | UpdateFrequency.Once;
    listener = IGC.RegisterBroadcastListener(TAG);
    listener.SetMessageCallback(TAG);
    Signal.thisProgram = this;
}
public bool Load()
{
    if (Storage.Length != 0)
    {
        Logger.Info("Loading session size: " + Storage.Length);
        if (StorageData.Load(Storage))
        {
            return true;

        }

        Logger.Warn("Unable to Load previous session due to different version");

    }
    return false;

}
public void Save()
{
    string str = clearStorage ? "" : StorageData.Save();
    Logger.Info("Saving session size: " + str.Length);
    Storage = str;

}
public static class StorageData
{
    public static string Save()
    {
        Serializer.InitPack();
        Serializer.Pack(STORAGE_VERSION);
        Serializer.Pack(DockData.currentDockCount);
        Serializer.Pack(DockData.docks);
        Serializer.Pack(Pilot.dock);
        List<int> selected = new List<int>();
        foreach (Dock d in DockData.selectedDocks)
        {
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
        return Serializer.serialized;

    }
    public static bool Load(string str)
    {
        Serializer.InitUnpack(str);
        if (STORAGE_VERSION != Serializer.UnpackString())
        {
            return false;

        }

        DockData.currentDockCount = Serializer.UnpackInt();
        DockData.docks = Serializer.UnpackListDock();
        Pilot.dock = Serializer.UnpackListDock();
        List<int> selected = Serializer.UnpackListInt();
        DockData.selectedDocks.Clear();
        foreach (int i in selected)
        {
            DockData.selectedDocks.Add(DockData.docks[i]);
        }
        DockData.dynamic.Clear();
        foreach (Dock d in DockData.docks)
        {
            if (d.gridEntityId == 0)
            {
                continue;
            }
            DockData.dynamic[d.blockEntityId] = d;
            if (Pilot.dock.Count != 0 && Pilot.dock[0].blockEntityId == d.blockEntityId)
            {
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
        return true;

    }
}
public void HandleCommand(ref string command)
{
    string[] parts = command.Trim().Split(' ');
    parts.DefaultIfEmpty("");
    string arg0 = parts.ElementAtOrDefault(0).ToUpper();
    string arg1 = parts.ElementAtOrDefault(1);
    arg1 = arg1 ?? "";
    try
    {
        switch (arg0)
        {
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
                switch (arg1.ToUpper())
                {
                    case "stance":
                    case "STANCE":
                        Pannels.ScreenHandle(Pannels.ScreenAction.AddStance);
                        break;
                    case "":
                        Pannels.ScreenHandle(Pannels.ScreenAction.Add);
                        break;
                    case "dock":
                    case "DOCK":
                        Dock otherDock;
                        if (parts.Length > 2)
                        {
                            otherDock = DockSystem.GetDockFromConnected(string.Join(" ", parts.Skip(2).ToArray()));
                        }
                        else
                        {
                            otherDock = DockSystem.GetDockFromConnected();
                        }
                        if (otherDock == null)
                        {
                            Logger.Err("Unable to add, ship not connected.");
                            break;
                        }
                        otherDock.Touch();
                        DockData.currentDockCount++;
                        DockData.docks.Add(otherDock);
                        DockData.docks.Sort();
                        DockData.selectedDocks.Add(otherDock);
                        DockData.BalanceDisplays();
                        break;
                    //default:
                    //    Pannels.ScreenHandle(Pannels.ScreenAction.Add);
                    //    break;
                    default:
                        if (String.Join(" ", arg1).StartsWith("GPS")) //GPS:Au/Co/Fe Vain:46333.05:31840.77:29921.83:
                        {
                            string GPS = command.Trim().Substring(4).Trim();
                            //string GPS = String.Join(" ", arg1);
                            //string[] GPSsegments = GPS.Split(':');
                            Dock dock = Waypoint.DockFromGPS(GPS);
                            if (dock != null)
                            {
                                DockData.currentDockCount++;
                                DockData.docks.Add(dock);
                                DockData.docks.Sort();
                                DockData.BalanceDisplays();
                            }
                        }
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
                switch (arg1.ToUpper())
                {
                    case "prev":
                    case "PREV":
                        Signal.Clear();
                        DockData.NAVScreenHandle(Pannels.ScreenAction.Prev);
                        Pilot.Start();
                        break;
                    case "next":
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
                switch (arg1.ToUpper())
                {
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
            case "UNDOCK":
                Signal.Clear();
                Pilot.StartUndock();
                break;
            case "SCAN":
                ScanGrid();
                break;
            default:
                Logger.Err("Unknown command ->" + arg0 + "<-");
                break;
        }
    }
    catch (Exception e)
    {
        Logger.Err("Command exception -< " + command + " >-< " + e.Message + " >-");
    }
    try
    {
        //SendSignals();
    }
    catch (Exception e)
    {
        Logger.Err($"Signal send exception during command handling. -<{e.Message}>-");
    }
}
public static class MainHelper
{
    public delegate void Updater(ref string msg);
    public static void TimedRunIf(ref UpdateType update, UpdateType what, Updater run, ref string argument)
    {
        if ((update & what) == 0)
        {
            return;

        }

        TimeStats.Start(what.ToString());
        run(ref argument);
        update &= ~what;
        TimeStats.Stop(what.ToString());

    }
    public static void TimedRunDefault(ref UpdateType update, Updater run, ref string argument)
    {
        TimedRunIf(ref update, update, run, ref argument);

    }
    public static void WriteStats(Program p)
    {
        string str = String.Format("SAM v{0}. . .{1}\n", VERSION, Animation.Rotator());
        str += TimeStats.Results();
        str += String.Format("Load:{0:F3}%\n", 100.0 * (double)p.Runtime.CurrentInstructionCount / (double)p.Runtime.MaxInstructionCount);
        p.Echo(str);

    }
}
public void Main(string argument, UpdateType updateSource)
{
    try
    {
        MainHelper.TimedRunIf(ref updateSource, UpdateType.Once, this.Once, ref argument);
        MainHelper.TimedRunIf(ref updateSource, UpdateType.Update100, this.Update100, ref argument);
        //MainHelper.TimedRunIf(ref updateSource, UpdateType.Antenna, this.Antenna, ref argument);
        MainHelper.TimedRunIf(ref updateSource, UpdateType.IGC, this.UpdateIGC, ref argument);
        MainHelper.TimedRunIf(ref updateSource, UpdateType.Update10, this.Update10, ref argument);
        MainHelper.TimedRunDefault(ref updateSource, this.HandleCommand, ref argument);
        MainHelper.WriteStats(this);
    }
    catch (Exception e)
    {
        Logger.Err("Main exception: " + e.Message);
        Echo("Main exception: " + e.Message);
    }
}
public void Once(ref string unused)
{
    try
    {
        this.ScanGrid();
    }
    catch (Exception e)
    {
        Logger.Err("Once ScanGrid exception: " + e.Message);
    }
}
public void Antenna(ref string msg)
{
    try
    {
        DockSystem.Listen(msg);
    }
    catch (Exception e)
    {
        Logger.Err("Update100 Docks.Listen exception: " + e.Message);
    }
}
private MyIGCMessage igcData;
public void UpdateIGC(ref string msg)
{
    if (msg != TAG)
    {
        return;

    }

    while (listener.HasPendingMessage)
    {
        igcData = listener.AcceptMessage();
        if (igcData.Tag != TAG)
        {
            continue;
        }
        try
        {
            DockSystem.Listen((string)igcData.Data);
        }
        catch (Exception e)
        {
            Logger.Err("Antenna Docks.Listen exception: " + e.Message);
        }
    }
}
public void Update10(ref string unused)
{
    try
    {
        Pannels.Print();
    }
    catch (Exception e)
    {
        Logger.Err("Update10 Pannels.Print exception: " + e.Message);
    }
    try
    {
        Animation.Run();
    }
    catch (Exception e)
    {
        Logger.Err("Update10 Animation.Run exception: " + e.Message);
    }
    try
    {
        Pilot.Tick();
    }
    catch (Exception e)
    {
        Logger.Err("Update10 Pilot.Tick exception: " + e.Message);
    }
    if (Me.CustomName.Contains("DEBUG"))
    {
        this.DebugPrintLogging();

    }
}
int scanCountdown;
public void Update100(ref string unused)
{
    try
    {
        if (scanCountdown <= 0 || Pilot.running)
        {
            this.ScanGrid();
            scanCountdown = 10;
        }
        else
        {

            scanCountdown--;
        }


    }
    catch (Exception e)
    {
        Logger.Err("Update100 ScanGrid exception: " + e.Message);
    }
    try
    {
        DockSystem.Advertise(this);
    }
    catch (Exception e)
    {
        Logger.Err("Update100 Docks.Advertise exception: " + e.Message);
    }
    try
    {
        ConnectorControl.CheckConnect();
    }
    catch (Exception e)
    {
        Logger.Err("Update100 ConnectorControl.CheckConnect exception: " + e.Message);
    }
    try
    {
        this.SendSignals();
    }
    catch (Exception e)
    {
        Logger.Err("Update100 SendSignals exception: " + e.Message);
    }
    try
    {
        this.CheckSignals();
    }
    catch (Exception e)
    {
        Logger.Err("Update100 CheckSignals exception: " + e.Message);
    }
}
public static class DockData
{
    public static int currentDockCount = 0;
    public static int selectedDockNAV = 0, selectedTopNAV = 0;
    public static int selectedDockCONF = 0, selectedTopCONF = 0;
    public static List<Dock> selectedDocks = new List<Dock>();
    public static List<Dock> docks = new List<Dock>();
    public static Dictionary<long, Dock> dynamic = new Dictionary<long, Dock>();
    public static Dock GetDock(long entityId)
    {
        for (int i = 0; i < docks.Count; i++)
        {
            if (docks[i].blockEntityId == entityId)
            {
                return docks[i];
            }
        }
        return null;

    }
    public static Dock GetDock(string dockName)
    {
        for (int i = 0; i < docks.Count; i++)
        {
            if (docks[i].blockName == dockName)
            {
                return docks[i];
            }
        }
        throw new Exception("Connector ->" + dockName + "<- was not found.");

    }
    public static Dock GetSelected()
    {
        if (selectedDocks.Count == 0)
        {
            return null;

        }

        return selectedDocks[selectedDockNAV];

    }
    private static void SelectCONF()
    {
        if (selectedDockCONF < 0 || selectedDockCONF >= docks.Count)
        {
            return;

        }

        dock = docks[selectedDockCONF];
        if (selectedDocks.Contains(dock))
        {
            selectedDocks.Remove(dock);

        }
        else
        {
            selectedDocks.Add(dock);

        }

        BalanceDisplays();

    }
    private static void AddDock(bool stance)
    {
        Dock d = Dock.NewDock(RemoteControl.block.CenterOfMass,
            stance ? RemoteControl.block.WorldMatrix.Forward : Vector3D.Zero,
            stance ? RemoteControl.block.WorldMatrix.Up : Vector3D.Zero,
            Helper.FormatedWaypoint(stance, currentDockCount));
        currentDockCount++;
        docks.Add(d);
        docks.Sort();
        BalanceDisplays();

    }
    private static void RemDock()
    {
        if (selectedDockCONF < 0 || selectedDockCONF >= docks.Count)
        {
            return;

        }

        dock = docks[selectedDockCONF];
        if (dock.gridEntityId != 0 && dock.Fresh())
        {
            return;

        }

        selectedDocks.Remove(dock);
        docks.Remove(dock);
        dynamic.Remove(dock.blockEntityId);
        BalanceDisplays();

    }
    public static void BalanceDisplays()
    {
        if (selectedDockNAV < 0)
        {
            selectedDockNAV = selectedDocks.Count - 1;

        }

        if (selectedDockNAV >= selectedDocks.Count)
        {
            selectedDockNAV = 0;

        }

        if (selectedDockNAV < selectedTopNAV)
        {
            selectedTopNAV = selectedDockNAV;

        }

        if (selectedDockNAV >= selectedTopNAV + MAX_ENTRIES_NAV)
        {
            selectedTopNAV = selectedDockNAV - MAX_ENTRIES_NAV + 1;

        }

        if (selectedDockCONF < 0)
        {
            selectedDockCONF = docks.Count - 1;

        }

        if (selectedDockCONF >= docks.Count)
        {
            selectedDockCONF = 0;

        }

        if (selectedDockCONF < selectedTopCONF)
        {
            selectedTopCONF = selectedDockCONF;

        }

        if (selectedDockCONF >= selectedTopCONF + MAX_ENTRIES_CONF)
        {
            selectedTopCONF = selectedDockCONF - MAX_ENTRIES_CONF + 1;

        }
    }
    public static void NAVScreenHandle(Pannels.ScreenAction sa)
    {
        switch (sa)
        {
            case Pannels.ScreenAction.Prev:
                --selectedDockNAV;
                break;

            case Pannels.ScreenAction.Next:
                ++selectedDockNAV;
                break;

            case Pannels.ScreenAction.Select: break;

            case Pannels.ScreenAction.Add:
                AddDock(false);
                break;

            case Pannels.ScreenAction.AddStance:
                AddDock(true);
                break;

            case Pannels.ScreenAction.Rem: break;

        }
        BalanceDisplays();

    }
    public static void CONFScreenHandle(Pannels.ScreenAction sa)
    {
        switch (sa)
        {
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
                AddDock(false);
                break;

            case Pannels.ScreenAction.AddStance:
                AddDock(true);
                break;

            case Pannels.ScreenAction.Rem:
                RemDock();
                break;

        }
        BalanceDisplays();

    }
    private static Dock dock;
    private static string str, status;
    private static int index;
    private static string NAV_HEADER_ACTIVE = " Navigation " + new String('=', 43 - 12) + "\n";
    private static string NAV_HEADER_NOT_ACTIVE = " Navigation " + new String('-', 43 - 12) + "\n";
    private static int MAX_ENTRIES_NAV = 11;
    public static string PrintBufferNAV(bool active)
    {
        str = Animation.Rotator() + (active ? NAV_HEADER_ACTIVE : NAV_HEADER_NOT_ACTIVE);
        status = Navigation.Status();
        str += "   " + ((status == "" && !Pilot.running) ? "disabled" : status) + "\n";
        if (selectedDocks.Count() == 0)
        {
            return str + "\n - No docks selected.\n   Use Configuration\n   screen to select\n   them.";
        }
        str += (selectedTopNAV > 0) ? "     /\\/\\/\\\n" : "     ------\n";
        for (int i = 0;
        i < selectedDocks.Count;
        ++i)
        {
            if (i < selectedTopNAV || i >= selectedTopNAV + MAX_ENTRIES_NAV)
            {
                continue;
            }
            dock = selectedDocks[i];
            str += ((selectedDockNAV == i) ? " >" : "  ") + (dock.Fresh() ? "" : "? ") + "[" + dock.gridName + "] " + dock.blockName + "\n";
        }
        str += (selectedTopNAV + MAX_ENTRIES_NAV < selectedDocks.Count) ? "     \\/\\/\\/\n" : "     ------\n";
        return str;

    }
    private static string DCS_HEADER_ACTIVE = " Configuration " + new String('=', 43 - 15) + "\n";
    private static string DCS_HEADER_NOT_ACTIVE = " Configuration " + new String('-', 43 - 15) + "\n";
    private static int MAX_ENTRIES_CONF = 12;
    public static string PrintBufferCONF(bool active)
    {
        str = Animation.Rotator() + (active ? DCS_HEADER_ACTIVE : DCS_HEADER_NOT_ACTIVE);
        if (docks.Count() == 0)
        {
            return str + "\n - No available docks\n   to configure.";
        }
        str += (selectedTopCONF > 0) ? "     /\\/\\/\\\n" : "     ------\n";
        for (int i = 0;
        i < docks.Count;
        i++)
        {
            if (i < selectedTopCONF || i >= selectedTopCONF + MAX_ENTRIES_CONF)
            {
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
public static class DockSystem
{
    private static string connectorName;
    private static string panelName;
    private static List<VectorPath> approachPath = new List<VectorPath>();
    private static string Serialize()
    {
        Serializer.InitPack();
        Serializer.Pack(ADVERT_ID);
        if (!Block.GetProperty(GridBlocks.MasterProgrammableBlock.EntityId, "Name", ref connectorName))
        {
            connectorName = GridBlocks.MasterProgrammableBlock.CubeGrid.CustomName;

        }

        Serializer.Pack(GridBlocks.MasterProgrammableBlock.CubeGrid.EntityId);
        Serializer.Pack(connectorName);
        Serializer.Pack(GridBlocks.MasterProgrammableBlock.CubeGrid.GridSizeEnum);
        Serializer.Pack(GridBlocks.shipConnectors.Count());
        foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
        {
            Serializer.Pack(connector.EntityId);
            if (!Block.GetProperty(connector.EntityId, "Name", ref connectorName))
            {
                connectorName = connector.CustomName;
                //********************** REMOVE BELOW IF SCRIPT BREAKS ***************************
                //continue;
            }

            Serializer.Pack(connectorName);
            Serializer.Pack(connector.GetPosition());
            //***** Remove if statement if this breaks, the code in "else" was originally here *********
            if (Block.HasProperty(connector.EntityId, CONNECTOR_REVERSE_TAG))
            {
                Serializer.Pack(connector.WorldMatrix.Backward);
            }
            else
            {
                Serializer.Pack(connector.WorldMatrix.Forward);
            }
            Serializer.Pack(connector.WorldMatrix.Up);
            approachPath.Clear();
            foreach (IMyTextPanel panel in GridBlocks.textPanels)
            {
                if (!Block.GetProperty(panel.EntityId, "Name", ref panelName))
                {
                    continue;
                }
                if (panelName != connectorName)
                {
                    continue;
                }
                approachPath.Add(new VectorPath(panel.GetPosition(), -panel.WorldMatrix.Forward));
            }
            Serializer.Pack(approachPath);

        }
        return Serializer.serialized;

    }
    public static void Advertise(Program p)
    {
        if (!Block.HasProperty(GridBlocks.MasterProgrammableBlock.EntityId, "ADVERTISE"))
        {
            return;

        }

        if (GridBlocks.shipConnectors.Count() == 0)
        {
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
    public static void Listen(string message)
    {
        Serializer.InitUnpack(message);
        if (ADVERT_ID != Serializer.UnpackString())
        {
            return;

        }

        gridEntityId = Serializer.UnpackLong();
        gridName = Serializer.UnpackString();
        gridSize = Serializer.UnpackCubeSize();
        count = Serializer.UnpackInt();
        for (int i = 0; i < count; ++i)
        {
            blockEntityId = Serializer.UnpackLong();
            if (DockData.dynamic.ContainsKey(blockEntityId))
            {
                dock = DockData.dynamic[blockEntityId];
                exists = true;
            }
            else
            {
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
            dock.cubeSize = gridSize;
            dock.approachPath = Serializer.UnpackListVectorPath();
            dock.SortApproachVectorsByDistance(dock.stance.position);
            if (!exists)
            {
                DockData.dynamic[blockEntityId] = dock;
                DockData.docks.Add(dock);
                DockData.docks.Sort();
            }
        }
    }

    public static Dock GetDockFromConnected(string gridname = null)
    {
        List<IMyShipConnector> connectors = GridBlocks.shipConnectors;
        if (connectors.Count == 0)
        {
            return null;
        }
        IMyShipConnector dockedConnector = null;
        foreach (IMyShipConnector c in connectors)
        {
            if (c.Status == MyShipConnectorStatus.Connected)
            {
                dockedConnector = c;
                break;
            }
        }
        if (dockedConnector == null)
        {
            return null;
        }

        IMyShipConnector otherConnector = dockedConnector.OtherConnector;
        Dock dock = new Dock();
        dock.blockEntityId = otherConnector.EntityId;
        dock.blockName = otherConnector.CustomName;
        dock.cubeSize = otherConnector.CubeGrid.GridSizeEnum;
        dock.gridEntityId = otherConnector.CubeGrid.EntityId;
        dock.gridName = otherConnector.CubeGrid.CustomName;
        if (gridname != null)
        {
            dock.gridName = gridname;
        }

        dock.stance = new Stance(otherConnector.GetPosition(), otherConnector.WorldMatrix.Forward, otherConnector.WorldMatrix.Up);
        //dock.stance.forward = ;
        //dock.stance.up = ;
        //dock.stance.position = ;

        return dock;
    }
}
public static class CustomData
{
    public static System.Text.RegularExpressions.Regex customDataRegex = new System.Text.RegularExpressions.Regex("\\s*" + TAG + "\\.([a-zA-Z0-9_]*)([:=]{1}([\\S ]*))?", System.Text.RegularExpressions.RegexOptions.IgnoreCase);
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
    //private static string trim;
    public static bool Sanitize(ref IMyTerminalBlock block, ref BlockProfile profile)
    {
        lines = block.CustomData.Split(lineSeparator);
        build = "";
        exclusiveFound = false;
        matched = false;
        entityId = block.EntityId;
        foreach (string line in lines)
        {
            string trimmed = line.Trim(); // ******** REMOVE THIS IF BREAKS ********************
            if (trimmed == "")
            {
                //build += line + "\n";
                continue;
            }
            match = customDataRegex.Match(trimmed); //Was passing in "line"
            matched = match.Success || matched;
            if (match.Groups.Count == 4)
            {
                if (match.Groups[1].Value != "")
                {
                    if (match.Groups[3].Value != "")
                    {
                        attributeCap = Helper.Capitalize(match.Groups[1].Value);
                        if (profile.attributes.Contains(attributeCap))
                        {
                            value = match.Groups[3].Value;
                            build += TAG + "." + attributeCap + "=" + value + "\n";
                            Block.UpdateProperty(entityId, attributeCap, value);
                            continue;
                        }
                    }
                    else
                    {
                        tagUpper = match.Groups[1].Value.ToUpper();
                        if (profile.exclusiveTags.Contains(tagUpper))
                        {
                            if (exclusiveFound)
                            {
                                build += line + "\n";
                                continue;
                            }
                            exclusiveFound = true;
                        }
                        else if (!profile.tags.Contains(tagUpper))
                        {
                            build += line + "\n";
                            continue;
                        }
                        Block.UpdateProperty(entityId, tagUpper, "");
                        build += TAG + "." + tagUpper + "\n";
                        continue;
                    }
                }
                build += TAG + ".\n";
                continue;
            }
            else
            {
                build += line + "\n";
            }
            //trim = line.Trim();
            //if (trim == "")
            //{
            //continue;
            //}
            //build += trim + "\n";
        }
        if (matched)
            block.CustomData = build;
        return matched;

    }
}
public static class CustomName
{
    private static char[] attributeSeparator = new char[] { //':', //********* uncomment this if the lack of the ":" breaks *************
        '=' };
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
    public static bool Sanitize(ref IMyTerminalBlock block, ref BlockProfile profile)
    {
        simpleMatch = tagSimpleRegex.Match(block.CustomName);
        if (!simpleMatch.Success)
        {
            return false;

        }

        match = tagRegex.Match(simpleMatch.Groups[1].Value);
        if (!match.Success)
        {
            return false;

        }

        entityId = block.EntityId;
        foundExclusive = false;
        build = "[" + TAG;
        for (int i = 1; i < match.Groups.Count; ++i)
        {
            subTag = match.Groups[i].Value;
            if (subTag == "")
            {
                break;
            }
            subTagUpper = subTag.ToUpper();
            if (profile.exclusiveTags.Contains(subTagUpper))
            {
                if (foundExclusive)
                {
                    continue;
                }
                foundExclusive = true;
                build += " " + subTagUpper;
                Block.UpdateProperty(entityId, subTagUpper, "");
                continue;
            }
            if (profile.tags.Contains(subTagUpper))
            {
                build += " " + subTagUpper;
                Block.UpdateProperty(entityId, subTagUpper, "");
                continue;
            }
            attributePair = subTag.Split(attributeSeparator);
            if (attributePair.Count() > 1)
            {
                attributeCap = Helper.Capitalize(attributePair[0]);
                if (profile.attributes.Contains(attributeCap))
                {
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
public static class Profiles
{
    private static string[] empty = new string[] { };
    private static string[] meTags = new string[] { "DEBUG", "ADVERTISE", ESCAPE_NOSE_UP_TAG, SLOW_ON_APPROACH_TAG, ALLOW_DIRECT_ALIGNMENT_TAG };
    private static string[] textPanelTags = new string[] { "OVR" };
    private static string[] textPanelExclusiveTags = new string[] { "LOG", "NAV", "CONF", "DATA" };
    private static string[] connectorTags = new string[] { CONNECTOR_REVERSE_TAG };
    private static string[] cockpitAttributes = new string[] { "Slot" };
    private static string[] pbAttributes = new string[] { "Name", "Speed", "Wait" , TAXI_SPEED_TAG, CONVERGING_SPEED_TAG, APPROACH_DISTANCE_TAG, DOCK_DISTANCE_TAG,
    DOCK_SPEED_TAG, UNDOCK_DISTANCE_TAG, APPROACH_SPEED_TAG, APPROACH_SAFE_DISTANCE_TAG, ARRIVAL_SPEED_TAG, ARRIVAL_DISTANCE_TAG, ESCAPE_NOSE_UP_ELEVATION_TAG,
    AUTO_CRUISE_ATTRIBUTE, DESCEND_NOSE_DOWN_ELEVATION_TAG};
    private static string[] namedAttributes = new string[] { "Name" };
    private static string[] timerTags = new string[] { "DOCKED", "NAVIGATED", "STARTED", "UNDOCKED", "APPROACHING" };
    public static Dictionary<Type, BlockProfile> perType = new Dictionary<Type, BlockProfile> { { typeof(IMyRemoteControl),
            new BlockProfile(ref empty, ref empty, ref empty) },
        { typeof(IMyCameraBlock),
            new BlockProfile(ref empty, ref empty, ref empty) },
        { typeof(IMyRadioAntenna), new BlockProfile(ref empty, ref empty, ref empty) },
        { typeof(IMyLaserAntenna), new BlockProfile(ref empty, ref empty, ref empty) },
        { typeof(IMyProgrammableBlock), new BlockProfile(ref empty, ref empty, ref pbAttributes) },
        { typeof(IMyShipConnector), new BlockProfile(ref connectorTags, ref empty, ref namedAttributes) },
        { typeof(IMyTextPanel), new BlockProfile(ref textPanelTags, ref textPanelExclusiveTags, ref namedAttributes) },
        { typeof(IMyTimerBlock), new BlockProfile(ref timerTags, ref empty, ref empty) },
        { typeof(IMyCockpit), new BlockProfile(ref empty, ref empty, ref cockpitAttributes) } };
    public static BlockProfile me = new BlockProfile(ref meTags, ref empty, ref pbAttributes);

}
public static class Block
{
    public static bool ValidType(ref IMyTerminalBlock block, Type type)
    {
        return ValidProfile(ref block, Profiles.perType[type]);

    }
    public static bool ValidProfile(ref IMyTerminalBlock block, BlockProfile profile)
    {
        bool valid = false; //********************** <---- REMOVE THIS IF IT BREAKS ****************************
        if (CustomName.Sanitize(ref block, ref profile))
        {
            //return true;
            valid = true;
        }

        if (CustomData.Sanitize(ref block, ref profile))
        {
            //return true;
            valid = true;
        }

        //return false;
        return valid;

    }
    private static Dictionary<long, Dictionary<string, string>> properties = new Dictionary<long, Dictionary<string, string>>();
    public static void UpdateProperty(long entityId, string property, string value)
    {
        if (properties.ContainsKey(entityId))
        {
            properties[entityId][property] = value;

        }
        else
        {
            properties[entityId] = new Dictionary<string, string> { { property, value } };

        }
    }
    public static void ClearProperties()
    {
        foreach (KeyValuePair<long, Dictionary<string, string>> entities in properties)
        {
            entities.Value.Clear();
        }
    }
    public static bool HasProperty(long entityId, string name)
    {
        if (!properties.ContainsKey(entityId))
        {
            return false;

        }

        if (!properties[entityId].ContainsKey(name))
        {
            return false;

        }

        return true;

    }
    public static bool GetProperty(long entityId, string name, ref string value)
    {
        if (!HasProperty(entityId, name))
        {
            return false;

        }

        value = properties[entityId][name];
        return true;

    }
}
public static class GridBlocks
{
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
    public static List<IMyCockpit> cockpitBlocks = new List<IMyCockpit>();
    public static List<long> timerBlocks = new List<long>();
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
    public static void Clear()
    {
        foreach (string key in blockCount.Keys)
        {
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

    }
    public static void UpdateCount(string key)
    {
        if (blockCount.ContainsKey(key))
        {
            blockCount[key].newC++;

        }
        else
        {
            blockCount[key] = new PairCounter();

        }
    }
    public static void LogDifferences()
    {
        foreach (string key in blockCount.Keys)
        {
            var diff = blockCount[key].Diff();
            if (diff > 0)
            {
                Logger.Info(String.Format("Found {0}x {1}", diff, key));

            }
            else if (diff < 0)
            {
                Logger.Info(String.Format("Lost {0}x {1}", -diff, key));

            }
        }
    }
    public static bool AddBlock(IMyTerminalBlock block)
    {
        if ((remoteControl = block as IMyRemoteControl) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyRemoteControl)))
            {
                return false;

            }

            remoteControls.Add(remoteControl);

        }
        else if ((cameraBlock = block as IMyCameraBlock) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyCameraBlock)))
            {
                return false;

            }

            cameraBlocks.Add(cameraBlock);

        }
        else if ((radioAntenna = block as IMyRadioAntenna) != null)
        {
            radioAntennas.Add(radioAntenna);
        }
        else if ((laserAntenna = block as IMyLaserAntenna) != null)
        {
            laserAntennas.Add(laserAntenna);
        }
        else if ((programmableBlock = block as IMyProgrammableBlock) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyProgrammableBlock)))
            {
                return false;

            }

            programmableBlocks.Add(programmableBlock);

        }
        else if ((shipConnector = block as IMyShipConnector) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyShipConnector)))
            {
                return false;

            }

            shipConnectors.Add(shipConnector);

        }
        else if ((textPanel = block as IMyTextPanel) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyTextPanel)))
            {
                return false;

            }

            textPanels.Add(textPanel);

        }
        else if ((gyroBlock = block as IMyGyro) != null)
        {
            gyroBlocks.Add(gyroBlock);
        }
        else if ((thrustBlock = block as IMyThrust) != null)
        {
            thrustBlocks.Add(thrustBlock);
        }
        else if ((timerBlock = block as IMyTimerBlock) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyTimerBlock)))
            {
                return false;

            }

            timerBlocks.Add(timerBlock.EntityId);

        }
        else if ((cockpitBlock = block as IMyCockpit) != null)
        {
            if (!Block.ValidType(ref block, typeof(IMyCockpit)))
            {
                return false;
            }

            cockpitBlocks.Add(cockpitBlock);
        }
        else
        {
            return false;
        }
        return true;

    }
    public static void AddMe(IMyProgrammableBlock me)
    {
        MasterProgrammableBlock = me;
        terminalBlock = me as IMyTerminalBlock;
        if (!Block.ValidProfile(ref terminalBlock, Profiles.me))
        {
            me.CustomName += " [" + TAG + "]";
            return;
        }
        var speed = "";
        var speedInt = 0;
        if (Block.GetProperty(terminalBlock.EntityId, "Speed", ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (MAX_SPEED != (float)speedInt)
                {
                    MAX_SPEED = (float)speedInt;
                    Logger.Info("Maximum speed changed to " + MAX_SPEED);
                }
            }
        }
        if (Block.GetProperty(terminalBlock.EntityId, TAXI_SPEED_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (TAXIING_SPEED != (float)speedInt)
                {
                    TAXIING_SPEED = (float)speedInt;
                    Logger.Info("Taxiing speed changed to " + TAXIING_SPEED);
                }
            }
        }
        var distanceInt = 3;
        if (Block.GetProperty(terminalBlock.EntityId, APPROACH_DISTANCE_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out distanceInt))
            {
                if (APPROACH_DISTANCE != (float)distanceInt)
                {
                    APPROACH_DISTANCE = (float)distanceInt;
                    Logger.Info("Approach distance changed to " + APPROACH_DISTANCE);
                }
            }
        }
        if (Block.GetProperty(terminalBlock.EntityId, DOCK_DISTANCE_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out distanceInt))
            {
                if (DOCK_DISTANCE != (float)distanceInt)
                {
                    DOCK_DISTANCE = (float)distanceInt;
                    Logger.Info("Dock distance changed to " + DOCK_DISTANCE);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, DOCK_SPEED_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (DOCK_SPEED != (float)speedInt)
                {
                    DOCK_SPEED = (float)speedInt;
                    Logger.Info("Dock speed changed to " + DOCK_SPEED);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, UNDOCK_DISTANCE_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out distanceInt))
            {
                if (UNDOCK_DISTANCE != (float)distanceInt)
                {
                    UNDOCK_DISTANCE = (float)distanceInt;
                    Logger.Info("Undocking distance changed to " + UNDOCK_DISTANCE);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, APPROACH_SPEED_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (APPROACH_SPEED != (float)speedInt)
                {
                    APPROACH_SPEED = (float)speedInt;
                    Logger.Info("Approach speed changed to " + APPROACH_SPEED);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, APPROACH_SAFE_DISTANCE_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out distanceInt))
            {
                if (APPROACH_SAFE_DISTANCE != (float)distanceInt)
                {
                    APPROACH_SAFE_DISTANCE = (float)distanceInt;
                    Logger.Info("Taxiing distance changed to " + APPROACH_SAFE_DISTANCE);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, CONVERGING_SPEED_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (CONVERGING_SPEED != (float)speedInt)
                {
                    CONVERGING_SPEED = (float)speedInt;
                    Logger.Info("Converging speed changed to " + CONVERGING_SPEED);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, ARRIVAL_SPEED_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (ARRIVAL_SPEED != (float)speedInt)
                {
                    ARRIVAL_SPEED = (float)speedInt;
                    Logger.Info("Arrival speed changed to " + ARRIVAL_SPEED);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, ARRIVAL_DISTANCE_TAG, ref speed))
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (ARRIVAL_DISTANCE != (float)speedInt)
                {
                    ARRIVAL_DISTANCE = (float)speedInt;
                    Logger.Info("Arrival distance changed to " + ARRIVAL_DISTANCE);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, ESCAPE_NOSE_UP_ELEVATION_TAG, ref speed) && Situation.allowEscapeNoseUp)
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (ESCAPE_NOSE_UP_ELEVATION != (float)speedInt)
                {
                    ESCAPE_NOSE_UP_ELEVATION = (float)speedInt;
                    Logger.Info("Escape nose up ground-to-air elevation changed to " + ESCAPE_NOSE_UP_ELEVATION);
                }
            }
        }

        if (Block.GetProperty(terminalBlock.EntityId, DESCEND_NOSE_DOWN_ELEVATION_TAG, ref speed) && Situation.allowEscapeNoseUp)
        {
            if (Int32.TryParse(speed, out speedInt))
            {
                if (Situation.noseDownElevation != (float)speedInt)
                {
                    Situation.noseDownElevation = (float)speedInt;
                    Logger.Info($"Descend nose down ground-to-air elevation changed to {Situation.noseDownElevation:N0}");
                }
            }
        }
        else if (Situation.noseDownElevation != ESCAPE_NOSE_UP_ELEVATION && Situation.allowEscapeNoseUp)
        {
            Logger.Warn($"Nose down elevation not set. Matching nose up elevation...");
            Logger.Info("Use the custom data to set nose up elevation");
            Logger.Info($"Custom data reference: SAM.{DESCEND_NOSE_DOWN_ELEVATION_TAG}=<number>");
            Situation.noseDownElevation = ESCAPE_NOSE_UP_ELEVATION;
        }

        string dist = string.Empty;
        double outDist = Situation.autoCruiseAltitude;
        if (Block.GetProperty(terminalBlock.EntityId, AUTO_CRUISE_ATTRIBUTE, ref dist))
        {
            if (double.TryParse(dist, out outDist))
            {
                if (Situation.autoCruiseAltitude != (float)outDist)
                {
                    Situation.autoCruiseAltitude = (float)outDist;
                    Logger.Info($"Autocruise set to {Situation.autoCruiseAltitude:N0}.");
                }
            }
        }
        else if(!double.IsNaN(Situation.autoCruiseAltitude))
        {
            Situation.autoCruiseAltitude = double.NaN;
            Logger.Info("Autocruise disabled. Atmospheric flight might be slow.");
        }

        bool noseUp = Block.HasProperty(terminalBlock.EntityId, ESCAPE_NOSE_UP_TAG);
        if (noseUp != Situation.allowEscapeNoseUp)
        {
            Situation.allowEscapeNoseUp = noseUp;
            Logger.Info(noseUp ? "Escape atmosphere nose up is now enabled" : "Escape atmosphere nose up is now disabled.");
        }

        bool slowDown = Block.HasProperty(terminalBlock.EntityId, SLOW_ON_APPROACH_TAG);
        if (slowDown != Situation.slowOnApproach)
        {
            Situation.slowOnApproach = slowDown;
            Logger.Info(slowDown ? "Slow down when approaching enabled" : "Slow down when approaching disabled");
        }

        bool alignDirectly = Block.HasProperty(terminalBlock.EntityId, ALLOW_DIRECT_ALIGNMENT_TAG);
        if (alignDirectly != Situation.alignDirectly)
        {
            Situation.alignDirectly = alignDirectly;
            Logger.Info(alignDirectly ? "Now aligning directly in space." : "aligning directly in space disabled");
        }

        /*if (Block.GetProperty(terminalBlock.EntityId, TAXI_SPEED_TAG, ref speed))
                {
                    if (Int32.TryParse(speed, out speedInt))
                    {
                        if (TAXIING_SPEED != (float)speedInt)
                        {
                            TAXIING_SPEED = (float)speedInt;
                            Logger.Info("Taxiinng speed changed to " + TAXIING_SPEED);
                        }
                    }
                }*/
    }
    private static int xB, yB;
    private static int CompareThrusters(IMyThrust x, IMyThrust y)
    {
        xB = yB = 0;
        if (x.DefinitionDisplayNameText.Contains("Hydrogen "))
        {
            xB += 4;

        }
        else if (x.DefinitionDisplayNameText.Contains("Ion "))
        {
            xB += 2;

        }

        if (x.DefinitionDisplayNameText.Contains("Large "))
        {
            xB += 1;

        }

        if (y.DefinitionDisplayNameText.Contains("Hydrogen "))
        {
            yB += 4;

        }
        else if (y.DefinitionDisplayNameText.Contains("Ion "))
        {
            yB += 2;

        }

        if (y.DefinitionDisplayNameText.Contains("Large "))
        {
            yB += 1;

        }

        return xB - yB;

    }
    public static void EvaluateRemoteControls()
    {
        if (remoteControls.Count() == 1)
        {
            RemoteControl.block = remoteControls[0];
            ErrorState.Reset(ErrorState.Type.NoRemoteController);
            ErrorState.Reset(ErrorState.Type.TooManyControllers);
            return;
        };
        RemoteControl.block = null;
        if (!ErrorState.Get(ErrorState.Type.TooManyControllers) && remoteControls.Count() > 1)
        {
            ErrorState.Set(ErrorState.Type.TooManyControllers);
            Logger.Err("Too many remote controllers");
        }
    }
    public static void EvaluateCameraBlocks()
    {
        foreach (IMyCameraBlock cameraBlock in cameraBlocks)
        {
            if (!cameraBlock.EnableRaycast)
            {
                cameraBlock.EnableRaycast = true;

            }
        }
    }
    public static void EvaluateThrusters()
    {
        thrustBlocks.Sort(CompareThrusters);

    }
}
public void ScanGrid()
{
    Block.ClearProperties();
    GridBlocks.Clear();
    GridBlocks.AddMe(Me);
    this.GridTerminalSystem.GetBlocks(GridBlocks.terminalBlocks);
    //this.GridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(GridBlocks.terminalBlocks, b => b.IsSameConstructAs(Me));
    foreach (IMyTerminalBlock block in GridBlocks.terminalBlocks)
    {
        //************** Remove Text Panel check if script breaks *************
        if (block.CubeGrid != Me.CubeGrid && !(block is IMyTextPanel) && !(block is IMyShipConnector))
        {
            //********** Uncomment below if script breaks ********
            continue;

        }

        //*************** Remove if below if script breaks ****************
        if (!block.IsSameConstructAs(Me))
        {
            continue;
        }

        if (block.EntityId == Me.EntityId)
        {
            continue;

        }

        if (GridBlocks.AddBlock(block))
        {
            GridBlocks.UpdateCount(block.DefinitionDisplayNameText);

        }
    }
    GridBlocks.EvaluateThrusters();
    GridBlocks.EvaluateCameraBlocks();
    GridBlocks.EvaluateRemoteControls();
    GridBlocks.LogDifferences();

}
public static class Signal
{
    public enum SignalType { DOCK, NAVIGATION, START, UNDOCK, APPROACH };
    public static HashSet<SignalType> list = new HashSet<SignalType>();
    public static long lastSignal = long.MaxValue;
    public static Program thisProgram;
    public static int signalAttempt = SIGNAL_MAX_ATTEMPTS;
    public const int SIGNAL_MAX_ATTEMPTS = 5;
    public static void Send(SignalType signal)
    {
        list.Add(signal);
        lastSignal = DateTime.Now.Ticks;
        if (signal == SignalType.UNDOCK)
        {
            Logger.Info("Undock signal received.");
            thisProgram.SendSignals();
        }
    }
    public static void Clear()
    {
        lastSignal = long.MaxValue;
        list.Clear();

    }
}
private string timeString;
private double timeSec;
private long timeTicks;
public void CheckSignals()
{
    if (!Block.GetProperty(Me.EntityId, "Wait", ref timeString))
    {
        return;

    }

    if (!Double.TryParse(timeString, out timeSec))
    {
        return;

    }

    timeTicks = TimeSpan.FromSeconds(timeSec).Ticks;
    if (DateTime.Now.Ticks - Signal.lastSignal < timeTicks)
    {
        return;

    }

    DockData.NAVScreenHandle(Pannels.ScreenAction.Next);
    Pilot.Start();
    Signal.lastSignal = long.MaxValue;
    Logger.Info("Wait time expired, resuming navigation.");

}
List<IMyTimerBlock> timers = new List<IMyTimerBlock>();
public void SendSignals()
{
    if (Signal.list.Count == 0)
    {
        Signal.signalAttempt = Signal.SIGNAL_MAX_ATTEMPTS;
        return;

    }
    bool sentSignal = false;

    timers.Clear();
    GridTerminalSystem.GetBlocksOfType<IMyTimerBlock>(timers);
    foreach (IMyTimerBlock timer in timers)
    {
        foreach (long entityID in GridBlocks.timerBlocks)
        {
            if (timer.EntityId == entityID)
            {
                if (Block.HasProperty(entityID, "DOCKED") && Signal.list.Contains(Signal.SignalType.DOCK))
                {
                    Logger.Info("Timer started due to Docking accomplished");
                    timer.StartCountdown();
                    sentSignal = true;
                    //timer.ApplyAction("Start");
                }
                if (Block.HasProperty(entityID, "NAVIGATED") && Signal.list.Contains(Signal.SignalType.NAVIGATION))
                {
                    Logger.Info("Timer started due to Navigation finished");
                    //timer.ApplyAction("Start");
                    timer.StartCountdown();
                    sentSignal = true;
                }
                if (Block.HasProperty(entityID, "STARTED") && Signal.list.Contains(Signal.SignalType.START))
                {
                    Logger.Info("Timer started due to Navigation start");
                    timer.StartCountdown();
                    sentSignal = true;
                }
                if (Block.HasProperty(entityID, "UNDOCKED") && Signal.list.Contains(Signal.SignalType.UNDOCK))
                {
                    Logger.Info("Timer triggered due to undocking started");
                    timer.Trigger();
                    sentSignal = true;
                }
                if (Block.HasProperty(entityID, "APPROACHING") && Signal.list.Contains(Signal.SignalType.APPROACH))
                {
                    Logger.Info("Timer started due to approaching destination");
                    timer.StartCountdown();
                    sentSignal = true;
                }
            }
        }
    }
    if (sentSignal || Signal.signalAttempt == 0)
    {
        Signal.list.Clear();
        Signal.signalAttempt = Signal.SIGNAL_MAX_ATTEMPTS;
        Logger.Info("Sent all queued signals");
    }
    else
    {
        Signal.signalAttempt--;
    }

}
public static class RemoteControl
{
    public static IMyRemoteControl block = null;
    public static bool Present()
    {
        return block != null;

    }
    public static bool PresentOrLog()
    {
        if (Present())
        {
            return true;

        }

        Logger.Err(MSG_NO_REMOTE_CONTROL);
        return false;

    }
}
public static class ConnectorControl
{
    //public static IMyShipConnector block;
    private static int connectAttempts = 0;
    public static void AttemptConnect()
    {
        connectAttempts = DOCK_ATTEMPTS;
        doConnect();

    }
    public static void CheckConnect()
    {
        if (connectAttempts == 0)
        {
            return;

        }

        if (0 == --connectAttempts)
        {
            Logger.Info(MSG_FAILED_TO_DOCK);

        }
        else
        {
            doConnect();

        }
    }
    private static void doConnect()
    {
        if (!Connect())
        {
            return;

        }

        connectAttempts = 0;
        Logger.Info(MSG_DOCKING_SUCCESSFUL);
        Signal.Send(Signal.SignalType.DOCK);

    }
    private static bool connected;
    private static bool Connect()
    {
        connected = false;
        foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
        {
            connector.Connect();
            if (connector.Status == MyShipConnectorStatus.Connected)
            {
                connected = true;
                if (!connector.OtherConnector.CubeGrid.IsStatic)
                {
                    RemoteControl.block.DampenersOverride = false;
                }
            }
        }
        return connected;

    }
    private static Vector3D retractVector;
    public static Vector3D Disconnect()
    {
        retractVector = Vector3D.Zero;
        foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
        {
            if (connector.Status == MyShipConnectorStatus.Connected)
            {
                connector.Disconnect();
                retractVector = -connector.WorldMatrix.Forward;
            }
        }
        return retractVector;

    }
    private static Dock retractDock;
    public static Dock DisconnectAndTaxiData()
    {
        retractDock = null;
        foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
        {
            if (connector.Status == MyShipConnectorStatus.Connected)
            {
                var dock = DockData.GetDock(connector.OtherConnector.EntityId);
                if (dock != null)
                {
                    retractDock = dock;
                }
                else
                {
                    retractDock = Dock.NewDock(connector.OtherConnector.GetPosition(), connector.OtherConnector.WorldMatrix.Forward, connector.OtherConnector.WorldMatrix.Up, "D");
                }
                //Logger.Info("Sending undock signal...");
                Signal.Send(Signal.SignalType.UNDOCK);
                connector.Disconnect();
            }
        }
        return retractDock;

    }
    public static IMyShipConnector GetConnector(Dock refDock)
    {
        if (Math.Abs(Vector3D.Dot(refDock.stance.forward, RemoteControl.block.WorldMatrix.Up)) < 0.5f)
        {
            foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
            {
                bool reverse = Block.HasProperty(connector.EntityId, CONNECTOR_REVERSE_TAG);
                if (Math.Abs(Vector3D.Dot(reverse ? connector.WorldMatrix.Backward : connector.WorldMatrix.Forward, RemoteControl.block.WorldMatrix.Up)) < 0.5f)
                {
                    return connector;
                }
            }
        }
        else
        {
            foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
            {
                bool reverse = Block.HasProperty(connector.EntityId, CONNECTOR_REVERSE_TAG);
                if (Vector3D.Dot(reverse ? connector.WorldMatrix.Backward : connector.WorldMatrix.Forward, -refDock.stance.forward) > 0.5f)
                {
                    return connector;
                }
            }
        }
        foreach (IMyShipConnector connector in GridBlocks.shipConnectors)
        {
            return connector;
        }
        return null;

    }
}


public static class Pannels
{
    private static List<string> types = new List<string> { "NAV", "CONF", "LOG", "DATA" };
    private static Dictionary<string, string> buffer = new Dictionary<string, string>();
    private static Queue<string> selected = new Queue<string>(new List<string> { "NAV", "CONF" });
    private static string printBuffer, screen;
    private static void ResetBuffers()
    {
        foreach (string s in types)
        {
            buffer[s] = "";

        }
    }
    private static void FillPrintBuffer(string type)
    {
        printBuffer = buffer[type];
        if (printBuffer != "")
        {
            return;

        }

        screen = selected.Peek();
        switch (type)
        {
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
    public static void Print()
    {
        if (GridBlocks.textPanels.Count() == 0 &&
            GridBlocks.cockpitBlocks.Count() == 0)
        {
            Logger.Err("No text panels or cockpits found");
            return;

        }

        ResetBuffers();
        foreach (IMyTextPanel panel in GridBlocks.textPanels)
        {
            if (Block.HasProperty(panel.EntityId, "Name"))
            {
                break;
            }
            printBuffer = "";
            foreach (string type in types)
            {
                if (Block.HasProperty(panel.EntityId, type))
                {
                    FillPrintBuffer(type);
                    break;
                }
            }
            if (printBuffer == "")
            {
                FillPrintBuffer(selected.Peek());

            }

            panel.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;
            if (!Block.HasProperty(panel.EntityId, "OVR"))
            {
                panel.FontSize = 1.180f;
                panel.Font = "Monospace";
                panel.TextPadding = 0.0f;
            }
            panel.WriteText(printBuffer);

        }
        //printBuffer = "";
        FillPrintBuffer(selected.Peek());
        foreach (IMyCockpit pit in GridBlocks.cockpitBlocks)
        {

            string slotNumberStr = "";
            if (Block.GetProperty(pit.EntityId, "Slot", ref slotNumberStr))
            {
                int slot = 0;
                int.TryParse(slotNumberStr, out slot);
                //Logger.Info("Found cockpit");
                if (slot < pit.SurfaceCount)
                {
                    //Logger.Info("Outputing cockpit");
                    IMyTextSurface surface = pit.GetSurface(slot);
                    surface.ContentType = VRage.Game.GUI.TextPanel.ContentType.TEXT_AND_IMAGE;

                    surface.WriteText(printBuffer);
                }
                else
                {
                    //Logger.Warn("Cockpit surface not found.");
                }
            }
        }
    }
    public static void NextScreen()
    {
        selected.Enqueue(selected.Dequeue());

    }
    public enum ScreenAction { Prev, Next, Select, Add, Rem, AddStance };
    public static void ScreenHandle(ScreenAction sa)
    {
        switch (selected.Peek())
        {
            case "NAV":
                DockData.NAVScreenHandle(sa);
                break;

            case "CONF":
                DockData.CONFScreenHandle(sa);
                break;

            case "LOG": break;

        }
    }
}
public static class Logger
{
    private static List<string> logger = new List<string>();
    private static string str;
    private static string HEADER_ACTIVE = " SAM v" + Program.VERSION + " Logger " + new String('=', 43 - 14 - Program.VERSION.Length);
    private static string HEADER_NOT_ACTIVE = " SAM v" + Program.VERSION + " Logger " + new String('-', 43 - 14 - Program.VERSION.Length);
    public static void Log(string line)
    {
        logger.Insert(0, line);
        if (logger.Count() > LOG_MAX_LINES)
        {
            logger.RemoveAt(logger.Count() - 1);

        }
    }
    public static void Clear()
    {
        logger.Clear();

    }
    public static void Info(string line)
    {
        Log("I: " + line);

    }
    public static void Warn(string line)
    {
        Log("W: " + line);

    }
    public static void Err(string line)
    {
        Log("E: " + line);

    }
    public static void Pos(string where, ref Vector3D pos)
    {
        Log("GPS:" + where + ":" + pos.X.ToString("F2") + ":" + pos.Y.ToString("F2") + ":" + pos.Z.ToString("F2") + ":");

    }
    public static string PrintBuffer(bool active)
    {
        str = Animation.Rotator() + (active ? HEADER_ACTIVE : HEADER_NOT_ACTIVE);
        foreach (string line in logger)
        {
            str += "\n " + line;

        }

        return str;

    }
}
public void DebugPrintLogging()
{
    List<IMyTextPanel> blocks = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(blocks);
    if (blocks.Count() == 0)
    {
        return;

    }

    Animation.DebugRun();
    var str = Animation.DebugRotator() + Logger.PrintBuffer(false);
    foreach (IMyTextPanel panel in blocks)
    {
        if (!panel.CustomName.Contains("LOG"))
        {
            continue;
        }
        panel.FontSize = 1.180f;
        panel.Font = "Monospace";
        panel.TextPadding = 0.0f;
        panel.WriteText(str);
    }
}
public static class Animation
{
    private static string[] ROTATOR = new string[] { "|", "/", "-", "\\" };
    private static int rotatorCount = 0;
    private static int debugRotatorCount = 0;
    public static void Run()
    {
        if (++rotatorCount > ROTATOR.Length - 1)
        {
            rotatorCount = 0;

        }
    }
    public static string Rotator()
    {
        return ROTATOR[rotatorCount];

    }
    public static void DebugRun()
    {
        if (++debugRotatorCount > ROTATOR.Length - 1)
        {
            debugRotatorCount = 0;

        }
    }
    public static string DebugRotator()
    {
        return ROTATOR[debugRotatorCount];

    }
}
public static class TimeStats
{
    public static Dictionary<string, DateTime> start = new Dictionary<string, DateTime>();
    public static Dictionary<string, TimeSpan> stats = new Dictionary<string, TimeSpan>();
    public static void Start(string key)
    {
        start[key] = DateTime.Now;

    }
    public static void Stop(string key)
    {
        stats[key] = DateTime.Now - start[key];

    }
    public static string Results()
    {
        string str = "";
        foreach (KeyValuePair<string, TimeSpan> stat in stats)
        {
            str += String.Format("{0}:{1:F4}ms\n", stat.Key, stat.Value.TotalMilliseconds);

        }

        return str;

    }
}
public static class Serializer
{
    public static string[] separator = new string[] { "\n" };
    public static string serialized;
    public static Queue<string> deserialized;
    public static void InitPack()
    {
        serialized = "";

    }
    public static void Pack(string str)
    {
        serialized += str + separator[0];

    }
    public static void Pack(int val)
    {
        serialized += val.ToString() + separator[0];

    }
    public static void Pack(long val)
    {
        serialized += val.ToString() + separator[0];

    }
    public static void Pack(float val)
    {
        serialized += val.ToString() + separator[0];

    }
    public static void Pack(double val)
    {
        serialized += val.ToString() + separator[0];

    }
    public static void Pack(bool val)
    {
        serialized += (val ? "1" : "0") + separator[0];

    }
    public static void Pack(VRage.Game.MyCubeSize val)
    {
        Pack((int)val);

    }
    public static void Pack(Vector3D val)
    {
        Pack(val.X);
        Pack(val.Y);
        Pack(val.Z);

    }
    public static void Pack(List<Vector3D> val)
    {
        Pack(val.Count);
        foreach (Vector3D v in val)
        {
            Pack(v);
        }
    }
    public static void Pack(List<int> val)
    {
        Pack(val.Count);
        foreach (int v in val)
        {
            Pack(v);
        }
    }
    public static void Pack(Stance val)
    {
        Pack(val.position);
        Pack(val.forward);
        Pack(val.up);

    }
    public static void Pack(Dock val)
    {
        Pack(val.stance);
        Pack(val.approachPath);
        Pack(val.cubeSize);
        Pack(val.gridEntityId);
        Pack(val.gridName);
        Pack(val.blockEntityId);
        Pack(val.blockName);
        Pack(val.lastSeen);

    }
    public static void Pack(List<Dock> val)
    {
        Pack(val.Count);
        foreach (Dock v in val)
        {
            Pack(v);
        }
    }
    public static void Pack(Waypoint val)
    {
        Pack(val.stance);
        Pack(val.maxSpeed);
        Pack((int)val.type);

    }
    public static void Pack(List<Waypoint> val)
    {
        Pack(val.Count);
        foreach (Waypoint v in val)
        {
            Pack(v);
        }
    }
    public static void Pack(VectorPath val)
    {
        Pack(val.position);
        Pack(val.direction);

    }
    public static void Pack(List<VectorPath> val)
    {
        Pack(val.Count);
        foreach (VectorPath v in val)
        {
            Pack(v);
        }
    }
    public static void InitUnpack(string str)
    {
        deserialized = new Queue<string>(str.Split(separator, StringSplitOptions.None));

    }
    public static string UnpackString()
    {
        return deserialized.Dequeue();

    }
    public static int UnpackInt()
    {
        return int.Parse(deserialized.Dequeue());

    }
    public static long UnpackLong()
    {
        return long.Parse(deserialized.Dequeue());

    }
    public static float UnpackFloat()
    {
        return float.Parse(deserialized.Dequeue());

    }
    public static double UnpackDouble()
    {
        return double.Parse(deserialized.Dequeue());

    }
    public static bool UnpackBool()
    {
        return deserialized.Dequeue() == "1";

    }
    public static VRage.Game.MyCubeSize UnpackCubeSize()
    {
        return (VRage.Game.MyCubeSize)UnpackInt();

    }
    public static Vector3D UnpackVector3D()
    {
        return new Vector3D(UnpackDouble(), UnpackDouble(), UnpackDouble());

    }
    public static List<Vector3D> UnpackListVector3D()
    {
        List<Vector3D> val = new List<Vector3D>();
        int count = UnpackInt();
        for (int i = 0; i < count; i++)
        {
            val.Add(UnpackVector3D());

        }

        return val;

    }
    public static List<int> UnpackListInt()
    {
        List<int> val = new List<int>();
        int count = UnpackInt();
        for (int i = 0; i < count; i++)
        {
            val.Add(UnpackInt());

        }

        return val;

    }
    public static Stance UnpackStance()
    {
        return new Stance(UnpackVector3D(), UnpackVector3D(), UnpackVector3D());

    }
    public static Dock UnpackDock()
    {
        Dock val = new Dock();
        val.stance = UnpackStance();
        val.approachPath = UnpackListVectorPath();
        val.cubeSize = UnpackCubeSize();
        val.gridEntityId = UnpackLong();
        val.gridName = UnpackString();
        val.blockEntityId = UnpackLong();
        val.blockName = UnpackString();
        val.lastSeen = UnpackLong();
        return val;

    }
    public static List<Dock> UnpackListDock()
    {
        List<Dock> val = new List<Dock>();
        int count = UnpackInt();
        for (int i = 0; i < count; i++)
        {
            val.Add(UnpackDock());

        }

        return val;

    }
    public static Waypoint UnpackWaypoint()
    {
        return new Waypoint(UnpackStance(), UnpackFloat(), (Waypoint.wpType)UnpackInt());

    }
    public static List<Waypoint> UnpackListWaypoint()
    {
        List<Waypoint> val = new List<Waypoint>();
        int count = UnpackInt();
        for (int i = 0; i < count; i++)
        {
            val.Add(UnpackWaypoint());

        }

        return val;

    }
    public static VectorPath UnpackVectorPath()
    {
        return new VectorPath(UnpackVector3D(), UnpackVector3D());

    }
    public static List<VectorPath> UnpackListVectorPath()
    {
        List<VectorPath> val = new List<VectorPath>();
        int count = UnpackInt();
        for (int i = 0; i < count; i++)
        {
            val.Add(UnpackVectorPath());

        }

        return val;

    }
}
public static class Helper
{
    public static string FormatedWaypoint(bool stance, int pos)
    {
        return (stance ? "Ori " : "Pos ") + (++pos).ToString("D2");

    }
    public static string Capitalize(string s)
    {
        if (string.IsNullOrEmpty(s))
        {
            return string.Empty;

        }

        return s.First().ToString().ToUpper() + s.Substring(1);

    }
    public static Vector3D UnserializeVector(string str)
    {
        var parts = str.Split(':');
        Vector3D v = Vector3D.Zero;
        if (parts.Length != 7)
        {
            return v;

        }

        try
        {
            v = new Vector3D(double.Parse(parts[2]), double.Parse(parts[3]), double.Parse(parts[4]));
        }
        catch { }
        return v;

    }
}
public static class ErrorState
{
    public enum Type { TooManyControllers, NoRemoteController };
    private static Dictionary<Type, bool> errorState = new Dictionary<Type, bool> { { Type.TooManyControllers, false } };
    public static void Set(Type type)
    {
        errorState[type] = true;

    }
    public static void Reset(Type type)
    {
        errorState[type] = false;

    }
    public static bool Get(Type type)
    {
        return errorState[type];

    }
}
public class Stance
{
    public Vector3D position;
    public Vector3D forward;
    public Vector3D up;
    public Stance(Vector3D p, Vector3D f, Vector3D u)
    {
        this.position = p;
        this.forward = f;
        this.up = u;

    }
}
public class VectorPath
{
    public Vector3D position;
    public Vector3D direction;
    public VectorPath(Vector3D p, Vector3D d)
    {
        this.position = p;
        this.direction = d;

    }
}
public class Waypoint
{
    public Stance stance;
    public float maxSpeed;
    [Flags]
    public enum wpType { ALIGNING = 1 << 0, DOCKING = 1 << 1, UNDOCKING = 1 << 2,
        CONVERGING = 1 << 3, APPROACHING = 1 << 4, NAVIGATING = 1 << 5, TESTING = 1 << 6,
        TAXIING = 1 << 7, CRUISING = 1 << 8 };
    public wpType type;
    public Waypoint(Stance s, float m, wpType wt)
    {
        stance = s;
        maxSpeed = m;
        type = wt;

    }
    public string GetTypeMsg()
    {
        switch (this.type)
        {
            case wpType.ALIGNING: return MSG_ALIGNING;

            case wpType.DOCKING: return MSG_DOCKING;

            case wpType.UNDOCKING: return MSG_UNDOCKING;

            case wpType.CONVERGING: return MSG_CONVERGING;

            case wpType.APPROACHING: return MSG_APPROACHING;

            case wpType.NAVIGATING: return MSG_NAVIGATING;

            case wpType.TAXIING: return MSG_TAXIING;

            case wpType.CRUISING: return String.Format(MSG_CRUISING_AT, Situation.autoCruiseAltitude, MathHelper.ToDegrees(Navigation.ClimbAngle));

            default: break;

        }
        return "Testing...";

    }


    public static Dock DockFromGPS(string coordinates)
    {
        string[] segment = coordinates.Split(':');
        //Logger.Info($"GPS --- {coordinates}");
        if (segment.Length == 7)
        {
            Waypoint wp = FromString(coordinates);
            Dock dock = Dock.NewDock(wp, segment[1]);
            dock.gridName = "GPS";
            return dock;
        }
        else
        {
            Logger.Err("Unable to add location,\ninvalid GPS coordinate");
            return null;
        }
    }

    //This is set to max_speed originally
    public static Waypoint FromString(string coordinates)
    {
        Waypoint wp = new Waypoint(new Stance(Helper.UnserializeVector(coordinates), Vector3D.Zero, Vector3D.Zero), CONVERGING_SPEED, wpType.CONVERGING);
        return wp;
        //return new Waypoint(new Stance(Helper.UnserializeVector(coordinates), Vector3D.Zero, Vector3D.Zero), MAX_SPEED, wpType.CONVERGING);

    }
}
public class PairCounter
{
    public int oldC;
    public int newC;
    public PairCounter()
    {
        this.oldC = 0;
        this.newC = 1;

    }
    public void Recount()
    {
        this.oldC = this.newC;
        this.newC = 0;

    }
    public int Diff()
    {
        return newC - oldC;

    }
}
public class BlockProfile
{
    public string[] tags;
    public string[] exclusiveTags;
    public string[] attributes;
    public BlockProfile(ref string[] tags, ref string[] exclusiveTags, ref string[] attributes)
    {
        this.tags = tags;
        this.exclusiveTags = exclusiveTags;
        this.attributes = attributes;

    }
}
public class Dock : IComparable<Dock>
{
    private static long STALE_TIME = TimeSpan.FromSeconds(60.0).Ticks;
    public static Dock NewDock(Vector3D p, Vector3D f, Vector3D u, string name)
    {
        Dock d = new Dock();
        d.stance = new Stance(p, f, u);
        d.gridName = "Manual";
        d.blockName = name;
        return d;

    }
    public static Dock NewDock(Waypoint wp, string name)
    {
        return NewDock(wp.stance, name);

    }

    public static Dock NewDock(Stance st, string name)
    {
        Dock d = new Dock();
        d.stance = st;
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
    public int CompareTo(Dock other)
    {
        if (this.gridEntityId != other.gridEntityId)
        {
            return (other.gridEntityId < this.gridEntityId) ? 1 : -1;

        }

        if (this.blockEntityId != other.blockEntityId)
        {
            return (other.blockEntityId < this.blockEntityId) ? 1 : -1;

        }

        return this.blockName.CompareTo(other.blockName);

    }
    public void SortApproachVectorsByDistance(Vector3D from)
    {
        approachPath.Sort(delegate (VectorPath a, VectorPath b)
        {
            return (int)(Vector3D.Distance(from, b.position) - Vector3D.Distance(from, a.position));
        });

    }
    public void Touch()
    {
        this.lastSeen = DateTime.Now.Ticks;

    }
    public bool Fresh()
    {
        if (lastSeen == 0)
        {
            return true;

        }

        return (DateTime.Now.Ticks - lastSeen) < STALE_TIME;

    }

}
