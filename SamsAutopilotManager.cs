// Sam's Autopilot Manager
public static string VERSION = "1.2.3";
//
// Documentation: http://steamcommunity.com/sharedfiles/filedetails/?id=1224507423
// 
// Github: https://github.com/sugardose/SpaceEngineers
//
// Owner: Sam
// Contributors:
//  Cameron Leger
//  Michael Lane
//
// Latest changes:
//  - Modified where the call is made for NotifyTimers to ensure it is called everytime you dock;


// Change the tag used to identify blocks        
public static string TAG = "SAM";

// -------------------------------------------------------
// If you change these, things will break.

public static double DISTANCE_THRESHOLD_DOCKING = 1.0;
public static double DISTANCE_THRESHOLD_HOPPING = 1000;

public static double LINEUP_DISTANCE_MULTIPLIER = 2.0;
public static double APPROACH_DISTANCE_MULTIPLIER = 1.0;
public static double FREE_GROUND_MULTIPLIER = 4.0;
public static double CORRECTION_STEP_MULTIPLIER = 5.0;
public static double REMOTE_MAX_DISTANCE = 5.0;
public static double THRUST_MIN_ACCELERATION = 2.0;

// -------------------------------------------------------
// -------------------------------------------------------
public void HandleCommand(string command) {
    if (!this.HasControlBlock()) this.Log("No Remote Controller block");
    var parts = command.ToUpper().Split(' ');
    parts.DefaultIfEmpty("");
    var arg0 = parts.ElementAtOrDefault(0);
    var arg1 = parts.ElementAtOrDefault(1);
    try {
        switch (arg0) {
            case "DOCK":
                switch (arg1) {
                    case "REGISTER": this.DockRegister(); this.Save(); break;
                    case "DELETE": this.DockDelete(); this.Save(); break;
                    case "PREV": this.DockSelectPrev(); break;
                    case "NEXT": this.DockSelectNext(); break;
                    default: this.Log("Unknown " + arg0 + " argument ->" + arg1 + "<-"); break;
                }
                break;
            case "NAV":
                switch (arg1) {
                    case "START": this.NavigateStart(); break;
                    case "STOP": this.NavigateStop(); break;
                    case "NEAR": this.NavigateNearest(); break;
                    case "FAR": this.NavigateFarthest(); break;
                    default: this.Log("Unknown " + arg0 + " argument ->" + arg1 + "<-"); break;
                }
                break;
            case "DIAG":
                this.diagnosticsTabs.Move(0, this.diagnosticsTabs.Count() - 1);
                break;
            default: this.Log("Unknown command ->" + arg0 + "<-"); break;
        }
    } catch (Exception e) {
        this.Log("Command exception -< " + command + " >-< " + e.Message + " >-");
    }
}

public void Update10() {
    if (!this.HasControlBlock()) return;
    try { this.PrintToNavigationPanels(); } catch (Exception e) { this.Log("Update10 PrintToNavigationPanels exception: " + e.Message); }
    try { this.PrintToGpsPanels(); } catch (Exception e) { this.Log("Update10 PrintToGpsPanels exception: " + e.Message); }
    try { this.PrintToDiagnosticsPanels(); } catch (Exception e) { this.Log("Update10 PrintToDiagnosticsPanels exception: " + e.Message); }
    try { this.NavigationMonitor(); } catch (Exception e) { this.Log("Update10 NavigationMonitor exception: " + e.Message); }
    try { this.CheckNotify(); } catch (Exception e) { this.Log("Update10 CheckNotify exception: " + e.Message); }
    try { this.PrintToLoggingPanels(); } catch (Exception e) { this.Log("Update10 PrintToLoggingPanels exception: " + e.Message); }
}

public void Update100() {
    try { this.ScanControlBlock(); } catch (Exception e) { this.Log("Update100 ScanControlBlock exception: " + e.Message); }
    try { this.ScanCameraBlocks(); } catch (Exception e) { this.Log("Update100 ScanCameraBlock exception: " + e.Message); }
    try { this.ScanPanels(); } catch (Exception e) { this.Log("Update100 ScanPanels exception: " + e.Message); }
    try { this.ScanProgramableBlocks(); } catch (Exception e) { this.Log("Update100 ScanProgramableBlocks exception: " + e.Message); }
    try { this.ScanConnectors(); } catch (Exception e) { this.Log("Update100 ScanConnectors exception: " + e.Message); }
    if (this.HasDiagnosticsPanels())
        try { this.ScanThrusters(); } catch (Exception e) { this.Log("Update100 ScanThrusters exception: " + e.Message); }
}

public void Main(string argument, UpdateType updateSource) {
    var update = updateSource;
    if ((update & UpdateType.Update10) != 0) this.Update10();
    update &= ~UpdateType.Update10;
    if ((update & UpdateType.Update100) != 0) this.Update100();
    update &= ~UpdateType.Update100;
    if (update != 0) this.HandleCommand(argument);
}

// -------------------------------------------------------
// -------------------------------------------------------
// Adjusters

public double LINEUP_DISTANCE;
public double APPROACH_DISTANCE;
public double FREE_GROUND;
public double CORRECTION_STEP;

public void AdjustDimentions() {
    var diameter = 2.0 * Me.CubeGrid.WorldVolume.Radius;
    this.Log("Grid diameter: " + diameter.ToString("F2") + "m");
    this.APPROACH_DISTANCE = Program.APPROACH_DISTANCE_MULTIPLIER * diameter;
    this.LINEUP_DISTANCE = Program.LINEUP_DISTANCE_MULTIPLIER * diameter;
    this.FREE_GROUND = Program.FREE_GROUND_MULTIPLIER * diameter;
    this.CORRECTION_STEP = Program.CORRECTION_STEP_MULTIPLIER * diameter;
}

// Adjusters
// -------------------------------------------------------
// Coordinate generation

public List<Waypoint> GenerateApproach(DockMetadata metadata) {
    this.AdjustDimentions();
    List<Waypoint> list = new List<Waypoint>();
    var lineup = this.LINEUP_DISTANCE / Math.Sqrt(2.0);
    var approach = this.APPROACH_DISTANCE;

    if (Program.NearAnyDock(2 * DISTANCE_THRESHOLD_DOCKING, this.currentPosition, this.docks)) {
        var currentUp = this.controlBlock.WorldMatrix.Up;
        var currentBackward = this.controlBlock.WorldMatrix.Backward;
        var makeSpaceWaypoint = this.currentPosition + (approach * currentBackward);
        list.Add(new Waypoint(false, true, true, 100.0f, makeSpaceWaypoint));
        makeSpaceWaypoint = this.currentPosition + (lineup * currentUp) + (lineup * currentBackward);
        list.Add(new Waypoint(false, true, true, 100.0f, makeSpaceWaypoint));
        this.Log("Near a Dock - withdrawing backwards");
    }

    var directionWaypoint = metadata.position + (-this.LINEUP_DISTANCE * metadata.front) + (this.LINEUP_DISTANCE * metadata.up);
    list.Add(new Waypoint(true, false, false, 100.0f, Program.LerpToDistance(2 * Program.DISTANCE_THRESHOLD_HOPPING, this.currentPosition, Program.QuarterSlerp(this.currentPosition, directionWaypoint, this.planetCenter))));

    list.Add(new Waypoint(false, false, true, 100.0f, directionWaypoint));
    list.Add(new Waypoint(false, false, true, 100.0f, metadata.position + (-approach * metadata.front)));
    list.Add(new Waypoint(false, false, true, 100.0f, metadata.position));
    return list;
}

// Coordinate generation
// -------------------------------------------------------
// Remote configuration

public void ConfigureRemote() {
    var wp = this.waypoints.First();
    this.controlBlock.SetAutoPilotEnabled(false);
    this.controlBlock.ClearWaypoints();
    this.controlBlock.AddWaypoint(wp.position, this.autopilotDestination + " " + this.waypoints.Count().ToString());
    this.controlBlock.SetCollisionAvoidance(false);
    this.controlBlock.FlightMode = FlightMode.OneWay;
    this.controlBlock.SpeedLimit = wp.maxSpeed;
    this.controlBlock.SetDockingMode(wp.dockingMode);
    this.controlBlock.Direction = (wp.reverse) ? (Base6Directions.Direction.Backward) : (Base6Directions.Direction.Forward);
    this.controlBlock.ControlThrusters = true;
    this.controlBlock.DampenersOverride = true;
    this.controlBlock.SetAutoPilotEnabled(true);
}

public void StopRemoteAutopilot() {
    this.controlBlock.SetAutoPilotEnabled(false);
    this.controlBlock.ClearWaypoints();
    this.NotifyTimers();
}

// Remote configuration
// -------------------------------------------------------
// Navigation interface

public bool autopilotRunning;
public string autopilotDestination;
public List<Waypoint> waypoints;

public void NavigateStart() {
    if (this.docksConfigured.Count() == 0) {
        this.Log("Navigation aborted - no docks configured");
        return;
    }
    if (!this.InPlanet()) {
        this.Log("Not in a planet");
        return;
    }
    this.ConnectorsDisconnect();
    this.NotifyDelay("NAV START", 0);
    this.Log("Starting navigation to " + this.docksConfigured[this.dockSelected]);
    this.autopilotRunning = true;
    this.autopilotDestination = this.docksConfigured[this.dockSelected];
    var dockMetadata = this.docks[autopilotDestination];
    this.UpdateTransients();
    this.waypoints = this.GenerateApproach(dockMetadata);
    this.ConfigureRemote();
    tragectoryAdjustments = 0;
}

public void NavigateNearest() {
    this.UpdateTransients();
    this.dockSelected = this.docksConfigured.IndexOf(Program.GetNearestDock(false, this.currentPosition, this.docks));
    this.NavigateStart();
}

public void NavigateFarthest() {
    this.UpdateTransients();
    this.dockSelected = this.docksConfigured.IndexOf(Program.GetNearestDock(true, this.currentPosition, this.docks));
    this.NavigateStart();
}

public void NavigateStop() {
    this.Log("Stopping navigation");
    this.autopilotRunning = false;
    if (!this.HasControlBlock()) return;
    this.StopRemoteAutopilot();
}

// Navigation interface
// -------------------------------------------------------
// Metadata

public DockMetadata CreateDockMetadata() {
    this.UpdateTransients();
    double surfaceAltitude;
    this.controlBlock.TryGetPlanetElevation(MyPlanetElevation.Surface, out surfaceAltitude);
    return new DockMetadata(
        this.currentPosition,
        this.controlBlock.WorldMatrix.GetDirectionVector(Base6Directions.Direction.Forward),
        -Vector3D.Normalize(this.controlBlock.GetNaturalGravity()),
        surfaceAltitude,
        Vector3D.Distance(this.planetCenter, this.currentPosition),
        this.controlBlock.GetNaturalGravity().Length());
}

// Metadata
// -------------------------------------------------------
// Navigation Mechanics

public Vector3D currentPosition;
public Vector3D planetCenter;
public double currentWaypointDistance;
public double nextWaypointDistance;
public int tragectoryAdjustments = 0;

public bool InPlanet() {
    return this.controlBlock.TryGetPlanetPosition(out planetCenter);
}

public void UpdateTransients() {
    this.currentPosition = this.controlBlock.GetPosition();
    if (this.waypoints.Count() == 0) return;
    this.currentWaypointDistance = Vector3D.Distance(this.waypoints.First().position, this.currentPosition);
    this.nextWaypointDistance = Vector3D.Distance(this.currentPosition, this.waypoints[Math.Min(this.waypoints.Count - 1, 1)].position);
}


public Vector3D? CheckCameraObstructedPath(IMyCameraBlock camera, Vector3D cast) {
    if (!camera.CanScan(Vector3D.Distance(camera.GetPosition(), cast))) return null;
    var info = camera.Raycast(cast);
    if (info.IsEmpty()) return null;
    switch (info.Type) {
        case MyDetectedEntityType.LargeGrid:
        case MyDetectedEntityType.Planet:
            return info.HitPosition;
    }
    return null;
}

public Vector3D? CheckObstructedPath() {
    if (!HasCameraBlock()) return null;
    var cast = Program.LerpToDistance(this.FREE_GROUND, this.waypoints.First().position, this.planetCenter);

    foreach(IMyCameraBlock camera in this.cameraBlocks) {
        var hit = CheckCameraObstructedPath(camera, cast);
        if (hit != null) return hit;
    }
    return null;
}

public void CorrectObstructedWaypoint(Vector3D hit) {
    var newPos = Program.MoveAwayFrom(this.CORRECTION_STEP, hit, this.planetCenter);
    this.waypoints.First().position = Program.LerpToDistance(2 * Program.DISTANCE_THRESHOLD_HOPPING, this.currentPosition, newPos);
}

public void SetDirectWaypoint() {
    var nextVector = this.waypoints[1].position - this.currentPosition;
    var nextDistance = nextVector.Length();
    if (nextDistance < Program.DISTANCE_THRESHOLD_HOPPING) {
        this.waypoints.RemoveAt(0);
        return;
    }
    this.waypoints.First().position = Program.LerpToDistance(2 * Program.DISTANCE_THRESHOLD_HOPPING, this.currentPosition, Program.QuarterSlerp(this.currentPosition, this.waypoints[1].position, this.planetCenter));
}

public void NextWaypoint(bool approach) {
    this.waypoints.RemoveAt(0);
    if (this.waypoints.Count() == 0) {
        this.NavigateStop();
        this.NotifyDelay("NAV STOP", 3.0);
        return;
    } else if (this.waypoints.Count() == 1) {
        this.NotifyDelay("NAV APPROACH", 0);
    }

    this.Log(approach ? "Starting approach" : "Navigating to next waypoint");
    ConfigureRemote();
}

public void NavigationMonitor() {
    if (!this.autopilotRunning) return;
    if (!this.InPlanet()) {
        this.Log("Not in a planet");
        this.NavigateStop();
        return;
    }
    this.UpdateTransients();
    var distanceThreshold = (this.waypoints.First().dockingMode) ? (Program.DISTANCE_THRESHOLD_DOCKING) : (Program.DISTANCE_THRESHOLD_HOPPING);
    if (this.waypoints.First().rayTrace) {
        var obstruction = this.CheckObstructedPath();
        if (obstruction != null) {
            this.Log("Avoiding colision (" + (tragectoryAdjustments++).ToString() + ")");
            this.CorrectObstructedWaypoint(obstruction.Value);
            this.ConfigureRemote();
            return;
        }
        if (this.nextWaypointDistance < distanceThreshold) {
            this.NextWaypoint(true);
            return;
        }
    }
    if (this.currentWaypointDistance > distanceThreshold) return;
    if (this.waypoints.First().rayTrace) {
        this.SetDirectWaypoint();
        this.Log("Adjusting tragectory (" + (tragectoryAdjustments++).ToString() + ")");
        this.ConfigureRemote();
        return;
    }
    this.NextWaypoint(false);
}

// Navigation Mechanics
// -------------------------------------------------------
// Thruster information

public Dictionary<string, Dictionary<string, TrustInfo>> thrustInfo = new Dictionary<string, Dictionary<string, TrustInfo>>() {
{ "Hydrogen", new Dictionary<string, TrustInfo>() { {"Forward", new TrustInfo(0.0f, 0.0f)}, { "Backward", new TrustInfo(0.0f, 0.0f)}, { "Up", new TrustInfo(0.0f, 0.0f)}, { "Down", new TrustInfo(0.0f, 0.0f)}, { "Left", new TrustInfo(0.0f, 0.0f)}, { "Right", new TrustInfo(0.0f, 0.0f)} } },
{ "Ion", new Dictionary<string, TrustInfo>() { {"Forward", new TrustInfo(0.0f, 0.0f)}, { "Backward", new TrustInfo(0.0f, 0.0f)}, { "Up", new TrustInfo(0.0f, 0.0f)}, { "Down", new TrustInfo(0.0f, 0.0f)}, { "Left", new TrustInfo(0.0f, 0.0f)}, { "Right", new TrustInfo(0.0f, 0.0f)} } },
{ "Atmospheric", new Dictionary<string, TrustInfo>() { {"Forward", new TrustInfo(0.0f, 0.0f)}, { "Backward", new TrustInfo(0.0f, 0.0f)}, { "Up", new TrustInfo(0.0f, 0.0f)}, { "Down", new TrustInfo(0.0f, 0.0f)}, { "Left", new TrustInfo(0.0f, 0.0f)}, { "Right", new TrustInfo(0.0f, 0.0f)} } },
{ "All thrusters", new Dictionary<string, TrustInfo>() { {"Forward", new TrustInfo(0.0f, 0.0f)}, { "Backward", new TrustInfo(0.0f, 0.0f)}, { "Up", new TrustInfo(0.0f, 0.0f)}, { "Down", new TrustInfo(0.0f, 0.0f)}, { "Left", new TrustInfo(0.0f, 0.0f)}, { "Right", new TrustInfo(0.0f, 0.0f)} } },
};

public void ClearTrustInfo() {
    foreach (KeyValuePair<string, Dictionary<string, TrustInfo>> entry in this.thrustInfo) {
        foreach (KeyValuePair<string, TrustInfo> innerEntry in entry.Value) {
            innerEntry.Value.effective = 0.0f;
            innerEntry.Value.theoretical = 0.0f;
        }
    }
}

public bool noReferenceCockpit = true;
public void ScanThrusters() {
    this.ClearTrustInfo();
    List<IMyThrust> blocks = new List<IMyThrust>();
    GridTerminalSystem.GetBlocksOfType<IMyThrust>(blocks, b => b.CubeGrid == Me.CubeGrid);
    this.noReferenceCockpit = false;
    foreach (IMyThrust block in blocks) {
        if (!block.IsWorking) continue;
        if (Vector3I.Zero == block.GridThrustDirection) {
            this.noReferenceCockpit = true;
            return;
        }
        var direction = Program.THRUST_DIRECTION[block.GridThrustDirection];
        var type = Program.THRUST_TYPE[block.MaxThrust];
        this.thrustInfo[type.type][direction].effective += block.MaxEffectiveThrust;
        this.thrustInfo[type.type][direction].theoretical += block.MaxThrust;
        this.thrustInfo["All thrusters"][direction].effective += block.MaxEffectiveThrust;
        this.thrustInfo["All thrusters"][direction].theoretical += block.MaxThrust;
    }
}

// Thruster information
// -------------------------------------------------------
// Textpanel printing

public DisplaySlider rotator = new DisplaySlider(Program.SLIDER_ROTATOR);

public void PrintToNavigationPanels() {
    if (!this.HasNavigationPanels()) return;
    if (this.controlBlock == null) {
        var warn = "SAM disabled.\nNo Remote Controll block found, or\n more than one and no " + Program.TAG + " tag found.";
        Program.Print(this.navigationPanels, warn);
        return;
    }
    if (!this.InPlanet()) {
        var warn = "SAM disabled.\nNot in a planet.\n";
        Program.Print(this.navigationPanels, warn);
        return;
    }
    var currentPosition = this.controlBlock.GetPosition();
    var up = Vector3D.Normalize(currentPosition - this.planetCenter);
    var linerVelocity = this.controlBlock.GetShipVelocities().LinearVelocity;

    string str = "SAM is " + ((this.autopilotRunning) ? ("ON!") : ("OFF")) + "\n\n";
    if (this.autopilotRunning) {
        var destinationWaypointDistance = Vector3D.Distance(this.waypoints.Last().position, currentPosition).ToString("F1") + "m";
        str += " " + this.autopilotDestination + new String(' ', 12 + Program.NATO_MAX_CODE_SIZE - this.autopilotDestination.Length - destinationWaypointDistance.Length) + destinationWaypointDistance + "\n";
        str += " " + this.rotator.GetString() + " ";
        if (this.waypoints.Count() >= 5) str += "withdrawing...";
        else if (this.waypoints.Count() >= 4) str += "navigating...";
        else if (this.waypoints.Count() >= 2) str += "lining up...";
        else str += "approaching...";
        str += "\n\n";
    } else {
        str += "\n\n\n";
    }
    str += "- configured docks --\n";
    if (this.dockSelected - 1 < 0) str += "   --\n";
    for (int i = 0; i < this.docksConfigured.Count(); i++) {
        if (i < this.dockSelected - 1 || i > this.dockSelected + 1) continue;
        str += ((i == this.dockSelected) ? (" >") : ("  "));
        str += ((this.docksConfigured[i] == this.autopilotDestination && this.autopilotRunning) ? ("*") : (" "));
        var code = this.docksConfigured[i];
        var distance = Vector3D.Distance(this.docks[this.docksConfigured[i]].position, currentPosition).ToString("F1") + "m";
        str += code + new String(' ', Program.NATO_MAX_CODE_SIZE - code.Length + 9 - distance.Length + 1) + distance + "\n";
    }
    if (this.dockSelected + 1 >= this.docksConfigured.Count()) str += "   --\n";
    if (this.docksConfigured.Count() == 0) str += "   --\n";
    str += "---------------------";
    str += "\n\n";
    str += "Speeds:\n";
    var climbSpeed = linerVelocity.Dot(up).ToString("F2");
    var driftSpeed = (linerVelocity - (linerVelocity.Dot(up) * up)).Length().ToString("F2");
    str += " climb: " + new String(' ', 8 - climbSpeed.Length) + climbSpeed + "m.s-1\n";
    str += " drift: " + new String(' ', 8 - driftSpeed.Length) + driftSpeed + "m.s-1";
    Program.Print(this.navigationPanels, str);
}

public void PrintToGpsPanels() {
    if (!this.HasGpsPanels()) return;
    var str = "Docks:\n";
    foreach (string code in this.docksConfigured) {
        var w = this.docks[code].position;
        str += "GPS:" + code + ":" + w.X.ToString("F2") + ":" + w.Y.ToString("F2") + ":" + w.Z.ToString("F2") + ":\n";
    }
    str += "Waypoints:\n";
    for (int i = 0; i < this.waypoints.Count(); ++i) {
        var w = this.waypoints[i].position;
        str += "GPS:wp" + i.ToString() + ":" + w.X.ToString("F2") + ":" + w.Y.ToString("F2") + ":" + w.Z.ToString("F2") + ":\n";
    }
    Program.Print(this.gpsPanels, str);
}

public List<string> diagnosticsTabs = Program.DIAGNOSTICS_TABS;
public bool diagnosticsBlink = false;

public void PrintToDiagnosticsPanels() {
    if (!this.HasDiagnosticsPanels()) return;
    var selectedTab = this.diagnosticsTabs.First();
    this.diagnosticsBlink = !this.diagnosticsBlink;

    var currentPosition = this.controlBlock.GetPosition();
    var centerOfMass = this.controlBlock.CenterOfMass;
    var totalMass = this.controlBlock.CalculateShipMass().TotalMass;
    var gravity = this.controlBlock.GetNaturalGravity().Length();


    if (selectedTab == "Menu") {
        string sub = selectedTab + "\n\n" +
            "  Issue the command \"Diag\"\n" +
            " to SAM's Programing block\n" +
            " to iterate through this\n" +
            " panel's diagnostics\n" +
            " screens:\n";
        foreach (string tab in Program.DIAGNOSTICS_TABS) sub += " - " + tab + "\n";
        Program.Print(this.diagnosticsPanels, sub);
        return;
    }

    if (selectedTab == "Guidelines") {
        string sub = selectedTab + "\n\n" +
            "  Keep the Remote close\n" +
            " to the center of Mass. If\n" +
            " not possible, then at\n" +
            " least in the same\n" +
            " vertical line that\n" +
            " crosses the center of\n" +
            " mass.\n\n" +
            "  Keep Forward, Backward,\n" +
            " Up & Down accelerations\n" +
            " above " + Program.THRUST_MIN_ACCELERATION.ToString("F0") + "m.s-2. For Down\n" +
            " acceleration also take in\n" +
            " consideration the Natural\n" +
            " Gravity. E.g.:\n" +
            "  DOWNacc > gravity + " + Program.THRUST_MIN_ACCELERATION.ToString("F0");
        Program.Print(this.diagnosticsPanels, sub);
        return;
    }

    if (selectedTab == "Ship dimensions") {
        var sub = selectedTab + "\n";
        sub += Me.CubeGrid.GridSize.ToString() + "\n";
        sub += Me.CubeGrid.WorldVolume.Radius.ToString() + "\n";
        Program.Print(this.diagnosticsPanels, sub);
        return;
    }

    if (selectedTab == "Remote controller") {
        string sub = selectedTab + "\n\n";
        var distance = Vector3D.Distance(currentPosition, centerOfMass);
        sub += " Distance from center\n  of mass: " + distance.ToString("F1") + "m\n\n";
        if (distance > Program.REMOTE_MAX_DISTANCE && this.diagnosticsBlink)
            sub += " Far from center of mass!";
        Program.Print(this.diagnosticsPanels, sub);
        return;
    }

    var str = this.diagnosticsTabs.First() + "\n\n";
    if (this.noReferenceCockpit) {
        str += " Grid has no reference\n cockpit for calculation!\n Please use a cockpit.";
        Program.Print(this.diagnosticsPanels, str);
        return;
    }

    var totalEffective = 0.0;
    var totalTheoretical = 0.0;
    var notSuitableWarning = false;
    var str2 = " Max:\n";
    str2 += "    Acceleration   Thrust\n";
    str2 += "         (m.s-2)     (MN)\n";

    foreach (KeyValuePair<string, TrustInfo> entry in this.thrustInfo[this.diagnosticsTabs.First()]) {
        str2 += " " + entry.Key;
        str2 += new String(' ', 8 - entry.Key.Length);
        totalEffective += entry.Value.effective;
        totalTheoretical += entry.Value.theoretical;
        var effective = (entry.Value.effective / 1000000.0).ToString("F3");
        var acceleration = (entry.Value.effective / totalMass).ToString("F2");

        if (entry.Key == "Right" || entry.Key == "Left") {
            str2 += new String(' ', 7 - acceleration.Length);
            str2 += acceleration;
        } else if (entry.Key == "Downwards") {
            var notSuitable = (entry.Value.effective / totalMass) <= (Program.THRUST_MIN_ACCELERATION + gravity);
            if (notSuitable && this.diagnosticsBlink) {
                str2 += new String(' ', 7);
            } else {
                str2 += new String(' ', 7 - acceleration.Length);
                str2 += acceleration;
            }
            notSuitableWarning |= notSuitable;
        } else {
            var notSuitable = (entry.Value.effective / totalMass) <= (Program.THRUST_MIN_ACCELERATION);
            if (notSuitable && this.diagnosticsBlink) {
                str2 += new String(' ', 7);
            } else {
                str2 += new String(' ', 7 - acceleration.Length);
                str2 += acceleration;
            }
            notSuitableWarning |= notSuitable;
        }

        str2 += new String(' ', 9 - effective.Length);
        str2 += effective;
        str2 += "\n";
    }
    str += " Efficiency: " + (100.0f * totalEffective / totalTheoretical).ToString("F2") + "%\n";
    str += " Ship Mass: " + totalMass.ToString() + "kg\n";
    str += " Gravity: " + gravity.ToString("F1") + "m.s-2\n";
    str += "\n" + str2 + "\n";
    if (notSuitableWarning && !this.diagnosticsBlink)
        str += " Not enough acceleration!";
    Program.Print(this.diagnosticsPanels, str);
}

public DisplaySlider loggerRotator = new DisplaySlider(Program.SLIDER_ROTATOR);
public void PrintToLoggingPanels() {
    if (!this.HasLoggingPanels()) return;
    var str = loggerRotator.GetString() + " SAM v" + Program.VERSION + " Logger ";
    str += new String('-', 37 - str.Length);
    foreach (string line in this.logger) str += "\n " + line;
    str += "\n" + new string('-', 37);
    Program.Print(this.loggingPanels, str);
}

// Textpanel printing
// -------------------------------------------------------
// Logging

public List<string> logger = new List<string>();
public void Log(string str) {
    Echo(str);
    this.logger.Add(str);
    if (this.logger.Count() > Program.LOG_MAX_LINES) this.logger.RemoveAt(0);
}

// Logging
// -------------------------------------------------------
// Notify

public List<Notification> notifications = new List<Notification>();

public void NotifyDelay(string notification, double delaySeconds) {
    this.notifications.Add(new Notification(DateTime.Now.AddSeconds(delaySeconds), notification));
}

public void CheckNotify() {
    List<Notification> remove = new List<Notification>();
    foreach (Notification n in this.notifications) {
        var now = DateTime.Now;
        if (n.when > DateTime.Now) continue;
        var message = n.notification;
        remove.Add(n);
        this.Log("Notifying for: " + n.notification);
        foreach (IMyProgrammableBlock block in this.programableBlocks) {
            block.TryRun(message);
        }
        if (message == "NAV STOP") {
            foreach (IMyShipConnector block in this.connectors) {
                block.Connect();
            }
        }
    }
    foreach (Notification n in remove) {
        this.notifications.Remove(n);
    }
}

// Due to some weird reason, timer blocks do not persist on lists. So notify directly.
public void NotifyTimers() {
    List<IMyTimerBlock> blocks = new List<IMyTimerBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyTimerBlock>(blocks, b => b.CubeGrid == Me.CubeGrid);
    foreach (IMyTimerBlock block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        switch (match.Groups[2].Value.ToUpper()) {
            case "TRIGGER":
                Program.FixNameTag(block, match.Groups[1].Value, " TRIGGER");
                block.GetActionWithName("TriggerNow").Apply(block);
                break;
            case "START":
                Program.FixNameTag(block, match.Groups[1].Value, " START");
                block.GetActionWithName("Start").Apply(block);
                break;
            default:
                Program.FixNameTag(block, match.Groups[1].Value, "");
                break;
        }
    }
}

public void ConnectorsDisconnect() {
    foreach (IMyShipConnector block in this.connectors) {
        block.Disconnect();
    }
}

// Notify
// -------------------------------------------------------
// Docking coordinate storage & handling

public Dictionary<string, DockMetadata> docks;
public List<string> docksConfigured;
public int dockSelected;

public void DockRegister() {
    if (!this.InPlanet()) {
        this.Log("Not in a planet");
        return;
    }
    foreach (string code in Program.NATO_CODES) {
        if (!this.docks.ContainsKey(code)) {
            this.docks.Add(code, this.CreateDockMetadata());
            this.docksConfigured.Add(code);
            this.dockSelected = this.docksConfigured.Count() - 1;
            this.Log("Created and stored dock position: " + code);
            return;
        }
    }
    this.Log("All docking slots used");
}

public int DockSelectNext() {
    this.dockSelected += 1;
    if (this.dockSelected >= this.docksConfigured.Count()) {
        this.dockSelected = 0;
    }
    return this.dockSelected;
}

public int DockSelectPrev() {
    this.dockSelected -= 1;
    if (this.dockSelected < 0) {
        this.dockSelected = this.docksConfigured.Count() - 1;
    }
    return this.dockSelected;
}

public void DockDelete() {
    if (!this.InPlanet()) {
        this.Log("Not in a planet");
        return;
    }
    if (this.docksConfigured.Count() == 0) {
        this.Log("No docks to delete");
        return;
    }
    var code = this.docksConfigured[this.dockSelected];
    this.docks.Remove(code);
    this.docksConfigured.RemoveAt(this.dockSelected);
    if (this.dockSelected >= this.docksConfigured.Count()) {
        this.dockSelected = this.docksConfigured.Count() - 1;
    }
    this.Log("Deleted dock: " + code);
}

// Docking coordinate storage & handling
// -------------------------------------------------------
// Blocks

public IMyRemoteControl controlBlock;
public List<IMyCameraBlock> cameraBlocks = new List<IMyCameraBlock>();
public List<IMyTextPanel> navigationPanels = new List<IMyTextPanel>();
public List<IMyTextPanel> gpsPanels = new List<IMyTextPanel>();
public List<IMyTextPanel> diagnosticsPanels = new List<IMyTextPanel>();
public List<IMyTextPanel> loggingPanels = new List<IMyTextPanel>();
public List<IMyProgrammableBlock> programableBlocks = new List<IMyProgrammableBlock>();
public List<IMyShipConnector> connectors = new List<IMyShipConnector>();

public bool HasControlBlock() { return this.controlBlock != null; }
public bool HasCameraBlock() { return this.cameraBlocks.Count() != 0; }
public bool HasNavigationPanels() { return this.navigationPanels.Count() != 0; }
public bool HasGpsPanels() { return this.gpsPanels.Count() != 0; }
public bool HasDiagnosticsPanels() { return this.diagnosticsPanels.Count() != 0; }
public bool HasLoggingPanels() { return this.loggingPanels.Count() != 0; }
public bool HasProgrammableBlocks() { return this.programableBlocks.Count() != 0; }
public bool HasConnectors() { return this.connectors.Count() != 0; }

public bool firstControlBlockScan = true;

public void ScanControlBlock() {
    Program.TagBlock(Me);
    this.controlBlock = null;
    List<IMyRemoteControl> blocks = new List<IMyRemoteControl>();
    GridTerminalSystem.GetBlocksOfType<IMyRemoteControl>(blocks, b => b.CubeGrid == Me.CubeGrid);
    if (blocks.Count() == 1) {
        this.controlBlock = blocks[0];
        Program.TagBlock(this.controlBlock);
        return;
    }
    foreach (IMyRemoteControl block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        Program.FixNameTag(block, match.Groups[1].Value, "");
        this.controlBlock = block;
        return;
    }
    if (firstControlBlockScan) {
        this.Log("Unable to find a Remote Controller");
        firstControlBlockScan = false;
        return;
    }
    if (!firstCameraScan && this.HasCameraBlock()) {
        this.Log("Found Remote Controller");
        firstControlBlockScan = true;
        return;
    }
}

public bool firstCameraScan = true;

public void ScanCameraBlocks() {
    this.cameraBlocks.Clear();
    List<IMyCameraBlock> blocks = new List<IMyCameraBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyCameraBlock>(blocks, b => b.CubeGrid == Me.CubeGrid);
    foreach (IMyCameraBlock block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        Program.FixNameTag(block, match.Groups[1].Value, "");
        block.EnableRaycast = true;
        this.cameraBlocks.Add(block);
    }
    if (firstCameraScan && !this.HasCameraBlock()) {
        this.Log("Unable to find a Camera");
        firstCameraScan = false;
        return;
    }
    if (!firstCameraScan && this.HasCameraBlock()) {
        this.Log("Found Camera(s)");
        firstCameraScan = true;
        return;
    }
}

public void ScanPanels() {
    this.navigationPanels.Clear();
    this.gpsPanels.Clear();
    this.diagnosticsPanels.Clear();
    this.loggingPanels.Clear();
    List<IMyTextPanel> blocks = new List<IMyTextPanel>();
    GridTerminalSystem.GetBlocksOfType<IMyTextPanel>(blocks);
    foreach (IMyTextPanel block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        switch (match.Groups[2].Value.ToUpper()) {
            case "DIAG":
                Program.FixNameTag(block, match.Groups[1].Value, " DIAG");
                Program.ConfigureTextPanel(block, 1.0f);
                this.diagnosticsPanels.Add(block);
                break;
            case "GPS":
                Program.FixNameTag(block, match.Groups[1].Value, " GPS");
                Program.ConfigureTextPanel(block, 0.75f);
                this.gpsPanels.Add(block);
                break;
            case "LOG":
                Program.FixNameTag(block, match.Groups[1].Value, " LOG");
                Program.ConfigureTextPanel(block, 0.71f); // 25x37
                this.loggingPanels.Add(block);
                break;
            default:
                Program.FixNameTag(block, match.Groups[1].Value, "");
                Program.ConfigureTextPanel(block, 1.2f);
                this.navigationPanels.Add(block);
                break;
        }
    }
}

public void ScanProgramableBlocks() {
    this.programableBlocks.Clear();
    List<IMyProgrammableBlock> blocks = new List<IMyProgrammableBlock>();
    GridTerminalSystem.GetBlocksOfType<IMyProgrammableBlock>(blocks, b => b.CubeGrid == Me.CubeGrid);
    foreach (IMyProgrammableBlock block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        switch (match.Groups[2].Value.ToUpper()) {
            case "NOTIFY":
                Program.FixNameTag(block, match.Groups[1].Value, " NOTIFY");
                this.programableBlocks.Add(block);
                break;
            default:
                Program.FixNameTag(block, match.Groups[1].Value, "");
                break;
        }
    }
}


public void ScanConnectors() {
    this.connectors.Clear();
    List<IMyShipConnector> blocks = new List<IMyShipConnector>();
    GridTerminalSystem.GetBlocksOfType<IMyShipConnector>(blocks, b => b.CubeGrid == Me.CubeGrid);
    foreach (IMyShipConnector block in blocks) {
        var match = Program.tagRegex.Match(block.CustomName);
        if (!match.Success) continue;
        Program.FixNameTag(block, match.Groups[1].Value, "");
        this.connectors.Add(block);
    }
}

// Blocks
// -------------------------------------------------------
// Class Constructor & Serializers

public Program() {
    this.Log("Sam's AutoPilot initialized");
    this.InitSerializables();
    try {
        if (this.Load()) this.Log("Loaded " + this.docks.Count().ToString() + " docks from Storage");
    } catch (Exception e) {
        this.Log("Unable to load Storage: " + e.Message);
        Storage = "";
        this.InitSerializables();
    }
    this.AdjustDimentions();
    Runtime.UpdateFrequency = UpdateFrequency.Update10 | UpdateFrequency.Update100;
}

public void InitSerializables() {
    this.autopilotRunning = false;
    this.autopilotDestination = "";
    this.waypoints = new List<Waypoint>();
    this.docks = new Dictionary<string, DockMetadata>();
    this.docksConfigured = new List<string>();
    this.dockSelected = 0;
}

public bool Load() {
    if (Storage.Length == 0) {
        return false;
    }
    var parts = Storage.Split(';');
    var part = 0;
    this.autopilotRunning = bool.Parse(parts[part++]);
    this.autopilotDestination = parts[part++];
    var waypointCount = int.Parse(parts[part++]);
    for (int i = 0; i < waypointCount; ++i) {
        var rayTrace = bool.Parse(parts[part++]);
        var reverse = bool.Parse(parts[part++]);
        var dockingMode = bool.Parse(parts[part++]);
        var maxSpeed = float.Parse(parts[part++]);
        this.waypoints.Add(new Waypoint(rayTrace, reverse, dockingMode, maxSpeed, Program.UnserializeVector(parts[part++])));
    }
    this.dockSelected = int.Parse(parts[part++]);
    var docksConfiguredCount = int.Parse(parts[part++]);
    for (int i = 0; i < docksConfiguredCount; ++i) {
        this.docksConfigured.Add(parts[part++]);
    }
    for (int i = 0; i < docksConfiguredCount; ++i) {
        this.docks.Add(this.docksConfigured[i], new DockMetadata(
            Program.UnserializeVector(parts[part++]),
            Program.UnserializeVector(parts[part++]),
            Program.UnserializeVector(parts[part++]),
            double.Parse(parts[part++]),
            double.Parse(parts[part++]),
            double.Parse(parts[part++])));
    }
    return true;
}

public void Save() {
    Storage = (this.autopilotRunning & this.waypoints.Count() != 0).ToString();
    Storage += ";" + this.autopilotDestination;

    Storage += ";" + this.waypoints.Count().ToString();
    for (int i = 0; i < this.waypoints.Count(); ++i) {
        Storage += ";" + this.waypoints[i].rayTrace.ToString();
        Storage += ";" + this.waypoints[i].reverse.ToString();
        Storage += ";" + this.waypoints[i].dockingMode.ToString();
        Storage += ";" + this.waypoints[i].maxSpeed.ToString();
        Storage += ";" + Program.SerializeVector(this.waypoints[i].position);
    }
    Storage += ";" + this.dockSelected;
    Storage += ";" + this.docksConfigured.Count().ToString();
    for (int i = 0; i < this.docksConfigured.Count(); i++) {
        Storage += ";" + this.docksConfigured[i];
    }
    for (int i = 0; i < this.docksConfigured.Count(); i++) {
        var metadata = this.docks[this.docksConfigured[i]];
        Storage += ";" + Program.SerializeVector(metadata.position);
        Storage += ";" + Program.SerializeVector(metadata.front);
        Storage += ";" + Program.SerializeVector(metadata.up);
        Storage += ";" + metadata.surfaceAltitude.ToString();
        Storage += ";" + metadata.centerAltitude.ToString();
        Storage += ";" + metadata.gravity.ToString();
    }
}

// Class Constructor & Serializers
// -------------------------------------------------------
// Static variables

public static int LOG_MAX_LINES = 23; // 25-2
public static int NATO_MAX_CODE_SIZE = 8;
public static List<string> NATO_CODES = new List<string>(new string[] { "Alfa", "Bravo", "Charlie", "Delta", "Echo", "Foxtrot", "Golf", "Hotel", "India", "Juliett", "Kilo", "Lima", "Mike", "November", "Oscar", "Papa", "Quebec", "Romeo", "Sierra", "Tango", "Uniform", "Victor", "Whiskey", "Xray", "Yankee", "Zulu" });
public static List<char> SLIDER_ROTATOR = new List<char>(new char[] { '-', '\\', '|', '/' });
public static List<char> SLIDER_EXCLAMATION_BLINK = new List<char>(new char[] { ' ', '!' });
public static Dictionary<Vector3I, string> THRUST_DIRECTION = new Dictionary<Vector3I, string>() {{ Vector3I.Forward, "Forward" }, { Vector3I.Backward, "Backward" }, { Vector3I.Left, "Left" }, { Vector3I.Right, "Right" }, { Vector3I.Up, "Up" }, { Vector3I.Down, "Down" },};
public static Dictionary<float, ThrusterBlockType> THRUST_TYPE = new Dictionary<float, ThrusterBlockType>() {{ 6000000, new ThrusterBlockType("Large", "Large", "Hydrogen") }, {  900000, new ThrusterBlockType("Large", "Small", "Hydrogen") }, {  400000, new ThrusterBlockType("Small", "Large", "Hydrogen") }, {   82000, new ThrusterBlockType("Small", "Small", "Hydrogen") },{ 5400000, new ThrusterBlockType("Large", "Large", "Atmospheric") }, {  420000, new ThrusterBlockType("Large", "Small", "Atmospheric") }, {  408000, new ThrusterBlockType("Small", "Large", "Atmospheric") }, {   80000, new ThrusterBlockType("Small", "Small", "Atmospheric") },{ 3600000, new ThrusterBlockType("Large", "Large", "Ion") }, {  288000, new ThrusterBlockType("Large", "Small", "Ion") }, {  144000, new ThrusterBlockType("Small", "Large", "Ion") }, {   12000, new ThrusterBlockType("Small", "Small", "Ion") },};
public static List<string> DIAGNOSTICS_TABS = new List<string>(new string[] { "Menu", "Guidelines", "Remote controller", "All thrusters", "Atmospheric", "Hydrogen", "Ion" });

static System.Text.RegularExpressions.Regex tagRegex = new System.Text.RegularExpressions.Regex("\\[" + Program.TAG + "(\\s+([0-9a-zA-Z]*)\\s*)*\\]", System.Text.RegularExpressions.RegexOptions.IgnoreCase);

// Static variables
// -------------------------------------------------------
// Static methods 

public static string SerializeVector(Vector3D vec) {
    return vec.GetDim(0).ToString() + ":" + vec.GetDim(1).ToString() + ":" + vec.GetDim(2).ToString();
}

public static Vector3D UnserializeVector(string str) {
    var parts = str.Split(':');
    return new Vector3D(double.Parse(parts[0]), double.Parse(parts[1]), double.Parse(parts[2]));
}

public static void Print(List<IMyTextPanel> panels, string str) {
    foreach (IMyTextPanel panel in panels) panel.WritePublicText(str);
}

public static IMyTextPanel ConfigureTextPanel(IMyTextPanel block, float fontSize) {
    block.FontSize = fontSize;
    block.Font = "Monospace";
    block.ShowPublicTextOnScreen();
    return block;
}

public static void FixNameTag(IMyTerminalBlock block, string oldStr, string newStr) {
    if (oldStr.Length == 0) return;
    block.CustomName = block.CustomName.Replace("[" + Program.TAG + oldStr + "]", "[" + Program.TAG + newStr + "]");
}

public static void TagBlock(IMyTerminalBlock block) {
    var match = tagRegex.Match(block.CustomName);
    if (!match.Success) {
        block.CustomName += " [" + Program.TAG + "]";
        return;
    }
    Program.FixNameTag(block, match.Groups[1].Value, "");
}

public static Vector3D QuarterSlerp(Vector3D source, Vector3D destination, Vector3D origin) {
    var vSrc = source - origin;
    var vDst = destination - origin;
    var vSrcLen = vSrc.Length();
    var vDstLen = vDst.Length();
    var qRotation = Quaternion.CreateFromTwoVectors(vSrc, vDst);
    var qSlerp = Quaternion.Slerp(Quaternion.Identity, qRotation, 0.25f);
    var vNew = ((vSrcLen + vDstLen) / 2.0) * Vector3D.Normalize(vSrc);
    return origin + Vector3D.Transform(vNew, qSlerp);
}

public static Vector3D LerpToDistance(double distance, Vector3D source, Vector3D destination) {
    return source + (distance * Vector3D.Normalize(destination - source));
}

public static Vector3D MoveAwayFrom(double distance, Vector3D moveAway, Vector3D from) {
    return moveAway + (distance * Vector3D.Normalize(moveAway - from));
}

public static bool NearAnyDock(double maxDistance, Vector3D position, Dictionary<string, DockMetadata> docks) {
    foreach (KeyValuePair<string, DockMetadata> dock in docks) {
        var distance = Vector3D.Distance(position, dock.Value.position);
        if (distance <= maxDistance) return true;
    }
    return false;
}

public static string GetNearestDock(bool farthest, Vector3D position, Dictionary<string, DockMetadata> docks) {
    string foundDock = "";
    double foundDistance = farthest ? -Double.MaxValue : Double.MaxValue;
    foreach (KeyValuePair<string, DockMetadata> dock in docks) {
        var distance = Vector3D.Distance(position, dock.Value.position);
        if (distance < foundDistance && !farthest || distance > foundDistance && farthest) {
            foundDock = dock.Key; foundDistance = distance;
        }
    }
    return foundDock;
}

// Static methods 
// -------------------------------------------------------
// Classes

public class DockMetadata {
    public Vector3D position;
    public Vector3D front;
    public Vector3D up;
    public double surfaceAltitude;
    public double centerAltitude;
    public double gravity;
    public DockMetadata(Vector3D p, Vector3D f, Vector3D u, double sA, double cA, double g) {
        this.position = p; this.front = f; this.up = u; this.surfaceAltitude = sA; this.centerAltitude = cA; this.gravity = g;
    }
}

public class DisplaySlider {
    public List<char> displayList;
    public DisplaySlider(List<char> l) {
        this.displayList = new List<char>(l);
    }
    public string GetString() {
        this.displayList.Move(this.displayList.Count() - 1, 0);
        return this.displayList.First().ToString();
    }
}

public class TrustInfo {
    public float theoretical;
    public float effective;
    public TrustInfo(float t, float e) {
        this.theoretical = t;
        this.effective = e;
    }
    public void Add(float t, float e) { this.theoretical += t; this.effective += e; }
}

public class ThrusterBlockType {
    public string grid;
    public string size;
    public string type;
    public ThrusterBlockType(string g, string size, string t) { this.grid = g; this.size = size; this.type = t; }
}

public class Waypoint {
    public bool rayTrace;
    public bool reverse;
    public bool dockingMode;
    public float maxSpeed;
    public Vector3D position;
    public Waypoint(bool rayTrace, bool reverse, bool dockingMode, float maxSpeed, Vector3D position) {
        this.rayTrace = rayTrace; this.reverse = reverse; this.dockingMode = dockingMode; this.maxSpeed = maxSpeed; this.position = position;
    }
}

public class Notification {
    public DateTime when;
    public string notification;
    public Notification(DateTime when, string notification) {
        this.when = when; this.notification = notification;
    }
}
// Classes
// -------------------------------------------------------
