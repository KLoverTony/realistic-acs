using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using Havok;
using Sandbox.Game.Entities;
using Sandbox.Game.World;
using MyAPIGateway = Sandbox.ModAPI.MyAPIGateway;
using Sandbox.ModAPI.Ingame;
using VRage.Game.Entity;
using VRage.FileSystem;
using VRage.Plugins;
using VRage.Utils;
using VRageMath;
using VRageRender.Import;

#if !LOCAL_BUILD
[assembly: AssemblyVersion("0.1.0.0")]
[assembly: AssemblyFileVersion("0.1.0.0")]
#endif

namespace ClientPlugin;

// ReSharper disable once UnusedType.Global
public sealed class Plugin : IPlugin
{
    public const string Name = "Realistic ACS";
    public static Plugin Instance { get; private set; }
    private long updateCount;
    // The superseded continuous-torque experiment remains opt-in only.  Normal
    // automatic control is provided by the combined vanilla/RCS controller.
    private bool experimentalPulseArmed;
    private bool chatHookRegistered;
    // Combined attitude control follows any player-controlled grid by default.
    // /acs attitude disarm remains a global opt-out.
    private bool attitudeControlArmed = true;
    private long attitudeControlGridId;
    private readonly Dictionary<long, RcsVisualState> rcsVisualStates = new();
    private readonly Dictionary<long, HydrogenOverrideState> hydrogenOverrides = new();
    private long hydrogenOverrideGridId;
    private readonly Dictionary<long, GridActuatorCache> actuatorCaches = new();
    private readonly ControlAllocationCache attitudeAllocationCache = new();
    private readonly ControlAllocationCache dampeningAllocationCache = new();
    private readonly ControlAllocationCache overrideCounterAllocationCache = new();
    private readonly Dictionary<long, AlwaysOnGridRegistration> alwaysOnGrids = new();
    private Dictionary<long, Vector3D> rcsVisualAllocatedForces = new();
    private long rcsVisualAllocationGridId;
    private bool rcsTiltDiagnosticActive;
    private long rcsTiltDiagnosticGridId;
    private Vector3D rcsTiltDiagnosticGridDirection;
    private bool rcsSharedVectorProbeActive;
    private long rcsSharedVectorProbeGridId;
    private Vector3D rcsSharedVectorProbeGridThrust;
    private RotationalDampeningMode rotationalDampeningMode = RotationalDampeningMode.Follow;
    // Persistent rotational inertia is enabled by default for the locally controlled grid.
    // /acs inertia off remains an immediate opt-out.
    private bool inertiaModeEnabled = true;
    private bool angularDampingOverridden;
    private long activeOverrideGridId;
    private long inertiaGridId;
    private float originalAngularDamping;
    private MyCubeGrid inertiaGrid;
    private HkRigidBody activeOverrideRigidBody;
    private Vector3 pulseStartAngularVelocity;
    private bool physicsTorquePulseActive;
    private Vector3 activePhysicsTorque;
    private Vector3 accumulatedTorqueImpulse;
    private float peakAppliedTorqueMagnitude;
    private bool torqueSafetyLimiting;
    private MatrixD activeControlFrame;
    private const float MinimumExperimentalTorqueScale = 0.001f;
    private const float DefaultExperimentalTorqueScale = 1.00f;
    private const float MaximumExperimentalTorqueScale = 1.00f;
    private const float AngularSpeedSafetyLimit = 2.00f;
    private const float AngularSpeedSafetyResume = 1.80f;
    private const float NominalSimulationStepSeconds = 1f / 60f;
    private const string RcsTag = "[ACS-RCS]";
    private const string RcsBlockSubtype = "RealisticAcsRcsLarge";
    private const string RcsHydrogenBlockSubtype = "RealisticAcsRcsHydrogenLarge";
    private const string RcsAzimuthSubpart = "RCS_Azimuth";
    private const string RcsElevationSubpart = "RCS_Elevation";
    private const string RcsAzimuthDummy = "subpart_" + RcsAzimuthSubpart;
    private const string RcsElevationDummy = "subpart_" + RcsElevationSubpart;
    private const string RcsForceAxisDummy = "RCS_FORCE_AXIS";
    private const string RcsAzimuthModel = "RCS_Pod_Azimuth";
    private const string RcsElevationModel = "RCS_Pod_Elevation";
    private const string RcsHydrogenAzimuthModel = "RCS_hydrogen_Pod_Azimuth";
    private const string RcsHydrogenElevationModel = "RCS_hydrogen_Pod_Elevation";
    // The Blender rig is authored for up to 80 degrees. Sixty degrees retains
    // generous clearance while giving each visual gimbal useful travel.
    // The nozzle may incline only 60 degrees from its neutral axis, but the
    // azimuth bearing must be able to swivel around that axis.  Limiting both
    // axes to 60 degrees incorrectly makes a pod unable to reach a valid
    // lateral direction after the block itself is rolled by 90 degrees.
    private const float RcsMaximumAzimuthRadians = MathHelper.Pi;
    private const float RcsMaximumElevationRadians = MathHelper.Pi / 3f;
    // A tiny numerical tolerance keeps directions sampled exactly at 60 degrees
    // from chattering across the gate because of floating-point rounding.
    private const double RcsConeMinimumAlignment = 0.499; // approximately cos(60 degrees)
    private const float RcsVisualRecenteringRadiansPerFrame = 0.06f;
    private const double RcsVisualInputDeadzone = 0.01;
    // Finite ion-RCS rating for the current non-conveyor cluster subtype.  A
    // future hydrogen subtype can supply a different value through
    // GetRcsMaximumForce without changing the allocator.
    private const double RcsIonMaximumForce = 50000.0;
    private const double RcsHydrogenMaximumForce = 150000.0;
    // Do not turn a deliberately unavailable requested axis into a visibly
    // incorrect cross-axis rotation.  The diagnostic still reports the
    // allocation, but live physics requires a reasonably aligned torque.
    private const double RcsMinimumTorqueAlignment = 0.85;
    // Vanilla-thruster geometry is used as a virtual attitude actuator model,
    // rather than writing native thrust overrides. Only an effectively pure
    // virtual couple is eligible, so the model never implies unaccounted-for
    // translation while it applies simulated attitude torque.
    private const double MaximumTranslationResidualRatio = 0.02;
    private const long ActuatorCacheRefreshFrames = 30;
    private const long AllocationSolveIntervalFrames = 6;
    private const long AlwaysOnValidationFrames = 3600;
    private const int MaximumAlwaysOnGrids = 16;
    private const double AllocationInputChangeThresholdPercent = 0.25;
    private const int MaximumVanillaAllocationIterations = 96;
    private const long LifecycleLogFlushFrames = 120;
    private const long LifecycleLogMaximumBytes = 1024 * 1024;
    private static readonly ConcurrentQueue<string> PendingLifecycleLogLines = new();
    private static readonly object LifecycleLogFileLock = new();
    private static string lifecycleLogPath;
    private static int lifecycleLogFlushScheduled;
    private const float RcsAxisDiagnosticRadians = MathHelper.Pi / 9f; // 20 degrees
    private const double RawRotationIndicatorFullScale = 9.0;
    private const double PilotInputMaximumTorquePercent = 25.0;
    // Initial motion test: sufficiently strong to see on a test grid, while
    // remaining bounded by the emergency angular-speed cutoff below.
    private const double LiveAttitudeTorqueScale = 1.00;
    // Vanilla engines are the coarse, high-authority *virtual* source. Their
    // geometry supplies simulated r x F authority without altering translation
    // thrust, player input, or native thruster overrides.
    private const double VanillaAttitudeTorqueScale = 1.00;
    // Damping uses the same full allocated authority as pilot rotation. Its
    // requested torque still tapers proportionally with angular velocity.
    private const double RotationalDampeningTorqueScale = 1.00;
    private const double AngularSpeedAtFullDampeningTorque = 0.20;
    // Physics never settles at an exact mathematical zero.  Below this speed the
    // rotation is visually imperceptible, so stop correcting instead of hunting.
    private const float AngularSpeedDampeningDeadzone = 0.0005f;
    private float experimentalTorqueScale = DefaultExperimentalTorqueScale;

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining)]
    public void Init(object gameInstance)
    {
        Instance = this;
        InitializeLifecycleLogPath();
        const string message = "Finite RCS plus virtual vanilla-thruster-geometry attitude control, persistent rotational inertia, and inertial-dampener-following rotational arrest are enabled by default for every player-controlled grid. A 2.00 rad/s emergency angular-speed cutoff is active.";
        MyLog.Default.WriteLine($"[{Name}] {message}");
        WriteLifecycleLog(message);
    }

    public void Dispose()
    {
        if (chatHookRegistered)
        {
            try
            {
                MyAPIGateway.Utilities?.MessageEntered -= OnMessageEntered;
            }
            catch
            {
                // Pulsar can dispose a plugin after the game API has already unloaded.
            }
        }

        RestoreExperimentalOverrides("plugin unload");
        RestoreHydrogenRcsOverrides("plugin unload");
        RestoreAngularDamping("plugin unload");
        RestoreAlwaysOnAngularDamping("plugin unload");
        alwaysOnGrids.Clear();
        const string message = "Unloaded.";
        MyLog.Default.WriteLine($"[{Name}] {message}");
        WriteLifecycleLog(message);
        FlushLifecycleLogSynchronously();
        Instance = null;
    }

    public void Update()
    {
        updateCount++;
        EnsureChatHook();

        var controlledGrid = MySession.Static?.ControlledGrid;
        RegisterAlwaysOnGridIfConfigured(controlledGrid);
        UpdateInertiaMode(controlledGrid);
        UpdateRcsVisuals(controlledGrid);
        UpdateAttitudeControl(controlledGrid);
        UpdateRotationalDampening(controlledGrid);
        UpdateManualOverrideTorque(controlledGrid);
        UpdateAlwaysOnGrids(controlledGrid);

        if (updateCount % LifecycleLogFlushFrames == 0)
            QueueLifecycleLogFlush();

    }

    private void EnsureChatHook()
    {
        if (chatHookRegistered)
            return;

        try
        {
            var utilities = MyAPIGateway.Utilities;
            if (utilities == null)
                return;

            utilities.MessageEntered += OnMessageEntered;
            chatHookRegistered = true;
            WriteLifecycleLog("Registered the local /acs chat-command handler.");
        }
        catch
        {
            // The game API may not be initialized yet. Try again on a later update.
        }
    }

    private void OnMessageEntered(string messageText, ref bool sendToOthers)
    {
        string command = messageText?.Trim();
        bool isInertiaCommand = command?.StartsWith("/acs inertia", StringComparison.OrdinalIgnoreCase) == true;
        bool isAttitudeCommand = command?.StartsWith("/acs attitude", StringComparison.OrdinalIgnoreCase) == true;
        bool isRotationalDampenersCommand = command?.StartsWith("/acs rotational-dampeners", StringComparison.OrdinalIgnoreCase) == true;
        if (!string.Equals(command, "/acs arm", StringComparison.OrdinalIgnoreCase)
            && !string.Equals(command, "/acs disarm", StringComparison.OrdinalIgnoreCase)
            && !string.Equals(command, "/acs status", StringComparison.OrdinalIgnoreCase)
            && !isInertiaCommand
            && !isAttitudeCommand
            && !isRotationalDampenersCommand)
        {
            return;
        }

        sendToOthers = false;
        if (isInertiaCommand)
        {
            SetInertiaMode(command);
            return;
        }

        if (isAttitudeCommand)
        {
            SetAttitudeControl(command);
            return;
        }

        if (isRotationalDampenersCommand)
        {
            SetRotationalDampeningMode(command);
            return;
        }

        if (string.Equals(command, "/acs disarm", StringComparison.OrdinalIgnoreCase))
        {
            DisarmAttitudeControl("master ACS control disarmed by chat command");
            inertiaModeEnabled = false;
            RestoreAngularDamping("master ACS control disarmed by chat command");
            RestoreAlwaysOnAngularDamping("master ACS control disarmed by chat command");
            return;
        }

        if (string.Equals(command, "/acs status", StringComparison.OrdinalIgnoreCase))
        {
            string status = $"ACS attitude control: {(attitudeControlArmed ? "armed" : "disarmed")}; persistent inertia: {(inertiaModeEnabled ? "on" : "off")}; rotational dampening: {rotationalDampeningMode.ToString().ToLowerInvariant()}; emergency angular-speed cutoff: {AngularSpeedSafetyLimit:F2} rad/s.";
            WriteLifecycleLog($"Chat status request: {status}");
            ShowChatFeedback(status);
            return;
        }

        attitudeControlArmed = true;
        attitudeControlGridId = 0;
        inertiaModeEnabled = true;
        const string armed = "ACS armed: finite RCS plus virtual vanilla-thruster-geometry attitude control, persistent inertia, and rotational dampening are enabled for the controlled grid.";
        WriteLifecycleLog(armed);
        ShowChatFeedback(armed);
    }

    private void SetExperimentalTorqueScale(string command)
    {
        const string prefix = "/acs scale";
        string valueText = command.Substring(prefix.Length).Trim();
        if (!float.TryParse(valueText, NumberStyles.Float, CultureInfo.InvariantCulture, out float percent)
            || percent < MinimumExperimentalTorqueScale * 100
            || percent > MaximumExperimentalTorqueScale * 100)
        {
            const string usage = "Invalid ACS scale. Use /acs scale <0.1-100>, where the value is a percent.";
            WriteLifecycleLog(usage);
            ShowChatFeedback(usage);
            return;
        }

        experimentalTorqueScale = percent / 100f;
        string message = $"ACS experimental torque scale set to {experimentalTorqueScale:P1}.";
        WriteLifecycleLog(message);
        ShowChatFeedback(message);
    }

    private void SetInertiaMode(string command)
    {
        const string prefix = "/acs inertia";
        string argument = command.Substring(prefix.Length).Trim();
        if (string.Equals(argument, "on", StringComparison.OrdinalIgnoreCase))
        {
            inertiaModeEnabled = true;
            WriteLifecycleLog("Persistent-inertia experiment enabled. Angular damping will be removed only from the currently controlled grid.");
            ShowChatFeedback("ACS persistent-inertia experiment enabled.");
            return;
        }

        if (string.Equals(argument, "off", StringComparison.OrdinalIgnoreCase))
        {
            inertiaModeEnabled = false;
            RestoreAngularDamping("disabled by chat command");
            RestoreAlwaysOnAngularDamping("disabled by chat command");
            WriteLifecycleLog("Persistent-inertia experiment disabled.");
            ShowChatFeedback("ACS persistent-inertia experiment disabled.");
            return;
        }

        const string usage = "Invalid inertia command. Use /acs inertia on or /acs inertia off.";
        WriteLifecycleLog(usage);
        ShowChatFeedback(usage);
    }

    private static bool TryGetPilotInputPercent(
        out double pitchPercent,
        out double yawPercent,
        out double rollPercent,
        out Vector2 rotation,
        out float roll)
    {
        var controller = MySession.Static?.ControlledEntity as IMyShipController;
        if (controller == null)
        {
            pitchPercent = 0;
            yawPercent = 0;
            rollPercent = 0;
            rotation = Vector2.Zero;
            roll = 0;
            return false;
        }

        rotation = controller.RotationIndicator;
        roll = controller.RollIndicator;
        pitchPercent = -Clamp(rotation.X / RawRotationIndicatorFullScale, -1, 1) * PilotInputMaximumTorquePercent;
        yawPercent = -Clamp(rotation.Y / RawRotationIndicatorFullScale, -1, 1) * PilotInputMaximumTorquePercent;
        rollPercent = Clamp(roll, -1, 1) * PilotInputMaximumTorquePercent;
        return true;
    }

    private void SetAttitudeControl(string command)
    {
        const string prefix = "/acs attitude";
        string argument = command.Substring(prefix.Length).Trim();
        if (string.Equals(argument, "arm", StringComparison.OrdinalIgnoreCase))
        {
            attitudeControlArmed = true;
            attitudeControlGridId = 0;
            const string armed = "ACS attitude control armed. Turn vanilla gyros off; working RCS blocks and powered vanilla-thruster geometry provide finite virtual attitude authority while you hold a rotation input.";
            WriteLifecycleLog(armed);
            ShowChatFeedback(armed);
            return;
        }

        if (string.Equals(argument, "disarm", StringComparison.OrdinalIgnoreCase))
        {
            DisarmAttitudeControl("disarmed by chat command");
            return;
        }

        const string usage = "Invalid attitude command. Use /acs attitude arm or /acs attitude disarm.";
        WriteLifecycleLog(usage);
        ShowChatFeedback(usage);
    }

    private void UpdateAttitudeControl(MyCubeGrid controlledGrid)
    {
        if (!attitudeControlArmed)
            return;

        if (controlledGrid?.Physics?.RigidBody == null)
            return;

        // A player changing cockpits/grids is not an opt-out. Rebind to the new
        // controlled grid and retain the user's global arm/disarm preference.
        attitudeControlGridId = controlledGrid.EntityId;
        if (!TryGetPilotInputPercent(out double pitchPercent, out double yawPercent, out double rollPercent, out _, out _)
            || (Math.Abs(pitchPercent) < 0.001 && Math.Abs(yawPercent) < 0.001 && Math.Abs(rollPercent) < 0.001))
            return;

        GetCachedControlAllocations(
            attitudeAllocationCache, controlledGrid, pitchPercent, yawPercent, rollPercent,
            out bool hasRcsAllocation, out RcsForceAllocation rcsAllocation,
            out bool hasVanillaAllocation, out AllocationResult vanillaAllocation);
        double rcsAlignment = hasRcsAllocation ? GetTorqueAlignment(rcsAllocation.RequestedTorque, rcsAllocation.AchievedTorque) : 0;
        double vanillaAlignment = hasVanillaAllocation ? GetTorqueAlignment(vanillaAllocation.RequestedTorque, vanillaAllocation.AchievedTorque) : 0;
        double rcsResidual = hasRcsAllocation ? GetTranslationResidualRatio(rcsAllocation.TranslationResidual, rcsAllocation.ForceCapacity) : double.PositiveInfinity;
        double vanillaResidual = hasVanillaAllocation ? GetTranslationResidualRatio(vanillaAllocation.TranslationResidual, vanillaAllocation.ForceCapacity) : double.PositiveInfinity;
        Vector3D combinedTorque = Vector3D.Zero;
        if (hasRcsAllocation && rcsAlignment >= RcsMinimumTorqueAlignment && rcsResidual <= MaximumTranslationResidualRatio)
            combinedTorque += rcsAllocation.AchievedTorque * LiveAttitudeTorqueScale;
        if (hasVanillaAllocation && vanillaAlignment >= RcsMinimumTorqueAlignment && vanillaResidual <= MaximumTranslationResidualRatio)
            combinedTorque += vanillaAllocation.AchievedTorque * VanillaAttitudeTorqueScale;

        if (combinedTorque.LengthSquared() <= 1)
        {
            if (updateCount % 60 == 0)
                WriteLifecycleLog($"ACS attitude control withheld an unbalanced virtual wrench: RCS alignment/residual {rcsAlignment:F2}/{rcsResidual:P1}; vanilla alignment/residual {vanillaAlignment:F2}/{vanillaResidual:P1}.");
            return;
        }

        Vector3 appliedTorque = ToVector3(combinedTorque);
        Vector3 angularVelocity = controlledGrid.Physics.RigidBody.AngularVelocity;
        if (angularVelocity.Length() >= AngularSpeedSafetyLimit && Vector3.Dot(angularVelocity, appliedTorque) > 0)
        {
            if (updateCount % 60 == 0)
                WriteLifecycleLog("ACS finite RCS attitude control is holding torque at the emergency angular-speed cutoff.");
            return;
        }

        controlledGrid.Physics.RigidBody.ApplyTorque(NominalSimulationStepSeconds, appliedTorque);
        if (updateCount % 60 == 0)
        {
            WriteLifecycleLog(
                $"ACS finite RCS attitude control — pilot request pitch/yaw/roll: {pitchPercent:F1}% / {yawPercent:F1}% / {rollPercent:F1}%; "
                + $"RCS torque/alignment: {(hasRcsAllocation ? rcsAllocation.AchievedTorque.ToString() : "unavailable")}/{rcsAlignment:F2}; "
                + $"vanilla torque/alignment: {(hasVanillaAllocation ? vanillaAllocation.AchievedTorque.ToString() : "unavailable")}/{vanillaAlignment:F2}; residual RCS/vanilla: {rcsResidual:P1}/{vanillaResidual:P1}; "
                + $"combined applied torque: {appliedTorque}."
            );
        }
    }

    private void DisarmAttitudeControl(string reason)
    {
        bool wasArmed = attitudeControlArmed;
        attitudeControlArmed = false;
        attitudeControlGridId = 0;
        RestoreHydrogenRcsOverrides(reason);
        if (!wasArmed)
            return;

        string message = $"ACS finite RCS attitude control disarmed ({reason}).";
        WriteLifecycleLog(message);
        ShowChatFeedback(message);
    }

    private void GetCachedControlAllocations(
        ControlAllocationCache cache,
        MyCubeGrid grid,
        double pitchPercent,
        double yawPercent,
        double rollPercent,
        out bool hasRcsAllocation,
        out RcsForceAllocation rcsAllocation,
        out bool hasVanillaAllocation,
        out AllocationResult vanillaAllocation)
    {
        bool inputChanged = Math.Abs(cache.PitchPercent - pitchPercent) > AllocationInputChangeThresholdPercent
            || Math.Abs(cache.YawPercent - yawPercent) > AllocationInputChangeThresholdPercent
            || Math.Abs(cache.RollPercent - rollPercent) > AllocationInputChangeThresholdPercent;
        if (cache.GridId != grid.EntityId
            || inputChanged
            || updateCount - cache.LastSolveFrame >= AllocationSolveIntervalFrames)
        {
            cache.GridId = grid.EntityId;
            cache.PitchPercent = pitchPercent;
            cache.YawPercent = yawPercent;
            cache.RollPercent = rollPercent;
            cache.LastSolveFrame = updateCount;
            cache.HasRcsAllocation = TryCalculateRcsForceAllocation(grid, pitchPercent, yawPercent, rollPercent, out cache.RcsAllocation);
            cache.HasVanillaAllocation = TryCalculateWholeGridAllocation(grid, pitchPercent, yawPercent, rollPercent, out cache.VanillaAllocation);
        }

        hasRcsAllocation = cache.HasRcsAllocation;
        rcsAllocation = cache.RcsAllocation;
        hasVanillaAllocation = cache.HasVanillaAllocation;
        vanillaAllocation = cache.VanillaAllocation;
    }

    private void SetRotationalDampeningMode(string command)
    {
        const string prefix = "/acs rotational-dampeners";
        string argument = command.Substring(prefix.Length).Trim();
        switch (argument.ToLowerInvariant())
        {
            case "follow":
                rotationalDampeningMode = RotationalDampeningMode.Follow;
                break;
            case "on":
                rotationalDampeningMode = RotationalDampeningMode.On;
                break;
            case "off":
                rotationalDampeningMode = RotationalDampeningMode.Off;
                break;
            default:
                const string usage = "Invalid rotational-dampeners command. Use /acs rotational-dampeners follow, on, or off.";
                WriteLifecycleLog(usage);
                ShowChatFeedback(usage);
                return;
        }

        string message = $"ACS rotational dampening mode set to {rotationalDampeningMode.ToString().ToLowerInvariant()}.";
        WriteLifecycleLog(message);
        ShowChatFeedback(message);
    }

    private void UpdateRcsVisuals(MyCubeGrid controlledGrid)
    {
        // This callback also runs at the main menu, where there is no controlled
        // grid.  Do not dereference it while clearing the previous session state.
        if (controlledGrid == null)
        {
            RestoreHydrogenRcsOverrides("control ended");
            rcsVisualStates.Clear();
            rcsVisualAllocatedForces.Clear();
            rcsVisualAllocationGridId = 0;
            return;
        }

        if (hydrogenOverrideGridId != 0 && hydrogenOverrideGridId != controlledGrid.EntityId)
            RestoreHydrogenRcsOverrides("controlled grid changed");

        // Initialize the authored force axes before planning.  The planner needs
        // each pod's neutral, mounted direction in order to honour its cone.
        GridActuatorCache actuatorCache = GetGridActuatorCache(controlledGrid);
        var rcsBlocks = actuatorCache.RcsBlocks;
        foreach (MyCubeBlock rcsBlock in rcsBlocks)
        {
            if (!rcsVisualStates.TryGetValue(rcsBlock.EntityId, out RcsVisualState state))
            {
                state = new RcsVisualState();
                rcsVisualStates[rcsBlock.EntityId] = state;
            }

            if (!state.Initialized && !state.InitializationAttempted)
            {
                state.InitializationAttempted = true;
                TryInitializeRcsVisuals(rcsBlock, state);
            }
        }

        double inputPitch = 0;
        double inputYaw = 0;
        double inputRoll = 0;
        if (TryGetPilotInputPercent(out double pilotPitch, out double pilotYaw, out double pilotRoll, out _, out _))
        {
            inputPitch = pilotPitch / PilotInputMaximumTorquePercent;
            inputYaw = pilotYaw / PilotInputMaximumTorquePercent;
            inputRoll = pilotRoll / PilotInputMaximumTorquePercent;
        }

        bool hasRcsInput = Math.Abs(inputPitch) > RcsVisualInputDeadzone
            || Math.Abs(inputYaw) > RcsVisualInputDeadzone
            || Math.Abs(inputRoll) > RcsVisualInputDeadzone;
        if (rcsSharedVectorProbeActive)
        {
            if (rcsSharedVectorProbeGridId != controlledGrid.EntityId)
            {
                rcsSharedVectorProbeActive = false;
                rcsSharedVectorProbeGridId = 0;
                rcsVisualAllocatedForces.Clear();
            }
            else
            {
                Vector3D sharedThrustWorld = Vector3D.Normalize(Vector3D.TransformNormal(
                    rcsSharedVectorProbeGridThrust,
                    controlledGrid.WorldMatrix));
                rcsVisualAllocatedForces = new Dictionary<long, Vector3D>();
                foreach (MyCubeBlock rcsBlock in rcsBlocks)
                {
                    if (IsRcsProbeBlock(rcsBlock) && rcsVisualStates.TryGetValue(rcsBlock.EntityId, out RcsVisualState state) && state.Initialized)
                        rcsVisualAllocatedForces[rcsBlock.EntityId] = sharedThrustWorld * GetRcsMaximumForce(rcsBlock);
                }
            }
        }
        else if (rcsTiltDiagnosticActive)
        {
            if (rcsTiltDiagnosticGridId != controlledGrid.EntityId)
            {
                rcsTiltDiagnosticActive = false;
                rcsTiltDiagnosticGridId = 0;
                rcsVisualAllocatedForces.Clear();
            }
            else
            {
                // The calibration bypasses force allocation.  Its purpose is to
                // verify each pod's local inverse-kinematic response to one
                // shared grid-relative tilt direction.
                rcsVisualAllocatedForces.Clear();
            }
        }
        else if (!hasRcsInput)
        {
            rcsVisualAllocatedForces.Clear();
        }
        else if (rcsVisualAllocationGridId != controlledGrid.EntityId || updateCount % 6 == 0)
        {
            rcsVisualAllocationGridId = controlledGrid.EntityId;
            if (TryCalculateRcsForceAllocation(
                    controlledGrid,
                    inputPitch * PilotInputMaximumTorquePercent,
                    inputYaw * PilotInputMaximumTorquePercent,
                    inputRoll * PilotInputMaximumTorquePercent,
                    out RcsForceAllocation allocation))
            {
                rcsVisualAllocatedForces = allocation.ForcesByBlock;
            }
            else
            {
                rcsVisualAllocatedForces.Clear();
            }
        }

        foreach (MyCubeBlock rcsBlock in rcsBlocks)
        {
            RcsVisualState state = rcsVisualStates[rcsBlock.EntityId];

            if (!state.Initialized)
                continue;

            if (rcsTiltDiagnosticActive && rcsTiltDiagnosticGridId == controlledGrid.EntityId)
            {
                ApplyRcsAxisTiltDiagnostic(rcsBlock, state, controlledGrid.WorldMatrix);
            }
            else if (rcsVisualAllocatedForces.TryGetValue(rcsBlock.EntityId, out Vector3D forceWorld)
                && forceWorld.LengthSquared() > 1)
            {
                // A pod may only aim thrust inside its own mounted 60-degree cone.
                // Do not show a best-effort/impossible pose: leave it neutral instead.
                Vector3D thrustWorld = Vector3D.Normalize(forceWorld);
                Vector3D neutralThrustWorld = GetRcsNeutralForceWorld(rcsBlock, state);
                bool withinPhysicalCone = Vector3D.Dot(thrustWorld, neutralThrustWorld) >= RcsConeMinimumAlignment;
                if (withinPhysicalCone || (rcsSharedVectorProbeActive && IsRcsProbeBlock(rcsBlock)))
                {
                    // The nozzle/exhaust points opposite the selected thrust vector.
                    Vector3D desiredNozzleWorld = -thrustWorld;
                    if (state.DesiredNozzleWorld.LengthSquared() <= 0
                        || Vector3D.Dot(state.DesiredNozzleWorld, desiredNozzleWorld) < 0.9999)
                    {
                        state.DesiredNozzleWorld = desiredNozzleWorld;
                        SolveRcsGimbalPose(state, rcsBlock.WorldMatrix, state.DesiredNozzleWorld, out float targetAzimuth, out float targetElevation);
                        state.TargetAzimuthRadians = targetAzimuth;
                        state.TargetElevationRadians = targetElevation;
                    }
                    state.AzimuthRadians = MoveRcsVisualAxis(state.AzimuthRadians, state.TargetAzimuthRadians);
                    state.ElevationRadians = MoveRcsVisualAxis(state.ElevationRadians, state.TargetElevationRadians);
                }
                else
                {
                    // Hold the existing pose.  Resetting to centre every time an
                    // allocator sample crosses the boundary causes a visible spasm.
                    state.DesiredNozzleWorld = Vector3D.Zero;
                }
            }
            else
            {
                state.DesiredNozzleWorld = Vector3D.Zero;
                state.AzimuthRadians = RecenterRcsVisualAxis(state.AzimuthRadians);
                state.ElevationRadians = RecenterRcsVisualAxis(state.ElevationRadians);
            }

            Matrix azimuthTransform = Matrix.CreateRotationZ(state.AzimuthRadians) * state.AzimuthInitialTransform;
            // The authored Blender rig defines elevation around local Y.
            Matrix elevationTransform = Matrix.CreateRotationY(state.ElevationRadians) * state.ElevationInitialTransform;
            state.Azimuth.PositionComp.SetLocalMatrix(ref azimuthTransform, this, true);
            state.Elevation.PositionComp.SetLocalMatrix(ref elevationTransform, this, true);
            UpdateHydrogenRcsFuelDemand(rcsBlock);
        }
    }

    private void UpdateHydrogenRcsFuelDemand(MyCubeBlock rcsBlock)
    {
        if (!attitudeControlArmed || !IsHydrogenRcsBlock(rcsBlock) || !(rcsBlock is IMyThrust hydrogenRcs))
            return;

        if (!hydrogenOverrides.ContainsKey(rcsBlock.EntityId))
        {
            hydrogenOverrides.Add(rcsBlock.EntityId, new HydrogenOverrideState(hydrogenRcs, hydrogenRcs.ThrustOverridePercentage));
            hydrogenOverrideGridId = rcsBlock.CubeGrid.EntityId;
        }

        double demand = rcsVisualAllocatedForces.TryGetValue(rcsBlock.EntityId, out Vector3D force)
            ? Clamp(force.Length() / GetRcsMaximumForce(rcsBlock), 0, 1)
            : 0;
        // The definition's native force is only 1 N. This override therefore
        // drives its hydrogen converter at the allocated duty cycle without
        // introducing meaningful translation force.
        hydrogenRcs.ThrustOverridePercentage = (float)demand;
    }

    private void RestoreHydrogenRcsOverrides(string reason)
    {
        if (hydrogenOverrides.Count == 0)
            return;

        foreach (HydrogenOverrideState state in hydrogenOverrides.Values)
        {
            try
            {
                state.Thruster.ThrustOverridePercentage = state.OriginalOverridePercentage;
            }
            catch
            {
                // A block may already have closed while changing worlds.
            }
        }

        hydrogenOverrides.Clear();
        hydrogenOverrideGridId = 0;
        WriteLifecycleLog($"Restored original hydrogen RCS thrust overrides ({reason}).");
    }

    private GridActuatorCache GetGridActuatorCache(MyCubeGrid grid)
    {
        if (!actuatorCaches.TryGetValue(grid.EntityId, out GridActuatorCache cache)
            || updateCount - cache.LastRefreshFrame >= ActuatorCacheRefreshFrames)
        {
            cache ??= new GridActuatorCache(grid.EntityId);
            cache.RcsBlocks.Clear();
            cache.VanillaThrusters.Clear();
            foreach (var slimBlock in grid.GetBlocks())
            {
                if (slimBlock.FatBlock is MyCubeBlock cubeBlock && IsCustomRcsBlock(cubeBlock))
                    cache.RcsBlocks.Add(cubeBlock);
                if (slimBlock.FatBlock is MyThrust thruster)
                    cache.VanillaThrusters.Add(thruster);
            }

            cache.LastRefreshFrame = updateCount;
            actuatorCaches[grid.EntityId] = cache;
        }

        return cache;
    }

    private void LogRcsVisualAlignment()
    {
        foreach (var pair in rcsVisualStates)
        {
            RcsVisualState state = pair.Value;
            if (!state.Initialized || state.DesiredNozzleWorld.LengthSquared() <= 0)
                continue;

            Vector3D actualForceWorld = Vector3D.Normalize(Vector3D.TransformNormal(
                state.ForceAxisElevationLocal,
                state.Elevation.WorldMatrix));
            Vector3D actualNozzleWorld = -actualForceWorld;
            double alignment = Vector3D.Dot(actualNozzleWorld, state.DesiredNozzleWorld);
            WriteLifecycleLog(
                $"ACS RCS visual alignment — block {pair.Key}; desired exhaust: {state.DesiredNozzleWorld}; "
                + $"actual exhaust: {actualNozzleWorld}; alignment: {alignment:F4}; "
                + $"azimuth/elevation radians: {state.AzimuthRadians:F3}/{state.ElevationRadians:F3}."
            );
        }
    }

    private void LogRcsSharedVectorProbe(MyCubeGrid grid)
    {
        if (!rcsSharedVectorProbeActive || grid == null || rcsSharedVectorProbeGridId != grid.EntityId)
            return;

        foreach (var slimBlock in grid.GetBlocks())
        {
            if (!(slimBlock.FatBlock is MyCubeBlock rcsBlock)
                || !IsRcsProbeBlock(rcsBlock)
                || !rcsVisualStates.TryGetValue(rcsBlock.EntityId, out RcsVisualState state)
                || !state.Initialized
                || state.DesiredNozzleWorld.LengthSquared() <= 0)
            {
                continue;
            }

            MatrixD inverseBlockWorld = MatrixD.Invert(rcsBlock.WorldMatrix);
            Vector3D requestedThrustWorld = -state.DesiredNozzleWorld;
            Vector3D requestedThrustLocal = Vector3D.Normalize(Vector3D.TransformNormal(requestedThrustWorld, inverseBlockWorld));
            Vector3D actualThrustWorld = Vector3D.Normalize(Vector3D.TransformNormal(state.ForceAxisElevationLocal, state.Elevation.WorldMatrix));
            WriteLifecycleLog(
                $"ACS RCS shared-vector probe — name: {GetRcsCustomName(rcsBlock)}; id: {rcsBlock.EntityId}; "
                + $"block right/up/forward: {rcsBlock.WorldMatrix.Right}/{rcsBlock.WorldMatrix.Up}/{rcsBlock.WorldMatrix.Forward}; "
                + $"requested thrust world/local: {requestedThrustWorld}/{requestedThrustLocal}; "
                + $"actual thrust world: {actualThrustWorld}; azimuth/elevation: {state.AzimuthRadians:F3}/{state.ElevationRadians:F3}."
            );
        }
    }

    private static void LogLinearMotion(MyCubeGrid grid)
    {
        if (grid?.Physics?.RigidBody == null)
            return;

        Vector3D worldVelocity = grid.Physics.RigidBody.LinearVelocity;
        double speed = worldVelocity.Length();
        // Avoid filling the normal log with stationary samples.  The threshold
        // is well below a speed that is meaningful on the vanilla HUD.
        if (speed < 0.01)
            return;

        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
        Vector3D cockpitVelocity = new(
            Vector3D.Dot(worldVelocity, controlFrame.Right),
            Vector3D.Dot(worldVelocity, controlFrame.Up),
            Vector3D.Dot(worldVelocity, controlFrame.Forward));
        WriteLifecycleLog(
            $"ACS linear-motion trace — speed: {speed:F3} m/s; world velocity: {worldVelocity}; "
            + $"cockpit pitch/right, yaw/up, forward: {cockpitVelocity}; native dampeners: {grid.DampenersEnabled}."
        );
    }

    private static bool IsRcsProbeBlock(MyCubeBlock rcsBlock)
    {
        return GetRcsCustomName(rcsBlock).StartsWith("ACS-RCS-", StringComparison.OrdinalIgnoreCase);
    }

    private static string GetRcsCustomName(MyCubeBlock rcsBlock)
    {
        return (rcsBlock as IMyTerminalBlock)?.CustomName ?? string.Empty;
    }

    private void CenterRcsVisuals()
    {
        foreach (RcsVisualState state in rcsVisualStates.Values)
        {
            if (!state.Initialized)
                continue;

            state.AzimuthRadians = 0;
            state.ElevationRadians = 0;
            Matrix azimuthTransform = state.AzimuthInitialTransform;
            Matrix elevationTransform = state.ElevationInitialTransform;
            state.Azimuth.PositionComp.SetLocalMatrix(ref azimuthTransform, this, true);
            state.Elevation.PositionComp.SetLocalMatrix(ref elevationTransform, this, true);
        }

        const string message = "ACS RCS visual pose centered.";
        WriteLifecycleLog(message);
        ShowChatFeedback(message);
    }

    private void SetRcsTiltDiagnostic(string command)
    {
        string[] parts = command.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
        string direction = parts.Length >= 4 ? parts[3].ToLowerInvariant() : string.Empty;
        if (direction == "off")
        {
            rcsTiltDiagnosticActive = false;
            rcsTiltDiagnosticGridId = 0;
            rcsVisualAllocatedForces.Clear();
            const string disabled = "ACS RCS tilt diagnostic disabled.";
            WriteLifecycleLog(disabled);
            ShowChatFeedback(disabled);
            return;
        }

        rcsTiltDiagnosticGridDirection = direction switch
        {
            "right" => Vector3D.Right,
            "left" => Vector3D.Left,
            "up" => Vector3D.Up,
            "down" => Vector3D.Down,
            "forward" => Vector3D.Forward,
            "back" => Vector3D.Backward,
            _ => Vector3D.Zero
        };
        MyCubeGrid grid = MySession.Static?.ControlledGrid;
        if (rcsTiltDiagnosticGridDirection.LengthSquared() <= 0 || grid == null)
        {
            const string usage = "Use /acs rcs tilt right|left|up|down|forward|back while controlling a grid, or /acs rcs tilt off.";
            WriteLifecycleLog(usage);
            ShowChatFeedback(usage);
            return;
        }

        rcsTiltDiagnosticActive = true;
        rcsTiltDiagnosticGridId = grid.EntityId;
        string enabled = $"ACS RCS tilt diagnostic: tilt each pod 20 degrees from neutral toward grid-local {direction}.";
        WriteLifecycleLog(enabled);
        ShowChatFeedback(enabled);
    }

    private void SetRcsSharedVectorProbe(string command)
    {
        string[] parts = command.Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
        string direction = parts.Length >= 4 ? parts[3].ToLowerInvariant() : string.Empty;
        if (direction == "off")
        {
            rcsSharedVectorProbeActive = false;
            rcsSharedVectorProbeGridId = 0;
            rcsVisualAllocatedForces.Clear();
            const string disabled = "ACS RCS shared-vector probe disabled.";
            WriteLifecycleLog(disabled);
            ShowChatFeedback(disabled);
            return;
        }

        rcsSharedVectorProbeGridThrust = direction switch
        {
            "right" => Vector3D.Right,
            "left" => Vector3D.Left,
            "up" => Vector3D.Up,
            "down" => Vector3D.Down,
            "forward" => Vector3D.Forward,
            "back" => Vector3D.Backward,
            _ => Vector3D.Zero
        };
        MyCubeGrid grid = MySession.Static?.ControlledGrid;
        if (rcsSharedVectorProbeGridThrust.LengthSquared() <= 0 || grid == null)
        {
            const string usage = "Use /acs rcs probe right|left|up|down|forward|back while controlling a grid, or /acs rcs probe off.";
            WriteLifecycleLog(usage);
            ShowChatFeedback(usage);
            return;
        }

        rcsTiltDiagnosticActive = false;
        rcsSharedVectorProbeActive = true;
        rcsSharedVectorProbeGridId = grid.EntityId;
        string enabled = $"ACS RCS shared-vector probe: named ACS-RCS-* pods target grid-local thrust {direction}. The log will report their local transforms and actual world results.";
        WriteLifecycleLog(enabled);
        ShowChatFeedback(enabled);
    }

    private void LogRcsForceAllocation(MyCubeGrid grid)
    {
        if (!TryGetPilotInputPercent(out double pitch, out double yaw, out double roll, out _, out _)
            || (Math.Abs(pitch) < 0.001 && Math.Abs(yaw) < 0.001 && Math.Abs(roll) < 0.001)
            || !TryCalculateRcsForceAllocation(grid, pitch, yaw, roll, out RcsForceAllocation allocation))
        {
            return;
        }

        WriteLifecycleLog(
            $"ACS RCS allocation — requested torque: {allocation.RequestedTorque}; achieved: {allocation.AchievedTorque}; "
            + $"net force residual: {allocation.TranslationResidual}; active blocks: {allocation.ForcesByBlock.Count}."
        );
        foreach (var pair in allocation.ForcesByBlock)
        {
            if (pair.Value.LengthSquared() > 1)
            {
                if (TryGetCustomRcsBlock(grid, pair.Key, out MyCubeBlock rcsBlock)
                    && rcsVisualStates.TryGetValue(pair.Key, out RcsVisualState state)
                    && state.Initialized)
                {
                    Vector3D offsetWorld = rcsBlock.PositionComp.GetPosition() - grid.Physics.CenterOfMassWorld;
                    Vector3D torqueWorld = Vector3D.Cross(offsetWorld, pair.Value);
                    MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
                    Vector3D offsetGrid = ToGridVector(offsetWorld, grid.WorldMatrix);
                    Vector3D forceGrid = ToGridVector(pair.Value, grid.WorldMatrix);
                    Vector3D neutralGrid = ToGridVector(GetRcsNeutralForceWorld(rcsBlock, state), grid.WorldMatrix);
                    WriteLifecycleLog(
                        $"ACS RCS allocated force — name: {GetRcsCustomName(rcsBlock)}; id: {pair.Key}; COM offset grid-local: {offsetGrid}; "
                        + $"neutral thrust grid-local: {neutralGrid}; allocated force grid-local: {forceGrid}; "
                        + $"torque cockpit pitch/yaw/roll: {Vector3D.Dot(torqueWorld, controlFrame.Right):F0}/"
                        + $"{Vector3D.Dot(torqueWorld, controlFrame.Up):F0}/{Vector3D.Dot(torqueWorld, controlFrame.Forward):F0}."
                    );
                }
                else
                {
                    WriteLifecycleLog($"ACS RCS allocated force vector — block {pair.Key}; force world: {pair.Value}; magnitude: {pair.Value.Length():F1}.");
                }
            }
        }
    }

    private bool TryCalculateRcsForceAllocation(
        MyCubeGrid grid,
        double pitchPercent,
        double yawPercent,
        double rollPercent,
        out RcsForceAllocation allocation)
    {
        allocation = default;
        if (grid?.Physics == null)
            return false;

        var pods = new List<RcsConePod>();
        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;
        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
        var grossTorqueByAxis = new double[3];

        foreach (MyCubeBlock rcsBlock in GetGridActuatorCache(grid).RcsBlocks)
        {
            if (!rcsBlock.IsWorking
                || !rcsVisualStates.TryGetValue(rcsBlock.EntityId, out RcsVisualState state)
                || !state.Initialized)
            {
                continue;
            }

            Vector3D leverArm = rcsBlock.PositionComp.GetPosition() - centerOfMass;
            Vector3D neutralForceWorld = GetRcsNeutralForceWorld(rcsBlock, state);
            double maximumForce = GetRcsMaximumForce(rcsBlock);
            pods.Add(new RcsConePod(rcsBlock.EntityId, leverArm, neutralForceWorld, CreateRcsConeDirections(neutralForceWorld), maximumForce));
            double grossTorque = leverArm.Length() * maximumForce;
            grossTorqueByAxis[0] += grossTorque;
            grossTorqueByAxis[1] += grossTorque;
            grossTorqueByAxis[2] += grossTorque;
        }

        if (pods.Count == 0)
            return false;

        double torqueScale = Math.Max(1, Math.Max(grossTorqueByAxis[0], Math.Max(grossTorqueByAxis[1], grossTorqueByAxis[2])));
        double forceScale = 0;
        foreach (RcsConePod pod in pods)
            forceScale += pod.MaximumForce;
        Vector3D requestedTorque = controlFrame.Right * (grossTorqueByAxis[0] * pitchPercent / 100)
            + controlFrame.Up * (grossTorqueByAxis[1] * yawPercent / 100)
            + controlFrame.Forward * (grossTorqueByAxis[2] * rollPercent / 100);

        SolveRcsConeAllocation(pods, requestedTorque / torqueScale, forceScale, torqueScale);
        var forcesByBlock = new Dictionary<long, Vector3D>();
        Vector3D achievedTorque = Vector3D.Zero;
        Vector3D translationResidual = Vector3D.Zero;
        foreach (RcsConePod pod in pods)
        {
            Vector3D force = pod.SelectedDirection * pod.MaximumForce * pod.Throttle;
            achievedTorque += Vector3D.Cross(pod.LeverArm, force);
            translationResidual += force;
            forcesByBlock[pod.EntityId] = force;
        }

        allocation = new RcsForceAllocation(requestedTorque, achievedTorque, translationResidual, forceScale, forcesByBlock);
        return true;
    }

    private static Vector3D GetRcsNeutralForceWorld(MyCubeBlock rcsBlock, RcsVisualState state)
    {
        return Vector3D.Normalize(Vector3D.TransformNormal(
            GetRcsPoseForceLocal(state, 0, 0),
            rcsBlock.WorldMatrix));
    }

    private static List<Vector3D> CreateRcsConeDirections(Vector3D neutralForceWorld)
    {
        // One central direction plus two rings gives the optimizer a circular 60-degree
        // physical cone without ever asking a pod to show an impossible pose.
        var directions = new List<Vector3D> { neutralForceWorld };
        Vector3D reference = Math.Abs(Vector3D.Dot(neutralForceWorld, Vector3D.Up)) < 0.9 ? Vector3D.Up : Vector3D.Right;
        Vector3D tangentA = Vector3D.Normalize(Vector3D.Cross(reference, neutralForceWorld));
        Vector3D tangentB = Vector3D.Normalize(Vector3D.Cross(neutralForceWorld, tangentA));
        foreach (double polarAngle in new[] { MathHelper.Pi / 6.0, MathHelper.Pi / 3.0 })
        for (int ringIndex = 0; ringIndex < 8; ringIndex++)
        {
            double azimuth = ringIndex * MathHelper.TwoPi / 8.0;
            Vector3D radial = tangentA * Math.Cos(azimuth) + tangentB * Math.Sin(azimuth);
            directions.Add(Vector3D.Normalize(neutralForceWorld * Math.Cos(polarAngle) + radial * Math.Sin(polarAngle)));
        }

        return directions;
    }

    private static double GetTorqueAlignment(Vector3D requestedTorque, Vector3D achievedTorque)
    {
        double requestedLength = requestedTorque.Length();
        double achievedLength = achievedTorque.Length();
        if (requestedLength <= 1e-6 || achievedLength <= 1e-6)
            return 0;

        return Vector3D.Dot(requestedTorque, achievedTorque) / (requestedLength * achievedLength);
    }

    private static double GetTranslationResidualRatio(Vector3D translationResidual, double forceCapacity)
    {
        return forceCapacity <= 1e-6 ? double.PositiveInfinity : translationResidual.Length() / forceCapacity;
    }

    private static void SolveRcsConeAllocation(List<RcsConePod> pods, Vector3D targetTorque, double forceScale, double torqueScale)
    {
        // Translation residual remains part of the solution, but it is not a
        // physical force in this finite-torque prototype.  Giving it forty
        // times the torque weight caused a controller request to settle for a
        // weak, off-axis answer merely to make the residual look tidy.
        const double translationPriority = 2;
        const double torquePriority = 20;
        Vector3D forceResidual = Vector3D.Zero;
        Vector3D torqueResidual = -targetTorque;

        // Each pass chooses one direction and throttle per pod.  Unlike treating every
        // cone sample as a separate thruster, this enforces one resultant nozzle force.
        for (int pass = 0; pass < 32; pass++)
        {
            double largestChange = 0;
            foreach (RcsConePod pod in pods)
            {
                Vector3D oldForce = pod.SelectedDirection * pod.MaximumForce * pod.Throttle;
                Vector3D oldForceNormalized = oldForce / forceScale;
                Vector3D oldTorqueNormalized = Vector3D.Cross(pod.LeverArm, oldForce) / torqueScale;
                Vector3D baseForceResidual = forceResidual - oldForceNormalized;
                Vector3D baseTorqueResidual = torqueResidual - oldTorqueNormalized;
                double bestObjective = double.MaxValue;
                Vector3D bestDirection = pod.SelectedDirection;
                double bestThrottle = 0;

                foreach (Vector3D direction in pod.ConeDirections)
                {
                    Vector3D forceColumn = direction * pod.MaximumForce / forceScale;
                    Vector3D torqueColumn = Vector3D.Cross(pod.LeverArm, direction * pod.MaximumForce) / torqueScale;
                    double curvature = translationPriority * forceColumn.LengthSquared() + torquePriority * torqueColumn.LengthSquared();
                    double gradient = translationPriority * Vector3D.Dot(forceColumn, baseForceResidual) + torquePriority * Vector3D.Dot(torqueColumn, baseTorqueResidual);
                    double throttle = curvature <= 1e-12 ? 0 : Clamp(-gradient / curvature, 0, 1);
                    Vector3D proposedForceResidual = baseForceResidual + forceColumn * throttle;
                    Vector3D proposedTorqueResidual = baseTorqueResidual + torqueColumn * throttle;
                    double objective = translationPriority * proposedForceResidual.LengthSquared() + torquePriority * proposedTorqueResidual.LengthSquared();
                    if (objective < bestObjective)
                    {
                        bestObjective = objective;
                        bestDirection = direction;
                        bestThrottle = throttle;
                    }
                }

                pod.SelectedDirection = bestDirection;
                pod.Throttle = bestThrottle;
                Vector3D newForce = bestDirection * pod.MaximumForce * bestThrottle;
                forceResidual = baseForceResidual + newForce / forceScale;
                torqueResidual = baseTorqueResidual + Vector3D.Cross(pod.LeverArm, newForce) / torqueScale;
                largestChange = Math.Max(largestChange, (newForce - oldForce).Length() / pod.MaximumForce);
            }

            if (largestChange < 0.0001)
                break;
        }
    }

    private static float RecenterRcsVisualAxis(float current)
    {
        if (Math.Abs(current) <= RcsVisualRecenteringRadiansPerFrame)
            return 0;

        return current - Math.Sign(current) * RcsVisualRecenteringRadiansPerFrame;
    }

    private static float MoveRcsVisualAxis(float current, float target)
    {
        if (Math.Abs(target - current) <= RcsVisualRecenteringRadiansPerFrame)
            return target;

        return current + Math.Sign(target - current) * RcsVisualRecenteringRadiansPerFrame;
    }

    private void ApplyRcsAxisTiltDiagnostic(MyCubeBlock rcsBlock, RcsVisualState state, MatrixD gridWorldMatrix)
    {
        // Project the selected grid direction onto the tangent plane of this pod's
        // neutral thrust vector.  This requests a reachable 20-degree thrust-vector
        // change, then the local solver couples azimuth and elevation correctly.
        Vector3D gridDirectionWorld = Vector3D.Normalize(Vector3D.TransformNormal(rcsTiltDiagnosticGridDirection, gridWorldMatrix));
        Vector3D neutralThrustWorld = GetRcsNeutralForceWorld(rcsBlock, state);
        Vector3D tangentWorld = gridDirectionWorld - neutralThrustWorld * Vector3D.Dot(gridDirectionWorld, neutralThrustWorld);
        if (tangentWorld.LengthSquared() <= 1e-8)
        {
            state.DesiredNozzleWorld = Vector3D.Zero;
            state.AzimuthRadians = 0;
            state.ElevationRadians = 0;
            return;
        }

        Vector3D desiredThrustWorld = Vector3D.Normalize(
            neutralThrustWorld * Math.Cos(RcsAxisDiagnosticRadians)
            + Vector3D.Normalize(tangentWorld) * Math.Sin(RcsAxisDiagnosticRadians));
        state.DesiredNozzleWorld = -desiredThrustWorld;
        SolveRcsGimbalPose(state, rcsBlock.WorldMatrix, state.DesiredNozzleWorld, out float targetAzimuth, out float targetElevation);
        state.AzimuthRadians = MoveRcsVisualAxis(state.AzimuthRadians, targetAzimuth);
        state.ElevationRadians = MoveRcsVisualAxis(state.ElevationRadians, targetElevation);
    }

    private static void SolveRcsGimbalPose(
        RcsVisualState state,
        MatrixD parentWorldMatrix,
        Vector3D desiredNozzleWorld,
        out float azimuthRadians,
        out float elevationRadians)
    {
        // A fixed cone map is independent of the last visual pose, avoiding
        // history-dependent results for pods rolled around their thrust axis.
        // Convert the requested world-space nozzle direction into this specific
        // block's local frame before choosing its two local gimbal axes.
        MatrixD inverseBlockWorld = MatrixD.Invert(parentWorldMatrix);
        Vector3D desiredNozzleLocal = Vector3D.Normalize(Vector3D.TransformNormal(desiredNozzleWorld, inverseBlockWorld));
        Vector3D neutralForceLocal = GetRcsPoseForceLocal(state, 0, 0);
        float bestAzimuth = 0;
        float bestElevation = 0;
        double bestScore = Vector3D.Dot(-neutralForceLocal, desiredNozzleLocal);
        foreach (RcsPoseCandidate candidate in state.PoseCandidates)
        {
            double score = Vector3D.Dot(-candidate.ForceLocal, desiredNozzleLocal);
            if (score > bestScore)
            {
                bestScore = score;
                bestAzimuth = candidate.AzimuthRadians;
                bestElevation = candidate.ElevationRadians;
            }
        }

        azimuthRadians = bestAzimuth;
        elevationRadians = bestElevation;
    }

    private static Vector3D GetRcsPoseForceLocal(
        RcsVisualState state,
        float azimuth,
        float elevation)
    {
        // Match the engine's actual hierarchy rather than approximating it with
        // inferred world axes: parent cube block → azimuth subpart → elevation subpart.
        MatrixD azimuthLocal = Matrix.CreateRotationZ(azimuth) * state.AzimuthInitialTransform;
        MatrixD elevationLocal = Matrix.CreateRotationY(elevation) * state.ElevationInitialTransform * azimuthLocal;
        return Vector3D.Normalize(Vector3D.TransformNormal(state.ForceAxisElevationLocal, elevationLocal));
    }

    private void TryInitializeRcsVisuals(MyCubeBlock rcsBlock, RcsVisualState state)
    {
        try
        {
            string azimuthModel = IsHydrogenRcsBlock(rcsBlock) ? RcsHydrogenAzimuthModel : RcsAzimuthModel;
            string elevationModel = IsHydrogenRcsBlock(rcsBlock) ? RcsHydrogenElevationModel : RcsElevationModel;
            MyEntitySubpart azimuth = GetOrCreateSubpart(rcsBlock, RcsAzimuthDummy, azimuthModel);
            if (azimuth == null)
            {
                WriteLifecycleLog($"ACS RCS visual setup failed for {rcsBlock.EntityId}: missing {RcsAzimuthDummy}.");
                return;
            }

            MyEntitySubpart elevation = GetOrCreateSubpart(azimuth, RcsElevationDummy, elevationModel);
            if (elevation == null)
            {
                WriteLifecycleLog($"ACS RCS visual setup failed for {rcsBlock.EntityId}: missing {RcsElevationDummy}.");
                return;
            }

            if (elevation.Model?.Dummies == null
                || !elevation.Model.Dummies.TryGetValue(RcsForceAxisDummy, out MyModelDummy forceAxisDummy))
            {
                WriteLifecycleLog($"ACS RCS visual setup failed for {rcsBlock.EntityId}: missing {RcsForceAxisDummy} in {elevationModel}.");
                return;
            }

            state.Azimuth = azimuth;
            state.Elevation = elevation;
            state.AzimuthInitialTransform = azimuth.PositionComp.LocalMatrixRef;
            state.ElevationInitialTransform = elevation.PositionComp.LocalMatrixRef;
            // Blender explicitly defines this dummy's local +Z as force. In SE's
            // matrix convention Backward is +Z, so its inverse is nozzle exhaust.
            state.ForceAxisElevationLocal = Vector3D.Normalize(forceAxisDummy.Matrix.Backward);
            BuildRcsPoseCandidates(state);
            state.Initialized = true;
            WriteLifecycleLog($"ACS RCS visual hierarchy initialized for block {rcsBlock.EntityId}: main → azimuth → elevation.");
        }
        catch (Exception exception)
        {
            WriteLifecycleLog($"ACS RCS visual setup failed for {rcsBlock.EntityId}: {exception.GetType().Name}: {exception.Message}");
        }
    }

    private static void BuildRcsPoseCandidates(RcsVisualState state)
    {
        const float poseStep = MathHelper.Pi / 36f; // five degrees
        Vector3D neutralForceLocal = GetRcsPoseForceLocal(state, 0, 0);
        for (float azimuth = -RcsMaximumAzimuthRadians; azimuth <= RcsMaximumAzimuthRadians + 0.001f; azimuth += poseStep)
        for (float elevation = -RcsMaximumElevationRadians; elevation <= RcsMaximumElevationRadians + 0.001f; elevation += poseStep)
        {
            Vector3D forceLocal = GetRcsPoseForceLocal(state, azimuth, elevation);
            if (Vector3D.Dot(forceLocal, neutralForceLocal) >= RcsConeMinimumAlignment)
                state.PoseCandidates.Add(new RcsPoseCandidate(azimuth, elevation, forceLocal));
        }
    }

    private static MyEntitySubpart GetOrCreateSubpart(MyEntity parent, string dummyName, string modelFile)
    {
        string subpartName = dummyName.Substring("subpart_".Length);
        if (parent.TryGetSubpart(subpartName, out MyEntitySubpart existing))
            return existing;

        if (parent.Model?.Dummies == null || !parent.Model.Dummies.TryGetValue(dummyName, out MyModelDummy dummy))
            return null;

        var data = new MyEntitySubpart.Data
        {
            Name = subpartName,
            File = modelFile,
            InitialTransform = dummy.Matrix
        };

        MethodInfo instantiateMethod = typeof(MyEntity).GetMethod(
            "InstantiateSubpart",
            BindingFlags.Instance | BindingFlags.NonPublic
        );
        if (instantiateMethod == null)
            throw new MissingMethodException(typeof(MyEntity).FullName, "InstantiateSubpart");

        instantiateMethod.Invoke(parent, new object[] { dummy, data });
        return parent.TryGetSubpart(subpartName, out MyEntitySubpart created) ? created : null;
    }

    private void UpdateRotationalDampening(MyCubeGrid controlledGrid)
    {
        if (!attitudeControlArmed || controlledGrid?.Physics?.RigidBody == null || !ShouldArrestRotation(controlledGrid))
            return;

        if (!TryGetPilotInputPercent(out double inputPitch, out double inputYaw, out double inputRoll, out _, out _))
            return;

        // Pilot authority wins. Releasing all rotation controls is the explicit
        // request for the dampening system to arrest existing angular velocity.
        if (Math.Abs(inputPitch) > 0.001 || Math.Abs(inputYaw) > 0.001 || Math.Abs(inputRoll) > 0.001)
            return;

        Vector3 angularVelocity = controlledGrid.Physics.RigidBody.AngularVelocity;
        if (angularVelocity.Length() <= AngularSpeedDampeningDeadzone)
            return;

        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? controlledGrid.WorldMatrix;
        double pitchPercent = Clamp(-Dot(angularVelocity, controlFrame.Right) / AngularSpeedAtFullDampeningTorque, -1, 1) * PilotInputMaximumTorquePercent;
        double yawPercent = Clamp(-Dot(angularVelocity, controlFrame.Up) / AngularSpeedAtFullDampeningTorque, -1, 1) * PilotInputMaximumTorquePercent;
        double rollPercent = Clamp(-Dot(angularVelocity, controlFrame.Forward) / AngularSpeedAtFullDampeningTorque, -1, 1) * PilotInputMaximumTorquePercent;
        if (Math.Abs(pitchPercent) < 0.001 && Math.Abs(yawPercent) < 0.001 && Math.Abs(rollPercent) < 0.001)
            return;

        GetCachedControlAllocations(
            dampeningAllocationCache, controlledGrid, pitchPercent, yawPercent, rollPercent,
            out bool hasRcsAllocation, out RcsForceAllocation rcsAllocation,
            out bool hasVanillaAllocation, out AllocationResult vanillaAllocation);
        double rcsAlignment = hasRcsAllocation ? GetTorqueAlignment(rcsAllocation.RequestedTorque, rcsAllocation.AchievedTorque) : 0;
        double vanillaAlignment = hasVanillaAllocation ? GetTorqueAlignment(vanillaAllocation.RequestedTorque, vanillaAllocation.AchievedTorque) : 0;
        double rcsResidual = hasRcsAllocation ? GetTranslationResidualRatio(rcsAllocation.TranslationResidual, rcsAllocation.ForceCapacity) : double.PositiveInfinity;
        double vanillaResidual = hasVanillaAllocation ? GetTranslationResidualRatio(vanillaAllocation.TranslationResidual, vanillaAllocation.ForceCapacity) : double.PositiveInfinity;
        Vector3D combinedTorque = Vector3D.Zero;
        if (hasRcsAllocation && rcsAlignment >= RcsMinimumTorqueAlignment && rcsResidual <= MaximumTranslationResidualRatio)
            combinedTorque += rcsAllocation.AchievedTorque;
        if (hasVanillaAllocation && vanillaAlignment >= RcsMinimumTorqueAlignment && vanillaResidual <= MaximumTranslationResidualRatio)
            combinedTorque += vanillaAllocation.AchievedTorque;
        if (combinedTorque.LengthSquared() <= 1)
            return;

        Vector3 appliedTorque = ToVector3(combinedTorque * RotationalDampeningTorqueScale);
        controlledGrid.Physics.RigidBody.ApplyTorque(NominalSimulationStepSeconds, appliedTorque);
        if (updateCount % 60 == 0)
        {
            WriteLifecycleLog(
                $"ACS rotational dampening — mode: {rotationalDampeningMode.ToString().ToLowerInvariant()}; "
                + $"counter-request pitch/yaw/roll: {pitchPercent:F1}% / {yawPercent:F1}% / {rollPercent:F1}%; "
                + $"RCS/vanilla alignment/residual: {rcsAlignment:F2}/{vanillaAlignment:F2} / {rcsResidual:P1}/{vanillaResidual:P1}; "
                + $"combined applied torque at {RotationalDampeningTorqueScale:P0}: {appliedTorque}; no thruster overrides were written."
            );
        }
    }

    private bool ShouldArrestRotation(MyCubeGrid grid)
    {
        bool dampeningRequested = rotationalDampeningMode == RotationalDampeningMode.On
            || (rotationalDampeningMode == RotationalDampeningMode.Follow && grid.DampenersEnabled);
        // Shift is a momentary inversion, not a second persistent mode:
        // dampeners on → hold Shift to drift; dampeners off → hold Shift to
        // arrest rotation. This preserves the player's normal SE expectation.
        bool shiftHeld = MyAPIGateway.Input?.IsAnyShiftKeyPressed() == true;
        return shiftHeld ? !dampeningRequested : dampeningRequested;
    }

    private void RegisterAlwaysOnGridIfConfigured(MyCubeGrid grid)
    {
        if (grid == null || alwaysOnGrids.ContainsKey(grid.EntityId))
            return;

        if (!TryGetAlwaysOnCockpit(grid, out MyCockpit cockpit, out string reason))
        {
            if (!string.IsNullOrEmpty(reason))
                WriteLifecycleLog($"ACS AlwaysOn registration skipped for grid {grid.EntityId}: {reason}.");
            return;
        }

        if (alwaysOnGrids.Count >= MaximumAlwaysOnGrids)
        {
            WriteLifecycleLog($"ACS AlwaysOn registration refused for grid {grid.EntityId}: limit of {MaximumAlwaysOnGrids} registered grids reached.");
            return;
        }

        alwaysOnGrids.Add(grid.EntityId, new AlwaysOnGridRegistration(grid, cockpit.EntityId, updateCount + AlwaysOnValidationFrames));
        WriteLifecycleLog($"ACS AlwaysOn registered grid {grid.EntityId} through vanilla Main Cockpit {cockpit.EntityId}.");
    }

    private void UpdateAlwaysOnGrids(MyCubeGrid controlledGrid)
    {
        if (alwaysOnGrids.Count == 0)
            return;

        var expired = new List<long>();
        foreach (AlwaysOnGridRegistration registration in alwaysOnGrids.Values)
        {
            MyCubeGrid grid = registration.Grid;
            if (grid == null || grid.MarkedForClose || grid.Closed)
            {
                expired.Add(registration.GridId);
                continue;
            }

            if (registration.GridId == controlledGrid?.EntityId)
                continue; // The controlled-grid path already applied this torque.

            if (updateCount >= registration.NextValidationFrame)
            {
                if (!TryGetAlwaysOnCockpit(grid, out MyCockpit cockpit, out _)
                    || cockpit.EntityId != registration.CockpitEntityId)
                {
                    expired.Add(registration.GridId);
                    continue;
                }

                registration.NextValidationFrame = updateCount + AlwaysOnValidationFrames;
            }

            UpdateAlwaysOnInertia(registration);
            UpdateManualOverrideTorque(grid);
        }

        foreach (long gridId in expired)
        {
            if (alwaysOnGrids.TryGetValue(gridId, out AlwaysOnGridRegistration registration))
                RestoreAlwaysOnAngularDamping(registration, "registration expired");
            alwaysOnGrids.Remove(gridId);
            WriteLifecycleLog($"ACS AlwaysOn unregistered grid {gridId} after slow validation.");
        }
    }

    private void UpdateAlwaysOnInertia(AlwaysOnGridRegistration registration)
    {
        if (!inertiaModeEnabled || registration.Grid.Physics == null)
        {
            RestoreAlwaysOnAngularDamping(registration, "inertia disabled or physics unavailable");
            return;
        }

        if (!registration.AngularDampingOverridden)
        {
            registration.OriginalAngularDamping = registration.Grid.Physics.AngularDamping;
            registration.AngularDampingOverridden = true;
        }

        registration.Grid.Physics.AngularDamping = 0;
    }

    private void RestoreAlwaysOnAngularDamping(string reason)
    {
        foreach (AlwaysOnGridRegistration registration in alwaysOnGrids.Values)
            RestoreAlwaysOnAngularDamping(registration, reason);
    }

    private static void RestoreAlwaysOnAngularDamping(AlwaysOnGridRegistration registration, string reason)
    {
        if (!registration.AngularDampingOverridden)
            return;

        try
        {
            if (registration.Grid?.Physics != null)
                registration.Grid.Physics.AngularDamping = registration.OriginalAngularDamping;
        }
        catch
        {
            // The grid can close before slow registration cleanup observes it.
        }

        registration.AngularDampingOverridden = false;
        registration.OriginalAngularDamping = 0;
    }

    private static bool TryGetAlwaysOnCockpit(MyCubeGrid grid, out MyCockpit result, out string reason)
    {
        result = null;
        reason = null;
        var cockpits = new List<MyCockpit>();
        foreach (var slimBlock in grid.GetBlocks())
            if (slimBlock.FatBlock is MyCockpit cockpit)
                cockpits.Add(cockpit);

        if (cockpits.Count == 0)
        {
            reason = "no cockpit exists";
            return false;
        }

        if (cockpits.Count == 1)
            result = cockpits[0];
        else
        {
            foreach (MyCockpit cockpit in cockpits)
            {
                if (!cockpit.IsMainCockpit)
                    continue;
                if (result != null)
                {
                    reason = "multiple vanilla Main Cockpits exist";
                    return false;
                }
                result = cockpit;
            }

            if (result == null)
            {
                reason = "multiple cockpits exist but none is the vanilla Main Cockpit";
                return false;
            }
        }

        string customData = (result as IMyTerminalBlock)?.CustomData ?? string.Empty;
        if (customData.IndexOf("AlwaysOn=true", StringComparison.OrdinalIgnoreCase) < 0)
        {
            reason = "the qualifying cockpit does not declare AlwaysOn=true";
            result = null;
            return false;
        }

        return true;
    }

    private void UpdateManualOverrideTorque(MyCubeGrid grid)
    {
        if (grid?.Physics?.RigidBody == null)
            return;

        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;
        MatrixD gridWorldMatrix = grid.WorldMatrix;
        Vector3D totalTorque = Vector3D.Zero;
        foreach (MyThrust thruster in GetGridActuatorCache(grid).VanillaThrusters)
        {
            var api = (IMyThrust)thruster;
            if (IsCustomRcsBlock(thruster) || !api.Enabled || !thruster.IsWorking || !thruster.IsPowered
                || api.ThrustOverridePercentage <= 0.001f || api.CurrentThrust <= 0)
            {
                continue;
            }

            Vector3D maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
            if (maximumForceWorld.LengthSquared() <= 0)
                continue;

            Vector3D forceWorld = Vector3D.Normalize(maximumForceWorld) * api.CurrentThrust;
            totalTorque += Vector3D.Cross(thruster.PositionComp.GetPosition() - centerOfMass, forceWorld);
        }

        if (totalTorque.LengthSquared() <= 1)
            return;

        Vector3D appliedTorqueWorld = totalTorque;
        if (ShouldArrestRotation(grid)
            && TryCalculateOverrideCounterTorque(grid, totalTorque, out Vector3D counterTorque))
        {
            appliedTorqueWorld += counterTorque;
        }

        Vector3 appliedTorque = ToVector3(appliedTorqueWorld);
        Vector3 angularVelocity = grid.Physics.RigidBody.AngularVelocity;
        if (angularVelocity.Length() >= AngularSpeedSafetyLimit && Vector3.Dot(angularVelocity, appliedTorque) > 0)
            return;

        grid.Physics.RigidBody.ApplyTorque(NominalSimulationStepSeconds, appliedTorque);
    }

    private bool TryCalculateOverrideCounterTorque(MyCubeGrid grid, Vector3D disturbanceTorque, out Vector3D counterTorque)
    {
        counterTorque = Vector3D.Zero;
        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
        double largestComponent = Math.Max(
            Math.Abs(Vector3D.Dot(disturbanceTorque, controlFrame.Right)),
            Math.Max(Math.Abs(Vector3D.Dot(disturbanceTorque, controlFrame.Up)), Math.Abs(Vector3D.Dot(disturbanceTorque, controlFrame.Forward))));
        if (largestComponent <= 1e-6)
            return false;

        double pitchPercent = Clamp(-Vector3D.Dot(disturbanceTorque, controlFrame.Right) / largestComponent, -1, 1) * PilotInputMaximumTorquePercent;
        double yawPercent = Clamp(-Vector3D.Dot(disturbanceTorque, controlFrame.Up) / largestComponent, -1, 1) * PilotInputMaximumTorquePercent;
        double rollPercent = Clamp(-Vector3D.Dot(disturbanceTorque, controlFrame.Forward) / largestComponent, -1, 1) * PilotInputMaximumTorquePercent;
        GetCachedControlAllocations(
            overrideCounterAllocationCache, grid, pitchPercent, yawPercent, rollPercent,
            out bool hasRcsAllocation, out RcsForceAllocation rcsAllocation,
            out bool hasVanillaAllocation, out AllocationResult vanillaAllocation);

        Vector3D requestedCounter = -disturbanceTorque;
        Vector3D availableCounter = Vector3D.Zero;
        if (hasRcsAllocation
            && GetTorqueAlignment(requestedCounter, rcsAllocation.AchievedTorque) >= RcsMinimumTorqueAlignment
            && GetTranslationResidualRatio(rcsAllocation.TranslationResidual, rcsAllocation.ForceCapacity) <= MaximumTranslationResidualRatio)
        {
            availableCounter += rcsAllocation.AchievedTorque;
        }
        if (hasVanillaAllocation
            && GetTorqueAlignment(requestedCounter, vanillaAllocation.AchievedTorque) >= RcsMinimumTorqueAlignment
            && GetTranslationResidualRatio(vanillaAllocation.TranslationResidual, vanillaAllocation.ForceCapacity) <= MaximumTranslationResidualRatio)
        {
            availableCounter += vanillaAllocation.AchievedTorque;
        }

        if (availableCounter.LengthSquared() <= 1)
            return false;

        // The allocator establishes a feasible counter direction. Scale it so
        // it cannot over-correct the manual override disturbance in one step.
        double scale = Math.Min(1, disturbanceTorque.Length() / availableCounter.Length());
        counterTorque = availableCounter * scale;
        return true;
    }

    private void UpdateInertiaMode(MyCubeGrid controlledGrid)
    {
        if (!inertiaModeEnabled || controlledGrid?.Physics == null)
        {
            RestoreAngularDamping(controlledGrid == null ? "control ended" : "grid physics unavailable");
            return;
        }

        if (!angularDampingOverridden || inertiaGridId != controlledGrid.EntityId)
        {
            RestoreAngularDamping("controlled grid changed");
            try
            {
                inertiaGrid = controlledGrid;
                inertiaGridId = controlledGrid.EntityId;
                originalAngularDamping = controlledGrid.Physics.AngularDamping;
                controlledGrid.Physics.AngularDamping = 0;
                angularDampingOverridden = true;
                WriteLifecycleLog(
                    $"Persistent-inertia experiment applied to grid {inertiaGridId}; "
                    + $"angular damping changed from {originalAngularDamping:F4} to 0."
                );
            }
            catch (Exception exception)
            {
                WriteLifecycleLog($"Persistent-inertia experiment failed: {exception.GetType().FullName}.");
                RestoreAngularDamping("inertia setup failure");
            }

            return;
        }

        try
        {
            controlledGrid.Physics.AngularDamping = 0;
        }
        catch (Exception exception)
        {
            WriteLifecycleLog($"Persistent-inertia update failed: {exception.GetType().FullName}.");
            RestoreAngularDamping("inertia update failure");
        }
    }

    private void RestoreAngularDamping(string reason)
    {
        if (!angularDampingOverridden)
            return;

        try
        {
            if (inertiaGrid?.Physics != null)
                inertiaGrid.Physics.AngularDamping = originalAngularDamping;
        }
        catch
        {
            // Physics can disappear during grid removal or plugin unload.
        }

        WriteLifecycleLog(
            $"Persistent-inertia experiment restored angular damping to {originalAngularDamping:F4} ({reason})."
        );
        angularDampingOverridden = false;
        inertiaGridId = 0;
        inertiaGrid = null;
        originalAngularDamping = 0;
    }

    private void LogControlledGrid()
    {
        var session = MySession.Static;
        var controlledEntity = session?.ControlledEntity;
        var grid = session?.ControlledGrid;

        if (grid == null)
        {
            WriteLifecycleLog($"Controlled entity: {controlledEntity?.GetType().FullName ?? "none"}; controlled grid: none.");
            return;
        }

        int blockCount = 0;
        var blockTypes = new Dictionary<string, int>(StringComparer.Ordinal);

        var thrusters = new List<MyThrust>();

        foreach (var block in grid.GetBlocks())
        {
            blockCount++;
            if (block.FatBlock == null)
                continue;

            if (block.FatBlock is MyThrust thruster)
                thrusters.Add(thruster);

            string typeName = block.FatBlock.GetType().FullName;
            blockTypes.TryGetValue(typeName, out int count);
            blockTypes[typeName] = count + 1;
        }

        string centerOfMass = grid.Physics == null ? "unavailable" : grid.Physics.CenterOfMassWorld.ToString();
        string mass = grid.Physics == null ? "unavailable" : grid.Physics.Mass.ToString("F1");
        string typeSummary = string.Join(", ", blockTypes);

        WriteLifecycleLog(
            $"Controlled entity: {controlledEntity.GetType().FullName}; grid entity ID: {grid.EntityId}; "
            + $"blocks: {blockCount}; block types: {typeSummary}; mass: {mass}; COM world: {centerOfMass}."
        );

        var thrusterSamples = new List<ThrusterSample>(thrusters.Count);
        Vector3D totalCurrentTorque = Vector3D.Zero;
        Vector3D totalMaximumTorque = Vector3D.Zero;
        var torqueByDirection = new Dictionary<string, DirectionTorqueSummary>(StringComparer.Ordinal);

        foreach (var thruster in thrusters)
        {
            var torque = LogThruster(thruster, grid.WorldMatrix, grid.Physics?.CenterOfMassWorld);
            thrusterSamples.Add(torque);
            totalCurrentTorque += torque.Current;
            totalMaximumTorque += torque.Maximum;

            if (!torque.IsAvailable)
                continue;

            torqueByDirection.TryGetValue(torque.DirectionKey, out DirectionTorqueSummary summary);
            summary.ThrusterCount++;
            summary.CurrentThrust += torque.CurrentThrust;
            summary.CurrentTorque += torque.Current;
            summary.MaximumTorque += torque.Maximum;
            summary.CurrentTorqueGrid += torque.CurrentGrid;
            summary.MaximumTorqueGrid += torque.MaximumGrid;
            torqueByDirection[torque.DirectionKey] = summary;
        }

        WriteLifecycleLog(
            $"Thruster torque totals about COM — current world: {totalCurrentTorque}; "
            + $"all-thrusters maximum world: {totalMaximumTorque}."
        );

        foreach (var pair in torqueByDirection)
        {
            DirectionTorqueSummary summary = pair.Value;
            WriteLifecycleLog(
                $"Direction geometry summary — force grid-local: {pair.Key}; thrusters: {summary.ThrusterCount}; "
                + $"current thrust: {summary.CurrentThrust:F1}; current torque grid-local: {summary.CurrentTorqueGrid}; "
                + $"maximum torque grid-local: {summary.MaximumTorqueGrid}; current torque world: {summary.CurrentTorque}; "
                + $"maximum torque world: {summary.MaximumTorque}."
            );
        }

        LogTorqueMinimizingSimulation(thrusterSamples, totalCurrentTorque);
    }

    private static ThrusterSample LogThruster(MyThrust thruster, MatrixD gridWorldMatrix, Vector3D? centerOfMassWorld)
    {
        // IMyThrust's force values are implemented explicitly by MyThrust. Casting is
        // read-only here; this plugin deliberately never writes an override or multiplier.
        var api = (IMyThrust)thruster;
        var worldPosition = thruster.PositionComp.GetPosition();
        if (!centerOfMassWorld.HasValue)
        {
            WriteLifecycleLog($"Thruster entity ID: {thruster.EntityId}; grid physics center of mass unavailable.");
            return ThrusterSample.Unavailable;
        }

        var comOffset = worldPosition - centerOfMassWorld.Value;
        var comOffsetGrid = ToGridVector(comOffset, gridWorldMatrix);
        var maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
        Vector3D currentForceWorld = maximumForceWorld.LengthSquared() == 0
            ? Vector3D.Zero
            : Vector3D.Normalize(maximumForceWorld) * api.CurrentThrust;
        var maximumTorqueWorld = Vector3D.Cross(comOffset, maximumForceWorld);
        var currentTorqueWorld = Vector3D.Cross(comOffset, currentForceWorld);
        Vector3D currentForceGrid = thruster.ThrustForce.LengthSquared() == 0
            ? Vector3D.Zero
            : Vector3D.Normalize(thruster.ThrustForce) * api.CurrentThrust;
        var maximumTorqueGrid = Vector3D.Cross(comOffsetGrid, thruster.ThrustForce);
        var currentTorqueGrid = Vector3D.Cross(comOffsetGrid, currentForceGrid);

        WriteLifecycleLog(
            $"Thruster entity ID: {thruster.EntityId}; world position: {worldPosition}; COM offset world: {comOffset}; "
            + $"COM offset grid-local: {comOffsetGrid}; "
            + $"max force grid-local: {thruster.ThrustForce}; max force world: {maximumForceWorld}; "
            + $"current thrust: {api.CurrentThrust:F1}; "
            + $"max thrust: {api.MaxThrust:F1}; max effective thrust: {api.MaxEffectiveThrust:F1}; "
            + $"current percentage: {api.CurrentThrustPercentage:F1}%; powered: {thruster.IsPowered}; "
            + $"current torque grid-local: {currentTorqueGrid}; max torque grid-local: {maximumTorqueGrid}; "
            + $"current torque world: {currentTorqueWorld}; max torque world: {maximumTorqueWorld}."
        );

        return new ThrusterSample(
            thruster.EntityId,
            thruster.ThrustForce.ToString(),
            api.CurrentThrust,
            api.MaxEffectiveThrust,
            maximumForceWorld.LengthSquared() == 0 ? Vector3D.Zero : maximumTorqueWorld / maximumForceWorld.Length(),
            currentTorqueWorld,
            maximumTorqueWorld,
            currentTorqueGrid,
            maximumTorqueGrid
        );
    }

    // This is the live, non-logging path used every simulation update while the
    // experiment is enabled. It measures only vanilla's present thrust demand;
    // it never allocates thrust or writes a thruster property.
    private static Vector3D CalculateCurrentTorque(MyCubeGrid grid)
    {
        if (grid.Physics == null)
            return Vector3D.Zero;

        Vector3D totalTorque = Vector3D.Zero;
        MatrixD gridWorldMatrix = grid.WorldMatrix;
        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;

        foreach (var block in grid.GetBlocks())
        {
            if (!(block.FatBlock is MyThrust thruster))
                continue;

            var api = (IMyThrust)thruster;
            Vector3D maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
            if (maximumForceWorld.LengthSquared() <= 0 || api.CurrentThrust <= 0)
                continue;

            Vector3D currentForceWorld = Vector3D.Normalize(maximumForceWorld) * api.CurrentThrust;
            totalTorque += Vector3D.Cross(thruster.PositionComp.GetPosition() - centerOfMass, currentForceWorld);
        }

        return totalTorque;
    }

    // Read-only Phase 4 diagnostic. A valid pair fires two thrusters with equal
    // and opposite force, so its linear forces cancel while its r x F torques add.
    // It deliberately does not allocate thrust or write any block property.
    private void LogRcsAuthority(MyCubeGrid grid)
    {
        if (grid?.Physics == null)
        {
            const string unavailable = "RCS diagnostics unavailable: control a grid with active physics first.";
            WriteLifecycleLog(unavailable);
            ShowChatFeedback(unavailable);
            return;
        }

        // Prefer the custom-cluster diagnostic when those blocks are present.
        // It reports the allocator's actual envelope rather than the legacy
        // tagged-thruster pair scan below.
        if (TryLogCustomRcsAuthority(grid))
            return;

        var samples = new List<RcsThrusterSample>();
        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;
        MatrixD gridWorldMatrix = grid.WorldMatrix;
        foreach (var block in grid.GetBlocks())
        {
            if (!(block.FatBlock is MyThrust thruster) || !IsRcsThruster(thruster))
                continue;

            var api = (IMyThrust)thruster;
            Vector3D maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
            if (maximumForceWorld.LengthSquared() <= 0 || api.MaxEffectiveThrust <= 0)
                continue;

            Vector3D forceDirectionWorld = Vector3D.Normalize(maximumForceWorld);
            Vector3D offsetGrid = ToGridVector(thruster.PositionComp.GetPosition() - centerOfMass, gridWorldMatrix);
            Vector3D forceDirectionGrid = ToGridVector(forceDirectionWorld, gridWorldMatrix);
            Vector3D torquePerNewtonWorld = Vector3D.Cross(
                thruster.PositionComp.GetPosition() - centerOfMass,
                forceDirectionWorld);
            samples.Add(new RcsThrusterSample(
                thruster.EntityId,
                thruster.CustomName?.ToString() ?? string.Empty,
                api.MaxEffectiveThrust,
                forceDirectionWorld,
                torquePerNewtonWorld));
            WriteLifecycleLog(
                $"RCS thruster — ID {thruster.EntityId}; name: {thruster.CustomName}; COM offset grid-local: {offsetGrid}; "
                + $"force direction grid-local: {forceDirectionGrid}; maximum effective thrust: {api.MaxEffectiveThrust:F0}."
            );
        }

        if (samples.Count == 0)
        {
            string none = $"RCS diagnostics: no powered thrusters tagged {RcsTag} were found on the controlled grid.";
            WriteLifecycleLog(none);
            ShowChatFeedback(none);
            return;
        }

        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? gridWorldMatrix;
        var positive = new double[3];
        var negative = new double[3];
        int oppositeForcePairs = 0;

        for (int first = 0; first < samples.Count; first++)
        for (int second = first + 1; second < samples.Count; second++)
        {
            RcsThrusterSample a = samples[first];
            RcsThrusterSample b = samples[second];
            if (Vector3D.Dot(a.ForceDirectionWorld, b.ForceDirectionWorld) > -0.999)
                continue;

            oppositeForcePairs++;
            double pairedThrust = Math.Min(a.MaximumThrust, b.MaximumThrust);
            Vector3D pairTorqueWorld = (a.TorquePerNewtonWorld + b.TorquePerNewtonWorld) * pairedThrust;
            var controlTorque = new[]
            {
                Vector3D.Dot(pairTorqueWorld, controlFrame.Right),
                Vector3D.Dot(pairTorqueWorld, controlFrame.Up),
                Vector3D.Dot(pairTorqueWorld, controlFrame.Forward)
            };

            WriteLifecycleLog(
                $"RCS opposite-force pair — IDs {a.EntityId}/{b.EntityId}; names: {a.CustomName} / {b.CustomName}; "
                + $"equal-force torque, cockpit pitch/yaw/roll (N·m): "
                + $"{controlTorque[0]:F0}, {controlTorque[1]:F0}, {controlTorque[2]:F0}."
            );

            for (int axis = 0; axis < controlTorque.Length; axis++)
            {
                positive[axis] = Math.Max(positive[axis], controlTorque[axis]);
                negative[axis] = Math.Min(negative[axis], controlTorque[axis]);
            }
        }

        string message = $"RCS diagnostics — tag {RcsTag}; tagged thrusters: {samples.Count}; translation-neutral opposite-force pairs: {oppositeForcePairs}. "
            + $"Single-pair authority, cockpit axes (N·m): pitch/right +{positive[0]:F0} / {negative[0]:F0}; "
            + $"yaw/up +{positive[1]:F0} / {negative[1]:F0}; roll/forward +{positive[2]:F0} / {negative[2]:F0}. "
            + $"Bidirectional: pitch {HasBidirectionalAuthority(positive[0], negative[0])}; "
            + $"yaw {HasBidirectionalAuthority(positive[1], negative[1])}; "
            + $"roll {HasBidirectionalAuthority(positive[2], negative[2])}. "
            + "This is a read-only single-pair envelope; simultaneous-axis allocation is the next phase.";
        WriteLifecycleLog(message);
        ShowChatFeedback("RCS authority logged. Use the Realistic ACS log for the axis values.");
    }

    private bool TryLogCustomRcsAuthority(MyCubeGrid grid)
    {
        if (!TryCalculateRcsForceAllocation(grid, 25, 0, 0, out RcsForceAllocation pitchPositive)
            || !TryCalculateRcsForceAllocation(grid, -25, 0, 0, out RcsForceAllocation pitchNegative)
            || !TryCalculateRcsForceAllocation(grid, 0, 25, 0, out RcsForceAllocation yawPositive)
            || !TryCalculateRcsForceAllocation(grid, 0, -25, 0, out RcsForceAllocation yawNegative)
            || !TryCalculateRcsForceAllocation(grid, 0, 0, 25, out RcsForceAllocation rollPositive)
            || !TryCalculateRcsForceAllocation(grid, 0, 0, -25, out RcsForceAllocation rollNegative))
        {
            return false;
        }

        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
        double pitchPlus = Vector3D.Dot(pitchPositive.AchievedTorque, controlFrame.Right);
        double pitchMinus = Vector3D.Dot(pitchNegative.AchievedTorque, controlFrame.Right);
        double yawPlus = Vector3D.Dot(yawPositive.AchievedTorque, controlFrame.Up);
        double yawMinus = Vector3D.Dot(yawNegative.AchievedTorque, controlFrame.Up);
        double rollPlus = Vector3D.Dot(rollPositive.AchievedTorque, controlFrame.Forward);
        double rollMinus = Vector3D.Dot(rollNegative.AchievedTorque, controlFrame.Forward);
        string message = "ACS RCS cluster authority at full allocation (N·m), cockpit axes: "
            + $"pitch +{pitchPlus:F0} / {pitchMinus:F0}; "
            + $"yaw +{yawPlus:F0} / {yawMinus:F0}; "
            + $"roll +{rollPlus:F0} / {rollMinus:F0}. "
            + $"Live control applies {LiveAttitudeTorqueScale:P0} of these values; active clusters: {pitchPositive.ForcesByBlock.Count}.";
        WriteLifecycleLog(message);
        foreach (var slimBlock in grid.GetBlocks())
        {
            if (!(slimBlock.FatBlock is MyCubeBlock rcsBlock)
                || !IsCustomRcsBlock(rcsBlock)
                || !rcsVisualStates.TryGetValue(rcsBlock.EntityId, out RcsVisualState state)
                || !state.Initialized)
            {
                continue;
            }

            Vector3D offsetGrid = ToGridVector(rcsBlock.PositionComp.GetPosition() - grid.Physics.CenterOfMassWorld, grid.WorldMatrix);
            Vector3D neutralGrid = ToGridVector(GetRcsNeutralForceWorld(rcsBlock, state), grid.WorldMatrix);
            WriteLifecycleLog(
                $"ACS RCS cluster layout — name: {GetRcsCustomName(rcsBlock)}; id: {rcsBlock.EntityId}; "
                + $"COM offset grid-local: {offsetGrid}; neutral thrust grid-local: {neutralGrid}; working: {rcsBlock.IsWorking}."
            );
        }
        ShowChatFeedback("ACS RCS cluster authority logged. Use the Realistic ACS log for each axis.");
        return true;
    }

    private static bool TryGetCustomRcsBlock(MyCubeGrid grid, long entityId, out MyCubeBlock result)
    {
        result = null;
        if (grid == null)
            return false;

        foreach (var slimBlock in grid.GetBlocks())
        {
            if (slimBlock.FatBlock is MyCubeBlock rcsBlock && rcsBlock.EntityId == entityId)
            {
                result = rcsBlock;
                return true;
            }
        }

        return false;
    }

    private static bool IsRcsThruster(MyThrust thruster)
    {
        return thruster.CustomName?.ToString().IndexOf(RcsTag, StringComparison.OrdinalIgnoreCase) >= 0;
    }

    private static bool HasWorkingCustomRcs(MyCubeGrid grid)
    {
        if (grid == null)
            return false;

        foreach (var slimBlock in grid.GetBlocks())
        {
            if (slimBlock.FatBlock is MyCubeBlock rcsBlock
                && IsCustomRcsBlock(rcsBlock)
                && rcsBlock.IsWorking)
            {
                return true;
            }
        }

        return false;
    }

    private static bool IsCustomRcsBlock(MyCubeBlock rcsBlock)
    {
        string subtype = rcsBlock?.BlockDefinition.Id.SubtypeName;
        return string.Equals(subtype, RcsBlockSubtype, StringComparison.Ordinal)
            || string.Equals(subtype, RcsHydrogenBlockSubtype, StringComparison.Ordinal);
    }

    private static bool IsHydrogenRcsBlock(MyCubeBlock rcsBlock)
    {
        return string.Equals(rcsBlock?.BlockDefinition.Id.SubtypeName, RcsHydrogenBlockSubtype, StringComparison.Ordinal);
    }

    private static double GetRcsMaximumForce(MyCubeBlock rcsBlock)
    {
        // The current buildable cluster is an electric/ion unit: it has no
        // conveyor dependency and provides its full rated authority whenever
        // its functional block is working.  Keep this subtype dispatch here so
        // hydrogen RCS can have a separate rating and fuel condition later.
        if (IsHydrogenRcsBlock(rcsBlock))
            return RcsHydrogenMaximumForce;
        return IsCustomRcsBlock(rcsBlock) ? RcsIonMaximumForce : 0;
    }

    private static string HasBidirectionalAuthority(double positive, double negative)
    {
        return positive > 1 && negative < -1 ? "yes" : "no";
    }

    // Read-only whole-grid allocation prototype. It optimizes changes from the
    // current vanilla throttle values, strongly preserving net translation while
    // seeking the requested cockpit-relative torque. No override is written.
    private void LogWholeGridAllocationPlan(string command, MyCubeGrid grid)
    {
        const string prefix = "/acs plan";
        string[] values = command.Substring(prefix.Length).Trim().Split(new[] { ' ' }, StringSplitOptions.RemoveEmptyEntries);
        if (values.Length != 3
            || !double.TryParse(values[0], NumberStyles.Float, CultureInfo.InvariantCulture, out double pitchPercent)
            || !double.TryParse(values[1], NumberStyles.Float, CultureInfo.InvariantCulture, out double yawPercent)
            || !double.TryParse(values[2], NumberStyles.Float, CultureInfo.InvariantCulture, out double rollPercent)
            || Math.Abs(pitchPercent) > 100 || Math.Abs(yawPercent) > 100 || Math.Abs(rollPercent) > 100)
        {
            const string usage = "Invalid allocation plan. Use /acs plan <pitch%> <yaw%> <roll%>, with each value from -100 to 100.";
            WriteLifecycleLog(usage);
            ShowChatFeedback(usage);
            return;
        }

        LogWholeGridAllocationPlan(pitchPercent, yawPercent, rollPercent, grid, true, true);
    }

    private void LogWholeGridAllocationPlan(
        double pitchPercent,
        double yawPercent,
        double rollPercent,
        MyCubeGrid grid,
        bool includeThrusterDetail,
        bool showFeedback)
    {

        if (grid?.Physics == null)
        {
            const string unavailable = "Allocation plan unavailable: control a grid with active physics first.";
            WriteLifecycleLog(unavailable);
            if (showFeedback)
                ShowChatFeedback(unavailable);
            return;
        }

        var candidates = new List<WrenchCandidate>();
        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;
        MatrixD gridWorldMatrix = grid.WorldMatrix;
        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? gridWorldMatrix;
        double forceScale = 0;
        var grossTorqueByAxis = new double[3];

        foreach (var block in grid.GetBlocks())
        {
            if (!(block.FatBlock is MyThrust thruster))
                continue;

            var api = (IMyThrust)thruster;
            // A terminal-disabled thruster can retain a powered resource sink
            // and non-zero effective capacity. It must not supply virtual ACS
            // authority unless it is explicitly enabled and working.
            if (!api.Enabled || !thruster.IsWorking || !thruster.IsPowered || api.MaxEffectiveThrust <= 0)
                continue;

            Vector3D maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
            if (maximumForceWorld.LengthSquared() <= 0)
                continue;

            Vector3D forceDirection = Vector3D.Normalize(maximumForceWorld);
            Vector3D torquePerNewton = Vector3D.Cross(thruster.PositionComp.GetPosition() - centerOfMass, forceDirection);
            double maximumThrust = api.MaxEffectiveThrust;
            candidates.Add(new WrenchCandidate(
                thruster.EntityId,
                thruster.CustomName?.ToString() ?? string.Empty,
                api.CurrentThrust,
                maximumThrust,
                forceDirection,
                torquePerNewton));
            forceScale += maximumThrust;
            grossTorqueByAxis[0] += Math.Abs(Dot(torquePerNewton, controlFrame.Right)) * maximumThrust;
            grossTorqueByAxis[1] += Math.Abs(Dot(torquePerNewton, controlFrame.Up)) * maximumThrust;
            grossTorqueByAxis[2] += Math.Abs(Dot(torquePerNewton, controlFrame.Forward)) * maximumThrust;
        }

        if (candidates.Count == 0 || forceScale <= 0)
        {
            const string none = "Allocation plan unavailable: no powered thrusters with effective thrust were found.";
            WriteLifecycleLog(none);
            if (showFeedback)
                ShowChatFeedback(none);
            return;
        }

        double torqueScale = Math.Max(1, Math.Max(grossTorqueByAxis[0], Math.Max(grossTorqueByAxis[1], grossTorqueByAxis[2])));
        Vector3D requestedTorque = controlFrame.Right * (grossTorqueByAxis[0] * pitchPercent / 100)
            + controlFrame.Up * (grossTorqueByAxis[1] * yawPercent / 100)
            + controlFrame.Forward * (grossTorqueByAxis[2] * rollPercent / 100);

        foreach (var candidate in candidates)
        {
            candidate.ForceContribution = candidate.ForceDirection * candidate.MaximumThrust / forceScale;
            candidate.TorqueContribution = candidate.TorquePerNewton * candidate.MaximumThrust / torqueScale;
        }

        Vector3D targetTorqueNormalized = requestedTorque / torqueScale;
        SolveReadOnlyAllocation(candidates, targetTorqueNormalized);

        Vector3D translationResidual = Vector3D.Zero;
        Vector3D achievedTorque = Vector3D.Zero;
        int changedThrusters = 0;
        foreach (var candidate in candidates)
        {
            translationResidual += candidate.ForceDirection * candidate.MaximumThrust * candidate.ThrottleDelta;
            achievedTorque += candidate.TorquePerNewton * candidate.MaximumThrust * candidate.ThrottleDelta;
            if (Math.Abs(candidate.ThrottleDelta) * candidate.MaximumThrust > 1)
                changedThrusters++;
        }

        Vector3D achievedControlTorque = new(
            Dot(achievedTorque, controlFrame.Right),
            Dot(achievedTorque, controlFrame.Up),
            Dot(achievedTorque, controlFrame.Forward));
        Vector3D requestedControlTorque = new(
            Dot(requestedTorque, controlFrame.Right),
            Dot(requestedTorque, controlFrame.Up),
            Dot(requestedTorque, controlFrame.Forward));

        WriteLifecycleLog(
            $"Whole-grid read-only allocation plan — request pitch/yaw/roll: {pitchPercent:F1}% / {yawPercent:F1}% / {rollPercent:F1}%; "
            + $"gross torque envelope (N·m): {grossTorqueByAxis[0]:F0}, {grossTorqueByAxis[1]:F0}, {grossTorqueByAxis[2]:F0}; "
            + $"requested torque (N·m): {requestedControlTorque}; achieved: {achievedControlTorque}; "
            + $"added-force residual (N): {translationResidual}; adjusted thrusters: {changedThrusters}/{candidates.Count}. "
            + "No thruster overrides were written."
        );

        foreach (var candidate in candidates)
        {
            double deltaThrust = candidate.ThrottleDelta * candidate.MaximumThrust;
            if (!includeThrusterDetail || Math.Abs(deltaThrust) <= 1)
                continue;

            WriteLifecycleLog(
                $"Allocation plan thruster — ID {candidate.EntityId}; name: {candidate.CustomName}; "
                + $"vanilla: {candidate.CurrentThrust:F1}; planned delta: {deltaThrust:F1}; "
                + $"planned thrust: {candidate.CurrentThrust + deltaThrust:F1}; maximum: {candidate.MaximumThrust:F1}."
            );
        }

        if (showFeedback)
            ShowChatFeedback("Whole-grid allocation plan logged. No thrust was changed.");
    }

    private bool TryCalculateWholeGridAllocation(
        MyCubeGrid grid,
        double pitchPercent,
        double yawPercent,
        double rollPercent,
        out AllocationResult result)
    {
        result = default;
        if (grid?.Physics == null)
            return false;

        var candidates = new List<WrenchCandidate>();
        Vector3D centerOfMass = grid.Physics.CenterOfMassWorld;
        MatrixD gridWorldMatrix = grid.WorldMatrix;
        MatrixD controlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? gridWorldMatrix;
        double forceScale = 0;
        var grossTorqueByAxis = new double[3];

        foreach (MyThrust thruster in GetGridActuatorCache(grid).VanillaThrusters)
        {
            var api = (IMyThrust)thruster;
            bool enabled = api.Enabled;
            bool working = thruster.IsWorking;
            bool powered = thruster.IsPowered;
            bool effective = api.MaxEffectiveThrust > 0;
            if (!enabled || !working || !powered || !effective)
                continue;

            Vector3D maximumForceWorld = ToWorldVector(thruster.ThrustForce, gridWorldMatrix);
            if (maximumForceWorld.LengthSquared() <= 0)
                continue;

            Vector3D forceDirection = Vector3D.Normalize(maximumForceWorld);
            Vector3D torquePerNewton = Vector3D.Cross(thruster.PositionComp.GetPosition() - centerOfMass, forceDirection);
            double maximumThrust = api.MaxEffectiveThrust;
            candidates.Add(new WrenchCandidate(
                thruster.EntityId,
                thruster.CustomName?.ToString() ?? string.Empty,
                api.CurrentThrust,
                maximumThrust,
                forceDirection,
                torquePerNewton));
            forceScale += maximumThrust;
            grossTorqueByAxis[0] += Math.Abs(Dot(torquePerNewton, controlFrame.Right)) * maximumThrust;
            grossTorqueByAxis[1] += Math.Abs(Dot(torquePerNewton, controlFrame.Up)) * maximumThrust;
            grossTorqueByAxis[2] += Math.Abs(Dot(torquePerNewton, controlFrame.Forward)) * maximumThrust;
        }

        if (candidates.Count == 0 || forceScale <= 0)
            return false;

        double torqueScale = Math.Max(1, Math.Max(grossTorqueByAxis[0], Math.Max(grossTorqueByAxis[1], grossTorqueByAxis[2])));
        Vector3D requestedTorque = controlFrame.Right * (grossTorqueByAxis[0] * pitchPercent / 100)
            + controlFrame.Up * (grossTorqueByAxis[1] * yawPercent / 100)
            + controlFrame.Forward * (grossTorqueByAxis[2] * rollPercent / 100);
        foreach (var candidate in candidates)
        {
            candidate.ForceContribution = candidate.ForceDirection * candidate.MaximumThrust / forceScale;
            candidate.TorqueContribution = candidate.TorquePerNewton * candidate.MaximumThrust / torqueScale;
        }

        SolveReadOnlyAllocation(candidates, requestedTorque / torqueScale);
        Vector3D achievedTorque = Vector3D.Zero;
        Vector3D translationResidual = Vector3D.Zero;
        foreach (var candidate in candidates)
        {
            achievedTorque += candidate.TorquePerNewton * candidate.MaximumThrust * candidate.ThrottleDelta;
            translationResidual += candidate.ForceDirection * candidate.MaximumThrust * candidate.ThrottleDelta;
        }

        result = new AllocationResult(requestedTorque, achievedTorque, translationResidual, forceScale);
        return true;
    }

    private static void SolveReadOnlyAllocation(List<WrenchCandidate> candidates, Vector3D targetTorque)
    {
        const double translationPriority = 40;
        const double torquePriority = 1;
        const double smoothnessPriority = 0.0001;
        Vector3D forceResidual = Vector3D.Zero;
        Vector3D torqueResidual = -targetTorque;

        // Coordinate descent uses the exact curvature for each throttle variable.
        // Unlike a fixed-step gradient, it converges reliably when force and torque
        // columns have very different scales or the requested axis is weak.
        for (int iteration = 0; iteration < MaximumVanillaAllocationIterations; iteration++)
        {
            double largestChange = 0;
            foreach (var candidate in candidates)
            {
                double curvature =
                    translationPriority * candidate.ForceContribution.LengthSquared()
                    + torquePriority * candidate.TorqueContribution.LengthSquared()
                    + smoothnessPriority;
                if (curvature <= 1e-12)
                    continue;

                double gradient =
                    translationPriority * Vector3D.Dot(candidate.ForceContribution, forceResidual)
                    + torquePriority * Vector3D.Dot(candidate.TorqueContribution, torqueResidual)
                    + smoothnessPriority * candidate.ThrottleDelta;
                double original = candidate.ThrottleDelta;
                double updated = Clamp(
                    original - gradient / curvature,
                    candidate.MinimumThrottleDelta,
                    candidate.MaximumThrottleDelta);
                double change = updated - original;
                if (Math.Abs(change) <= 1e-12)
                    continue;

                candidate.ThrottleDelta = updated;
                forceResidual += candidate.ForceContribution * change;
                torqueResidual += candidate.TorqueContribution * change;
                largestChange = Math.Max(largestChange, Math.Abs(change));
            }

            if (largestChange <= 1e-8)
                break;
        }
    }

    private static double Clamp(double value, double minimum, double maximum)
    {
        return Math.Max(minimum, Math.Min(maximum, value));
    }

    private static AllocationPlan LogTorqueMinimizingSimulation(List<ThrusterSample> samples, Vector3D vanillaTorque)
    {
        var candidates = new List<AllocationCandidate>();

        foreach (var sample in samples)
        {
            if (!sample.IsAvailable)
                continue;

            candidates.Add(new AllocationCandidate(sample));
        }

        // A pairwise constrained descent retains the exact vanilla thrust total for
        // every grid-local force direction. It only moves demand between thrusters
        // that already propel in that same direction; it is a diagnostic simulation.
        Vector3D simulatedTorque = vanillaTorque;
        for (int pass = 0; pass < 12; pass++)
        {
            bool changed = false;
            for (int first = 0; first < candidates.Count; first++)
            for (int second = first + 1; second < candidates.Count; second++)
            {
                var a = candidates[first];
                var b = candidates[second];
                if (!string.Equals(a.DirectionKey, b.DirectionKey, StringComparison.Ordinal))
                    continue;

                Vector3D torqueChangePerNewton = a.TorquePerNewton - b.TorquePerNewton;
                double denominator = torqueChangePerNewton.LengthSquared();
                if (denominator <= 1e-9)
                    continue;

                double idealTransferToA = -Vector3D.Dot(simulatedTorque, torqueChangePerNewton) / denominator;
                double minimumTransferToA = Math.Max(-a.AllocatedThrust, -(b.MaximumThrust - b.AllocatedThrust));
                double maximumTransferToA = Math.Min(b.AllocatedThrust, a.MaximumThrust - a.AllocatedThrust);
                double transferToA = Math.Max(minimumTransferToA, Math.Min(maximumTransferToA, idealTransferToA));
                if (Math.Abs(transferToA) <= 0.01)
                    continue;

                a.AllocatedThrust += transferToA;
                b.AllocatedThrust -= transferToA;
                simulatedTorque += torqueChangePerNewton * transferToA;
                changed = true;
            }

            if (!changed)
                break;
        }

        double vanillaMagnitude = vanillaTorque.Length();
        double simulatedMagnitude = simulatedTorque.Length();
        double reductionPercent = vanillaMagnitude <= 0.01 ? 0 : (1 - simulatedMagnitude / vanillaMagnitude) * 100;
        WriteLifecycleLog(
            $"Read-only allocation simulation — vanilla torque magnitude: {vanillaMagnitude:F1}; "
            + $"simulated torque: {simulatedTorque}; simulated magnitude: {simulatedMagnitude:F1}; "
            + $"reduction: {reductionPercent:F1}%."
        );

        foreach (var candidate in candidates)
        {
            if (Math.Abs(candidate.AllocatedThrust - candidate.OriginalThrust) > 0.1)
            {
                WriteLifecycleLog(
                    $"Simulated allocation — thruster entity ID: {candidate.EntityId}; direction: {candidate.DirectionKey}; "
                    + $"vanilla: {candidate.OriginalThrust:F1}; simulated: {candidate.AllocatedThrust:F1}; "
                    + $"maximum effective: {candidate.MaximumThrust:F1}."
                );
            }
        }

        return new AllocationPlan(candidates, reductionPercent);
    }

    private void TryStartExperimentalPulse(MyCubeGrid grid)
    {
        if (!experimentalPulseArmed || IsExperimentalPulseActive)
            return;

        try
        {
            activeOverrideGridId = grid.EntityId;
            activeOverrideRigidBody = grid.Physics?.RigidBody;
            if (activeOverrideRigidBody == null)
            {
                experimentalPulseArmed = false;
                WriteLifecycleLog("Experimental pulse refused: controlled grid has no rigid body.");
                return;
            }

            pulseStartAngularVelocity = activeOverrideRigidBody?.AngularVelocity ?? Vector3.Zero;
            activeControlFrame = (MySession.Static?.ControlledEntity as MyCockpit)?.WorldMatrix ?? grid.WorldMatrix;
            physicsTorquePulseActive = true;
            accumulatedTorqueImpulse = Vector3.Zero;
            peakAppliedTorqueMagnitude = 0;
            torqueSafetyLimiting = false;
            WriteLifecycleLog(
                $"Continuous torque experiment started on grid {grid.EntityId}; torque scale: {experimentalTorqueScale:P1}; "
                + $"angular velocity at start: {pulseStartAngularVelocity}; "
                + $"cockpit-local pitch/yaw/roll at start: {ToCockpitAngularVelocity(pulseStartAngularVelocity)}. "
                + "Each simulation update derives r x F from current vanilla thrust. No thruster overrides are written."
            );
        }
        catch (Exception exception)
        {
            WriteLifecycleLog($"Continuous torque experiment failed: {exception.GetType().FullName}. Restoring state.");
            RestoreExperimentalOverrides("pulse failure");
        }
    }

    private void ApplyExperimentalTorqueStep(MyCubeGrid grid)
    {
        try
        {
            Vector3D currentVanillaTorque = CalculateCurrentTorque(grid);
            activePhysicsTorque = ToVector3(currentVanillaTorque * experimentalTorqueScale);
            Vector3 angularVelocity = activeOverrideRigidBody?.AngularVelocity ?? Vector3.Zero;
            float angularSpeed = angularVelocity.Length();
            if (angularSpeed >= AngularSpeedSafetyLimit && Vector3.Dot(angularVelocity, activePhysicsTorque) > 0)
            {
                if (!torqueSafetyLimiting)
                {
                    torqueSafetyLimiting = true;
                    WriteLifecycleLog(
                        $"Emergency angular-speed cutoff engaged at {angularSpeed:F3} rad/s. "
                        + "Torque that would increase rotation is temporarily suppressed."
                    );
                }

                return;
            }

            if (torqueSafetyLimiting && angularSpeed <= AngularSpeedSafetyResume)
            {
                torqueSafetyLimiting = false;
                WriteLifecycleLog($"Emergency angular-speed cutoff released at {angularSpeed:F3} rad/s.");
            }

            accumulatedTorqueImpulse += activePhysicsTorque * NominalSimulationStepSeconds;
            peakAppliedTorqueMagnitude = Math.Max(peakAppliedTorqueMagnitude, activePhysicsTorque.Length());
            activeOverrideRigidBody?.ApplyTorque(NominalSimulationStepSeconds, activePhysicsTorque);
        }
        catch (Exception exception)
        {
            WriteLifecycleLog($"Continuous torque application failed: {exception.GetType().FullName}. Restoring state.");
            RestoreExperimentalOverrides("torque application failure");
        }
    }

    private void RestoreExperimentalOverrides(string reason)
    {
        if (!IsExperimentalPulseActive)
            return;

        LogPulseAngularVelocity($"before restoration ({reason})");
        WriteLifecycleLog(
            $"Continuous torque experiment summary — peak applied torque: {peakAppliedTorqueMagnitude:F1}; "
            + $"integrated applied torque impulse: {accumulatedTorqueImpulse}."
        );

        activeOverrideGridId = 0;
        activeOverrideRigidBody = null;
        pulseStartAngularVelocity = Vector3.Zero;
        activePhysicsTorque = Vector3.Zero;
        accumulatedTorqueImpulse = Vector3.Zero;
        peakAppliedTorqueMagnitude = 0;
        torqueSafetyLimiting = false;
        physicsTorquePulseActive = false;
        activeControlFrame = MatrixD.Identity;
        WriteLifecycleLog($"Continuous torque experiment stopped ({reason}). No thruster overrides were changed.");
    }

    private bool IsExperimentalPulseActive => physicsTorquePulseActive;

    private void LogPulseAngularVelocity(string phase)
    {
        if (activeOverrideRigidBody == null)
        {
            WriteLifecycleLog($"Angular-velocity sample unavailable ({phase}): rigid body missing.");
            return;
        }

        try
        {
            Vector3 angularVelocity = activeOverrideRigidBody.AngularVelocity;
            Vector3 delta = angularVelocity - pulseStartAngularVelocity;
            Vector3 cockpitAngularVelocity = ToCockpitAngularVelocity(angularVelocity);
            WriteLifecycleLog(
                $"Angular-velocity sample ({phase}) — world: {angularVelocity}; "
                + $"cockpit-local pitch/right: {cockpitAngularVelocity.X:F6}; yaw/up: {cockpitAngularVelocity.Y:F6}; "
                + $"roll/forward: {cockpitAngularVelocity.Z:F6}; delta since pulse start: {delta}; "
                + $"magnitude: {angularVelocity.Length():F6}; current applied torque: {activePhysicsTorque}; "
                + $"peak applied torque: {peakAppliedTorqueMagnitude:F1}."
            );
        }
        catch
        {
            WriteLifecycleLog($"Angular-velocity sample unavailable ({phase}): rigid body access failed.");
        }
    }

    private static void ShowChatFeedback(string message)
    {
        try
        {
            MyAPIGateway.Utilities.ShowMessage(Name, message);
        }
        catch
        {
            // The log remains the authoritative diagnostic channel.
        }
    }

    private static Vector3D ToWorldVector(Vector3 gridLocalVector, MatrixD gridWorldMatrix)
    {
        return gridWorldMatrix.Right * gridLocalVector.X
            + gridWorldMatrix.Up * gridLocalVector.Y
            + gridWorldMatrix.Backward * gridLocalVector.Z;
    }

    private static Vector3D ToGridVector(Vector3D worldVector, MatrixD gridWorldMatrix)
    {
        return new Vector3D(
            Vector3D.Dot(worldVector, gridWorldMatrix.Right),
            Vector3D.Dot(worldVector, gridWorldMatrix.Up),
            Vector3D.Dot(worldVector, gridWorldMatrix.Backward)
        );
    }

    private static Vector3 ToVector3(Vector3D vector)
    {
        return new Vector3((float)vector.X, (float)vector.Y, (float)vector.Z);
    }

    private Vector3 ToCockpitAngularVelocity(Vector3 worldAngularVelocity)
    {
        return new Vector3(
            (float)Dot(worldAngularVelocity, activeControlFrame.Right),
            (float)Dot(worldAngularVelocity, activeControlFrame.Up),
            (float)Dot(worldAngularVelocity, activeControlFrame.Forward)
        );
    }

    private static double Dot(Vector3 vector, Vector3D axis)
    {
        return vector.X * axis.X + vector.Y * axis.Y + vector.Z * axis.Z;
    }

    private static double Dot(Vector3D vector, Vector3D axis)
    {
        return Vector3D.Dot(vector, axis);
    }

    private struct DirectionTorqueSummary
    {
        public int ThrusterCount;
        public double CurrentThrust;
        public Vector3D CurrentTorque;
        public Vector3D MaximumTorque;
        public Vector3D CurrentTorqueGrid;
        public Vector3D MaximumTorqueGrid;
    }

    private enum RotationalDampeningMode
    {
        Follow,
        On,
        Off
    }

    private sealed class RcsVisualState
    {
        public bool InitializationAttempted;
        public bool Initialized;
        public MyEntitySubpart Azimuth;
        public MyEntitySubpart Elevation;
        public Matrix AzimuthInitialTransform;
        public Matrix ElevationInitialTransform;
        public Vector3D ForceAxisElevationLocal;
        public Vector3D DesiredNozzleWorld;
        public float AzimuthRadians;
        public float ElevationRadians;
        public float TargetAzimuthRadians;
        public float TargetElevationRadians;
        public List<RcsPoseCandidate> PoseCandidates { get; } = new();
    }

    private readonly struct RcsPoseCandidate(float azimuthRadians, float elevationRadians, Vector3D forceLocal)
    {
        public float AzimuthRadians { get; } = azimuthRadians;
        public float ElevationRadians { get; } = elevationRadians;
        public Vector3D ForceLocal { get; } = forceLocal;
    }

    private sealed class HydrogenOverrideState(IMyThrust thruster, float originalOverridePercentage)
    {
        public IMyThrust Thruster { get; } = thruster;
        public float OriginalOverridePercentage { get; } = originalOverridePercentage;
    }

    private sealed class GridActuatorCache(long gridId)
    {
        public long GridId { get; } = gridId;
        public long LastRefreshFrame { get; set; } = long.MinValue;
        public List<MyCubeBlock> RcsBlocks { get; } = new();
        public List<MyThrust> VanillaThrusters { get; } = new();
    }

    private sealed class AlwaysOnGridRegistration(MyCubeGrid grid, long cockpitEntityId, long nextValidationFrame)
    {
        public long GridId { get; } = grid.EntityId;
        public MyCubeGrid Grid { get; } = grid;
        public long CockpitEntityId { get; } = cockpitEntityId;
        public long NextValidationFrame { get; set; } = nextValidationFrame;
        public bool AngularDampingOverridden { get; set; }
        public float OriginalAngularDamping { get; set; }
    }

    private sealed class ControlAllocationCache
    {
        public long GridId;
        public long LastSolveFrame = long.MinValue;
        public double PitchPercent;
        public double YawPercent;
        public double RollPercent;
        public bool HasRcsAllocation;
        public bool HasVanillaAllocation;
        public RcsForceAllocation RcsAllocation;
        public AllocationResult VanillaAllocation;
    }

    private readonly struct ThrusterSample(
        long entityId,
        string directionKey,
        double currentThrust,
        double maximumThrust,
        Vector3D torquePerNewton,
        Vector3D current,
        Vector3D maximum,
        Vector3D currentGrid,
        Vector3D maximumGrid)
    {
        public static ThrusterSample Unavailable { get; } = new(0, string.Empty, 0, 0, Vector3D.Zero, Vector3D.Zero, Vector3D.Zero, Vector3D.Zero, Vector3D.Zero);
        public bool IsAvailable => EntityId != 0;
        public long EntityId { get; } = entityId;
        public string DirectionKey { get; } = directionKey;
        public double CurrentThrust { get; } = currentThrust;
        public double MaximumThrust { get; } = maximumThrust;
        public Vector3D TorquePerNewton { get; } = torquePerNewton;
        public Vector3D Current { get; } = current;
        public Vector3D Maximum { get; } = maximum;
        public Vector3D CurrentGrid { get; } = currentGrid;
        public Vector3D MaximumGrid { get; } = maximumGrid;
    }

    private readonly struct RcsThrusterSample(
        long entityId,
        string customName,
        double maximumThrust,
        Vector3D forceDirectionWorld,
        Vector3D torquePerNewtonWorld)
    {
        public long EntityId { get; } = entityId;
        public string CustomName { get; } = customName;
        public double MaximumThrust { get; } = maximumThrust;
        public Vector3D ForceDirectionWorld { get; } = forceDirectionWorld;
        public Vector3D TorquePerNewtonWorld { get; } = torquePerNewtonWorld;
    }

    private sealed class WrenchCandidate(
        long entityId,
        string customName,
        double currentThrust,
        double maximumThrust,
        Vector3D forceDirection,
        Vector3D torquePerNewton)
    {
        public long EntityId { get; } = entityId;
        public string CustomName { get; } = customName;
        public double CurrentThrust { get; } = currentThrust;
        public double MaximumThrust { get; } = maximumThrust;
        public Vector3D ForceDirection { get; } = forceDirection;
        public Vector3D TorquePerNewton { get; } = torquePerNewton;
        public Vector3D ForceContribution { get; set; }
        public Vector3D TorqueContribution { get; set; }
        public double ThrottleDelta { get; set; }
        public double MinimumThrottleDelta => -CurrentThrust / MaximumThrust;
        public double MaximumThrottleDelta => 1 - CurrentThrust / MaximumThrust;
    }

    private readonly struct AllocationResult(
        Vector3D requestedTorque,
        Vector3D achievedTorque,
        Vector3D translationResidual,
        double forceCapacity)
    {
        public Vector3D RequestedTorque { get; } = requestedTorque;
        public Vector3D AchievedTorque { get; } = achievedTorque;
        public Vector3D TranslationResidual { get; } = translationResidual;
        public double ForceCapacity { get; } = forceCapacity;
    }

    private readonly struct RcsForceAllocation(
        Vector3D requestedTorque,
        Vector3D achievedTorque,
        Vector3D translationResidual,
        double forceCapacity,
        Dictionary<long, Vector3D> forcesByBlock)
    {
        public Vector3D RequestedTorque { get; } = requestedTorque;
        public Vector3D AchievedTorque { get; } = achievedTorque;
        public Vector3D TranslationResidual { get; } = translationResidual;
        public double ForceCapacity { get; } = forceCapacity;
        public Dictionary<long, Vector3D> ForcesByBlock { get; } = forcesByBlock;
    }

    private sealed class RcsConePod(
        long entityId,
        Vector3D leverArm,
        Vector3D neutralForceWorld,
        List<Vector3D> coneDirections,
        double maximumForce)
    {
        public long EntityId { get; } = entityId;
        public Vector3D LeverArm { get; } = leverArm;
        public Vector3D NeutralForceWorld { get; } = neutralForceWorld;
        public List<Vector3D> ConeDirections { get; } = coneDirections;
        public double MaximumForce { get; } = maximumForce;
        public Vector3D SelectedDirection { get; set; } = neutralForceWorld;
        public double Throttle { get; set; }
    }

    private sealed class AllocationCandidate(ThrusterSample sample)
    {
        public long EntityId { get; } = sample.EntityId;
        public string DirectionKey { get; } = sample.DirectionKey;
        public double OriginalThrust { get; } = sample.CurrentThrust;
        public double AllocatedThrust { get; set; } = sample.CurrentThrust;
        public double MaximumThrust { get; } = sample.MaximumThrust;
        public Vector3D TorquePerNewton { get; } = sample.TorquePerNewton;
    }

    private sealed class AllocationPlan(List<AllocationCandidate> candidates, double reductionPercent)
    {
        public List<AllocationCandidate> Candidates { get; } = candidates;
        public double ReductionPercent { get; } = reductionPercent;
    }

    private static void InitializeLifecycleLogPath()
    {
        try
        {
            string directory = Path.Combine(MyFileSystem.UserDataPath, "RealisticAcs");
            Directory.CreateDirectory(directory);
            lifecycleLogPath = Path.Combine(directory, "realistic-acs.log");
        }
        catch
        {
            lifecycleLogPath = null;
        }
    }

    private static void WriteLifecycleLog(string message)
    {
        PendingLifecycleLogLines.Enqueue($"{DateTime.UtcNow:O} [{Name}] {message}{Environment.NewLine}");
    }

    private static void QueueLifecycleLogFlush()
    {
        if (string.IsNullOrEmpty(lifecycleLogPath) || PendingLifecycleLogLines.IsEmpty
            || System.Threading.Interlocked.Exchange(ref lifecycleLogFlushScheduled, 1) != 0)
            return;

        System.Threading.ThreadPool.QueueUserWorkItem(_ =>
        {
            try { FlushLifecycleLog(); }
            finally { System.Threading.Interlocked.Exchange(ref lifecycleLogFlushScheduled, 0); }
        });
    }

    private static void FlushLifecycleLogSynchronously()
    {
        if (string.IsNullOrEmpty(lifecycleLogPath))
            return;

        lock (LifecycleLogFileLock)
            FlushLifecycleLogCore();
    }

    private static void FlushLifecycleLog()
    {
        lock (LifecycleLogFileLock)
            FlushLifecycleLogCore();
    }

    private static void FlushLifecycleLogCore()
    {
        try
        {
            var batch = new System.Text.StringBuilder();
            while (PendingLifecycleLogLines.TryDequeue(out string line))
                batch.Append(line);
            if (batch.Length == 0)
                return;

            if (File.Exists(lifecycleLogPath) && new FileInfo(lifecycleLogPath).Length >= LifecycleLogMaximumBytes)
            {
                string previousPath = lifecycleLogPath + ".previous";
                if (File.Exists(previousPath))
                    File.Delete(previousPath);
                File.Move(lifecycleLogPath, previousPath);
            }

            File.AppendAllText(lifecycleLogPath, batch.ToString());
        }
        catch
        {
            // Diagnostics must never prevent the game from loading the plugin.
        }
    }
}
