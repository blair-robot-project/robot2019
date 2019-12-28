package org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.TwoSideMPSubsystem.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.usfirst.frc.team449.robot.components.PathRequester;
import org.usfirst.frc.team449.robot.other.Waypoint;
import org.usfirst.frc.team449.robot.subsystem.interfaces.AHRS.SubsystemAHRS;
import org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.TwoSideMPSubsystem.SubsystemMPTwoSides;
import org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.commands.GetPathFromJetson;

/**
 * A command that drives the given subsystem to the vision target detected by the Limelight.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class GoToVisionTargetLimelight<T extends Subsystem & SubsystemMPTwoSides & SubsystemAHRS> extends CommandGroup {

    /**
     * Network table for pulling info from the Jetson
     */
    private final NetworkTable table;

    /**
     * Offsets added to x and y, respectively, after receiving the raw values from the Limelight.
     */
    private final double xOffset, yOffset;

    /**
     * Whether to run this command repeatedly.
     */
    private final boolean adaptive;

    /**
     * Whether a vision target was detected by the Limelight. Null until the pose is pulled.
     */
    @Nullable
    private Boolean targetDetected;

    /**
     * The pose array given by the Limelight of the form {x, y, z, pitch, yaw, roll}.
     */
    private double[] camtran;

    /**
     * Default constructor
     *
     * @param subsystem     The subsystem to run the path gotten from the Jetson on.
     * @param pathRequester The object for interacting with the Jetson.
     * @param maxVel        The maximum velocity, in feet/second.
     * @param maxAccel      The maximum acceleration, in feet/(second^2)
     * @param maxJerk       The maximum jerk, in feet/(second^3)
     * @param deltaTime     The time between setpoints in the profile, in seconds.
     * @param xOffset       Offset added to x after receiving the raw value from the Limelight.
     * @param yOffset       Offset added to y after receiving the raw value from the Limelight.
     * @param adaptive      Whether to run this command repeatedly.
     * @param timeout       How long this command is allowed to run. Should be set if using adaptive mode.
     */
    @JsonCreator
    public GoToVisionTargetLimelight(@NotNull @JsonProperty(required = true) T subsystem,
                                     @NotNull @JsonProperty(required = true) PathRequester pathRequester,
                                     @JsonProperty(required = true) double maxVel,
                                     @JsonProperty(required = true) double maxAccel,
                                     @JsonProperty(required = true) double maxJerk,
                                     @JsonProperty(required = true) double deltaTime,
                                     double xOffset,
                                     double yOffset,
                                     boolean adaptive,
                                     double timeout) {
        requires(subsystem);
        if (timeout > 0) {
            setTimeout(timeout);
        }
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        GetPathFromJetson getPath = new GetPathFromJetson(
                pathRequester, null, deltaTime, maxVel, maxAccel, maxJerk, false);
        GoToPositionRelative goToPositionRelative = new GoToPositionRelative<>(getPath, subsystem);
        goToPositionRelative.setWaypoints(this::getWaypoints);
        addSequential(goToPositionRelative);
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        this.adaptive = adaptive;
    }

    /**
     * @return The points for the path to hit, relative to the robot's current position.
     */
    @NotNull
    private Waypoint[] getWaypoints() {
        pullCamtran();
        double x = getX(), y = getY(), theta = getTheta();

        targetDetected = (x != xOffset);

        System.out.println("X: " + x);
        System.out.println("Y: " + y);
        System.out.println("THETA: " + theta);

        Waypoint[] toRet = new Waypoint[1];
        toRet[0] = new Waypoint(x, y, theta);
        return toRet;
    }

    /**
     * Populate the camtran array by pulling the values from NetworkTables.
     */
    private void pullCamtran() {
        camtran = table.getEntry("camtran").getDoubleArray(new double[] {0, 0, 0, 0, 0, 0});
    }

    /**
     * @return The relative X distance to the setpoint, in feet.
     */
    private double getX() {
        return camtran[0] + xOffset;
    }

    /**
     * @return The relative Y distance to the setpoint, in feet.
     */
    private double getY() {
        return camtran[1] + yOffset;
    }

    /**
     * @return The relative angular distance to the setpoint, in degrees.
     */
    private double getTheta() {
        return Math.toDegrees(camtran[4]);
    }

    /**
     * Stop the command group either for the normal reason or if no target was detected by the Limelight.
     * @return if the command group is finished or no target was detected.
     */
    @Override
    protected boolean isFinished() {
        return super.isFinished() || (targetDetected != null && !targetDetected);
    }

    /**
     * Run this command again if in adaptive mode and a target was detected.
     */
    @Override
    protected void end() {
        //Avoid null pointer
        if (targetDetected == null) {
            targetDetected = false;
        }

        if (adaptive && targetDetected) {
            Scheduler.getInstance().add(this);
        }
    }
}
