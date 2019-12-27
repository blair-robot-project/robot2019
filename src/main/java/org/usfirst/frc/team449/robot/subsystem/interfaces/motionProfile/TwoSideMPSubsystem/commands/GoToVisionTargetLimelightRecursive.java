package org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.TwoSideMPSubsystem.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.components.PathRequester;
import org.usfirst.frc.team449.robot.other.Logger;
import org.usfirst.frc.team449.robot.subsystem.interfaces.AHRS.SubsystemAHRS;
import org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.TwoSideMPSubsystem.SubsystemMPTwoSides;

/**
 * A command that drives the given subsystem the vision target detected by the Limelight.
 * Every interval, the vision target's pose is pulled from the Limelight again and the path being followed is updated.
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class GoToVisionTargetLimelightRecursive<T extends Subsystem & SubsystemMPTwoSides & SubsystemAHRS> extends Command {

    /**
     * The subsystem to run the path gotten from the Jetson on.
     */
    @NotNull
    private final T subsystem;

    /**
     * The object for interacting with the Jetson.
     */
    @NotNull
    private final PathRequester pathRequester;

    /**
     * The time between setpoints in the profile, in seconds.
     */
    private final double deltaTime;

    /**
     * The maximum velocity, in feet/second.
     */
    private final double maxVel;

    /**
     * The maximum acceleration, in feet/(second^2)
     */
    private final double maxAccel;

    /**
     * The maximum jerk, in feet/(second^3)
     */
    private final double maxJerk;

    /**
     * How long to follow the current path before regenerating an updated path, in seconds.
     */
    private final double regenInterval;

    private final double xOffset, yOffset;

    GoToVisionTargetLimelight<T> goNow;

    /**
     * Default constructor
     *
     * @param subsystem     The subsystem to run the path gotten from the Jetson on.
     * @param pathRequester The object for interacting with the Jetson.
     * @param maxVel        The maximum velocity, in feet/second.
     * @param maxAccel      The maximum acceleration, in feet/(second^2)
     * @param maxJerk       The maximum jerk, in feet/(second^3)
     * @param deltaTime     The time between setpoints in the profile, in seconds.
     * @param regenInterval How long to follow the current path before regenerating an updated path, in seconds.
     */
    @JsonCreator
    public GoToVisionTargetLimelightRecursive(@NotNull @JsonProperty(required = true) T subsystem,
                                              @NotNull @JsonProperty(required = true) PathRequester pathRequester,
                                              @JsonProperty(required = true) double maxVel,
                                              @JsonProperty(required = true) double maxAccel,
                                              @JsonProperty(required = true) double maxJerk,
                                              @JsonProperty(required = true) double deltaTime,
                                              double xOffset,
                                              double yOffset,
                                              @JsonProperty(required = true) double regenInterval) {
        this.subsystem = subsystem;
        requires(subsystem);
        this.pathRequester = pathRequester;
        this.deltaTime = deltaTime;
        this.maxVel = maxVel;
        this.maxAccel = maxAccel;
        this.maxJerk = maxJerk;
        this.regenInterval = regenInterval;
        this.xOffset = xOffset;
        this.yOffset = yOffset;
        goNow = new GoToVisionTargetLimelight<>(
                subsystem, pathRequester, maxVel, maxAccel, maxJerk, deltaTime, xOffset, yOffset, regenInterval);
    }

    /**
     * Log when this command is initialized and start the GoToVisionTarget command.
     */
    @Override
    protected void initialize() {
        Logger.addEvent("GoToVisionTargetLimelightRecursive init", this.getClass());
        goNow.start();
    }

    /**
     * Stop when the path is completely followed or the time limit is hit.
     *
     * @return whether the path was completed or ran out of time.
     */
    @Override
    protected boolean isFinished() {
        return goNow.isCompleted() || goNow.isCanceled();
    }

    /**
     * Log that the command has ended and start another instance of this class, unless no pose was received last time.
     */
    @Override
    protected void end() {
        Logger.addEvent("GoToVisionTargetLimelightRecursive end", this.getClass());
        if (goNow.wasPoseReceived()) {
            GoToVisionTargetLimelightRecursive<T> goAgain = new GoToVisionTargetLimelightRecursive<>(
                    subsystem, pathRequester, maxVel, maxAccel, maxJerk, deltaTime, xOffset, yOffset, regenInterval);
            goAgain.start();
        }
    }

    /**
     * Cancel the GoToVisionTarget command.
     */
    @Override
    public synchronized void cancel() {
        if (goNow != null && goNow.isRunning()) {
            goNow.cancel();
        }
        super.cancel();
    }

    /**
     * Log that the command has been interrupted and cancel the GoToVisionTarget command.
     */
    @Override
    protected void interrupted() {
        Logger.addEvent("GoToVisionTargetLimelightRecursive interrupted!", this.getClass());
        if (goNow != null && goNow.isRunning()) {
            goNow.cancel();
        }
    }

}
