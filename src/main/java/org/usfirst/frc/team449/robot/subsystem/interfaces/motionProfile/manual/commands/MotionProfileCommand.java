package org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.manual.commands;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.other.Clock;
import org.usfirst.frc.team449.robot.other.Logger;
import org.usfirst.frc.team449.robot.other.MotionProfileData;
import org.usfirst.frc.team449.robot.subsystem.interfaces.motionProfile.manual.SubsystemMPManual;


/**
 * Executes the motion profile
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class MotionProfileCommand<T extends Subsystem & SubsystemMPManual> extends Command {
    /**
     * Amount of time this command is allowed to run for, in milliseconds
     */
    private final long timeout;

    /**
     * The subsystem to run the motion profile on
     */
    private T subsystem;

    /**
     * The motion profile points that the command will execute
     */
    private MotionProfileData data;

    /**
     * The time this command started running at.
     */
    private long startTime;

    /**
     *
     */
    private int index;


    /**
     * @param timeout   the time that this command will run for in seconds
     * @param subsystem the subsystem the motion profile will run on
     * @param data      the motion profile points
     */
    @JsonCreator
    public MotionProfileCommand(@JsonProperty(required = true) double timeout,
                                @NotNull @JsonProperty(required = true) T subsystem,
                                @NotNull @JsonProperty(required = true) MotionProfileData data) {

        this.timeout = (long) (timeout * 1000.);

        this.subsystem = subsystem;
        requires(subsystem);

        this.data = data;

    }

    /**
     * Returns whether this command is finished. If it is, then the command will be removed and {@link Command#end()
     * end()} will be called.
     *
     * <p>It may be useful for a team to reference the {@link Command#isTimedOut() isTimedOut()}
     * method for time-sensitive commands.
     *
     * <p>Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Returning true will result in the command executing once
     * and finishing immediately. We recommend using {@link InstantCommand} for this.
     *
     * @return whether this command is finished.
     * @see Command#isTimedOut() isTimedOut()
     */
    protected boolean isFinished() {
        return (index == data.getData().length - 1 || timeout <= Clock.currentTimeMillis() - startTime);
    }

    @Override
    protected void initialize() {
        //Record the start time.
        startTime = Clock.currentTimeMillis();
        Logger.addEvent("RunLoadedProfile init", this.getClass());

    }

    @Override
    protected void execute() {
        index = Math.min((int) (Clock.currentTimeMillis() - startTime) / data.getPointTimeMillis(), data.getData().length - 1);
        double[] profileData = data.getData()[index];
        subsystem.runMPPoint(profileData[0], profileData[1], profileData[2]);
    }

    @Override
    protected void end() {
        double pos = data.getData()[data.getData().length - 1][0];
        subsystem.holdPosition(pos);
        Logger.addEvent("RunLoadedProfile end.", this.getClass());
    }

}
