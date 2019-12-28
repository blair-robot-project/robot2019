package org.usfirst.frc.team449.robot.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import edu.wpi.first.networktables.NetworkTableInstance;

import java.util.function.DoubleSupplier;

/**
 * Determines the distance from the Limelight to a vision target. Currently using a ton of hardcoded values.
 * Assumes that the Limelight is at an angle with respect to the vision targets
 * If the Limelight isn't tilted, use LimeLightDistanceComponentSimple
 */
public class LimeLightDistanceComponentTilted implements DoubleSupplier {

    /**
     * The height of the Limelight
     */
    private final double limelightHeight;
    /**
     * The tilt downwards of the Limelight, in degrees
     */
    private final double limelightAngleDown;

    /**
     * Default constructor
     *
     * @param limelightHeight The height of the Limelight
     * @param limelightAngleDown The angle of the Limelight, in degrees
     */
    @JsonCreator
    public LimeLightDistanceComponentTilted(@JsonProperty(required = true) double limelightHeight,
                                            @JsonProperty(required = true) double limelightAngleDown) {
        this.limelightHeight = limelightHeight;
        this.limelightAngleDown = limelightAngleDown;
    }

    /**
     * @return Gets the distance from the robot to the vision target, accounting for the tilt of the Limelight
     */
    @Override
    public double getAsDouble() {
        double robotToTargAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        return limelightHeight * Math.tan(Math.toRadians(90 - limelightAngleDown + robotToTargAngle));
    }
}
