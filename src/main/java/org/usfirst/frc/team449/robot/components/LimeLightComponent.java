package org.usfirst.frc.team449.robot.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.jetbrains.annotations.NotNull;
import java.util.function.DoubleSupplier;

/**
 * The component that supplies distances from the limelight to a vision target
 */
@JsonIdentityInfo(generator = ObjectIdGenerators.StringIdGenerator.class)
public class LimeLightComponent implements DoubleSupplier {

    /**
     * The NetworkTableEntry that supplies the desired value
     * Determined by the ReturnValue value
     */
    @NotNull NetworkTableEntry entry;
    /**
     * Whether the limelight has a valid target in sight. Will return 0 for no, 1 for yes
     */
    private final NetworkTableEntry tv;
    /**
     * Which value to ask from the limelight
     * Can be:
     *  x: x distance from target
     *  y: y distance from target
     *  area: area of the target seen (0% to 100% of the whole target seen)
     *  poseX: x position of the target in a 3D model
     *  poseY: y position of the target in a 3D model
     *  poseZ: z position of the target in a 3D model
     *  pitch: angle from limelight to target on x-z coordinate plane, determined by the 3D model from camtran
     *  yaw: same as pitch, but on x-y coordinate plane
     *  roll: same as pitch, on y-z coordinate plane
     */
    private final ReturnValue value;
    /**
     * Added to the output double
     */
    private final double offset;

    enum ReturnValue {
        x, y, area, poseX, poseY, poseZ, pitch, yaw, roll
    }

    /**
     * Default creator
     * @param value what to request from the Limelight
     */
    @JsonCreator
    public LimeLightComponent(@JsonProperty(required = true) ReturnValue value,
                              double offset){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        this.value = value;
        this.offset = offset;
        switch(value) {
            case x:
                entry = table.getEntry("tx");
                break;
            case y:
                entry = table.getEntry("ty");
                break;
            case area:
                entry = table.getEntry("ta");
                break;
            default:
                entry = table.getEntry("camtran");
        }
        tv = table.getEntry("tv");
    }

    /**
     * @return requested value from LimeLight
     */
    @Override
    public double getAsDouble() {
        if (tv.getDouble(0) == 0){
            return Double.NaN;
        }
        double[] camtran = entry.getDoubleArray(new double[6]);
        switch(value) {
            case poseX:
                return camtran[0] + offset;
            case poseY:
                return camtran[1] + offset;
            case poseZ:
                return camtran[2] + offset;
            case pitch:
                return camtran[3] + offset;
            case yaw:
                return camtran[4] + offset;
            case roll:
                return camtran[5] + offset;
            default:
                return entry.getDouble(0.0) + offset;
        }
    }
}