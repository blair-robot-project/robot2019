package org.usfirst.frc.team449.robot.generalInterfaces.doubleUnaryOperator.feedForwardComponent;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.Nullable;

/**
 * A feedforward component for arms.
 */
public class FeedForwardArmComponent extends FeedForwardKaKvComponent {

    private final double angleOffset;

    private final double cosTerm;

    /**
     * Field to avoid garbage collection
     */
    private double angleFactor;

    /**
     * Default constructor.
     *
     * @param kVFwd               The voltage required to raise the arm at a steady-state velocity of 1 rotation per second.
     * @param kVRev               The voltage required to lower tha arm at a steady-state velocity of 1 rotation per second.
     *                            Defaults to kVFwd.
     * @param kAFwd               The voltage required to accelerate the arm up at one rotation per second^2 with the arm horizontal. Defaults to zero, meaning we don't use acceleration feed-forward.
     * @param kARev               The voltage required to accelerate the arm down at one foot per second^2 with the arm horizontal. Defaults to kAFwd.
     * @param interceptVoltageFwd The voltage required to overcome static friction in the forwards direction with the arm vertical. Vintercept in the drive characterization paper. Defaults to 0.
     * @param interceptVoltageRev The voltage required to overcome static friction in the reverse direction with the arm vertical. Vintercept in the drive characterization paper. Defaults to interceptVoltageFwd.
     * @param cosTerm The extra voltage required to move the arm when it's horizontal vs vertical, in volts.
     * @param startingAngle The starting angle of the arm, in degrees. The angles are defined such that zero degrees is horizontal. Defaults to zero.
     */
    @JsonCreator
    public FeedForwardArmComponent(@JsonProperty(required = true) double kVFwd,
                                   @Nullable Double kVRev,
                                   double kAFwd,
                                   @Nullable Double kARev,
                                   double interceptVoltageFwd,
                                   @Nullable Double interceptVoltageRev,
                                   double cosTerm,
                                   double startingAngle) {
        super(kVFwd, kVRev, kAFwd, kARev, interceptVoltageFwd, interceptVoltageRev);
        this.cosTerm = cosTerm;
        this.angleOffset = startingAngle/360.;
    }

    @Override
    public double calcMPVoltage(double positionSetpoint, double velSetpoint, double accelSetpoint) {
        angleFactor = Math.cos((talon.getPositionFeet() + angleOffset)*2*Math.PI);
        if (velSetpoint > 0 || (velSetpoint == 0 && accelSetpoint > 0)) {
            return velSetpoint * kVFwd + angleFactor * accelSetpoint * kAFwd + interceptVoltageFwd + angleFactor * cosTerm;
        } else if (velSetpoint < 0 || (velSetpoint == 0 && accelSetpoint < 0)) {
            return velSetpoint * kVRev + angleFactor * accelSetpoint * kARev - interceptVoltageRev + angleFactor * cosTerm;
        } else {
            return angleFactor * cosTerm;
        }
    }

    @Override
    public double applyAsDouble(double operand) {
        return super.applyAsDouble(operand) + cosTerm * Math.cos((talon.getPositionFeet() + angleOffset)*2*Math.PI);
    }
}
