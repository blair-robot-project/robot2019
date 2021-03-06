package org.usfirst.frc.team449.robot.components;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import org.jetbrains.annotations.NotNull;
import org.usfirst.frc.team449.robot.jacksonWrappers.FPSTalon;

import java.util.function.DoubleSupplier;

public class TalonDataComponent implements DoubleSupplier {
    @NotNull private final FPSTalon talon;
    @NotNull private final ReturnValue value;

    enum ReturnValue{
        position, velocity, current, voltage;
    }

    /**
     *
     * @param talon the talon to get a value from
     * @param value whether to get the position, velocity, current, or voltage
     */
    @JsonCreator
    public TalonDataComponent(@NotNull @JsonProperty(required = true) FPSTalon talon,
                              @NotNull @JsonProperty(required = true) ReturnValue value) {
        this.talon = talon;
        this.value = value;
    }

    /**
     * @return the requested value from the talon - 0 if none specified
     */
    @Override
    public double getAsDouble() {
        switch(value) {
            case position:
                return talon.getPositionFeet();
            case velocity:
                return talon.getVelocity();
            case current:
                return talon.getOutputCurrent();
            case voltage:
                return talon.getOutputVoltage();
            default:
                return 0;
        }
    }
}
