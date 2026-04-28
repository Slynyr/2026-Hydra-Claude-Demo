package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;

public final class SerializerConstants {
    public static final double  SERIALIZING_VOLTAGE = 12;
    public static final Current CURRENT_LIMIT       = Amps.of(25);
    public static final Distance PULLEY_DIAMETER = Inches.of(2);
    public static final Distance PULLEY_CIRCUMFERENCE = PULLEY_DIAMETER.times(Math.PI);
}
