package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.Current;

import static edu.wpi.first.units.Units.Amps;

public final class SerializerConstants {
    public static final double  SERIALIZING_VOLTAGE = 10;
    public static final Current CURRENT_LIMIT       = Amps.of(20);
    public static final double  GEARING             = 1.0 / 3.0;
}
