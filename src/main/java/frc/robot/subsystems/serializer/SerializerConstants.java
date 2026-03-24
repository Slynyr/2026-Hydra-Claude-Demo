package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.Current;

import static edu.wpi.first.units.Units.Amps;

public final class SerializerConstants {
    public static final double  SERIALIZING_VOLTAGE = 12;
    public static final Current CURRENT_LIMIT       = Amps.of(30);
    public static final double  GEARING             = 1.0 / 3.0;
}
