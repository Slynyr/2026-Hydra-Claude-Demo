package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;


public final class SerializerConstants {
    
    public static final Current ORTONA_SPARK_MAX_CURRENT_LIMIT = Amps.of(30);
    public static final boolean ORTONA_INDEXER_MOTOR_INVERTED = false;
        public static final boolean ORTONA_FEEDER_MOTOR_INVERTED = false;

    public static final Current TALON_FX_CURRENT_LIMIT = Amps.of(30);
    public static final int INDEXER_ID = 1;
    public static final int FEEDER_ID = 2;

}
