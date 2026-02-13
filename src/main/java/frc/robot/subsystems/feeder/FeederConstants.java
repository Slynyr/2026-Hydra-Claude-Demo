package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Current;

public final class FeederConstants {
        public static final int FEEDER_ID = 2;

        public static final PIDConstants SIM_PID = new PIDConstants(0.0175, 0, 0);
        public static final PIDConstants TALONFX_PID = new PIDConstants(0.001, 0, 0);
        public static final double kG = 0.0;
        public static final double kS = 0.1;
        public static final double kV = 0.12;

        public static final boolean ORTONA_FEEDER_MOTOR_INVERTED = false;
        public static final Current ORTONA_SPARK_MAX_CURRENT_LIMIT = Amps.of(30);

        public static final Current TALON_FX_CURRENT_LIMIT = Amps.of(30);
}  