package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;

public final class FeederConstants {

        public static final PIDConstants SIM_PID = new PIDConstants(1, 0, 0);
        public static final PIDController PID = new PIDController(1.0,0.0,0.0);
	public static final PIDConstants TALONFX_PID = new PIDConstants(PID.getP(), PID.getI(), PID.getD());

        public static final double kV = 0.0;
        public static final double GEARING = 1.0/3.0;

        public static final boolean ORTONA_FEEDER_MOTOR_INVERTED = false;
        public static final Current ORTONA_SPARK_MAX_CURRENT_LIMIT = Amps.of(30);

        public static final Current TALON_FX_CURRENT_LIMIT = Amps.of(30);
}  