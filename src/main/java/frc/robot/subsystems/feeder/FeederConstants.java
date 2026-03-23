package frc.robot.subsystems.feeder;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;

import static edu.wpi.first.units.Units.Amps;

public final class FeederConstants {
    public static final PIDConstants SIM_PID     = new PIDConstants(1, 0, 0);
    public static final PIDConstants TALONFX_PID = new PIDConstants(0.1, 0.05, 0.0);

    public static final double kV      = 0.035;
    public static final double kS      = 0.31;
    public static final double GEARING = 1.0 / 3.0;

    public static final Current CURRENT_LIMIT = Amps.of(30);
    public static final double FEEDER_REVERSE_VOLTAGE = 6;
}