package frc.robot.subsystems.feeder;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;

public final class FeederConstants {
    public static final PIDConstants SIM_PID     = new PIDConstants(1, 0, 0);
    public static final PIDConstants TALONFX_PID_UPPER = new PIDConstants(0.2, 0.0, 0.0);
    public static final PIDConstants TALONFX_PID_LOWER = new PIDConstants(0.025, 0.0, 0.0);

    public static final double kV_UPPER      = 0.035;
    public static final double kS_UPPER      = 0.31;

    public static final double kV_LOWER      = 0.035;
    public static final double kS_LOWER      = 0.31;    

    public static final double GEARING = 1.0 / 3.0;

    public static final Current CURRENT_LIMIT = Amps.of(30);
    public static final double FEEDER_REVERSE_VOLTAGE = 6;

    public static final Distance FEEDER_ROLLER_DIAMETER = Inches.of(2);
    public static final Distance FEEDER_ROLLER_CIRCUMFERENCE = FEEDER_ROLLER_DIAMETER.times(Math.PI);
}