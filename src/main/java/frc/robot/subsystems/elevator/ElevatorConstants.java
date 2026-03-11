package frc.robot.subsystems.elevator;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

import static edu.wpi.first.units.Units.*;

public final class ElevatorConstants {
    public static final Current  CURRENT_LIMIT = Amps.of(30.0);
    public static final double   GEARING       = 20.0 / 1.0;
    public static final Distance DRUM_RADIUS   = Inches.of(20.0);

    public static final Current SPIKE_CURRENT = Amps.of(50);

    public static final PIDConstants TALONFX_SLOW_PID = new PIDConstants(6.0, 0.67, 0.0);
    public static final PIDConstants TALONFX_FAST_PID = new PIDConstants(0.0, 0.0, 0.0);
    public static final PIDConstants SIM_PID          = new PIDConstants(10, 0, 0);
    public static final double       kG               = 0.0;

    public static final Mass     ELEVATOR_MASS       = Pound.of(1);
    public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(23.2);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(27.5);

    public static final class kSetpoints {
        public static final Distance ELEVATOR_UP   = Meters.of(1);
        public static final Distance ELEVATOR_DOWN = Meters.of(0.5);
    }
}