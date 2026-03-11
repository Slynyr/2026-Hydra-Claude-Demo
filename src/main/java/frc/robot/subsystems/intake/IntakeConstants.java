package frc.robot.subsystems.intake;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public final class IntakeConstants {

    public static final class Extension {
        public static final Distance EXTENSION_MIN_DISTANCE = Centimeters.of(0.5);
        public static final Distance EXTENSION_DISTANCE     = Centimeters.of(26.5);
        public static final Distance EXTENSION_MAX_DISTANCE = Centimeters.of(27);

        public static final PIDController SIM_PID     = new PIDController(0.0, 0.0, 0.0);
        public static final PIDConstants  TALONFX_PID = new PIDConstants(5.5, 0.75, 0);

        public static final Voltage MAX_VOLTAGE            = Volts.of(12.0);
        public static final Current CURRENT_LIMIT          = Amps.of(20.0);
        public static final double  GEARING                = 9.0 / 1.0;
        public static final double  UNIT_CONVERSION_FACTOR = 0.11 / 1.58;
        public static final Mass    INTAKE_MASS            = Kilograms.of(3.1818);

        public static final Distance EXTEND_POINT  = Centimeters.of(23.5);
        public static final Distance RETRACT_POINT = EXTEND_POINT.minus(Centimeters.of(7.5));

        public static final int UPDATE_FREQUENCY = 50; // in Hz

        public static final Current CRASH_CURRENT_THRESHOLD = Amps.of(50.0); //TODO: test it
    }

    public static final class Roller {
        public static final Current CURRENT_LIMIT = Amps.of(30.0);
        public static final double  GEARING       = 1.5 / 1.0;

        public static final double INTAKE_VOLTAGE  = 10;
        public static final double AGITATE_VOLTAGE = 6;
    }
}