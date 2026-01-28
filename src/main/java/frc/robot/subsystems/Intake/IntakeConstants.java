package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

public final class IntakeConstants {

	public static final class Extension {

		public static final int MOTORID = 5;

		public static final Distance EXTENSION_MIN_DISTANCE = Meters.of(0.0);
		public static final Distance EXTENSION_MAX_DISTANCE = Meters.of(0.5);

		public static final PIDConstants TALONFX_PID = new PIDConstants(1.0, 0.01, 0.0);
		public static final PIDConstants SIM_PID = new PIDConstants(1.0, 0.0, 0.0);
		public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
		public static final Current MAX_CURRENT = Amps.of(40.0);
		public static final double GEARING = 10.0;
		public static final Mass INTAKE_MASS = Kilograms.of(1.0);
		public static final Distance INTAKE_DRUMRADIUS = Meters.of(0.0254);
		public static final Distance INTAKE_MIN_DISTANCE = Meters.of(0.0);
		public static final Distance INTAKE_MAX_DISTANCE = Meters.of(0.5); 

		public static final int UPDATE_FREQUENCY = 50; // in Hz

	}

    public static final class Roller {

		public static final int MOTORID = 4;

        public static final Voltage MAX_VOLTAGE = Volts.of(12.0);
        public static final Current MAX_CURRENT = Amps.of(40.0);
        public static final double GEARING = 10.0;
        public static final Mass ROLLER_MASS = Kilograms.of(1.0);
        public static final Distance ROLLER_DRUMRADIUS = Meters.of(0.0254);

    }
}