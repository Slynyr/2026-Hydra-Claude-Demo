package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public final class HopperConstants {

    public static final Current CURRENT_LIMIT = Amps.of(20.0);
    public static final Current DAMAGE_DETECTION_CURRENT = Amps.of(50.0); //TODO: test on real robot
    public static final boolean CRASH_DETECTION_ENABLED = true;

    public static final double GEARING = 9.0 / 1.0;

    public static final Distance HOPPER_DRUMRADIUS = Meters.of(1.0);
    public static final Mass HOPPER_MASS = Pounds.of(10.561);    

    public static final PIDConstants TALONFX_PID = new PIDConstants(4.5,0.55,0);

    public static final double UNIT_CONVERSION_FACTOR = 0.29/2.715;

    public static final PIDConstants SIM_PID = new PIDConstants(4, 0, 3);

    public static final Distance HOPPER_MIN_EXTENSION = Centimeters.of(1.0);
    public static final Distance HOPPER_MAX_EXTENSION = Centimeters.of(39.0);
}
