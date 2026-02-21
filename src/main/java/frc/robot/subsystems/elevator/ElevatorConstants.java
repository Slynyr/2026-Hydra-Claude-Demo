package frc.robot.subsystems.elevator;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import com.pathplanner.lib.config.PIDConstants;

public final class ElevatorConstants {

    public static final Current CURRENT_LIMIT = Units.Amps.of(30.0); // TODO: Replace with real climber current limit after testing
    public static final double GEARING = 9.0/1.0; // TODO: Replace with actual gearbox ratio
    public static final Distance ELEVATOR_DRUMRADIUS = Units.Inches.of(20.0);
    public static final Distance ELEVATOR_DRUMCIRCUMFERENCE = Units.Inches.of(2 * Math.PI * ELEVATOR_DRUMRADIUS.in(Inches));
    public static final Distance ROTATION_CONVERTER = ELEVATOR_DRUMCIRCUMFERENCE.div(GEARING);
    public static final PIDConstants TALONFX_PID = new PIDConstants(0, 0, 0); 
    public static final PIDConstants SIM_PID = new PIDConstants(10, 0, 0);
    public static final Mass ELEVATOR_MASS = Pound.of(1);
    public static final Distance ELEVATOR_MIN_HEIGHT = Units.Inches.of(10.0);
    public static final Distance ELEVATOR_MAX_HEIGHT = Units.Inches.of(20.0);

    public static final Distance ELEVATOR_PREP_HEIGHT = Meters.of(15.0);

    public static final Distance IDLING_HEIGHT = Meters.of(11.0);

}