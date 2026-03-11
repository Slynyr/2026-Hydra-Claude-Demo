package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public interface LauncherIO {

    @AutoLog
    class LauncherInputs {
        // CANcoder
        public boolean           isCANCoderConnected = false;
        public MagnetHealthValue magnetHealth        = MagnetHealthValue.Magnet_Invalid;

        // Launcher
        public boolean         isLauncherConnected = false;
        public double          launcherTemperature = 0.0;
        public Voltage         launcherVoltage     = Volts.of(0.0);
        public Current         launcherCurrent     = Amps.of(0);
        public AngularVelocity launcherVelocity    = RotationsPerSecond.of(0.0);

        public boolean         isLauncherFollowerConnected = false;
        public double          launcherFollowerTemperature = 0.0;
        public Voltage         launcherFollowerVoltage     = Volts.of(0.0);
        public Current         launcherFollowerCurrent     = Amps.of(0);
        public AngularVelocity launcherFollowerVelocity    = RotationsPerSecond.of(0.0);

        // Hood
        public Distance hoodServo1Pos    = Meters.of(0.0);
        public Distance hoodServo2Pos    = Meters.of(0.0);
        public Distance hoodServo1Target = Meters.of(0.0);
        public Distance hoodServo2Target = Meters.of(0.0);
    }

    // Launcher
    default void runVelocity(Supplier<AngularVelocity> velocity) {}

    default void stopLauncher() {}

    // Hood
    default Distance getHoodExtension() {
        return Meters.of(0);
    }

    default AngularVelocity getVelocity() {
        return RotationsPerSecond.of(0);
    }

    default void updateHood(Distance setpoint) {}

    default void updateInputs(LauncherInputs inputs) {}
}