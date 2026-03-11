package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class LauncherIOSim implements LauncherIO {
    private static final double FLYWHEEL_INERTIA = 100.0;
    private static final double LAUNCHER_GEARING = 1.0;

    private final FlywheelSim   flywheel;
    private final PIDController motorController;

    private Distance hoodPos = Meters.of(0.0);

    private boolean isRunning;

    public LauncherIOSim() {
        DCMotor motorLauncher = DCMotor.getKrakenX60Foc(2);
        flywheel = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        motorLauncher,
                        FLYWHEEL_INERTIA,
                        LAUNCHER_GEARING
                ),
                motorLauncher,
                0.02);

        flywheel.update(0.01);

        motorController = new PIDController(0, 0, 0);

        isRunning = true;
    }

    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        flywheel.setAngularVelocity(velocity.get().in(RadiansPerSecond));
    }

    @Override
    public AngularVelocity getVelocity() {
        return flywheel.getAngularVelocity();
    }

    @Override
    public Distance getHoodExtension() {
        return hoodPos;
    }

    @Override
    public void updateHood(Distance setpoint) {
        hoodPos = setpoint;
    }

    @Override
    public void stopLauncher() {
        flywheel.setInputVoltage(0.0);
        flywheel.setAngularVelocity(0);
        motorController.reset();
        isRunning = false;
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        double voltageLauncher = 0;
        double currentLauncher = 0;

        if (isRunning) {
            voltageLauncher = MathUtil.clamp(
                    motorController.calculate(flywheel.getAngularVelocityRPM()), -12, 12);
            currentLauncher = flywheel.getCurrentDrawAmps();
        }

        inputs.isLauncherConnected = true;
        inputs.launcherTemperature = 0.0;
        inputs.launcherVoltage = Volts.of(voltageLauncher);
        inputs.launcherCurrent = Current.ofBaseUnits(currentLauncher, Amps);
        inputs.launcherVelocity = flywheel.getAngularVelocity();

        inputs.hoodServo1Pos = hoodPos;
        inputs.hoodServo2Pos = hoodPos;
    }
}