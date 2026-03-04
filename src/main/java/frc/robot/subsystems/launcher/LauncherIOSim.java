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

    private final FlywheelSim   flywheelSim;
    private final PIDController controllerLauncher;

    private Distance hoodPos = Meters.of(0.0);

    private boolean isRunning;

    public LauncherIOSim() {
        DCMotor motorLauncher = DCMotor.getKrakenX60Foc(2);
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(
                        motorLauncher,
                        FLYWHEEL_INERTIA,
                        LAUNCHER_GEARING
                ),
                motorLauncher,
                0.02);

        flywheelSim.update(0.01);

        controllerLauncher = new PIDController(
                LauncherConstants.Launcher.PID.getP(), LauncherConstants.Launcher.PID.getI(),
                LauncherConstants.Launcher.PID.getD());

        isRunning = true;
    }

    @Override
    public void runVelocity(Supplier<AngularVelocity> velocity) {
        flywheelSim.setAngularVelocity(velocity.get().in(RadiansPerSecond) * 60);
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
        flywheelSim.setInputVoltage(0.0);
        flywheelSim.setAngularVelocity(0);
        controllerLauncher.reset();
        isRunning = false;
    }

    @Override
    public void updateInputs(LauncherInputs inputs) {
        double voltageLauncher = 0;
        double currentLauncher = 0;

        if (isRunning) {
            voltageLauncher = MathUtil.clamp(
                    controllerLauncher.calculate(flywheelSim.getAngularVelocityRPM()), -12, 12);
            currentLauncher = flywheelSim.getCurrentDrawAmps();
        }

        inputs.isLauncherConnected = true;
        inputs.launcherTemperature = 0.0;
        inputs.launcherVoltage = Volts.of(voltageLauncher);
        inputs.launcherCurrent = Current.ofBaseUnits(currentLauncher, Amps);
        inputs.launcherVelocity = flywheelSim.getAngularVelocity();

        inputs.hoodServo1Pos = hoodPos;
        inputs.hoodServo2Pos = hoodPos;
    }
}