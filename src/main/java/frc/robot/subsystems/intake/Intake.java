package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.*;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;
    private Distance position;

    private static Pose3d extenderPose;

    public Intake(IntakeIO intakeIO) {

        this.intakeIO = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();
        extenderPose = new Pose3d();

        Checkmate.register("Should fully extend Intake", () -> {

            double extendTarget = Extension.EXTENSION_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - extendTarget) > 0.05) {
                return TestResult.fail("Intake failed to extend, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake extension ok");
        });

        Checkmate.register("Should fully retract Intake", () -> {

            double retractTarget = Extension.EXTENSION_MIN_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - retractTarget) > 0.05) {
                return TestResult.fail("Intake failed to retract, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake retraction ok");
        });


        Checkmate.register("Should spin roller", () -> {

            intakeIO.setRollerVoltage(6.0);

            Timer.delay(2.0);

            double current = inputs.rollerCurrent.in(Amps);
            intakeIO.setRollerVoltage(0.0);
            if (Math.abs(current) < 1.0) {
                return TestResult.fail("Intake roller failed to spin up, current: " + current);
            }
            return TestResult.success("Intake roller ok, current: " + current);
        });
        ;

    }

    public Command setRollerVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);
    }

    public Command stopRoller() {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(0.0), this);
    }

    public Command brakemode() {
        return Commands.runOnce(() -> intakeIO.brakeMode(), this);
    }

    public Command extend() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE), this);
    }

    public Command retract() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE), this);
    }

    public Command move(Distance position) {
        return Commands.runOnce(() -> intakeIO.setSetpoint(position), this);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> intakeIO.stopMotor(), this);
    }

    public Command coastMode() {
        return Commands.runOnce(() -> intakeIO.coastMode(), this);
    }

    public Command setExtensionVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setExtensionVoltage(voltage), this);
    }

    public Distance getPosition() {
        return intakeIO.getPosition();
    }

    @Override
    public void periodic() {
        Logger.processInputs("Intake", inputs);
        extenderPose = new Pose3d(

            inputs.extensionPosition, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, Math.toRadians(0.0))

        );

        boolean overCurrent = inputs.extensionTorqueCurrent.gt(IntakeConstants.Extension.CRASH_CURRENT_THRESHOLD);
            
        if (DriverStation.isEnabled()){
            if (overCurrent && !inputs.isCrashDetected) {
                position = getPosition();
                inputs.isCrashDetected = true;
                intakeIO.coastMode();
            } else if (!overCurrent && inputs.isCrashDetected) {
                intakeIO.setSetpoint(position);
                inputs.isCrashDetected = false;
                intakeIO.brakeMode();
            }

        intakeIO.updateInputs(inputs);
        Logger.recordOutput("Components/Intake", extenderPose);
        SmartDashboard.putData("Intake/PID", Extension.PID);

        }
    
    }
}