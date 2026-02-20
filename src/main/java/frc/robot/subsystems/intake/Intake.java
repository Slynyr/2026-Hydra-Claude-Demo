package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;

    private static Pose3d extenderPose;

    public Intake(IntakeIO intakeIO) {

        this.intakeIO = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();
        extenderPose = new Pose3d();

        Checkmate.register("Should fully extend Intake", () -> {

            double extendTarget = Extension.EXTENSION_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - extendTarget) > 0.02) {
                return TestResult.fail("Intake extension failed to extend, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake extension ok, position: " + inputs.extensionPosition);
        });

        Checkmate.register("Should fully retract Intake", () -> {

            double retractTarget = Extension.EXTENSION_MIN_DISTANCE.in(Meters);

            intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);

            Timer.delay(2.0);

            if (Math.abs(inputs.extensionPosition - retractTarget) > 0.02) {
                return TestResult.fail("Intake extension failed to retract, position: " + inputs.extensionPosition);
            }
            return TestResult.success("Intake extension ok, position: " + inputs.extensionPosition);
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

    public Command intake(double voltage) {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage), this);
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

    public Command stopMotor() {
        return Commands.runOnce(() -> intakeIO.stopMotor(), this);
    }

    public Distance getPosition() {
        return intakeIO.getPosition();
    }

    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
        extenderPose = new Pose3d(

            inputs.extensionPosition, 0.0, 0.0,
            new Rotation3d(0.0, 0.0, Math.toRadians(0.0))

        );

        Logger.recordOutput("Components/Intake", extenderPose);

    }

}