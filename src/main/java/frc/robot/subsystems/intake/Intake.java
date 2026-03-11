package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.Extension;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;

public class Intake extends SubsystemBase {

    private final IntakeIO intakeIO;
    private final IntakeInputsAutoLogged inputs;
    private Distance position;

    private static Pose3d extenderPose;

    public Intake(IntakeIO intakeIO) {

        this.intakeIO = intakeIO;
        this.inputs = new IntakeInputsAutoLogged();
        extenderPose = new Pose3d();

//        Checkmate.register("Should fully extend Intake", () -> {
//
//            double extendTarget = Extension.EXTENSION_DISTANCE.in(Meters);
//
//            intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE);
//
//            Timer.delay(2.0);
//
//            if (Math.abs(inputs.extensionPosition - extendTarget) > 0.05) {
//                return TestResult.fail("Intake failed to extend, position: " + inputs.extensionPosition);
//            }
//            return TestResult.success("Intake extension ok");
//        });
//
//        Checkmate.register("Should fully retract Intake", () -> {
//
//            double retractTarget = Extension.EXTENSION_MIN_DISTANCE.in(Meters);
//
//            intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE);
//
//            Timer.delay(2.0);
//
//            if (Math.abs(inputs.extensionPosition - retractTarget) > 0.05) {
//                return TestResult.fail("Intake failed to retract, position: " + inputs.extensionPosition);
//            }
//            return TestResult.success("Intake retraction ok");
//        });

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


    }

/**
* Sets the voltage of the roller
* @param voltage The voltage to set the roller to, in volts. Should be between -12 and 12.
 * @return A command that sets the roller voltage when executed.
 */
    public Command setRollerVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(voltage));
    }
/**
 * Stops the roller by setting the voltage to 0.0 volts.
 * @return A command that stops the roller when executed.
 */
    public Command stopRoller() {
        return Commands.runOnce(() -> intakeIO.setRollerVoltage(0.0));
    }
/**
* Sets brakeMode for the motors
* @return A command that sets the extension motor to brake mode when executed.
 */
    public Command brakemode() {
        return Commands.runOnce(() -> intakeIO.brakeMode());
    }
/**
* Extends intake to constant distance
* @return A command that extends the intake when executed.
 */
    public Command extend() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_DISTANCE));
    }

    /**
     * Moves intake to given position
     * @param setpoint The position to move the intake to.
     * @return A command that moves the intake to the specified position when executed.
     */
    public Command move(Supplier<Distance> setpoint) {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Meters.of(setpoint.get().in(Meters))));
    }

/**
* Retracts intake to constant distance
* @return A command that retracts the intake when executed.
 */
    public Command retract() {
        return Commands.runOnce(() -> intakeIO.setSetpoint(Extension.EXTENSION_MIN_DISTANCE));
    }

    /**
     * Moves intake to given position
     * @param position The position to move the intake to.
     * @return A command that moves the intake to the specified position when executed.
     */
    public Command move(Distance position) {
        return Commands.runOnce(() -> intakeIO.setSetpoint(position));
    }
/**
* Stops the motors
* @return A command that stops the extension motor when executed.
 */
    public Command stopMotor() {
        // intakeIO.stopMotor();
        return Commands.runOnce(() -> intakeIO.stopMotor());
    }

    public Distance getSetpoint(){
        return inputs.extensionSetpoint;
    }
/**
* Sets the motors to coastMode
* @return A command that sets the extension motor to coast mode when executed.
 */
    public Command coastMode() {
        return Commands.runOnce(() -> intakeIO.coastMode());
    }
/**
* Sets the voltage of the extension motor
* @param voltage The voltage to set the extension motor to.
 * @return A command that sets the extension motor voltage when executed.
 */
    public Command setExtensionVoltage(double voltage) {
        return Commands.runOnce(() -> intakeIO.setExtensionVoltage(voltage));
    }
/**
* Gets the current position of the intake extension.
* @return The current position of the intake extension.
 */
    public Distance getPosition() {
        return intakeIO.getPosition();
    }

/**
 * Updates the inputs, and runs a consistent check for a "crash"
 */
    @Override
    public void periodic() {
        intakeIO.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        extenderPose = new Pose3d(

            inputs.extensionPosition.in(Meters), 0.0, 0.0,
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
        }

        Logger.recordOutput("Intake/pos", getPosition().in(Meters));
        Logger.recordOutput("Components/Intake", extenderPose);
        SmartDashboard.putData("Intake/PID", Extension.PID);
    }
}