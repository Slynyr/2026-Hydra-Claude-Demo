package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs;
    private static Pose3d hopperPose = new Pose3d();
    private Distance lastCrashPosition;

    public Hopper(HopperIO io) {
        this.io = io;
        inputs = new HopperInputsAutoLogged();
        // SmartDashboard.putData("Hopper/PID", HopperConstants.PID);

        hopperPose = new Pose3d();

        Checkmate.register("Hopper extends fully", () -> {
            CommandScheduler.getInstance().schedule(this.fullExtend());
            Timer.delay(2);
            Distance extensionLength = this.getPosition();
            if (extensionLength.isNear(HopperConstants.HOPPER_MAX_EXTENSION, Inches.of(0.02))) {
                return TestResult.success();
            } else if (extensionLength.isNear(HopperConstants.HOPPER_MIN_EXTENSION, Inches.of(0.02))) {
                return TestResult.fail("Hopper did not start! " + 
                                (inputs.isMotorConnected ? "(Motor connected)" : "(Motor not connected)"));
            } else {
                return TestResult.fail("Hopper not extending fully! Current Position: " + io.getPosition());
            }
        });

        Checkmate.register("Hopper retracts fully", () -> {
            CommandScheduler.getInstance().schedule(this.fullRetract());
            Timer.delay(2);
            Distance extensionLength = this.getPosition();
            if (extensionLength.isNear(HopperConstants.HOPPER_MIN_EXTENSION, 0)) {
                return TestResult.success();
            } else if (extensionLength.isNear(HopperConstants.HOPPER_MAX_EXTENSION,0)) {
                return TestResult.fail("Hopper did not start! " + 
                            (inputs.isMotorConnected ? "(Motor connected)" : "(Motor not connected)"));
            } else {
                return TestResult.fail("Hopper not retracting fully! Current Position: " + io.getPosition());
            }
        });
    }

    /** 
     * Extends hopper 12 inches out 
     */
    public Command fullExtend() {
        return Commands.runOnce(() ->
            io.setSetpoint(() -> HopperConstants.HOPPER_MAX_EXTENSION), this
        );
    }

    /** 
     * Retracts hopper all the way to 0 inches
     */
    public Command fullRetract() {
        return Commands.runOnce(
            () -> io.setSetpoint(() -> HopperConstants.HOPPER_MIN_EXTENSION), this
        );
    }

    /** 
     * Positive voltage extends, Negative voltage retracts (MAX of 12 inches and MIN of 0 inches)
     */
    public Command setVoltage(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command stopMotor() {
        return Commands.runOnce(
           () -> 
           io.stopMotor(), this
       );
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    public Command brakeMode() {
        return Commands.runOnce(() -> io.brakeMode(), this);
    }

    public Command coastMode() {
        return Commands.runOnce(() -> io.coastMode(), this);
    }

    public Distance getPosition() {
        return io.getPosition();
    }

    public Distance getPositionIntakeZero() {
        return io.getPositionIntakeZero();
    }


    public Distance getSetpoint(){
        return io.getSetpoint();
    }

    public Command setSetpoint(Supplier<Distance> setpoint) {
        return Commands.runOnce(() -> io.setSetpoint(setpoint));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
        hopperPose = new Pose3d(inputs.motorPosition.in(Meters), 0, 0, new Rotation3d());
        Logger.recordOutput("Components/Hopper", hopperPose);

        // SmartDashboard.putData("Hopper/PID", HopperConstants.PID);

        boolean overCurrent = inputs.torqueCurrent.gt(HopperConstants.DAMAGE_DETECTION_CURRENT);
    
        if(DriverStation.isEnabled() && HopperConstants.CRASH_DETECTION_ENABLED) {
            if (overCurrent && !inputs.isCrashDetected) {
                inputs.isCrashDetected = true;
                lastCrashPosition = io.getPosition();
                io.coastMode();
                io.setMotorVoltage(0);
            } else if (!overCurrent && inputs.isCrashDetected) {
                inputs.isCrashDetected = false;
                io.brakeMode();
                io.setSetpoint(() -> lastCrashPosition);
            }
        }
    }
}
