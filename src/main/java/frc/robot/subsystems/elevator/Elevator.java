package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class Elevator extends SubsystemBase{

    private final ElevatorIO io;
    private final ElevatorInputsAutoLogged inputs;

    private static Pose3d elevatorPose;

    // Setup alerts for elevator motors connection
    private final Alert ElevatorAlert  = new Alert("The Left Elevator Motor is Disconnected " + ElevatorConstants.MAIN_MOTOR_ID, AlertType.kError);

    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        elevatorPose = new Pose3d();
    }

    /**
     * Sets voltage of elevator to given voltage
     * @param voltage voltage value
     */
    public Command startManualMove(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    /**
     * Zeros elevator encoder position 
     */
    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoder(), this);
    }

    /**
     * Sets the position of the elevator
     * Command ends when the elevator is within 25mm range of the targeted position
     * @param setpoint setpoint value
     */
    public Command elevatorGo(Distance setpoint) {
        return Commands.sequence(
            Commands.runOnce(() -> io.setSetpoint(setpoint), this),
            Commands.waitUntil(() -> setpoint.isNear(getPosition(), Meters.of(0.025)))
        );
    }

    /**
     * Stop all motor 
     */
    public Command stopAll() {
        return Commands.runOnce(() -> io.stopMotor(), this);
    }
    
    /**
     * Gets position of the elevator
     * @return The encoders position
     */
    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        Logger.recordOutput("Components/Elevator", elevatorPose);

        //Alert if motors are disconnected
        ElevatorAlert.set(!inputs.isMainMotorConnected);

        elevatorPose = new Pose3d(0,0,inputs.mainMotorPosition.in(Units.Meters), new Rotation3d());
    }
}