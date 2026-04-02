package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

@Deprecated
public class Elevator extends SubsystemBase {
    private final ElevatorIO               io;
    private final ElevatorInputsAutoLogged inputs;

    private static Pose3d pose;

    public Elevator(ElevatorIO io) {
        this.io = io;
        inputs = new ElevatorInputsAutoLogged();

        pose = new Pose3d();
    }

    /**
     * Sets voltage of elevator to given voltage
     *
     * @param voltage voltage value
     */
    public Command setVoltage(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    /**
     * Runs a set voltage until the absolute current spikes above {@link ElevatorConstants#SPIKE_CURRENT}
     */
    public Command runVoltageUntilSpike(double voltage) {
        return Commands.sequence(
                setVoltage(voltage),
                Commands.waitUntil(() -> Amps.of(inputs.torqueCurrent.abs(Amp)).gte(ElevatorConstants.SPIKE_CURRENT)),
                stop(),
                zeroEncoder()
        );
    }

    /**
     * Zeros elevator encoder position
     */
    public Command zeroEncoder() {
        return Commands.runOnce(io::zeroEncoder, this);
    }

    /**
     * Sets the position of the elevator Command ends when the elevator is within 25mm range of the targeted position
     *
     * @param setpoint setpoint value
     */
    public Command setSetpointAndWait(Distance setpoint, int slot) {
        return Commands.sequence(
                Commands.runOnce(() -> io.setSetpoint(setpoint, slot), this),
                Commands.waitUntil(() -> setpoint.isNear(getPosition(), Meters.of(0.025)))
        );
    }

    /**
     * Stop all motor
     */
    public Command stop() {
        return Commands.runOnce(io::stopMotor, this);
    }

    /**
     * Gets position of the elevator
     *
     * @return The encoders position
     */
    public Distance getPosition() {
        return io.getPosition();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        // TODO: write as a triggered command if the elevator is implemented
        // Safety: Stop elevator if current exceeds 50A
//        if (inputs.supplyCurrent.gte(ElevatorConstants.SPIKE_CURRENT)) {
//            io.stopMotor();
//        }

        Logger.processInputs("Elevator", inputs);

        pose = new Pose3d(0, 0, inputs.position.in(Units.Meters), new Rotation3d());
        Logger.recordOutput("Components/Elevator", pose);
    }
}