package frc.robot.subsystems.serializer;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Checkmate;
import frc.robot.util.Checkmate.TestResult;
import frc.robot.util.MathUtils;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class Serializer extends SubsystemBase {

    private final SerializerIO               io;
    private final SerializerInputsAutoLogged inputs;

    public Serializer(SerializerIO io) {
        this.io = io;
        inputs = new SerializerInputsAutoLogged();

        Checkmate.register(
                "Should spin towards feeder", () -> {
                    CommandScheduler.getInstance().schedule(this.setVoltage(2));

                    if (this.getVelocity().in(RotationsPerSecond) > 0) {
                        return TestResult.success("Serializer spins the right way");
                    } else if (this.getVelocity().in(RotationsPerSecond) < 0) {
                        return TestResult.fail("Serializer spins the wrong way");
                    } else {
                        return TestResult.fail("Serializer is not spinning!");
                    }
                });
    }

    public Command setVoltage(double voltage) {
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    public Command stop() {
        return Commands.runOnce(io::stopMotors, this);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    @AutoLogOutput(key = "Serializer/BeltSpeed", unit = "m/s")
    public LinearVelocity getBeltSpeed() {
        return MathUtils.calculateSurfaceSpeed(getVelocity(), SerializerConstants.PULLEY_CIRCUMFERENCE);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
    }
}