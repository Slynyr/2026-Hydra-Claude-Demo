package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Checkmate;
import frc.robot.util.Checkmate.TestResult;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.RotationsPerSecond;

public class Feeder extends SubsystemBase {
    private final FeederInputsAutoLogged inputs;
    private final FeederIO               io;

    public Feeder(FeederIO io) {
        this.io = io;
        inputs = new FeederInputsAutoLogged();

        Checkmate.register(
                "Should spin towards launcher", () -> {
                    CommandScheduler.getInstance().schedule(runVelocity(() -> RotationsPerSecond.of(10)));

                    if (this.getVelocity().in(RotationsPerSecond) > 0) {
                        return TestResult.success("Feeder spins the right way");
                    } else if (this.getVelocity().in(RotationsPerSecond) < 0) {
                        return TestResult.fail("Feeder spins the wrong way");
                    } else {
                        return TestResult.fail("Feeder is not spinning!");
                    }
                });
    }

    public Command setVoltage(double voltage) {
        return Commands.runOnce(() -> io.setMotorVoltage(voltage), this);
    }

    public Command runVelocity(Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> io.runVelocity(velocity), this);
    }

    public Command stop() {
        return Commands.runOnce(io::stopMotor, this);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }
}
