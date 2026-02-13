package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Feeder extends SubsystemBase {

    private FeederInputsAutoLogged inputs;
    private FeederIO io;
    
    public Feeder(FeederIO io) {
        this.io = io;
        inputs = new FeederInputsAutoLogged();

        Checkmate.register("Should spin to move FUEL towards launcher", () -> {
            Command cmd = this.setVoltage(2);
            cmd.initialize();
            cmd.execute();
            if(this.getVelocity().in(RotationsPerSecond) > 0) {
                return TestResult.success("Feeder spins the right way");
            } else if (this.getVelocity().in(RotationsPerSecond) < 0) {
                return TestResult.fail("Feeder spins the wrong way");
            } else {
                return TestResult.fail("Feeder is not spinning!");
            }
        });
    }

    public Command setVoltage(double voltage){
        return Commands.runOnce(() -> {
            io.setMotorVoltage(voltage);
        }, this);
    }

    public Command runRPS(double RPS) {
        return Commands.runOnce(() -> {
            io.runRPS(RPS);
        }, this);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> {
            io.stopMotor();
        }, this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> {
            io.zeroEncoder();
        }, this);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocityRPS();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Feeder", inputs);
    }
}
