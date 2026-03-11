package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Checkmate;
import frc.robot.utils.Checkmate.TestResult;

public class Serializer extends SubsystemBase{
    
    private SerializerIO io;
    private final SerializerInputsAutoLogged inputs;

    public Serializer(SerializerIO io) {

        this.io = io;
        inputs = new SerializerInputsAutoLogged();
        
        Checkmate.register("Should spin towards feeder", () -> {
            Command cmd = this.setVoltage(2);
            cmd.initialize();
            cmd.execute();
            if(this.getVelocity().in(RotationsPerSecond) > 0) {
                return TestResult.success("Serializer spins the right way");
            } 
            else if (this.getVelocity().in(RotationsPerSecond) < 0) {
                return TestResult.fail("Serializer spins the wrong way");
            } 
            else {
                return TestResult.fail("Serializer is not spinning!");
            }
        });
    }

    public Command setVoltage(double voltage){
        return Commands.runOnce(() -> io.setVoltage(voltage), this);
    }

    public Command stopMotor() {
        return Commands.runOnce(() -> io.stopMotors(), this);
    }

    public Command zeroEncoder() {
        return Commands.runOnce(() -> io.zeroEncoders(), this);
    }

    public AngularVelocity getVelocity() {
        return io.getVelocity();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Serializer", inputs);
    }
}