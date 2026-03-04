package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record LaunchConfig(Angle angle, AngularVelocity speed) {}
