package frc.robot.subsystems.launcher.interpolator;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public record LaunchConfig(Distance hoodExtension, AngularVelocity speed) {}
