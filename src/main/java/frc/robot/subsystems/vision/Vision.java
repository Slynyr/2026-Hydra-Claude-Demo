package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LogHelper;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.vision.VisionConstants.*;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class Vision extends SubsystemBase {
    private final VisionIO               io;
    private final VisionInputsAutoLogged inputs;
    private final Alert                  disconnectedAlert = new Alert(
            "Limelight appears to be disconnected. (TIMEOUT)", AlertType.kError);

    private final Alert tempAlert = new Alert("LL Temp", AlertType.kWarning);

    public Vision(VisionIO io) {
        this.io = io;
        inputs = new VisionInputsAutoLogged();

        io.setCameraOffset();
    }

    /**
     * Estimates the robot pose given the AprilTags on the field
     *
     * @param drive Drive subsystem to adjust odometry of
     */
    public void addPoseEstimate(Drive drive) {
        LimelightHelpers.PoseEstimate estimate = io.estimatePose(drive);

        // if estimate is invalid, don't update pose
        if (estimate == null || estimate.tagCount < FIDUCIAL_TRUST_THRESHOLD) return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, deriveStdDevs(estimate.avgTagDist));
    }

    public void captureClip() {
        LogHelper.logEvent("Vision/CaptureClip");
        io.captureClip();
    }

    /**
     * Derives the standard deviation of background noise in the fiducial pose estimation given the factors from
     * constants
     *
     * @param avgTagDist average tag distance, retrieved from pose estimate
     *
     * @return standard deviations as a 3rd-degree matrix
     */
    private Vector<N3> deriveStdDevs(double avgTagDist) {
        double xy = XY_STDDEV_BASE_METERS + XY_STDDEV_PER_METER * avgTagDist * avgTagDist;
        double theta = Math.toRadians(THETA_STDDEV_BASE_DEG + THETA_STDDEV_PER_METER * avgTagDist * avgTagDist);

        Logger.recordOutput("Vision/TranslationalStdDev", Meters.of(xy));
        Logger.recordOutput("Vision/RotationalStdDev", Radians.of(theta));

        return VecBuilder.fill(xy, xy, theta);
    }

    /**
     * Sets the rotation of the robot, used for LL MT2
     *
     * @param rotation The rotation of the robot
     */
    public void setRotation(Rotation2d rotation) {
        io.setRotation(rotation);
    }

    /**
     * @return true if vision has a target
     */
    public boolean hasTarget() {
        return inputs.hasTarget;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        disconnectedAlert.set(!inputs.isConnected && Constants.CURRENT_MODE != Constants.Mode.SIM);
        tempAlert.set(inputs.sysTemp >= 70.0);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
