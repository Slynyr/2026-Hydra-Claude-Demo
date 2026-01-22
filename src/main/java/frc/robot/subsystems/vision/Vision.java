package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class Vision extends SubsystemBase {
    public static final String PRIMARY_CAM_NAME = "limelight";

    public static final int FIDUCIAL_TRUST_THRESHOLD = 1;
    public static final int DISCONNECTION_TIMEOUT    = 5;
    public static final int THROTTLE_DISABLED        = 200;

    /**
     * If enabled, the {@link VisionIOLimelight} will use
     * {@link frc.robot.subsystems.vision.VisionIOLimelight.IMUMode#FUSED} estimations if the detected AprilTag is
     * strong enough.
     */
    public static final boolean ALLOW_FUSED_GYRO_ESTIMATIONS = true;

    /**
     * 1σ translation error at zero distance (meters)
     */
    public static final double XY_STDDEV_BASE_METERS  = 0.10;
    /**
     * Additional translation error per meter of tag distance
     */
    public static final double XY_STDDEV_PER_METER    = 0.05;
    /**
     * 1σ rotation error at zero distance (deg)
     */
    public static final double THETA_STDDEV_BASE_DEG  = 2.0;
    /**
     * Additional rotation error per meter of tag distance (deg / meter)
     */
    public static final double THETA_STDDEV_PER_METER = 1.5;

    // TODO: update these to camera offset
    public static final Transform3d OFFSET_FROM_ROBOT_ORIGIN = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0));

    private final VisionIO               io;
    private final VisionInputsAutoLogged inputs;

    private final Alert disconnectedAlert = new Alert(
            "Limelight appears to be disconnected. (TIMEOUT)", Alert.AlertType.kError);

    private final Alert tempAlert = new Alert("LL Temp", Alert.AlertType.kWarning);

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
        if (estimate == null || estimate.tagCount < Vision.FIDUCIAL_TRUST_THRESHOLD) return;

        drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, deriveStdDevs(estimate.avgTagDist));
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
        double xy = XY_STDDEV_BASE_METERS + XY_STDDEV_PER_METER * avgTagDist;
        // TODO: this should be tested
        return VecBuilder.fill(
                xy, xy,
                Math.toRadians(THETA_STDDEV_BASE_DEG + THETA_STDDEV_PER_METER * avgTagDist)
        );
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

        disconnectedAlert.set(!inputs.isConnected && Constants.currentMode != Constants.Mode.SIM);
        tempAlert.set(inputs.sysTemp >= 70.0);
    }

    @Override
    public void simulationPeriodic() {
        io.simulationPeriodic();
    }
}
