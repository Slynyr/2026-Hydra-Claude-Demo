package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

public class VisionConstants {
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

    // TODO: update these to camera offset (done) - Double check hoodExtension
    public static final Transform3d OFFSET_FROM_ROBOT_ORIGIN = new Transform3d(
            new Translation3d(Inches.of(-11.639), Inches.of(-1.623), Inches.of(20.027)),
            new Rotation3d(Degrees.of(0.0), Degrees.of(5.0), Degrees.of(180.0))
    );

    public static final double CAPTURE_VIDEO_DURATION = 25;
}
