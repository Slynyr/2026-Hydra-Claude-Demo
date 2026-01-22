package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class VisionIOLimelight implements VisionIO {
    private double lastPrxLatency     = 0;
    private double disconnectedFrames = 0;

    public VisionIOLimelight() {
        new Trigger(DriverStation::isDisabled)
                .onTrue(
                        Commands.parallel(
                                setIMUMode(IMUMode.FUSED),
                                setThrottle(Vision.THROTTLE_DISABLED)
                        )
                ).onFalse(
                        Commands.parallel(
                                setIMUMode(IMUMode.INTERNAL),
                                setThrottle(0)
                        )
                );

        // TODO: add throttle debug commands, i'll do this when my FRC Checkmate library is done
//        DebugCommand.register("No Throttle LL", setThrottle(0));
//        DebugCommand.register("Throttle LL", setThrottle(Vision.THROTTLE_DISABLED));
//        DebugCommand.register("Fused LL", setIMUMode(IMUMode.FUSED));
//        DebugCommand.register("Internal LL", setIMUMode(IMUMode.INTERNAL));

        LimelightHelpers.SetThrottle(Vision.PRIMARY_CAM_NAME, Vision.THROTTLE_DISABLED);
        LimelightHelpers.SetIMUMode(Vision.PRIMARY_CAM_NAME, IMUMode.FUSED.ID);
    }

    private Command setThrottle(int throttle) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetThrottle(Vision.PRIMARY_CAM_NAME, throttle)
        ).ignoringDisable(true);
    }

    private Command setIMUMode(IMUMode mode) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetIMUMode(Vision.PRIMARY_CAM_NAME, mode.ID)
        ).ignoringDisable(true);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(Vision.PRIMARY_CAM_NAME);
        inputs.ty = LimelightHelpers.getTY(Vision.PRIMARY_CAM_NAME);
        inputs.ta = LimelightHelpers.getTA(Vision.PRIMARY_CAM_NAME);
        inputs.hasTarget = LimelightHelpers.getTV(Vision.PRIMARY_CAM_NAME);
        inputs.targetId = LimelightHelpers.getFiducialID(Vision.PRIMARY_CAM_NAME);
        inputs.imgLatency = LimelightHelpers.getLatency_Capture(Vision.PRIMARY_CAM_NAME);
        inputs.prxLatency = LimelightHelpers.getLatency_Pipeline(Vision.PRIMARY_CAM_NAME);

        double[] hw = LimelightHelpers.getLimelightNTTableEntry(Vision.PRIMARY_CAM_NAME, "hw")
                                      .getDoubleArray(new double[]{ 0.0, 0.0, 0.0, 0.0 });

        try {
            inputs.fps = hw[3];
            inputs.cpuTemp = hw[2];
            inputs.ramUsage = hw[1];
            inputs.sysTemp = hw[0];
        } catch (Exception e) {
            inputs.fps = -1.0;
            inputs.cpuTemp = -1.0;
            inputs.ramUsage = -1.0;
            inputs.sysTemp = -1.0;
        }

        // check if disconnected by comparing prx latency
        if (lastPrxLatency != inputs.prxLatency) {
            disconnectedFrames = 0;
            inputs.isConnected = true;
        } else {
            inputs.isConnected = ++disconnectedFrames <= Vision.DISCONNECTION_TIMEOUT;
        }

        lastPrxLatency = inputs.prxLatency;
    }

    @Override
    public void setCameraOffset() {
        LimelightHelpers.setCameraPose_RobotSpace(
                Vision.PRIMARY_CAM_NAME,
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getX(),
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getY(),
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getZ(),
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureX().in(Units.Degrees),
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureY().in(Units.Degrees),
                Vision.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureZ().in(Units.Degrees));
    }

    /**
     * Logs the mode used for the bot's {@link Rotation2d} (gyro) in the pose estimator
     *
     * @param mode mode used for estimating rotation
     */
    private void logGryoMode(IMUMode mode) {
        Logger.recordOutput("Vision/Gyro-Mode", mode.name());
    }

    /**
     * Updates the current robot orientation in {@link LimelightHelpers}, then gets the
     * {@link frc.robot.util.LimelightHelpers.PoseEstimate} using WPI Blue MegaTag2.
     *
     * @param drive Drive subsystem to get rotation from
     */
    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        Rotation2d yaw = drive.getRotation();

        if (Vision.ALLOW_FUSED_GYRO_ESTIMATIONS &&
            DriverStation.isEnabled() && // enabled
            LimelightHelpers.getTA(Vision.PRIMARY_CAM_NAME) >= 1.5 && // confident tag
            Math.abs(speeds.vxMetersPerSecond) < 0.1 && // bot not moving
            Math.abs(speeds.vyMetersPerSecond) < 0.1 &&
            Math.abs(speeds.omegaRadiansPerSecond) < 0.1) {
            LimelightHelpers.SetIMUMode(Vision.PRIMARY_CAM_NAME, IMUMode.FUSED.ID); // use fused IMU
            // ...and get estimate for bot pose in FUSED mode
            yaw = LimelightHelpers.getBotPoseEstimate_wpiBlue(Vision.PRIMARY_CAM_NAME).pose.getRotation();
            logGryoMode(IMUMode.FUSED);
        } else {
            logGryoMode(IMUMode.EXTERNAL);
        }

        LimelightHelpers.SetRobotOrientation(
                Vision.PRIMARY_CAM_NAME,
                yaw.getDegrees(), 0, 0, 0, 0, 0);

        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Vision.PRIMARY_CAM_NAME);
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        // use fused IMU when setting rotation
        LimelightHelpers.SetIMUMode(Vision.PRIMARY_CAM_NAME, IMUMode.FUSED.ID);
        LimelightHelpers.SetRobotOrientation(
                Vision.PRIMARY_CAM_NAME, rotation.getDegrees(),
                0, 0, 0, 0, 0);
    }

    /**
     * Forward limelight ports (5800-5809) so it can be used over USB
     */
    public static void forwardLimelightPorts() {
        for (int i = 5800; i <= 5809; i++)
            PortForwarder.add(i, Vision.PRIMARY_CAM_NAME + ".local", i);
    }

    public enum IMUMode {
        EXTERNAL(0),
        FUSED(1),
        INTERNAL(2);

        public final int ID;

        IMUMode(int num) {
            ID = num;
        }
    }
}

