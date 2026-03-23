package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.vision.VisionConstants.MINIMUM_TARGET_AREA;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class VisionIOLimelight implements VisionIO {
    private final String limelightName;

    private double  lastPrxLatency     = 0;
    private double  disconnectedFrames = 0;
    private boolean forceFusedIMU      = false;

    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;

        new Trigger(DriverStation::isDisabled)
                .onTrue(setIMUMode(IMUMode.FUSED).alongWith(setThrottle(VisionConstants.THROTTLE_DISABLED)))
                .onFalse(setIMUMode(IMUMode.INTERNAL).alongWith(setThrottle(0)));

       SmartDashboard.putData("Throttle-0 LL", setThrottle(0).ignoringDisable(true));
       SmartDashboard.putData("Throttle-100 LL", setThrottle(100).ignoringDisable(true));
       SmartDashboard.putData("Fused LL", setIMUMode(IMUMode.FUSED).ignoringDisable(true));
       SmartDashboard.putData("Internal LL", setIMUMode(IMUMode.INTERNAL).ignoringDisable(true));

        // disable throttle
        LimelightHelpers.SetThrottle(limelightName, VisionConstants.THROTTLE_DISABLED);
        LimelightHelpers.SetIMUMode(limelightName, IMUMode.FUSED.id);

        LimelightHelpers.SetFiducialIDFiltersOverride(limelightName, VisionConstants.TAG_FILTER);

        // enable rewind
        LimelightHelpers.setRewindEnabled(limelightName, true);

        // forward ports to network
        forwardLimelightPorts();
    }

    private Command setThrottle(int throttle) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetThrottle(limelightName, throttle)
        ).ignoringDisable(true);
    }

    private Command setIMUMode(IMUMode mode) {
        return Commands.runOnce(
                () -> LimelightHelpers.SetIMUMode(limelightName, mode.id)
        ).ignoringDisable(true);
    }

    public void captureClip() {
        LimelightHelpers.triggerRewindCapture(limelightName, VisionConstants.CAPTURE_VIDEO_DURATION);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.tx = LimelightHelpers.getTX(limelightName);
        inputs.ty = LimelightHelpers.getTY(limelightName);
        inputs.ta = LimelightHelpers.getTA(limelightName);
        inputs.hasTarget = LimelightHelpers.getTV(limelightName);
        inputs.targetId = LimelightHelpers.getFiducialID(limelightName);
        inputs.imgLatency = LimelightHelpers.getLatency_Capture(limelightName);
        inputs.prxLatency = LimelightHelpers.getLatency_Pipeline(limelightName);

        double[] hw = LimelightHelpers.getLimelightNTTableEntry(limelightName, "hw")
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
            inputs.isConnected = ++disconnectedFrames <= VisionConstants.DISCONNECTION_TIMEOUT;
        }

        lastPrxLatency = inputs.prxLatency;
    }

    @Override
    public void setCameraOffset() {
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getMeasureX().in(Meters),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getMeasureY().in(Meters),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getTranslation().getMeasureZ().in(Meters),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureX().in(Units.Degrees),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureY().in(Units.Degrees),
                VisionConstants.OFFSET_FROM_ROBOT_ORIGIN.getRotation().getMeasureZ().in(Units.Degrees));
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
     * {@link LimelightHelpers.PoseEstimate} using WPI Blue MegaTag2.
     *
     * @param drive Drive subsystem to get rotation from
     */
    @Override
    public LimelightHelpers.PoseEstimate estimatePose(Drive drive) {
        ChassisSpeeds speeds = drive.getChassisSpeeds();
        Rotation2d yaw = drive.getRotation();

        if (LimelightHelpers.getTA(limelightName) < MINIMUM_TARGET_AREA.getAsDouble()) {
            Logger.recordOutput("Vision/PoseEstimateStatus", "REJECT");
            return null;
        }

        if (VisionConstants.ALLOW_FUSED_GYRO_ESTIMATIONS &&
             DriverStation.isEnabled() && // enabled
             LimelightHelpers.getTA(limelightName) >= 1.5 && // confident tag
             Math.abs(speeds.vxMetersPerSecond) < 0.1 && // bot not moving
             Math.abs(speeds.vyMetersPerSecond) < 0.1 &&
             Math.abs(speeds.omegaRadiansPerSecond) < 0.1) {
            LimelightHelpers.SetIMUMode(limelightName, IMUMode.FUSED.id); // use fused IMU
            // ...and get estimate for bot pose in FUSED mode
            yaw = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).pose.getRotation();
            logGryoMode(IMUMode.FUSED);
            Logger.recordOutput("Vision/PoseEstimateStatus", "MEGA_TAG_1");
            LimelightHelpers.SetIMUMode(limelightName, IMUMode.FUSED.id);
        } else {
            logGryoMode(IMUMode.EXTERNAL);
            LimelightHelpers.SetIMUMode(limelightName, IMUMode.EXTERNAL.id);
            Logger.recordOutput("Vision/PoseEstimateStatus", "MEGA_TAG_2");
        }

        Logger.recordOutput("Vision/ForceFusedIMU", forceFusedIMU);

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                yaw.getDegrees(), 0, 0, 0, 0, 0);

        return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    }

    public void setForceFusedIMU(boolean forceFusedIMU) {
        this.forceFusedIMU = forceFusedIMU;
    }

    @Override
    public void setRotation(Rotation2d rotation) {
        // use fused IMU when setting rotation
        LimelightHelpers.SetIMUMode(limelightName, IMUMode.FUSED.id);
        LimelightHelpers.SetRobotOrientation(
                limelightName, rotation.getDegrees(),
                0, 0, 0, 0, 0);
    }

    /**
     * Forward limelight ports (5800-5809) so it can be used over USB
     */
    public void forwardLimelightPorts() {
        System.out.printf("Forwarding limelight ports 5800..5909 on %s.local", limelightName);
        for (int i = 5800; i <= 5809; i++)
            PortForwarder.add(i, limelightName + ".local", i);
    }

    public enum IMUMode {
        EXTERNAL(0),
        FUSED(1),
        INTERNAL(2);

        public final int id;

        IMUMode(int num) {
            id = num;
        }
    }
}

