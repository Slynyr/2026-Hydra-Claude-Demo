package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LimelightHelpers.PoseEstimate;
import frc.robot.util.LimelightHelpers.RawFiducial;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import static frc.robot.subsystems.vision.VisionConstants.FIDUCIAL_TRUST_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.OFFSET_FROM_ROBOT_ORIGIN;

/**
 * @author Logan Dhillon, FRC 5409 Chargers
 */
public class VisionIOSim implements VisionIO {
    private final SwerveDriveSimulation sim_drive;
    private final VisionSystemSim       sim_vision;
    private final PhotonPoseEstimator   poseEstimator;
    private final PhotonCamera          cam;

    private boolean hasTarget = false;

    private static volatile VisionIOSim globalThis = null;

    public VisionIOSim(SwerveDriveSimulation sim) {
        if (globalThis != null) throw new IllegalArgumentException("You cannot create more than one VisionIOSim!");

        this.sim_drive = sim;
        this.sim_vision = new VisionSystemSim("PV-simsystem");

        SimCameraProperties prop = new SimCameraProperties();
        prop.setCalibration(1280, 800, Rotation2d.fromDegrees(97.6524449259));
        prop.setCalibError(0.25, 0.08);
        prop.setFPS(20);
        prop.setAvgLatencyMs(35);
        prop.setLatencyStdDevMs(5);

        this.cam = new PhotonCamera("PV-simcamera");
        PhotonCameraSim cameraSim = new PhotonCameraSim(cam, prop);

        try {
            AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.kDefaultField.m_resourceFile);
            sim_vision.addAprilTags(tagLayout);

            poseEstimator = new PhotonPoseEstimator(tagLayout, OFFSET_FROM_ROBOT_ORIGIN);
        } catch (IOException e) {
            throw new RuntimeException("Failed to load PV AprilTag layout '" + AprilTagFields.kDefaultField + "': " +
                                       AprilTagFields.kDefaultField.m_resourceFile);
        }

        sim_vision.addCamera(cameraSim, OFFSET_FROM_ROBOT_ORIGIN);

        globalThis = this;
    }

    /**
     * builds {@link RawFiducial} data from a {@link PhotonTrackedTarget}
     *
     * @param t photonlib target data
     *
     * @return output fiducial data
     */
    private static RawFiducial buildFiducialData(PhotonTrackedTarget t) {
        return new RawFiducial(
                t.fiducialId,
                t.bestCameraToTarget.getX(),
                t.getBestCameraToTarget().getY(),
                t.area,
                t.bestCameraToTarget.getTranslation().getNorm(),
                t.bestCameraToTarget.getTranslation()
                                    .plus(OFFSET_FROM_ROBOT_ORIGIN.getTranslation())
                                    .getNorm(),
                t.poseAmbiguity);
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.isConnected = true;
        inputs.fps = 20;
        inputs.prxLatency = 35;
        inputs.hasTarget = hasTarget;
    }

    @Override
    public PoseEstimate estimatePose(Drive drive) {
        List<PhotonPipelineResult> results = cam.getAllUnreadResults();
        if (results.isEmpty()) return null;

        PhotonPipelineResult latest = results.get(results.size() - 1);
        Optional<EstimatedRobotPose> pe = poseEstimator.estimateCoprocMultiTagPose(latest);

        if (pe.isEmpty()) {
            hasTarget = false;
            return null;
        }

        hasTarget = true;

        //noinspection SizeReplaceableByIsEmpty
        if (pe.get().targetsUsed.size() < FIDUCIAL_TRUST_THRESHOLD) return null;

        return new PoseEstimate(
                pe.get().estimatedPose.toPose2d(),
                pe.get().timestampSeconds,
                0,
                pe.get().targetsUsed.size(),
                0,
                0,
                0,
                pe.get().targetsUsed.stream()
                                    .map(VisionIOSim::buildFiducialData)
                                    .toArray(RawFiducial[]::new),
                false);
    }

    @Override
    public void simulationPeriodic() {
        sim_vision.update(sim_drive.getSimulatedDriveTrainPose());
        Logger.recordOutput("Simulation/Vision/RobotPose", sim_vision.getRobotPose());
    }
}

