package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.SuperstructureManager;
import frc.robot.subsystems.Drive.Drivetrain;
import frc.robot.util.LimelightLib.LimelightHelpers;
import frc.robot.util.LimelightLib.LimelightHelpers.LimelightResults;

import java.util.Optional;

public final class Vision extends SubsystemBase {
    public enum Goal {
        SPEAKER, AMP, TRAP, SOURCE, NOTE
    }

    private Optional<Goal> state;

    private final Drivetrain m_drive;
    private final SuperstructureManager m_superstructure;

    private final PhotonCamera backCamera = new PhotonCamera(VisionConstants.kBackCameraName);
    private final PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), PoseStrategy.CLOSEST_TO_REFERENCE_POSE, backCamera,
            VisionConstants.kBackCameraToRobot);

    public Vision(Drivetrain drive, SuperstructureManager superstructure) {
        this.m_drive = drive;
        this.m_superstructure = superstructure;

        LimelightHelpers.setPipelineIndex(VisionConstants.kFrontCameraName,
                VisionConstants.kFrontAprilTagPipeline);
    }

    @Override
    public void periodic() {
        var back = this.backCamera.getLatestResult();
        var front = LimelightHelpers.getLatestResults(VisionConstants.kFrontCameraName);

        updatePose(back, front);

        if (this.state.isPresent()) {
            switch (this.state.get()) {
                case AMP:
                    break;
                case NOTE:
                    break;
                case SOURCE:
                    break;
                case SPEAKER:
                    break;
                case TRAP:
                    break;
            }
        }
    }

    public Command track(Goal desiredTarget) {
        return this.startEnd(() -> {
            this.state = Optional.of(desiredTarget);
            if (desiredTarget == Goal.NOTE) {
                LimelightHelpers.setPipelineIndex(VisionConstants.kFrontCameraName, VisionConstants.kFrontNNPipeline);
            }
        }, () -> {
            this.state = Optional.empty();
            if (desiredTarget == Goal.NOTE) {
                LimelightHelpers.setPipelineIndex(VisionConstants.kFrontCameraName,
                        VisionConstants.kFrontAprilTagPipeline);
            }
        });
    }

    private void updatePose(PhotonPipelineResult back, LimelightResults front) {
        if (back.hasTargets() && back.getMultiTagResult().estimatedPose.isPresent) {
            photonPoseEstimator.setReferencePose(m_drive.getPose());
            photonPoseEstimator.update().ifPresent((EstimatedRobotPose result) -> {
                this.m_drive.addVisionMeasurement(result.estimatedPose.toPose2d(), result.timestampSeconds);
            });
        }
        if (front.targetingResults.valid
                && front.targetingResults.pipelineID == VisionConstants.kFrontAprilTagPipeline) {
            this.m_drive.addVisionMeasurement(front.targetingResults.getBotPose3d_wpiBlue().toPose2d(),
                    Timer.getFPGATimestamp() - getLatency(front));
        }
    }

    private Transform2d note(LimelightResults front) {
        if (front.targetingResults.valid
                && front.targetingResults.pipelineID == VisionConstants.kFrontNNPipeline) {
                    var targets = front.targetingResults.targets_Detector;
                    if (targets.length == 1) {

                    }
        }
    }

    private double getLatency(LimelightResults front) {
        return (front.targetingResults.latency_capture / 1000)
                    + (front.targetingResults.latency_jsonParse / 1000)
                    + (front.targetingResults.latency_pipeline / 1000);
    }
}