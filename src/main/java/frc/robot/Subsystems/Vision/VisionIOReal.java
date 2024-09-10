package frc.robot.Subsystems.Vision;

import frc.robot.FieldConstants;
import frc.robot.Constants.visionConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class VisionIOReal implements VisionIO {

    private final PhotonCamera camera1;
    private final PhotonCamera camera2;
    private final PhotonPoseEstimator poseEstimator1;
    private final PhotonPoseEstimator poseEstimator2;

    public VisionIOReal() {
        camera1 = new PhotonCamera(visionConstants.CAMERA_NAME_1);
        camera2 = new PhotonCamera(visionConstants.CAMERA_NAME_2);

        poseEstimator1 = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                visionConstants.ROBOT_TO_CAMERA1);

        poseEstimator2 = new PhotonPoseEstimator(
                FieldConstants.aprilTags,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera2,
                visionConstants.ROBOT_TO_CAMERA2);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        var result1 = camera1.getLatestResult();
        var result2 = camera2.getLatestResult();

        inputs.timestampCamera1 = result1.getTimestampSeconds();
        inputs.latencyCamera1 = result1.getLatencyMillis();
        inputs.numTargetsCamera1 = result1.getTargets().size();
        inputs.targetYawsCamera1 = new double[inputs.numTargetsCamera1];
        inputs.targetDistancesCamera1 = new double[inputs.numTargetsCamera1];

        for (int i = 0; i < inputs.numTargetsCamera1; i++) {
            PhotonTrackedTarget target = result1.getTargets().get(i);
            inputs.targetYawsCamera1[i] = target.getYaw();
            inputs.targetDistancesCamera1[i] = target.getBestCameraToTarget().getTranslation().getNorm();
        }

        inputs.timestampCamera2 = result2.getTimestampSeconds();
        inputs.latencyCamera2 = result2.getLatencyMillis();
        inputs.numTargetsCamera2 = result2.getTargets().size();
        inputs.targetYawsCamera2 = new double[inputs.numTargetsCamera2];
        inputs.targetDistancesCamera2 = new double[inputs.numTargetsCamera2];

        for (int i = 0; i < inputs.numTargetsCamera2; i++) {
            PhotonTrackedTarget target = result2.getTargets().get(i);
            inputs.targetYawsCamera2[i] = target.getYaw();
            inputs.targetDistancesCamera2[i] = target.getBestCameraToTarget().getTranslation().getNorm();
        }

        inputs.numTargets = inputs.numTargetsCamera1 + inputs.numTargetsCamera2;
        inputs.targetYaws = new double[inputs.numTargets];
        inputs.targetDistances = new double[inputs.numTargets];

        System.arraycopy(inputs.targetYawsCamera1, 0, inputs.targetYaws, 0, inputs.numTargetsCamera1);
        System.arraycopy(inputs.targetYawsCamera2, 0, inputs.targetYaws, inputs.numTargetsCamera1, inputs.numTargetsCamera2);

        System.arraycopy(inputs.targetDistancesCamera1, 0, inputs.targetDistances, 0, inputs.numTargetsCamera1);
        System.arraycopy(inputs.targetDistancesCamera2, 0, inputs.targetDistances, inputs.numTargetsCamera1, inputs.numTargetsCamera2);

        Optional<EstimatedRobotPose> estimatedPose1 = poseEstimator1.update();
        Optional<EstimatedRobotPose> estimatedPose2 = poseEstimator2.update();
        var chosenPose = chooseBestPose(estimatedPose1, estimatedPose2);

        if (chosenPose.isPresent()) {
            var pose = chosenPose.get().estimatedPose;
            inputs.chosenPoseTranslation[0] = pose.getX();
            inputs.chosenPoseTranslation[1] = pose.getY();
            inputs.chosenPoseTranslation[2] = pose.getZ();

            var rotation = pose.getRotation();
            inputs.chosenPoseRotation[0] = rotation.getX(); // roll
            inputs.chosenPoseRotation[1] = rotation.getY(); // pitch
            inputs.chosenPoseRotation[2] = rotation.getZ(); // yaw
        }
    }

    private Optional<EstimatedRobotPose> chooseBestPose(Optional<EstimatedRobotPose> pose1,
                                                        Optional<EstimatedRobotPose> pose2) {
        if (pose1.isPresent() && pose2.isPresent()) {
            double ambiguity1 = pose1.get().targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average().orElse(1.0);
            double ambiguity2 = pose2.get().targetsUsed.stream()
                    .mapToDouble(t -> t.getPoseAmbiguity())
                    .average().orElse(1.0);
            return ambiguity1 < ambiguity2 ? pose1 : pose2;
        } else {
            return pose1.isPresent() ? pose1 : pose2;
        }
    }
}
