package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.visionConstants;

public class Vision {
    private final PhotonCamera cameraShrek;
    private final PhotonCamera cameraDonkey;
    private Matrix<N3, N1> curStdDevs;
    private final PhotonPoseEstimator poseEstimatorShrek;
    private final PhotonPoseEstimator poseEstimatorDonkey;

    public Vision() {
        cameraShrek = new PhotonCamera("Shrek");
        cameraDonkey = new PhotonCamera("Donkey");

        poseEstimatorShrek = new PhotonPoseEstimator(visionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, visionConstants.kRobotToCamShrek);
        poseEstimatorDonkey = new PhotonPoseEstimator(visionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, visionConstants.kRobotToCamDonkey);
    
        poseEstimatorShrek.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        poseEstimatorDonkey.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    
}
