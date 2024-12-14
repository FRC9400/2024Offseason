package frc.robot;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.visionConstants;

public class Vision {
    private final PhotonCamera camera;
    private Matrix<N3, N1> curStdDevs;
    private final PhotonPoseEstimator poseEstimator;

    public Vision(int id) {
        camera = new PhotonCamera(visionConstants.name[id]);

        poseEstimator = new PhotonPoseEstimator(visionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, visionConstants.kRobotToCam[id]);
        
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(){
        Optional<EstimatedRobotPose> poseEst = Optional.empty();
        for(var change : camera.getAllUnreadResults()){
            poseEst = poseEstimator.update(change);
            updateStdDevs(poseEst, change.getTargets());
        }
        return poseEst;
    }

    private void updateStdDevs(Optional<EstimatedRobotPose> robotPose, List<PhotonTrackedTarget> targets){
        if(robotPose.isEmpty()){
            curStdDevs = visionConstants.kSingleTagStdDevs;
        }
        else {
            var estStdDevs = visionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            for(var tgt : targets){
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.fiducialId);
                if(tagPose.isEmpty()){continue;}
                numTags++;
                avgDist+=tagPose.get().toPose2d().getTranslation().getDistance(robotPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if(numTags==0){
                curStdDevs=visionConstants.kSingleTagStdDevs;
            }
            else{
                avgDist/=numTags;
                if(numTags>1){estStdDevs=visionConstants.kMultiTagStdDevs;}
                if(numTags==1&&avgDist>4){
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE,Double.MAX_VALUE);
                }
                else{estStdDevs = estStdDevs.times(1+(avgDist*avgDist)/30);}//values copied(idk if they should be adjusted)
                curStdDevs = estStdDevs;
            }
        }
    }

    public Matrix<N3,N1> getEstimationStdDevs(){
        return curStdDevs;
    }
}
