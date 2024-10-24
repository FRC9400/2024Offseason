package frc.robot.autons.modes;

import com.choreo.lib.Choreo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

import frc.robot.Subsystems.Swerve.Swerve;

public class ChoreoTuning extends SequentialCommandGroup {

    public ChoreoTuning(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.3257739543914795,5.590954303741455),new Rotation2d(0)))),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("ChoreoTuning"))
        );
    }
    
}
