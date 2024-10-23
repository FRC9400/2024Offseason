package frc.robot.autons.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import com.choreo.lib.Choreo;

public class TestAuto extends SequentialCommandGroup{
     public TestAuto(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.397372841835022,5.555154800415039),new Rotation2d(0))))
           // swerve.runChoreoTrajStandard(Choreo.getTrajectory("new1"))

        );}
    
}
