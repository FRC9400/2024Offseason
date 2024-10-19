package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import com.choreo.lib.Choreo;
import frc.robot.Subsystems.Swerve.Swerve;

public class PreloadAmp extends SequentialCommandGroup {

    public PreloadAmp(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.6304683089256287,6.717046737670898),new Rotation2d(1.088283140601115)))),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }
    
}
