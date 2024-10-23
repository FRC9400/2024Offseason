package frc.robot.autons.modes;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import com.choreo.lib.Choreo;
import frc.robot.Subsystems.Swerve.Swerve;

public class FourNoteMid extends SequentialCommandGroup {

    public FourNoteMid(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(1.3615734577178962,5.555154800415039),new Rotation2d(0)))),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("4NoteMidA")),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("4NoteMidB")),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(2),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("4NoteMidC")),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("4NoteMidD")),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(2),
            swerve.runChoreoTrajStandard(Choreo.getTrajectory("4NoteMidE")),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }
    
}