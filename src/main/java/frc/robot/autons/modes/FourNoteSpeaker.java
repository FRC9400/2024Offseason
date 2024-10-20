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

public class FourNoteSpeaker extends SequentialCommandGroup {

    public FourNoteSpeaker(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.6813831329345703,4.373771667480469),new Rotation2d(-1.0679532436948613)))),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerA"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerB"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerC"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerD"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerE"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitCommand(2),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteSpeakerF"), true),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }
    
}
