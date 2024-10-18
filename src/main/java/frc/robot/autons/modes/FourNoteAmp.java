package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import com.choreo.lib.Choreo;
import frc.robot.Subsystems.Swerve.Swerve;

public class FourNoteAmp extends SequentialCommandGroup {

    public FourNoteAmp(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 0 : 0)),
            new InstantCommand(() -> swerve.resetPose(new Pose2d(new Translation2d(0.6304683089256287,6.717046737670898),new Rotation2d(1.088283140601115)))),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteAmpA"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteAmpB"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteAmpC"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteAmpD"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteAmpE"), true),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );
    }
    
}
