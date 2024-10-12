package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import com.choreo.lib.Choreo;
import frc.robot.Subsystems.Swerve.Swerve;

public class NoteMid extends SequentialCommandGroup {

    public NoteMid(Swerve swerve, Superstructure superstructure){/* 
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteMidA"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteMidB"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteMidC"), true),
            new InstantCommand(() -> superstructure.requestPreShoot()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteMidD"), true),
            new InstantCommand(() -> superstructure.requestIntake()),
            new WaitUntilCommand(superstructure.getState()==SuperstructureStates.IDLE),
            swerve.runChoreoTraj(Choreo.getTrajectory("4NoteMidE"), true),
            new InstantCommand(() -> superstructure.requestPreShoot())
        );*/
    }
    
}
