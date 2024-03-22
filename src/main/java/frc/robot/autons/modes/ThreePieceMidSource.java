package frc.robot.autons.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Turn;

public class ThreePieceMidSource extends SequentialCommandGroup{
    Timer timer = new Timer();
    public ThreePieceMidSource(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            // this is rlly fuckin bad lol
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new WaitCommand(1.25),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)),
            new RunCommand(() -> swerve.requestDesiredState(1.5, 0, 0, true, false))
            .until(() -> superstructure.getIntakeCurrent() > 40).withTimeout(1.5),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(0.5),
            new InstantCommand(() -> timer.reset()),
            new InstantCommand(() -> timer.start()),
            new RunCommand(() -> swerve.requestDesiredState(-1.5, 0, 0, true, false))
            .withTimeout(1),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(1.25),
            new WaitCommand(1.25),


            new RunCommand(() -> swerve.requestDesiredState(1.5, 0, 0, true, false))
            .withTimeout(1.1),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(0.25),
            new Turn(swerve,  new Rotation2d(-Math.PI/2.0))
            .until(() -> Math.abs(swerve.getGyroPositionDegrees() - (-90)) < 3),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)),
            new WaitCommand(0.25),
            new RunCommand(() -> swerve.requestDesiredState(0, -1.5, 0, true, false))
            .until(() -> superstructure.getIntakeCurrent() > 40).withTimeout(1.5),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(0.5),
            new RunCommand(() -> swerve.requestDesiredState(0, 1.5, 0, true, false))
            .withTimeout(1.25),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(0.25),
            new Turn(swerve, new Rotation2d(0))
            .until(() ->Math.abs( swerve.getGyroPositionDegrees()) < 3),
            new RunCommand(() -> swerve.requestDesiredState(-1.5, 0, 0, true, false))
            .until(() -> swerve.getDriveCurrent() > 90 ).withTimeout(1),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .withTimeout(0.25),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID))

            
            );


        
    }
}

