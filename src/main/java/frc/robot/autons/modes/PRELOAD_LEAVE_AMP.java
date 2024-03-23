package frc.robot.autons.modes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Turn;

public class PRELOAD_LEAVE_AMP extends SequentialCommandGroup{
    double startingPosDegrees;
    SuperstructureStates shootSide = DriverStation.getAlliance().equals(Alliance.Blue) ? SuperstructureStates.SPIN_UP_LEFT : SuperstructureStates.SPIN_UP_LEFT;
    public PRELOAD_LEAVE_AMP(Swerve swerve, Superstructure superstructure){
         startingPosDegrees = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? 60 : -60;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(startingPosDegrees)),
            new InstantCommand(() -> superstructure.setState(shootSide)),
            new WaitCommand(1.0),
            new RunCommand(() -> swerve.requestDesiredState(0, 1.5, 0, false, false))
            .withTimeout(1.5),
            new Turn(swerve,  new Rotation2d(0))
            .until(() ->Math.abs( swerve.getGyroPositionDegrees()) < 3),
            new RunCommand(() -> swerve.requestDesiredState(1.5, 0, 0, false, false))
            .withTimeout(1),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, false, false))
    
        );    
    }
    

    
}
