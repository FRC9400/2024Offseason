package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class PreLoadLeaveSourceSide extends SequentialCommandGroup{
    double startingPosDegrees;
    double rot_speed = 2;
    public PreLoadLeaveSourceSide(Swerve swerve, Superstructure superstructure){
        startingPosDegrees = DriverStation.getAlliance().equals(Alliance.Blue) ? -60 : 60;
        if(DriverStation.getAlliance().equals(Alliance.Blue)){
            rot_speed *= -1;
        }
        else{
            rot_speed *= 1; 
        }
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(startingPosDegrees)),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_LEFT)),
            new WaitCommand(1.0),
            new RunCommand(() -> swerve.requestDesiredState(3, 0, 0, false, false))
            .withTimeout(1),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, rot_speed, false, false))
            .until(() -> Math.abs(swerve.getGyroPositionDegrees() ) < 3),
            new RunCommand(() -> swerve.requestDesiredState(3, 0, 0, false, false))
            .withTimeout(1)
    
        );    
    }
    

    
}
