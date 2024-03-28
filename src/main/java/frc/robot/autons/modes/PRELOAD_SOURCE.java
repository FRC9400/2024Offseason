package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class PRELOAD_SOURCE extends SequentialCommandGroup{
    
    public PRELOAD_SOURCE(Swerve swerve, Superstructure superstructure){
        SuperstructureStates shootSide = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? SuperstructureStates.SPIN_UP_RIGHT : SuperstructureStates.SPIN_UP_RIGHT;
        double gyroStartAngle = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? -60 :  60;
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(gyroStartAngle)),
            new InstantCommand(() -> superstructure.setState(shootSide))
        );
    
    }
}
