package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class PRELOAD_AMP extends SequentialCommandGroup{
    SuperstructureStates shootSide = DriverStation.getAlliance().equals(Alliance.Blue) ? SuperstructureStates.SPIN_UP_LEFT : SuperstructureStates.SPIN_UP_RIGHT;
    public PRELOAD_AMP(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> superstructure.setState(shootSide))
        );
    
    }
}
