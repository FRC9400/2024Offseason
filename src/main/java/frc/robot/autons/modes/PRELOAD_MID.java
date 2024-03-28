package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class PRELOAD_MID extends SequentialCommandGroup{
    
    public PRELOAD_MID(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID))
        );
    
    }
}
