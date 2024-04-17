package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TWOPIECE_MID extends SequentialCommandGroup{
    public TWOPIECE_MID(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new WaitCommand(1.25),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE_A)),
            new RunCommand(() -> swerve.requestDesiredState(1.5, 0, 0, true, false))
            .until(() -> superstructure.getIntakeCurrent() > 40).withTimeout(1.5),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            .until(() -> superstructure.getState() == SuperstructureStates.IDLE),
            new RunCommand(() -> swerve.requestDesiredState(-1.5, 0, 0, true, false))
            .withTimeout(1.15),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SPIN_UP_MID)),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            );
        
    }
}
