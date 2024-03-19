package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class TwoPieceMidHardCoded extends SequentialCommandGroup{
    public TwoPieceMidHardCoded(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID)),
            new WaitCommand(1.0),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.INTAKE)),
            new RunCommand(() -> swerve.requestDesiredState(1.5, 0, 0, false, false))
            .until(() -> superstructure.getState() == SuperstructureStates.IDLE),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, false, false))
            .withTimeout(0.5),
            new RunCommand(() -> swerve.requestDesiredState(-1.5, 0, 0, false, false))
            .withTimeout(1.43),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.SHOOT_MID))
            );
        
    }
}
