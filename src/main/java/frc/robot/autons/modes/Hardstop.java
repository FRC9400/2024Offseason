package frc.robot.autons.modes;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;
import frc.robot.Subsystems.Swerve.Swerve;

public class Hardstop extends SequentialCommandGroup{
    Timer timer = new Timer();
    public Hardstop(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> timer.reset()),
            new InstantCommand(() -> timer.start()),
            new RunCommand(() -> swerve.requestDesiredState(-1, 0, 0, true, false))
            .until(() -> swerve.getDriveCurrent() > 90 ).withTimeout(1.5),
            new InstantCommand(() -> timer.reset()),
            new RunCommand(() -> swerve.requestDesiredState(0, 0, 0, true, false))
            
            );
        
    }
}

