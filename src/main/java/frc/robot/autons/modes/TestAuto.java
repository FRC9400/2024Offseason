package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

import frc.robot.Subsystems.Swerve.Swerve;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new InstantCommand(() -> superstructure.setState(SuperstructureStates.PREPARE_SHOOT))
            //swerve.runChoreoTraj(Choreo.getTrajectory("TestPath"),true)
        );
    }
    
}
