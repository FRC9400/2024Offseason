package frc.robot.autons.modes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import com.choreo.lib.Choreo;
import frc.robot.Subsystems.Swerve.Swerve;

public class TestAuto extends SequentialCommandGroup {

    public TestAuto(Swerve swerve, Superstructure superstructure){
        addRequirements(swerve, superstructure);
        addCommands(
            new InstantCommand(() -> swerve.setGyroStartingPosition(0)),
            new InstantCommand(() -> superstructure.requestPreShoot())//,
            //swerve.runChoreoTraj(Choreo.getTrajectory("TestPath"),true)
        );
    }
    
}
