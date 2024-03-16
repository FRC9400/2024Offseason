package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.modes.preloadLeft;
import frc.robot.autons.modes.preloadMid;
import frc.robot.autons.modes.preloadRight;

public class AutonomousSelector {
    private SendableChooser<SequentialCommandGroup> autonomousSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        autonomousSelector.setDefaultOption(
            "PRELOAD_MID", new preloadMid(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_LEFT", new preloadLeft(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_RIGHT", new preloadRight(swerve, superstructure));

        


    }
    
}
