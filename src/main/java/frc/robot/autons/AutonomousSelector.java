package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;



public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            DO_NOTHING,
            PRELOAD_AMP,
            PRELOAD_MID,
            PRELOAD_SOURCE,
            FOUR_NOTE_MID,
            FOUR_NOTE_AMP,
            FOUR_NOTE_SOURCE,
            TWO_NOTE_MID

        };
    
    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        
        autonomousSelector.setDefaultOption(
            "Do Nothing", modes.DO_NOTHING
        );

        autonomousSelector.addOption("Preload Amp", modes.PRELOAD_AMP);

        autonomousSelector.addOption("Preload Mid", modes.PRELOAD_MID);

        autonomousSelector.addOption("Preload Source", modes.PRELOAD_SOURCE);

        autonomousSelector.addOption("Four Note Mid", modes.FOUR_NOTE_MID);

        autonomousSelector.addOption("Two Note Mid", modes.TWO_NOTE_MID);

        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}