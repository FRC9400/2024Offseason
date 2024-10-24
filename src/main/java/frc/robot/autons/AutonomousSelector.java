package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;



public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
        public enum modes{
            TEST,
            PRELOAD_AMP,
            PRELOAD_MID,
            PRELOAD_SOURCE,
            TUNING
        };
    
    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        
        autonomousSelector.setDefaultOption(
            "TEST", modes.TEST
        );

        autonomousSelector.addOption("HELp", modes.TEST);

        autonomousSelector.addOption("Preload Amp", modes.PRELOAD_AMP);

        autonomousSelector.addOption("Preload Mid", modes.PRELOAD_MID);

        autonomousSelector.addOption("Preload Source", modes.PRELOAD_SOURCE);

        autonomousSelector.addOption("Tune Choreo", modes.TUNING);

        SmartDashboard.putData("Auto Choices", autonomousSelector);
    }

    public modes get(){
        return autonomousSelector.getSelected();
    }
}