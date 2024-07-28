package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.Swerve;



public class AutonomousSelector {
    private SendableChooser<modes> autonomousSelector = new SendableChooser<modes>();
    String mode;
     public enum modes{
        MID_PRELOAD,
        MID_2p_B,
        MID_4p_B_C_A,
        TESTABLE_MID_FOUR_HALF_PIECE,
        TESTABLE_MID_FOUR_RACE_THREE,
        MID_PRELOAD_LEAVE,
        AMP_PRELOAD,
        AMP_PRELOAD_LEAVE,
        TESTABLE_AMP_TWO_PIECE_ONE,
        TESTABLE_SOURCE_TWO_PIECE,
        TESTABLE_SOURCE_TWO_PIECE_THREE,
        SOURCE_PRELOAD,
        SOURCE_PRELOAD_LEAVE
        }
        
    public AutonomousSelector(Swerve swerve){

       
        autonomousSelector.setDefaultOption(
        
            "MID_PRELOAD", modes.MID_PRELOAD);

        autonomousSelector.addOption("MID_2p_B", modes.MID_2p_B);

        autonomousSelector.addOption("MID_4p_B_C_A", modes.MID_4p_B_C_A);
        
        autonomousSelector.addOption("TESTABLE_MID_FOUR_HALF_PIECE", modes.TESTABLE_MID_FOUR_HALF_PIECE);

        autonomousSelector.addOption("TESTABLE_MID_FOUR_RACE_THREE", modes.TESTABLE_MID_FOUR_RACE_THREE);

        autonomousSelector.addOption("MID_PRELOAD_LEAVE", modes.MID_PRELOAD_LEAVE);
        
        autonomousSelector.addOption("AMP_PRELOAD", modes.AMP_PRELOAD);

        autonomousSelector.addOption("AMP_PRELOAD_LEAVE", modes.AMP_PRELOAD_LEAVE);

        autonomousSelector.addOption("TESTABLE_AMP_TWO_PIECE_ONE", modes.TESTABLE_AMP_TWO_PIECE_ONE);

        autonomousSelector.addOption("TESTABLE_SOURCE_TWO_PIECE", modes.TESTABLE_SOURCE_TWO_PIECE);

        autonomousSelector.addOption("TESTABLE_SOURCE_TWO_PIECE_THREE", modes.TESTABLE_SOURCE_TWO_PIECE_THREE);

        autonomousSelector.addOption("SOURCE_PRELOAD", modes.SOURCE_PRELOAD); 
        
        autonomousSelector.addOption("SOURCE_PRELOAD_LEAVE", modes.SOURCE_PRELOAD_LEAVE);

        
        SmartDashboard.putData("Auto Choices", autonomousSelector);
        
    }
    
public modes get(){
    return autonomousSelector.getSelected();
}
}
