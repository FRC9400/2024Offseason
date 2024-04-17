package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.modes.TWO_PIECE_SOURCE;
import frc.robot.autons.modes.TWO_PIECE_MID;
import frc.robot.autons.modes.FOUR_HALF_PIECE;
import frc.robot.autons.modes.FOUR_PIECE;
import frc.robot.autons.modes.FOUR_PIECE_THREE_B_A;
import frc.robot.autons.modes.PRELOAD_LEAVE_AMP;
import frc.robot.autons.modes.PRELOAD_LEAVE_MID;
import frc.robot.autons.modes.PRELOAD_LEAVE_SOURCE;
import frc.robot.autons.modes.PRELOAD_AMP;
import frc.robot.autons.modes.PRELOAD_MID;
import frc.robot.autons.modes.PRELOAD_SOURCE;
import frc.robot.autons.modes.TWO_PIECE_SOURCE_THREE;

public class AutonomousSelector {
    private SendableChooser<SequentialCommandGroup> autonomousSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        autonomousSelector.setDefaultOption(
        
            "MID_PRELOAD", new PRELOAD_MID(swerve, superstructure));

        autonomousSelector.addOption("MID_2p_B", new TWO_PIECE_MID(swerve, superstructure));

        autonomousSelector.addOption("MID_4p_B_C_A", new FOUR_PIECE(swerve, superstructure));
        
        autonomousSelector.addOption("TESTABLE_MID_FOUR_HALF_PIECE", new FOUR_HALF_PIECE(swerve, superstructure));

        autonomousSelector.addOption("TESTABLE_MID_FOUR_RACE_THREE", new FOUR_PIECE_THREE_B_A(swerve, superstructure));

        autonomousSelector.addOption("MID_PRELOAD_LEAVE", new PRELOAD_LEAVE_MID(swerve, superstructure));
        
        autonomousSelector.addOption("AMP_PRELOAD", new PRELOAD_AMP(swerve, superstructure));

        autonomousSelector.addOption("AMP_PRELOAD_LEAVE", new PRELOAD_LEAVE_AMP(swerve, superstructure));

        autonomousSelector.addOption("TESTABLE_SOURCE_TWO_PIECE", new TWO_PIECE_SOURCE(swerve, superstructure));

        autonomousSelector.addOption("TESTABLE_SOURCE_TWO_PIECE_THREE", new TWO_PIECE_SOURCE_THREE(swerve, superstructure));

        autonomousSelector.addOption("SOURCE_PRELOAD", new PRELOAD_SOURCE(swerve, superstructure)); 
        
        autonomousSelector.addOption("SOURCE_PRELOAD_LEAVE", new PRELOAD_LEAVE_SOURCE(swerve, superstructure));

        
        SmartDashboard.putData("Auto Choices", autonomousSelector);
        
    }
    
public SequentialCommandGroup get(){
    return autonomousSelector.getSelected();
}
}
