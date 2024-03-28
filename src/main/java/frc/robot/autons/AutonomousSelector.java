package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.modes.TWOPieceMid;
import frc.robot.autons.modes.ThreePieceMidSource;
import frc.robot.autons.modes.TWOPIECE_MID;
import frc.robot.autons.modes.Hardstop;
import frc.robot.autons.modes.PRELOAD_LEAVE_AMP;
import frc.robot.autons.modes.PRELOAD_LEAVE_MID;
import frc.robot.autons.modes.PRELOAD_LEAVE_SOURCE;
import frc.robot.autons.modes.PRELOAD_AMP;
import frc.robot.autons.modes.PRELOAD_MID;
import frc.robot.autons.modes.PRELOAD_SOURCE;

public class AutonomousSelector {
    private SendableChooser<SequentialCommandGroup> autonomousSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        autonomousSelector.setDefaultOption(
            "PRELOAD_MID", new PRELOAD_MID(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_AMP", new PRELOAD_AMP(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_SOURCE", new PRELOAD_SOURCE(swerve, superstructure));

        //autonomousSelector.addOption("TwoPieceMID", new TWOPieceMid(swerve, superstructure));

        autonomousSelector.addOption("TWOPIECE_MID", new TWOPIECE_MID(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_LEAVE_MID", new PRELOAD_LEAVE_MID(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_LEAVE_AMP", new PRELOAD_LEAVE_AMP(swerve, superstructure));
        
        autonomousSelector.addOption("PRE_LOAD_LEAVE_SOURCE", new PRELOAD_LEAVE_SOURCE(swerve, superstructure));

        //autonomousSelector.addOption("ThreePieceMidSource", new ThreePieceMidSource(swerve, superstructure));

        //autonomousSelector.addOption("testHardstop", new Hardstop(swerve, superstructure));

        SmartDashboard.putData("Auto Choices", autonomousSelector);


    }
    
public SequentialCommandGroup get(){
    return autonomousSelector.getSelected();
}
}
