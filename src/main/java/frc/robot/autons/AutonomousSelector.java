package frc.robot.autons;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autons.modes.TWOPieceMid;
import frc.robot.autons.modes.TwoPieceMidHardCoded;
import frc.robot.autons.modes.PreLoadLeaveAmpSide;
import frc.robot.autons.modes.PreLoadLeaveSourceSide;
import frc.robot.autons.modes.PreloadAmpSide;
import frc.robot.autons.modes.preloadMid;
import frc.robot.autons.modes.PreLoadSourceSide;

public class AutonomousSelector {
    private SendableChooser<SequentialCommandGroup> autonomousSelector = new SendableChooser<SequentialCommandGroup>();

    public AutonomousSelector(Swerve swerve, Superstructure superstructure){
        autonomousSelector.setDefaultOption(
            "PRELOAD_MID", new preloadMid(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_LEFT", new PreloadAmpSide(swerve, superstructure));

        autonomousSelector.addOption("PRELOAD_RIGHT", new PreLoadSourceSide(swerve, superstructure));

        //autonomousSelector.addOption("TwoPieceMID", new TWOPieceMid(swerve, superstructure));

        autonomousSelector.addOption("TwoPieceMidHardCoded", new TwoPieceMidHardCoded(swerve, superstructure));

        autonomousSelector.addOption("preloadLeaveLeft", new PreLoadLeaveAmpSide(swerve, superstructure));
        
        autonomousSelector.addOption("preloadRightLeave", new PreLoadLeaveSourceSide(swerve, superstructure));

        SmartDashboard.putData("Auto Choices", autonomousSelector);


    }
    
public SequentialCommandGroup get(){
    return autonomousSelector.getSelected();
}
}
