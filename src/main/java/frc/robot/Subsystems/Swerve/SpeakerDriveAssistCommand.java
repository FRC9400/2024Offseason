package frc.robot.Subsystems.Swerve;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Superstructure;
import frc.robot.Subsystems.Superstructure.SuperstructureStates;

public class SpeakerDriveAssistCommand extends Command{
    private final Swerve swerve;
    private final PIDController thetaController = new PIDController(4, 0, 0);
    private Rotation2d headingGoal;
    
    public SpeakerDriveAssistCommand(Swerve swerve){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        headingGoal = new Rotation2d(0);
    }

    @Override
    public void execute(){
        double x = Math.pow(MathUtil.applyDeadband(RobotContainer.controller.getLeftY(), 0.1), 3);
        double y = Math.pow(MathUtil.applyDeadband(RobotContainer.controller.getLeftX(), 0.1),3);
        double dx;
        double dy;

        if(DriverStation.getAlliance().equals(Alliance.Blue)){
            dx = x * -1;
            dy = y * -1;
        } else{
            dx = x;
            dy = y;
        }
        double thetaFeedback = thetaController.calculate(
            swerve.getGyroPositionRadians(),
            headingGoal.getRadians()
        );
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);

        swerve.requestDesiredState(dx * 4.72, dy * 4.72, thetaFeedback, true, false);

    }

    @Override
    public void end(boolean interrupted){

    }
}

