package frc.robot.Subsystems.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Superstructure;

public class Turn extends Command{
    private final Swerve swerve;
    private final PIDController thetaController = new PIDController(5, 0, 0.25);
    private Rotation2d headingGoal;
    
    public Turn(Swerve swerve, Rotation2d headingGoal){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.headingGoal = headingGoal;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        
        double thetaFeedback = thetaController.calculate(
            swerve.getGyroPositionRadians(),
            headingGoal.getRadians()
        );
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);

        swerve.requestDesiredState(0, 0, thetaFeedback, true, false);

    }

    @Override
    public void end(boolean interrupted){

    }
}

