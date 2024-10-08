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

public class AmpDriveAssistCommand extends Command{
    private final Swerve swerve;
    private final Superstructure superstructure;
    private final PIDController thetaController = new PIDController(4, 0, 0.25);
    private Rotation2d headingGoal;
    
    public AmpDriveAssistCommand(Swerve swerve, Superstructure superstructure){
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.swerve = swerve;
        this.superstructure = superstructure;
        addRequirements(swerve, superstructure);
    }

    @Override
    public void initialize(){
        headingGoal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? new Rotation2d( Math.PI/2.0) : new Rotation2d( - Math.PI/2.0);
        superstructure.setState(SuperstructureStates.AMP_A); 
    }

    @Override
    public void execute(){
        double x = MathUtil.applyDeadband(RobotContainer.controller.getLeftY(), 0.1);
        double y = MathUtil.applyDeadband(RobotContainer.controller.getLeftX(), 0.1);
        double dx;
        double dy;

        dx = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? x * -1 : x;
        dy =  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? y : y *-1;
       
        double thetaFeedback = thetaController.calculate(
            swerve.getGyroPositionRadians(),
            headingGoal.getRadians()
        );
        thetaFeedback = MathUtil.clamp(thetaFeedback, -5, 5);

        swerve.requestDesiredState(dx * 2, dy * 2, thetaFeedback, true, false);

    }

    @Override
    public void end(boolean interrupted){

    }
}