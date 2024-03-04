package frc.robot.Commands;

import frc.robot.Constants.swerveConstants;
import frc.robot.Subsystems.Swerve.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;




public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;


    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
   
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.4);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.4);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), 0.4);

        double x_speed = translationVal * swerveConstants.moduleConstants.maxSpeed;
        double y_speed = strafeVal * swerveConstants.moduleConstants.maxSpeed;
        double rot_speed = rotationVal * swerveConstants.moduleConstants.maxAngularVelocity;

        /* Drive */
        s_Swerve.requestVoltage(
            x_speed, 
            y_speed,
            rot_speed, 
            true
        );
    }
}
