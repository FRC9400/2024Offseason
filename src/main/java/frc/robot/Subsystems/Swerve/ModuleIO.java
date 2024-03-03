package frc.robot.Subsystems.Swerve;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVelocityMetersPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public double driveTempCelcius = 0.0;
        public double driveDistanceMeters = 0.0;
        public double driveOutputPercent = 0.0;
        public double rawDriveRPS = 0.0;

        public double moduleAngleRads = 0.0;
        public double moduleAngleDegs = 0.0;
        public double rawAbsolutePositionRotations = 0.0;
        public double absolutePositionRadians = 0.0;
        public double absolutePositionDegrees = 0.0; 
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0; 
        public double turnTempCelcius = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(ModuleIOInputs inputs) {}

    public default void setDesiredState(SwerveModuleState desiredState){}

    /** Run the drive motor at a specified velocity */
    public default void setDriveVelocity(double velocityMetersPerSecond, boolean auto) {}

    /** Run the drive motor at a specified percent */
    public default void setDrivePercent(double percent) {}

    public default void testDriveVoltage() {}

    public default void setDriveVoltage(double setVoltage) {}

    /** Set the turn motor to a particular angle */
    public default void setTurnAngle(double positionDegs) {}

    /** Enable or disable drive brake mode */
    public default void setDriveBrakeMode(boolean enable) {}

    /** Enable or disable turn brake mode */
    public default void setTurnBrakeMode(boolean enable) {}

    /** Resets the turn motor to its absolute position */
    public default void resetToAbsolute() {}

    /** Updates set of tunable numbers */
    public default void updateTunableNumbers() {}
}
