package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class shooterConstants {
    /* Motor Inverts */
    public static final InvertedValue leftShooterInvert = InvertedValue.Clockwise_Positive;

    /* Current Limits */
    public static final double statorCurrentLimit = 0.0;

    /* PID values */
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;

    /* MotionMagic Values */
    public static final double CruiseVelocity = 0;
    public static final double Acceleration = 0;
    public static final double Jerk = 0;

}
