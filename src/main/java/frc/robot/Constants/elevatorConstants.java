package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class elevatorConstants {
    public static final InvertedValue leftMotorInvert = InvertedValue.Clockwise_Positive;

    public static final double gearRatio = 12;
    public static final double wheelCircumferenceMeters = 5.51873699838;

    /* Current Limits */
    public static final double statorCurrentLimit = 100;

    /* PID Values*/
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    /* Climber PID Values */
    public static final double ClimberkP = 0.0;
    public static final double ClimberkD = 0.0;
    public static final double ClimberkS = 0.0;
    public static final double ClimberkV = 0.0;
    public static final double ClimberkG = 0.0;

    /* MotionMagic Values */
    public static final double CruiseVelocity = 0;
    public static final double Acceleration = 0;
    public static final double Jerk = 0;

}
