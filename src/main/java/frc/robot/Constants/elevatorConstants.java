package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.util.Units;

public class elevatorConstants {
    public static final InvertedValue leftMotorInvert = InvertedValue.Clockwise_Positive;

    public static final double gearRatio = 25;
    public static final double wheelCircumferenceMeters = Units.inchesToMeters(5.51873699838);
    public static final double minHeightMeters = 0.0;
    public static final double maxHeightMeters = 0.45;

    public static final double minHeightInRotations = 0;
    public static final double maxHeightInRotations = 0.0;

    /* Current Limits */
    public static final double statorCurrentLimit = 100;

    /* PID Values*/
    public static final double kP = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    /* MotionMagic Values */
    public static final double CruiseVelocity = 0;
    public static final double Acceleration = 0;
    public static final double Jerk = 0;


}
