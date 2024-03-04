package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class swerveConstants {
    public static final class moduleConstants {
        /* Inverts FL, FR, BL, BR */
        public static final InvertedValue[] driveMotorInverts = {InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final InvertedValue[] steerMotorInverts = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final SensorDirectionValue[] CANcoderInverts = {SensorDirectionValue.Clockwise_Positive, SensorDirectionValue.Clockwise_Positive, SensorDirectionValue.Clockwise_Positive, SensorDirectionValue.Clockwise_Positive};
        public static final double[] CANcoderOffsets = {0.518311, 0.271240, 0.386230, 0.173340}; //degrees or rotations

        /* CANcoder Offset FL, FR, BL, BR */
        public static final double[] CANcoderOffset = {};

        /* Gear Ratios */
        public static final double driveGearRatio = 6.55;
        public static final double steerGearRatio = 10.28;

        /* Max Speeds */
        public static final double maxSpeed = 2;
        public static final double maxAngularVelocity = 2.0;
        
        /* Current Limits */
        public static final double driveStatorCurrentLimit = 120;
        public static final double steerStatorCurrentLimit = 50;

        /* PID Values */
        public static final double drivekP = 0.0;
        public static final double drivekD = 0.0;
        public static final double drivekS = 0.0;
        public static final double drivekV = 0.0;

        public static final double anglekP = 0.0;
        public static final double anglekD = 0.0;
        public static final double anglekS = 0.0;
        public static final double anglekV = 0.0;

        /* Wheel Circumference */
        public static final double wheelCircumferenceMeters = 0.0;
    }

    public static final class kinematicsConstants{
        /* Drivetrain Constants */
        public static final double robotLength = Units.inchesToMeters(28);
        public static final double robotWidth = Units.inchesToMeters(28.5);

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d FR = new Translation2d(robotLength / 2.0, -robotWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-robotLength / 2.0, robotWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-robotLength / 2.0, -robotWidth / 2.0);

    }
}
