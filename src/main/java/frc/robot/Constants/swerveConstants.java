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
        public static final SensorDirectionValue[] CANcoderInverts = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
        public static final double[] CANcoderOffsets = {0.478516, 0.722412, 0.617920, 0.819824}; //degrees or rotations 178.418, 204.17, 220.254, 358.154
 // 0.518311, 0.271240, 0.386230, 0.173340
        /* CANcoder Offset FL, FR, BL, BR */
        

        /* Gear Ratios */
        public static final double driveGearRatio = 6.55;
        public static final double steerGearRatio = 10.28;

        /* Max Speeds */
        public static final double maxSpeed = 4.72; 
        public static final double maxAngularVelocity = 2; 
        
        /* Current Limits */
        public static final double driveStatorCurrentLimit = 80;
        public static final double steerStatorCurrentLimit = 50;

        /* Ramp Rate */
        public static final double rampRate = 0.02;
        
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
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(4* Math.PI);
    }

    public static final class kinematicsConstants{
        /* Drivetrain Constants */
        public static final double robotLength = Units.inchesToMeters(28);
        public static final double robotWidth = Units.inchesToMeters(28.5);

        public static final double trackWidth = 0.60325;
        public static final double wheelBase = 0.59055;

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d FR = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

    }
}
