package frc.robot.Constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class swerveConstants {
    public static final class moduleConstants {
        /* Inverts FL, FR, BL, BR */
        public static final InvertedValue[] driveMotorInverts = {InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.CounterClockwise_Positive};
        public static final InvertedValue[] steerMotorInverts = {InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive};
        public static final SensorDirectionValue[] CANcoderInverts = {SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive, SensorDirectionValue.CounterClockwise_Positive};
        /* CANcoder Offset FL, FR, BL, BR */
        public static final double[] CANcoderOffsets = {0.184570, 0.099584, 0.334229, 0.103516};//{-0.010986,0.098145,0.000244,0.105713}; 
      //  {0.989014,0.098145,0.000244,0.15469};
    
        
        /* Gear Ratios */
        public static final double driveGearRatio = 6.12;
        public static final double steerGearRatio = 150.0 / 7.0;

        /* Max Speeds */
        public static final double maxSpeedMeterPerSecond = Units.feetToMeters(16); 
        public static final double maxAngularVelocity = 2.5; 
        
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
        public static final double wheelCircumferenceMeters = Units.inchesToMeters(4) * Math.PI;
    }

    public static final class kinematicsConstants{
        /* Drivetrain Constants */
        public static final double wheelBase = Units.inchesToMeters(22.75);
        public static final double trackWidth = Units.inchesToMeters(20.75);

        /* Swerve Kinematics */
        public static final Translation2d FL = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d FR = new Translation2d(wheelBase / 2.0, -trackWidth / 2.0);
        public static final Translation2d BL = new Translation2d(-wheelBase / 2.0, trackWidth / 2.0);
        public static final Translation2d BR = new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0);

    }
}