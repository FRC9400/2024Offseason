package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShotData {
    
    private final Rotation2d rotation;
    private final double leftRPS;
    private final double rightRPS;
    private final double flightTimeSeconds;

    public ShotData(Rotation2d rotation, double leftRPS, double rightRPS, double flightTimeSeconds) {
        this.rotation = rotation;
        this.leftRPS = leftRPS;
        this.rightRPS = rightRPS;
        this.flightTimeSeconds = flightTimeSeconds;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public double getLeftRPS() {
        return leftRPS;
    }

    public double getRightRPS() {
        return rightRPS;
    }

    public double getFlightTimeSeconds() {
        return flightTimeSeconds;
    }
}
