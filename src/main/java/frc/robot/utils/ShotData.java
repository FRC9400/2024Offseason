package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShotData {
    
    private final Rotation2d rotation;
    private final double velocity;
    private final double ratio;
    private final double flightTimeSeconds;

    public ShotData(Rotation2d rotation, double velocity, double ratio, double flightTimeSeconds) {
        this.rotation = rotation;
        this.velocity = velocity;
        this.ratio = ratio;
        this.flightTimeSeconds = flightTimeSeconds;
    }

    public Rotation2d getRotation() {
        return rotation;
    }

    public double velocity() {
        return velocity;
    }

    public double ratio() {
        return ratio;
    }

    public double getFlightTimeSeconds() {
        return flightTimeSeconds;
    }
}
