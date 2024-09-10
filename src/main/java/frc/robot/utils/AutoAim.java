package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAim {

    private static final InterpolatingShotTree shotMap = new InterpolatingShotTree();

    static {
        shotMap.put(1.0, new ShotData(Rotation2d.fromDegrees(55), 20.0, 1.5, 1.2));
        //populate this through testing
    }

    public static ShotData getShotData(double distance) {
        return shotMap.get(distance);
    }

    public static void clearShotMap() {
        shotMap.clear();
    }

    public static void removeShotData(double distance) {
        shotMap.remove(distance);
    }
}
