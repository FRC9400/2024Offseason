package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {

    public static final PIDController choreoTransController = new PIDController(6, 0, 0);
    public static final PIDController choreoRotController = new PIDController(1.5,0,0);
    
}