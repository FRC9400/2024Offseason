package frc.robot.autons;

import edu.wpi.first.math.controller.PIDController;

public class AutoConstants {

    public static final PIDController choreoTransController = new PIDController(3, 0, 0);
    public static final PIDController choreoRotController = new PIDController(1.25,0,0);

    public static double subwooferVelM = 70;
    public static double subwooferRatioM = 0.5;
    public static double subwooferDegM = 45;

    public static double subwooferVelR = 70;
    public static double subwooferRatioR = 0.5;
    public static double subwooferDegR = 45;

    public static double subwooferVelL = 45.6;
    public static double subwooferRatioL = 1.5;
    public static double subwooferDegL = 50;

    public static double VelM = 70;
    public static double RatioM = 0.6;
    public static double DegM = 30;

    public static double VelR = 70;
    public static double RatioR = 0.5;
    public static double DegR = 30;

    public static double VelL = 45.6;
    public static double RatioL = 1.5;
    public static double DegL = 30;

    
}