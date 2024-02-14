
package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double appliedVolts = 0.0;
        public double setPoint = 0;
        public double elevatorHeightMeters = 0;
         
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};
        public double[] rotorVelocityRPS = new double[] {};
        

    }


public default void updateInputs(ElevatorIOInputs inputs){}

public default void updateTunableNumbers(){}

public default void setHeight(double setPoint){}

public default void testOutput(){}

public default void setVoltage(double output){}

public default void setElevator(double setPointMeters){}

public default void resetHeight() {}

public default void elevatorConfiguration(){}
}
