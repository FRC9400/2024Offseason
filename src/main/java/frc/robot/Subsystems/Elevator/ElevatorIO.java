
package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double appliedVolts = 0.0;
        public double setPointMeters = 0;
        public double elevatorHeightMeters = 0;
        public double elevatorVelMPS = 0;
        public double[] currentAmps = new double[] {};
        public double[] tempFahrenheit = new double[] {};

    }

public default void updateInputs(ElevatorIOInputs inputs){}

public default void updateTunableNumbers(){}

public default void zeroSensor(){}

public default void setOutput(double output){}

public default void setMotionMagicConfigs(boolean down){}

public default void goToSetpoint(double setPointMeters){}

public default void elevatorConfiguration(){}
}
