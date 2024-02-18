package frc.robot.Subsystems.Elevator;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIO elevatorIO;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();


    @Override
    public void periodic(){
  
        elevatorIO.updateInputs(inputs);
        elevatorIO.updateTunableNumbers(); 
        Logger.processInputs("Elevator", inputs);
    }

    public Elevator(ElevatorIO elevatorIO) {
        this.elevatorIO = elevatorIO;
      }

      public void setHeight(double setPoint){
        elevatorIO.setHeight(setPoint);
    }
    public void testOutput(){
        elevatorIO.testOutput();
    }

    public void setOutput(double output){
        elevatorIO.setOutput(output);
    }
    
    public void setElevator(double x){
        elevatorIO.setElevator(x);
    }

    public void homing(){
        elevatorIO.homing();
    }
 
    public void elevatorConfiguration(){
        elevatorIO.elevatorConfiguration();
    }
    
}

