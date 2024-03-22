/* 
package frc.robot.Subsystems.LEDs;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.canIDConstants;

public class LEDs {
    private CANdle candle = new CANdle(canIDConstants.candle, "rio");
    private LEDStates state = LEDStates.IDLE;
    Animation yellowSlowBlink = new Animation(1, 2, 64, 0);
    Animation orangeFastBlink = new Animation();
    Animation redSlowBlink = new Animation();
    Animation redFastBlink = new Animation();
    Animation blueFastBlink = new Animation();
   
    
    public LEDs(){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGBW;
        config.brightnessScalar = 0.5;
        candle.configAllSettings(config);
    }

    public enum LEDStates{
        DISABLED,
        IDLE,
        INTAKING,
        PREPARING_ELEVATOR_AMP,
        ELEVATOR_AMP,
        SHOOT
    }

    public void Loop(){
        Logger.recordOutput("LEDState", state);
        switch(state){
            case DISABLED:
        
                break;
            case IDLE:
            Animation 
            candle.;
                break;
            case INTAKING:
                break;
            case PREPARING_ELEVATOR_AMP:
                break;
            case ELEVATOR_AMP:
                break;
            case SHOOT:
                break;
        }
    }

    public void setState(LEDStates nextState){
        state = nextState;
    }
    
}
*/
