
package frc.robot.Subsystems.LEDs;
import org.littletonrobotics.junction.Logger;
 
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants.canIDConstants;

public class LEDs {
    private CANdle candle = new CANdle(canIDConstants.candle, "rio");
    private LEDStates state = LEDStates.IDLE;
    private BlinkPattern blinkPattern = BlinkPattern.SOLID;
    private Timer blinkTimer = new Timer();
    private Color color = new Color();
    private double onDuration;
    private double offDuration;
  
   
    public LEDs(){
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGBW; // set the strip type to RGB
        config.brightnessScalar = 0.5; // dim the LEDs to half brightness
        candle.configAllSettings(config);
        onDuration = 0;
        offDuration = 0;
        
    }

    public enum LEDStates{
        DISABLED,
        IDLE,
        INTAKING,
        READY_TO_SCORE,
        PREPARING_ELEVATOR_AMP,
        ELEVATOR_AMP,
        SHOOT
    }

    public enum BlinkPattern{
        SOLID,
        BLINK_SLOW,
        BLINK_FAST
    }

    public void Loop(){
        Logger.recordOutput("LEDState", state);
        switch(state){
            case DISABLED:
                color = Color.kRed;
                blinkPattern = BlinkPattern.SOLID;
                break;
            case IDLE:
                color = Color.kOrange;
                blinkPattern = BlinkPattern.BLINK_SLOW;
                break;
            case INTAKING:
                color = Color.kCrimson;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case READY_TO_SCORE:
                color = Color.kGreen;
                blinkPattern = BlinkPattern.SOLID;
            case PREPARING_ELEVATOR_AMP:
                color = Color.kAquamarine;
                blinkPattern = BlinkPattern.BLINK_SLOW;
                break;
            case ELEVATOR_AMP:
                color = Color.kAquamarine;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
            case SHOOT:
                color = Color.kAliceBlue;
                blinkPattern = BlinkPattern.BLINK_FAST;
                break;
        }

        Color8Bit color8Bit = new Color8Bit(color);

        if (blinkPattern == BlinkPattern.SOLID) {
            candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
          } else {
            double time = blinkTimer.get();
            double onDuration = 0;
            double offDuration = 0;
      
            if (blinkPattern == BlinkPattern.BLINK_FAST) {
              onDuration = 0.08;
              offDuration = 0.08 * 2;
            } else if (blinkPattern == BlinkPattern.BLINK_SLOW) {
              onDuration = 0.25;
              offDuration = 0.25 * 2;
            }
      
            if (time >= offDuration) {
              blinkTimer.reset();
              candle.setLEDs(0, 0, 0);
            } else if (time >= onDuration) {
              candle.setLEDs(color8Bit.red, color8Bit.green, color8Bit.blue);
            }
          }
    }

    public void setState(LEDStates nextState){
        state = nextState;
    }
    
}

