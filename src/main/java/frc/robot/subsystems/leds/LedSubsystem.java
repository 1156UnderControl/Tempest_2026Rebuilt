package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
    private final CANBus kCANBus = new CANBus("rio");
    private final CANdle candle = new CANdle(0, kCANBus);
    //private final RainbowAnimation rainbowAnimation = new RainbowAnimation(0, 8)
    //.withSlot(0)
    //.withBrightness(1)
    //.withDirection(AnimationDirectionValue.Backward)
    //.withFrameRate(Hertz.of(63.798));
    private final FireAnimation fireAnimation= new FireAnimation(0, 8)
    .withSlot(0)
    .withBrightness(1)
    .withDirection(AnimationDirectionValue.Backward)
    .withFrameRate(Hertz.of(63.798));
    

    private final SolidColor[] colors = new SolidColor[]{

    };

    public LedSubsystem(){
        setDefaultCommand(updateLEDs());
    }

    public Command updateLEDs(){
        return run (()-> {
            for (var solidColor : colors ){
                candle.setControl(solidColor);
            }
        candle.setControl(fireAnimation);
    });
}
}
