package frc.robot.subsystems.oled;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.oled.oledHAL.SSD1306HAL;
public class SSD1306 extends SubsystemBase{
    private SSD1306HAL oledInstance;

    public SSD1306(){
        oledInstance = new SSD1306HAL(0x78); // 0x78
        oledInstance.fillScreen();
    }

    @Override
    public void periodic() {
        oledInstance.sendBuffer();
    }
}
