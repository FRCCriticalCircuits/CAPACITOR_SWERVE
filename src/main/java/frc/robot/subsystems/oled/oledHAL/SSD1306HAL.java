package frc.robot.subsystems.oled.oledHAL;

import edu.wpi.first.wpilibj.I2C;

public class SSD1306HAL {
    I2C SSD1306_Handle;

    private enum SSD1306_I2C_ADDR
    {
        SSD1306_ADDR_SA0_0(0x78),
        SSD1306_ADDR_SA0_1(0x7A);
        
        /** Addr value. */
        public final int value;

        SSD1306_I2C_ADDR(int value) {
            this.value = value;
        }
    };
}
