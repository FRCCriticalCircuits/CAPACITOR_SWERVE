package frc.robot.subsystems.oled.oledHAL;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class SSD1306 {
    I2C i2c;

    /**
     * 128x64 SSD1306(I2C) Screen Initialization
     * @param address I2C address of SSD1306
     */
    public SSD1306(SSD1306_I2C_ADDR address){
        i2c = new I2C(Port.kOnboard, address.value);
    }

    private void writeCommand(int cmd){
        i2c.write(0x00, cmd);
    }

    private void writeData(int data){
        i2c.write(0x40, data);
    }
    
    private void writeDataArray(int dataArray[])
    {
        byte[] dataBuffer = new byte[129];
        dataBuffer[0] = 0x40;

        for(int i = 0; i < dataArray.length; i++){
            dataBuffer[i + 1] = (byte) dataArray[i];
        }
        
        i2c.writeBulk(dataBuffer);
    }   

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
