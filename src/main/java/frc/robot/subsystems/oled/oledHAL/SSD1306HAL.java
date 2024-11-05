package frc.robot.subsystems.oled.oledHAL;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class SSD1306HAL {
    I2C oledHandle;
    private byte[] dataBuffer = new byte[128 * 64 / 8];

    public SSD1306HAL(int deviceAddress) {
        oledHandle = new I2C(Port.kOnboard, deviceAddress);
        _init();
        
        clearScreen();
    }

    /** Datasheet Page 28-32 */
    private void _init() {
        /** Set Multiplex Ratio [16, 63], 0x3F for 63 */
        writeCmd(0xA8, 0x3F);

        /** Set Display Offset, 0x00 for non-Offset */
        writeCmd(0xD3, 0x00);

        /** Set Display Start Line [0x40, 0x7F], 0x40 for 0 as Start Line, 0x7F for 63 */
        writeCmd(0x40);

        /** Set Segment Reverse, 0xA0 for Normal */
        writeCmd(0xA1); // Test

        /** Set COM Reverse, 0xC8 for Reverse */
        writeCmd(0xC0); // Test

        /** Set COM Pins Hardware Configuration */
        writeCmd(0xDA, 0x02); // Test 000a0010 a0 for sequential

        /** Set Contrast in [0x00, 0x100] */
        writeCmd(0x81, 0x7F);

        /** Output RAM to Display (0xA4) */
        writeCmd(0xA4);

        /** Set Memory Addressing Mode, 0x00 -> Horizontal, 0x01 -> Vertical, 0x10 -> Page */
        writeCmd(0x20, 0x10);

        /** Set Clock Divide Ratio [3:0] (starts from 1) & oscillator frequency [4:7] (1000b for 100Hz) */
        writeCmd(0xD5, 0x80);

        /** 
         * Datasheet Page 62
         * Set Charge Pump Enable (0x14) / Disable (0x10) 
         */
        writeCmd(0x8D, 0x14);

        /** None Inverted */
        writeCmd(0xA6);

        /** Display On */
        writeCmd(0xAF);
    }

    /**
     * Send command to SSD1306
     * @param cmd the command to send
     */
    private void writeCmd (int cmd) {
        oledHandle.write(0x00, cmd);
    }

    /**
     * Send two commands to SSD 1306
     * @param cmd the first command
     * @param argument the follow command
     */
    private void writeCmd (int cmd, int... arguments) {
        writeCmd(cmd);
        for (int argument : arguments) writeCmd(argument);
    }

    /**
     * Sends data array to SSD1306
     * @param data  the data array containing data to be sent to the chip
     * @param index the starting index in the data array from which to begin sending
     * @param size  the number of bytes to send, usually is 128 which is the width
     */
    private void writeData (byte[] data, int index, int size) {
        byte[] buffer = new byte[size + 1];
        buffer[0] = 0x40;
        for (int i = index; i < (index + size); i++) {
            buffer[i - index + 1] = data[i];
        }
    }

    /** 
     * Apply buffer to SSD1306
     */
    private void sendBuffer () {
        for(int page = 0; page < 8; page++){
            // Set Page Address & Column Address
            writeCmd(0xB0 + page, 0x00, 0x10);

            // Write Multi Data
            writeData(dataBuffer, page * 128, 1024);
        }
    }

    /**
     * Fill screen with specific color
     * @param col the {@link Color} to set
     */
    private void fillScreen (Color col) {
        for(int i = 0; i < dataBuffer.length; i++) dataBuffer[i] = col.value;
    }

    /** Clear OLED Display */
    private void clearScreen () {
        fillScreen(Color.WHITE);
    }

    public enum Color{
        BLACK((byte) 0xFF),
        WHITE((byte) 0x00);

        public final byte value;

        Color(byte value) {
            this.value = value;
        }
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

