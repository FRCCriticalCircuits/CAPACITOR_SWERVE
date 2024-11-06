package frc.robot.subsystems.oled.oledHAL;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;

public class SSD1306HAL {
    I2C oledHandle;
    private byte[] dataBuffer = new byte[128 * 8];  // 8 page, 128 bytes each page

    public SSD1306HAL(int deviceAddress) {
        oledHandle = new I2C(Port.kMXP, deviceAddress);
        _init();
    }

    /** From Datasheet Page 28-32 & U8G2 */
    private void _init() {
        Timer.delay(0.1);

        /** Display OFF */
        writeCmd(0xAE);

        /** Set Clock Divide Ratio [3:0] (starts from 1) & oscillator frequency [4:7] */
        writeCmd(0xD5, 0x80);

        /** Set Multiplex Ratio [16, 63], 0x3F for 63 */
        writeCmd(0xA8, 0x3F);

        /** Set Display Offset, 0x00 for non-Offset */
        writeCmd(0xD3, 0x00);

        /** Set Display Start Line [0x40, 0x7F], 0x40 for 0 as Start Line, 0x7F for 63 */
        writeCmd(0x40);

        /** Set Charge Pump Enable (0x14) / Disable (0x10) */
        writeCmd(0x8D, 0x14);

        /** Set Memory Addressing Mode, 0x00 -> Horizontal, 0x01 -> Vertical, 0x10 -> Page */
        writeCmd(0x20, 0x10);

        /** Set Segment Reverse, 0xA0 for Normal */
        writeCmd(0xA1); // Test

        /** Set COM Reverse, 0xC8 for Reverse */
        writeCmd(0xC8);

        /** Set COM Pins Hardware Configuration */
        writeCmd(0xDA, 0x12); // Test 000a0010 a0 for sequential

        /** Set Contrast in [0x00, 0x100] */
        writeCmd(0x81, 0xCF);

        writeCmd(0xD9, 0xF1);

        writeCmd(0xDB, 0x40);

        /** Disable Scrolling */
        writeCmd(0x2E);

        /** Output RAM to Display (0xA4) */
        writeCmd(0xA4);

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
     * @param arguments the follow commands
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
        oledHandle.writeBulk(buffer);
    }

    /** 
     * Send buffer to SSD1306 RAM
     */
    public void sendBuffer () {
        for(byte page = 0; page < 8; page++){
            // Set Page Address & Column Address
            writeCmd((0xB0 + page), 0x00, 0x10);
            writeData(dataBuffer, page * 128, 128);
        }
    }

    /**
     * Draw Pixel
     * @param row the row to set
     * @param col the column to set
     * @param status on/off {@link PixelState} 
     * @apiNote result will be save to buffer, call sendBuffer to apply it
     */
    public void drawPixel(int row, int col, PixelState status){
        if(col > 128 || col < 1) return;
        if(row > 64 || row < 1) return;

        int bufferIndex = (row - (row % 8)) / 8 * 128 + (col - 1);
        int dataIndex = row % 8;

        if(status.value){
            dataBuffer[bufferIndex] |= (byte) (1 << (8 - dataIndex));   // (buffer OR ( 1 << (row - 1) ) )
        }else{
            dataBuffer[bufferIndex] &= (byte) ~(1 << (8 - dataIndex));  // (buffer AND ( NOT (1 << (row - 1)) ) )
        }
    }

    /**
     * Fill screen with specific color
     * @param status the {@link PixelState} to set
     */
    public void fillScreen () {
        for(int i = 0; i < dataBuffer.length; i++) dataBuffer[i] = (byte) 0xFF;
    }

    /** Fill screen with {@link PixelState} OFF */
    public void clearScreen () {
        fillScreen();
    }

    public enum PixelState{
        OFF(false),
        ON(true);

        public final boolean value;

        PixelState(boolean value) {
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

