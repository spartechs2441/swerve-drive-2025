package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LEDManager {
    private final AddressableLED led;
    private final AddressableLEDBuffer buf;
    private final LEDStatus status;

    public LEDManager(AddressableLED led, final int length) {
        this.led = led;
        this.buf = new AddressableLEDBuffer(length);
        this.status = LEDStatus.DEFAULT;
        led.setLength(length);
    }

    public LEDStatus getStatus() {
        return status;
    }

    public void setStatus(LEDStatus status) {
        if (status == this.status) {
            return;
        }
        switch (status) {
            case DEFAULT -> setDefault();
            case SHOOTING -> setShooting();
            case L2 -> setL2();
            case L3 -> setL3();
            case DOWN -> setDown();
            case RANDOM -> setRandom();
        }
        this.led.setData(this.buf);
    }

    private void setDefault() {
        setAll(Color.kBlack);
    }

    private void setShooting() {
        setAll(Color.kYellow);
    }

    private void setL2() {
        setAll(Color.kGreen);
    }

    private void setL3() {
        setAll(Color.kRed);
    }

    private void setDown() {
        setAll(Color.kBlue);
    }

    private void setRandom() {
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setRGB(i, (int) (Math.random() * 255), (int) (Math.random() * 255), (int) (Math.random() * 255));
        }
    }

    private void setAll(Color color) {
        for (int i = 0; i < buf.getLength(); i++) {
            buf.setLED(i, color);
        }
    }
}
