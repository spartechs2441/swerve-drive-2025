package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Queue;

public class LEDManager {
    private static final HashMap<Character, boolean[]> map;

    static {
        // dot is false, dash is true
        HashMap<Character, boolean[]> charMap = new HashMap<>();
        charMap.put('a', new boolean[]{false, true}); // .-
        charMap.put('b', new boolean[]{true, false, false, false}); // -...
        charMap.put('c', new boolean[]{true, false, true, false}); // -.-.
        charMap.put('d', new boolean[]{true, false, false}); // -..
        charMap.put('e', new boolean[]{false}); // .
        charMap.put('f', new boolean[]{false, false, true, false}); // ..-.
        charMap.put('g', new boolean[]{true, true, false}); // --.
        charMap.put('h', new boolean[]{true, true, true, true}); // ....
        charMap.put('i', new boolean[]{false, false}); // ..
        charMap.put('j', new boolean[]{false, true, true, true}); // .---
        charMap.put('k', new boolean[]{true, false, true}); // -.-
        charMap.put('l', new boolean[]{false, true, false, false}); // .-..
        charMap.put('m', new boolean[]{true, true}); // --
        charMap.put('n', new boolean[]{true, false}); // -.
        charMap.put('o', new boolean[]{true, true, true}); // ---
        charMap.put('p', new boolean[]{false, true, true, false}); // .--.
        charMap.put('q', new boolean[]{true, true, false, true}); // --.-
        charMap.put('r', new boolean[]{false, true, false}); // .-.
        charMap.put('s', new boolean[]{false, false, false}); // ...
        charMap.put('t', new boolean[]{true}); // -
        charMap.put('u', new boolean[]{false, false, true}); // ..-
        charMap.put('v', new boolean[]{false, false, false, true}); // ...-
        charMap.put('w', new boolean[]{false, true, true}); // .--
        charMap.put('x', new boolean[]{true, false, false, true}); // -..-
        charMap.put('y', new boolean[]{true, false, true, true}); // -.--
        charMap.put('z', new boolean[]{true, true, false, false}); // --..
        charMap.put('1', new boolean[]{false, true, true, true, true}); // .----
        charMap.put('2', new boolean[]{false, false, true, true, true}); // ..---
        charMap.put('3', new boolean[]{false, false, false, true, true}); // ...--
        charMap.put('4', new boolean[]{false, false, false, false, true}); // ....-
        charMap.put('5', new boolean[]{false, false, false, false, false}); // .....
        charMap.put('6', new boolean[]{true, false, false, false, false}); // -....
        charMap.put('7', new boolean[]{true, true, false, false, false}); // --...
        charMap.put('8', new boolean[]{true, true, true, false, false}); // ---..
        charMap.put('9', new boolean[]{true, true, true, true, false}); // ----.
        charMap.put('0', new boolean[]{true, true, true, true, true}); // -----
        map = charMap;
    }

    private final AddressableLED led;
    private final AddressableLEDBuffer buf;
    private LEDStatus status;
    private final Queue<Color> queue;
    private final int length;
    private final long lastUpdated = System.currentTimeMillis();

    public LEDManager(AddressableLED led, final int length) {
        this.led = led;
        this.buf = new AddressableLEDBuffer(length);
        this.length = length;
        this.status = LEDStatus.DEFAULT;
        this.queue = new ArrayDeque<>();
        updateLEDs(LEDStatus.DEFAULT);
        led.setLength(length);
    }

    /**
     * Convert string to morse code blinking code
     * @param input Input string to transform
     * @return pair where the Boolean is whether an error has occurred
     */
    private Pair<ArrayList<Boolean>, Boolean> stringToBlinkMorse(String input) {
        var array = new ArrayList<Boolean>(this.length);
        boolean bad = false;
        for (int i = 0; i < input.length(); i++) {
            var c = Character.toLowerCase(input.charAt(i));
            var sequence = map.get(c);
            if (sequence == null) {
                bad = true;
                sequence = new boolean[]{};
            }

            for (boolean b : sequence) {
                if (b) {
                    array.add(true);
                    array.add(true);
                } else {
                    array.add(true);
                }
                array.add(false);
            }
        }
        return Pair.of(array, bad);
    }

    public LEDStatus getStatus() {
        return status;
    }

    public void setStatus(LEDStatus status) {
        if (status == this.status) {
            return;
        }
        updateLEDs(status);
    }

    public void updateStream() {
        if (this.lastUpdated + 1000 > System.currentTimeMillis()) {
            return;
        }
        Color color = this.queue.poll();
        switch (status) {
            case BUFFER_ALL -> setAll(color);
        }
    }

    public void queueAdd(ArrayList<Color> add) {
        this.queue.addAll(add);
    }

    public void updateLEDs(LEDStatus status) {
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
