package frc.robot.simulation;

import java.util.Arrays;

public class LEDSim {


    public String[] buffer;
    int trailIndex;

    LEDSim(int len) {
        buffer = new String[len];
    }

    public void glow(String s) {
        for (int i = 0; i < buffer.length; i++)
            buffer[i] = s;
    }

    void trail(String bg, String mv, int len) {
        glow(bg);
        for (int i = trailIndex; i < len + trailIndex; i++)
            buffer[i % buffer.length] = mv;
        trailIndex++;
    }

    public static void main(String[] args) {
        LEDSim trail = new LEDSim(20);
        for (int i = 0; i < 20; i++) {
            trail.trail("\u001B[31mO\u001B[0m", "\u001B[32mO\u001B[0m", 15);
            System.out.println(Arrays.toString(trail.buffer));
        }
    }
    
}
