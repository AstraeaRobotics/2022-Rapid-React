package frc.robot.simulation;

import java.util.Arrays;

public class LEDSim {

    private long prevTime = 0;

    private int interval = 100;

    public String[] buffer;
    int simIndex;

    LEDSim(int len) {
        buffer = new String[len];
    }

    public void glow(String s) {
        for (int i = 0; i < buffer.length; i++)
            buffer[i] = s;
    }

    public void setData() {
        System.out.println(Arrays.toString(buffer));
    }

    void trail(String bg, String mv, int len) {
        long currentTime = System.currentTimeMillis();
        if ((currentTime - prevTime) >= interval) {
            glow(bg);
            prevTime = currentTime;
            for (int i = simIndex; i < len + simIndex; i++)
                buffer[i % buffer.length] = mv;
            simIndex++;
        }
        
    }
    
    public static void main(String[] args) {
        
        LEDSim sim = new LEDSim(20);
        
        // simulates periodic
        while (true) {
            sim.trail("\u001B[31mO\u001B[0m", "\u001B[32mO\u001B[0m", 6);
            // sim.glow("\u001B[31mO\u001B[0m");
            sim.setData();
        }

    }

}
