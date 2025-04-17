package combine;

import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class combine {

    public static void main(String[] args) {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceProvider.sampleSize()];

        int fullSpeed = 400;
        int slowSpeed = 150;
        float slowDownDistance = 0.5f;
        float stopDistance = 0.2f;

        Motor.A.setSpeed(fullSpeed);
        Motor.B.setSpeed(fullSpeed);
        Motor.A.forward();
        Motor.B.forward();

        while (!Button.ESCAPE.isDown()) {
            distanceProvider.fetchSample(sample, 0);
            float distance = sample[0];

            LCD.clear();
            LCD.drawString("Distance: " + distance + " m", 0, 0);

            if (distance < stopDistance) {
                Motor.A.stop(true);
                Motor.B.stop(true);
                Motor.A.backward();
                Motor.B.forward();
                Delay.msDelay(500);
                Motor.A.setSpeed(fullSpeed);
                Motor.B.setSpeed(fullSpeed);
                Motor.A.forward();
                Motor.B.forward();
            } else if (distance < slowDownDistance) {
                Motor.A.setSpeed(slowSpeed);
                Motor.B.setSpeed(slowSpeed);
            } else {
                Motor.A.setSpeed(fullSpeed);
                Motor.B.setSpeed(fullSpeed);
            }

            Delay.msDelay(100);
        }

        ultrasonicSensor.close();
        Button.waitForAnyPress();
    }
}
