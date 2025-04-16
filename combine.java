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
        Motor.A.forward();
        Motor.B.forward();
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S2);
        SampleProvider distanceProvider = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceProvider.sampleSize()];

        // Set initial motor speed
        Motor.A.setSpeed(300);
        Motor.B.setSpeed(300);

        // Define the critical stopping distance in meters (0.07 meters = 7 cm)
        float criticalDistance = 0.20f;

        while (!Button.ESCAPE.isDown()) {
            distanceProvider.fetchSample(sample, 0);
            float distance = sample[0];

            // Display distance
            LCD.clear();
            LCD.drawString("Distance: " + distance + " m", 0, 0);

            if (distance < criticalDistance) {
                Motor.A.stop(true);
                Motor.B.stop(true);

                // Spin until the distance is greater than the critical distance
                do {
                    Motor.A.backward();
                    Motor.B.forward();
                    distanceProvider.fetchSample(sample, 0);
                    distance = sample[0];
                } while (distance <= criticalDistance);

                // Continue forward motion
                Motor.A.forward();
                Motor.B.forward();
            }

            Delay.msDelay(100);
        }

        // Close the sensor
        ultrasonicSensor.close();
        Button.waitForAnyPress();
    }
}