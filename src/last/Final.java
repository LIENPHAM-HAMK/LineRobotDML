package last;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class Final {

    static EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(MotorPort.A); 
    static EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(MotorPort.B); 

    static EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S1);
    static EV3UltrasonicSensor ultraSensor = new EV3UltrasonicSensor(SensorPort.S2);

    public static void main(String[] args) {

        SampleProvider lightSP = colorSensor.getRedMode();
        float[] lightSample = new float[lightSP.sampleSize()];

        SampleProvider distSP = ultraSensor.getDistanceMode();
        float[] dist = new float[distSP.sampleSize()];

        int baseSpeed = 180;
        float kp = 250f;
        float kd = 500f;
        float target = 0.35f;
        float filteredLight = target;
        float lastError = 0f;

        boolean foundObstacle = false;
        boolean rotatedOnce = false;
        boolean rotatedTwice = false;
        boolean searchingLine = false;

        while (true) {
            distSP.fetchSample(dist, 0);
            float distance = dist[0];

            if (!foundObstacle && distance <= 0.20f) {
                leftMotor.stop(true);
                rightMotor.stop();
                System.out.println("Obstacle detected. Stopping and starting rotation...");
                foundObstacle = true;

                leftMotor.setSpeed(100);
                rightMotor.setSpeed(100);

                leftMotor.backward();
                rightMotor.forward();
            }

            if (foundObstacle && !rotatedOnce) {
                if (distance > 0.5f) {
                    System.out.println("Object disappeared. Continuing rotation for fixed angle...");

                    Delay.msDelay(240); 

                    leftMotor.stop(true);
                    rightMotor.stop();
                    System.out.println("Second rotation finished. Start moving to find the line...");

                    
                    int rightSpeed = 150;
                    int leftSpeed = (int)(rightSpeed * 1.24);

                    leftMotor.setSpeed(leftSpeed);
                    rightMotor.setSpeed(rightSpeed);

                    leftMotor.forward();
                    rightMotor.forward();

                    rotatedOnce = true;
                    rotatedTwice = true;
                    searchingLine = true;
                }
            }

            if (searchingLine) {
                lightSP.fetchSample(lightSample, 0);
                float light = lightSample[0];

                if (Math.abs(light - target) < 0.05f) {
                    searchingLine = false;
                    foundObstacle = false;
                    rotatedOnce = false;
                    rotatedTwice = false;
                    System.out.println("Line found. Resuming normal line following...");
                }

                Delay.msDelay(10);
                continue;
            }

            if (!foundObstacle && !searchingLine) {
                lightSP.fetchSample(lightSample, 0);
                float rawLight = lightSample[0];

                filteredLight = 0.5f * filteredLight + 0.5f * rawLight;

                float error = filteredLight - target;
                float derivative = error - lastError;

                int correction = (int)(kp * error + kd * derivative);
                lastError = error;

                int leftSpeed = baseSpeed - correction;
                int rightSpeed = baseSpeed + correction;

                leftSpeed = Math.max(80, Math.min(450, leftSpeed));
                rightSpeed = Math.max(80, Math.min(450, rightSpeed));

                leftMotor.setSpeed(leftSpeed);
                rightMotor.setSpeed(rightSpeed);

                leftMotor.forward();
                rightMotor.forward();
            }

            Delay.msDelay(5);
        }
    }
}
