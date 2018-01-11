package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * this run method handles all steps to move the correct jewel off of the platform.
 *  it handles lowering and raising the arm, turning on the LED on the color sensor, reading
 *  color from the color sensor, moving the appropriate direction to move ball off the holder
 *  and resetting the robots position on the balancing stone.
 */

public class JewelMover {

    ColorSensor colorSensor;
    Servo jewelpusher;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // settings for the lift release servo
    static final double MOVER_UP = 0.06;     // Maximum rotational position
    static final double MOVER_OUT = 0.80;     // Minimum rotational position
    static final double MOVER_STEP = 0.02;

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    int numbersteps = 0;

    // bLedOn true state of the LED.
    boolean bLedOn = true;
    // bLedOn true state of the LED.
    boolean bLedOff = false;
    /* Local OpMode members. */
    private TurnWheels turnWheels = new TurnWheels();

    boolean bMovedForward;

    // inits

    String side = null;
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public JewelMover() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap, String whichSide) {
        // save reference to HW Map
        hwMap = ahwMap;

        //what side are we on
        side = whichSide;

        // get a reference to our ColorSensor object.
        colorSensor = hwMap.colorSensor.get("color sensor");
        colorSensor.enableLed(bLedOff);
        jewelpusher = hwMap.servo.get("jewel pusher");
 //       jewelpusher.setDirection(Servo.Direction.REVERSE);
        //position the servo to Minimum position
        jewelpusher.setPosition(MOVER_UP);

        turnWheels.init(hwMap);

    }

    public void run() {

        // Set the LED on in the beginning
        colorSensor.enableLed(bLedOn);

        //move the arm out between the jewels so we can look at their colors
        numbersteps = (int)((MOVER_OUT - MOVER_UP) / MOVER_STEP);
        for (int i = 0; i <= numbersteps; i++) {
            jewelpusher.setPosition(MOVER_UP + (i * MOVER_STEP));
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }


        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if (side == "RED"){
            if(hsvValues[0] < 100 || hsvValues[0] > 300){ // red side, red ball
        //  move in the opposite direction of the color sensor
                turnWheels.encoderDrive(.3,-4,-4,10);
                bMovedForward = false;
            } else { // red side, blue ball
        //  move in the same direction of the color sensor
                turnWheels.encoderDrive(.3,4,4,10);
                bMovedForward = true;
            }
        }
        if (side == "BLUE"){
            if(hsvValues[0] < 100 || hsvValues[0] > 300){ // blue side, red ball
        //  move in the same direction of the color sensor
                turnWheels.encoderDrive(.3,4,4,10);
                bMovedForward = true;
            } else { // blue side, blue ball
        //  move in the opposite direction of the color sensor
                turnWheels.encoderDrive(.3,-4,-4,10);
                bMovedForward = false;
            }
        }
        // Set the LED off in the end
        colorSensor.enableLed(bLedOff);

        //move the arm back to the starting/home position
        numbersteps = (int)((MOVER_OUT - MOVER_UP) / MOVER_STEP);
        for (int i = 0; i <= numbersteps; i++) {
            jewelpusher.setPosition(MOVER_OUT - (i * MOVER_STEP));
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        //move back into position
        if (bMovedForward){
            turnWheels.encoderDrive(.3,-4,-4,10);
        }
        if (!bMovedForward){
            turnWheels.encoderDrive(.3,4,4,10);
        }
    }
}
