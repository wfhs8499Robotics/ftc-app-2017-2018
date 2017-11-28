package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mohai on 11/27/2017.
 */

public class JewelMover {

    ColorSensor colorSensor;
    Servo jewelpusher;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // settings for the lift release servo
    static final double LIFT_MAX_POS = 0.50;     // Maximum rotational position
    static final double LIFT_MIN_POS = 0.05;     // Minimum rotational position

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // bLedOn true state of the LED.
    boolean bLedOn = true;
    // bLedOn true state of the LED.
    boolean bLedOff = false;
    /* Local OpMode members. */

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

        jewelpusher = hwMap.servo.get("jewel pusher");
//        jewelpusher.setDirection(Servo.Direction.REVERSE);
        //position the servo to Minimum position
        jewelpusher.setPosition(LIFT_MIN_POS);

    }

    public void run() {
        //move the arm out between the jewels so we can look at their colors
        jewelpusher.setPosition(LIFT_MAX_POS);

        // Set the LED on in the beginning
        colorSensor.enableLed(bLedOn);

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if (side == "RED" && (hsvValues[0] < 100 || hsvValues[0] > 300)) { // red side, red ball
//  move in the opposite direction of the color sensor
        }
        if (side == "RED" && (hsvValues[0] > 100 || hsvValues[0] < 300)) { // red side, blue ball
//  move in the same direction of the color sensor
        }
        if (side == "BLUE" && (hsvValues[0] < 100 || hsvValues[0] > 300)) { // blue side, red ball
//  move in the same direction of the color sensor
        }
        if (side == "BLUE" && (hsvValues[0] > 100 || hsvValues[0] < 300)) { // blue side, blue ball
//  move in the opposite direction of the color sensor
        }
        // Set the LED off in the end
        colorSensor.enableLed(bLedOff);

        //move the arm back to the starting/home position
        jewelpusher.setPosition(LIFT_MIN_POS);

        //move back into position

    }
}
