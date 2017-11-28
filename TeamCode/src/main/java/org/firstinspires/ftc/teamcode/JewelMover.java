package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by mohai on 11/27/2017.
 */

public class JewelMover {

    ColorSensor colorSensor;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    final float values[] = hsvValues;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;
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



    }

    public void run(){

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
    }
}

