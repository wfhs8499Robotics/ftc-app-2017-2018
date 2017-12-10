package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;

/**
 * TurnWheels.
 *
 * couple of methods to move the robot around..  one for inches and one for MM.
 *
 */

public class PlaceGlyph {

    Servo leftGrabber = null; // Hardware Device Object
    Servo rightGrabber = null; // Hardware Device Object
    static final double RIGHT_MAX_POS = 0.70;     // Maximum rotational position
    static final double RIGHT_MIN_POS = 0.60;     // Minimum rotational position    static final double MAX_POS = 0.70;     // Maximum rotational position
    static final double LEFT_MAX_POS = 0.60;     // Maximum rotational position
    static final double LEFT_MIN_POS = 0.50;     // Minimum rotational position
    TurnWheels turnWheels = new TurnWheels();

    /* Constructor */
    public PlaceGlyph() {
    }

    public void init(HardwareMap hwMap) {
        leftGrabber = hwMap.servo.get("left grabber");
        rightGrabber = hwMap.servo.get("right grabber");
        leftGrabber.setDirection(Servo.Direction.REVERSE);
        //position the servo to the minimum position
        leftGrabber.setPosition(LEFT_MIN_POS);
        rightGrabber.setPosition(RIGHT_MIN_POS);
    }

    public void run(RelicRecoveryVuMark vuMark) {
        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // turn
            turnWheels.encoderDrive(.3, -4, 4, 10);
            //straight
            turnWheels.encoderDrive(.3, 14, 14, 10);
            //turn
            turnWheels.encoderDrive(.3, 4, -4, 10);
            //straight
            turnWheels.encoderDrive(.3, 6, 6, 10);
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // turn
            turnWheels.encoderDrive(.3, 4, -4, 10);
            //straight
            turnWheels.encoderDrive(.3, 14, 14, 10);
            //turn
            turnWheels.encoderDrive(.3, -4, 4, 10);
            //straight
            turnWheels.encoderDrive(.3, 6, 6, 10);
        }
        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {
            turnWheels.encoderDrive(.3, 17, 17, 10);
        }
        //position the servo to the maximum position
        leftGrabber.setPosition(LEFT_MAX_POS);
        rightGrabber.setPosition(RIGHT_MAX_POS);

        //backup
        turnWheels.encoderDrive(.3, -10, -10, 10);
        turnWheels.encoderDrive(.3, 24,-24, 10);
        // backup again
        turnWheels.encoderDrive(.3, -10, -10, 10);
    }
}