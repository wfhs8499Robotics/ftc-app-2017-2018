package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;

/**
 * TurnWheels.
 *
 * couple of methods to move the robot around..  one for inches and one for MM.
 *
 */

public class PlaceGlyph {

    private Servo leftGrabber = null; // Hardware Device Object
    private Servo rightGrabber = null; // Hardware Device Object
    private static final double RIGHT_MAX_POS = 0.60;     // Maximum rotational position
    private static final double RIGHT_MIN_POS = 0.39;     // Minimum rotational position    static final double MAX_POS = 0.70;     // Maximum rotational position
    private static final double LEFT_MAX_POS = 0.60;     // Maximum rotational position
    private static final double LEFT_MIN_POS = 0.42;     // Minimum rotational position
    private static final double LIFT_MAX_POWER = 0.30;
    private static final double LIFT_MIN_POWER = 0.15;
    private TurnWheels turnWheels = new TurnWheels();
    private DcMotor liftmotor = null;   // Hardware Device Object

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
        turnWheels.init(hwMap);
        liftmotor = hwMap.dcMotor.get("lift");
        liftmotor.setPower(LIFT_MAX_POWER);
        try {
            sleep(500);   // optional pause after each move
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        liftmotor.setPower(LIFT_MIN_POWER);
    }

    public void run(RelicRecoveryVuMark vuMark) {
/*        if (vuMark == RelicRecoveryVuMark.LEFT) {
            // turn
            turnWheels.left33();
            //straight
            turnWheels.gyroDrive(.3, 14, turnWheels.getRobotHeading());
                    //encoderDrive(.3, 14, 14, 10);
            //turn
            turnWheels.right33();
            //straight
            turnWheels.gyroDrive(.3, 6, turnWheels.getRobotHeading());
                    //encoderDrive(.3, 6, 6, 10);
        }
        if (vuMark == RelicRecoveryVuMark.RIGHT) {
            // turn
            turnWheels.right33();
            //straight
            turnWheels.gyroDrive(.3, 14, turnWheels.getRobotHeading());
                    //encoderDrive(.3, 14, 14, 10);
            //turn
            turnWheels.left33();
            //straight
            turnWheels.gyroDrive(.3, 6, turnWheels.getRobotHeading());
                    //encoderDrive(.3, 6, 6, 10);
        }
        if (vuMark == RelicRecoveryVuMark.CENTER || vuMark == RelicRecoveryVuMark.UNKNOWN) {*/
        turnWheels.gyroDrive(.3, 17, turnWheels.getRobotHeading());
                    //encoderDrive(.3, 17, 17, 10);
 //       }
        //position the servo to the maximum position
        liftmotor.setPower(0.00);
        leftGrabber.setPosition(LEFT_MAX_POS);
        rightGrabber.setPosition(RIGHT_MAX_POS);

        //backup
        turnWheels.gyroDrive(.3, -3, turnWheels.getRobotHeading());
        // encoderDrive(.3, -18, -18, 10);
//        if (vuMark == RelicRecoveryVuMark.LEFT) {
//            turnWheels.left180();
//        } else {
//            turnWheels.right180();
//        }
//        // backup again
//        turnWheels.gyroDrive(.25, -12, turnWheels.getRobotHeading());
//                //encoderDrive(.25, -12, -12, 10);
    }
}