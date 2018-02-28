package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    private DcMotor liftmotor = null;   // Hardware Device Object
    private LinearOpMode opMode;
    private TurnWheels turnWheels = new TurnWheels(opMode);

    /* Constructor */
    public PlaceGlyph(LinearOpMode opMode) {
        this.opMode = opMode;
    }

    public void preinit(HardwareMap hwMap) {
        leftGrabber = hwMap.servo.get("left grabber");
        rightGrabber = hwMap.servo.get("right grabber");
        leftGrabber.setDirection(Servo.Direction.REVERSE);
        leftGrabber.setPosition(99);
        rightGrabber.setPosition(99);
        turnWheels.init(hwMap);
        liftmotor = hwMap.dcMotor.get("lift");
    }

    public void init(HardwareMap hwMap) {
        if (opMode.opModeIsActive()) {
            //position the servo to the minimum position
            leftGrabber.setPosition(LEFT_MIN_POS);
            rightGrabber.setPosition(RIGHT_MIN_POS);
            liftmotor.setPower(LIFT_MAX_POWER);
        }
        try {
            sleep(500);   // optional pause after each move
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        if (opMode.opModeIsActive()) {
            liftmotor.setPower(LIFT_MIN_POWER);
        }
    }

    public void run(RelicRecoveryVuMark vuMark) {
        if (opMode.opModeIsActive()) {
            turnWheels.gyroDrive(.3, 17, turnWheels.getRobotHeading());
        }

        //position the servo to the maximum position
        liftmotor.setPower(0.00);
        if (opMode.opModeIsActive()) {
            leftGrabber.setPosition(LEFT_MAX_POS);
            rightGrabber.setPosition(RIGHT_MAX_POS);
        }
        //backup
        if (opMode.opModeIsActive()) {
            turnWheels.gyroDrive(.3, -3, turnWheels.getRobotHeading());
        }

    }
}