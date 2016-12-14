package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

//import org.firstinspires.ftc.robotcontroller.*;

/**
 * Paul Vohs (robotics stuff)
 */

@TeleOp(name="Basis Test", group="TeleOp")
public class BasisTeleop extends OpMode
{
    public enum DriveMode // Setting constants for the drive mode (it's either forwards or backwards)
    {
        FORWARD, REVERSE
    }

    DriveMode drivemode;// Create a DriveMode variable named drivemode

    public enum ServoPosition // Setting constants for servo positions
    {
        OPEN, CLOSE
    }

    ServoPosition servoposition; // Create a ServoPosition variable

    /* Initialize controllers (DC and servo), DC motors, servos, and variables for positions and powers */

    private DcMotorController dcDriveController;
    private DcMotorController Sweeper;
    private ServoController servoController;

    private DcMotor dcLeft;
    private DcMotor dcRight;
    private DcMotor sweeper;

    private Servo servoLeft;
    private Servo servoRight;

    private float up, right, leftMotorPower, rightMotorPower;
    private double servoLeftPosition, servoRightPosition;
    private int REVERSE;
    private boolean LASTSTATE;
    //private boolean APRESSED;
    private final double THRESHOLD=0.2;

    OpticalDistanceSensor distance;  // Hardware Device Object

    @Override
    public void init() // Initialization method
    {
        drivemode=DriveMode.FORWARD; // Set the initial drive mode to be forward

        // Get the dc and servo controllers
        dcDriveController=hardwareMap.dcMotorController.get("drive_controller");
        Sweeper=hardwareMap.dcMotorController.get("Sweeper");


        // Get the left and right dc motors
        dcLeft=hardwareMap.dcMotor.get("drive_left");
        dcRight=hardwareMap.dcMotor.get("drive_right");
        sweeper=hardwareMap.dcMotor.get("sweeper");

        // Reverse the direction of the right motor (so that both motors can use positive powers to indicate forwards)
        dcLeft.setDirection(DcMotor.Direction.REVERSE);

        REVERSE=1;
        //APRESSED=false;
        LASTSTATE=false;
        dcLeft.setPower(0);
	    dcRight.setPower(0);
        leftMotorPower=0;
        rightMotorPower=0;
	// Get the left and right servos (ASSUMING THERE'S A CLAW)
        //servoLeft=hardwareMap.servo.get("servo_left");
        //servoRight=hardwareMap.servo.get("servo_right");

        // Reverse the right servo
        //servoRight.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() // This method loops
    {
        leftMotorPower=0;
        rightMotorPower=0;
        /*
        For each value (up and right), if the value of the input is greater than the threshold, set it
        to the opposite of the gamepad's value (since joystick up is negative).
        If it's not above the threshold, set it to zero, since the joystick will be meant to be at zero
        */
        if (gamepad1.a!=LASTSTATE)
        {
            //APRESSED=true;
            if (!LASTSTATE)
            {
               REVERSE*=-1;
            }
            LASTSTATE=gamepad1.a;
        }

        up=(Math.abs(gamepad1.left_stick_y)>THRESHOLD)?-gamepad1.left_stick_y:0;
        right=(Math.abs(gamepad1.left_stick_x)>THRESHOLD)?-gamepad1.left_stick_x:0;

        // If the robot needs to move forwards, the drive mode is FORWARD, or else it's REVERSE
        drivemode=(up >= 0)?DriveMode.FORWARD:DriveMode.REVERSE;

        // If the claw needs to open, the servo position is open, or else it's closed
        //servoposition=(gamepad1.right_bumper)?ServoPosition.OPEN:ServoPosition.CLOSE;

        /*
        The x and y values of the single joystick determine what each motor power needs to be
        so that the wheels turn in the right direction for the desired movement. If the power would
        exceed the maximum (or minimum) power, set them to the max or min, or else the sum of values that
        are input from the controller
        */

        // This is the toggle for the button that will be used for reversing the robot direction

        if (REVERSE==-1)
        {
            leftMotorPower = (up + right > 1) ?REVERSE: REVERSE * (up - right);
            rightMotorPower = (up - right < -1) ?REVERSE*-1: REVERSE * (up + right);
        }
        else
        {
            leftMotorPower = (up + right > 1) ? 1 : (up + right);
            rightMotorPower = (up - right < -1) ? -1 : (up - right);
        }
        switch (drivemode) // Depending on the drive mode, either have the robot's wheels move regularly or reverse them
        {
            case FORWARD:
                // Keep the motor powers the same
                break;
            case REVERSE:
                // Reverse the motors
                dcLeft.setDirection(DcMotor.Direction.REVERSE);
                dcRight.setDirection(DcMotor.Direction.FORWARD);
                break;
                // Remember that the right motor was reversed in the init()
        }
/*
        switch (servoposition) // Sets the values for the servo positions based on the controller input
        {
            case OPEN:
                // Set the servos to be open
                servoLeftPosition=servoRightPosition=0.25;
                break;
            case CLOSE:
                // Set the servos to be closed
                servoLeftPosition=servoRightPosition=0;
                break;
        }
*/
        // Set all the powers for motors and positions for servos
        //servoLeft.setPosition(servoLeftPosition);
        //servoRight.setPosition(servoRightPosition);
        dcLeft.setPower(leftMotorPower);
        dcRight.setPower(rightMotorPower);

        if (gamepad1.right_trigger == 1 || gamepad2.right_trigger == 1)
        {
            sweeper.setPower(1);
        }
        else if (gamepad1.right_bumper || gamepad2.right_bumper)
        {
            sweeper.setPower(-1);
        }
        else
        {
            sweeper.setPower(0);
        }
    }
}
