package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by student on 11/30/16.
 */

@Autonomous(name = "Basis Autonomous Code 1", group = "Autonomous OpMode")
public class autonomous extends LinearOpMode {

    private DcMotor dcLeft;
    private DcMotor dcRight;

    public void runOpMode() throws InterruptedException {

        dcLeft = hardwareMap.dcMotor.get("drive_left");
        dcRight = hardwareMap.dcMotor.get("drive_right");

        dcLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        dcLeft.setPower(1.0d);
        dcRight.setPower(1.0d);
        sleep(2200);

        dcLeft.setPower(1.0d);
        dcRight.setPower(0);
        sleep(2200);

        /*
        dcLeft.setPower(1.0d);
        dcRight.setPower(1.0d);
        sleep(2000);

        dcLeft.setPower(1.0d);
        dcRight.setPower(0);
        sleep(1500);
        */

        dcLeft.setPower(1.0d);
        dcRight.setPower(1.0d);
        sleep(4000);


    }

}
