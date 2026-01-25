package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "IndividualWheelTest", group = "Test")

public class IndividualWheelTest extends OpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    public void init () {

        // Initialize motors
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Reverse left side
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

    }

    public void loop () {

        if (gamepad1.a) {

            frontLeft.setPower(-1);

        } else if (gamepad1.b) {

            frontRight.setPower(1);

        } else if (gamepad1.x) {

            backLeft.setPower(-1);

        } else if (gamepad1.y) {

            backRight.setPower(-1);

        } else {

            backRight.setPower(0);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            frontRight.setPower(0);

        }

    }

}



