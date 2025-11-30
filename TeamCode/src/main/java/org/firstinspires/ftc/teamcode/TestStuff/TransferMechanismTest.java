package org.firstinspires.ftc.teamcode.TestStuff;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(group = "Test", name = "TransferMechanismTest")
public class TransferMechanismTest extends OpMode {

   // private DcMotor spin;
  //  private Servo kick;
    private CRServo intake;
/*
    private int revolutionValue = 2790;
    private int segmentTicks = revolutionValue / 3;

    private int segmentCount = 0;
    private int currentTarget = 0;

    private boolean aWasPressed = false;   // edge-trigger state
    private boolean busy = false;          // prevents double counts
*/
    @Override
    public void init() {
       // spin = hardwareMap.get(DcMotor.class, "ferrisWheel");
       // kick = hardwareMap.get(Servo.class, "kick");
        intake = hardwareMap.get(CRServo.class, "intake");
/*
        spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spin.setTargetPosition(0);
        spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
    }

    @Override
    public void loop() {
/*
        // --- EDGE TRIGGER DETECTION ---
        boolean aPressed = gamepad1.a && !aWasPressed; // true only once per press
        aWasPressed = gamepad1.a;

        // --- If A is pressed and the motor is NOT already moving ---
        if (aPressed && !busy) {

            busy = true;   // block further presses until done

            segmentCount++;
            currentTarget += segmentTicks;

            // --- Full revolution: reset encoder ---
            if (segmentCount > 3) {
                spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                currentTarget = 0;
                spin.setTargetPosition(0);
                spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                segmentCount = 0;
            }

            spin.setTargetPosition(currentTarget);
            spin.setPower(0.8);
        }

        // --- Detect when the motor finishes reaching its target ---
        if (busy) {
            if (!spin.isBusy()) {
                spin.setPower(0); // stop cleanly
                busy = false;     // allow next button press
            }
        }

        // kick ball out otherwise if b otherwise stay at default position
        if (gamepad1.b) {

            kick.setPosition(0.15);

        } else {

            kick.setPosition(0);

        }
*/
        // intake
        if (gamepad1.a) {

            intake.setPower(1);

        }

        //outtake
        if (gamepad1.b) {

            intake.setPower(-1);

        }

        if (gamepad1.x){

            intake.setPower(0);

        }
/*
        telemetry.addData("Busy", busy);
        telemetry.addData("Segment", segmentCount + "/3");
        telemetry.addData("Target", currentTarget);
        telemetry.addData("Position", spin.getCurrentPosition());
        telemetry.update(); */
    }
}
