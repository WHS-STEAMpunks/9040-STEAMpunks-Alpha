package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Aditya (Blocks to Java)")
@Disabled
public class Aditya extends LinearOpMode {

  private DcMotor frontleft;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontrightAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");

    waitForStart();
    // Put initialization blocks here.
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (gamepad1.left_stick_y > 0.1) {
          frontleft.setPower(0.9);
          backleft.setPower(0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
        }
        if (gamepad1.right_stick_y > 0.1) {
          backright.setPower(-0.9);
          frontrightAsDcMotor.setPower(-0.9);
        } else {
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontleft.setPower(-0.9);
          backleft.setPower(-0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
        }
        if (gamepad1.right_stick_y < -0.1) {
          backright.setPower(0.9);
          frontrightAsDcMotor.setPower(0.9);
        } else {
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
          backleft.setPower(-0.9);
          frontleft.setPower(0.9);
          backright.setPower(-0.9);
          frontrightAsDcMotor.setPower(0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_bumper) {
          backleft.setPower(0.9);
          frontleft.setPower(-0.9);
          backright.setPower(0.9);
          frontrightAsDcMotor.setPower(-0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
