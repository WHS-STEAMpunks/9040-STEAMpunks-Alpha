package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "scuffed (Blocks to Java)")
public class scuffed extends LinearOpMode {

  private DcMotor carousel;
  private DcMotor frontleft;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontright;
  private DcMotor arm;
  private DcMotor intake;
  double power = 0.1;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    carousel = hardwareMap.get(DcMotor.class, "carousel");
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");
    frontright = hardwareMap.get(DcMotor.class, "front right");
    arm = hardwareMap.get(DcMotor.class, "arm");
    intake = hardwareMap.get(DcMotor.class, "intake");

    // Put initialization blocks here.
    carousel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    waitForStart();
    
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        if (gamepad1.a) {
          power = 0.9;
        } else {
          power = 0.1;
        }
        if (gamepad1.right_stick_x < -0.1) {
          frontleft.setPower(power);
          backleft.setPower(power);
          backright.setPower(power);
          frontright.setPower(power);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.1) {
          frontleft.setPower(power*-1);
          backleft.setPower(power*-1);
          backright.setPower(power*-1);
          frontright.setPower(power*-1);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
          frontleft.setPower(power);
          backleft.setPower(power);
          backright.setPower(power);
          frontright.setPower(power);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          frontright.setPower(0);
          backright.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontleft.setPower(power*-1);
          backleft.setPower(power*-1);
          backright.setPower(power);
          frontright.setPower(power);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.left_bumper) {
          backleft.setPower(power*-1);
          frontleft.setPower(power);
          backright.setPower(power*-1);
          frontright.setPower(power);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.right_bumper) {
          backleft.setPower(power);
          frontleft.setPower(power*-1);
          backright.setPower(power);
          frontright.setPower(power*-1);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad2.left_bumper) {
          arm.setPower(-0.45);
        } else {
          arm.setPower(0);
        }
        if (gamepad2.right_bumper) {
          arm.setPower(-0.1);
        } else {
          arm.setPower(0);
        }
        if (gamepad2.b) {
          intake.setPower(1);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.x) {
          intake.setPower(-100);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.a) {
          carousel.setPower(0.8);
        } else {
          carousel.setPower(0);
        }
        if (gamepad2.y) {
          carousel.setPower(-10);
        } else {
          carousel.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
