package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "luckythirteen")
public class luckythirteen extends LinearOpMode {

  private DcMotor carousel;
  private DcMotor frontleft;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontright;
  private DcMotor arm;
  private DcMotor intake;

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
        if (gamepad1.right_stick_x < -0.1) { // drivetrain
          frontleft.setPower(.8);
          backleft.setPower(.8);
          backright.setPower(.8);
          frontright.setPower(.8);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.1) { // reverse
          frontleft.setPower(-.8);
          backleft.setPower(-.8);
          backright.setPower(-.8);
          frontright.setPower(-.8);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
          frontleft.setPower(0.9);
          backleft.setPower(0.9);
          backright.setPower(-0.9);
          frontright.setPower(-0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          frontright.setPower(0);
          backright.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontleft.setPower(-1);
          backleft.setPower(-0.7);
          backright.setPower(0.7);
          frontright.setPower(0.7);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.left_bumper) {//strafe
          backleft.setPower(-0.7);
          frontleft.setPower(.9);
          backright.setPower(-0.7);
          frontright.setPower(0.7);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }
        if (gamepad1.right_bumper) {
          backleft.setPower(0.7);
          frontleft.setPower(-0.7);
          backright.setPower(0.7);
          frontright.setPower(-0.7);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontright.setPower(0);
        }/*
        if (gamepad2.left_bumper) {
          arm.setPower(-0.4);
        } else {
          arm.setPower(0);
        }
        if (gamepad2.right_bumper) {
          arm.setPower(-0.5);
        } else {
          arm.setPower(0);
        }
        if (gamepad2.b) { 
          intake.setPower(0.4);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.x) { 
          intake.setPower(-0.4);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.a) { // intake [in]
          carousel.setPower(.75);
        } else {
          carousel.setPower(0);
        }
        if (gamepad2.y) { // intake [out]
          carousel.setPower(-1);
        } else {
          carousel.setPower(0);
        }*/
        telemetry.update();
      }
    }
  }
}