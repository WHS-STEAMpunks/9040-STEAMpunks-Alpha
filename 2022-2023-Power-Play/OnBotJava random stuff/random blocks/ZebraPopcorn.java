package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ZebraPopcorn (Blocks to Java)")
@Disabled
public class ZebraPopcorn extends LinearOpMode {

  private DcMotor intake;
  private DcMotor carouselAsDcMotor;
  private DcMotor armAsDcMotor;
  private DcMotor frontleft;
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontrightAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    intake = hardwareMap.get(DcMotor.class, "intake");
    carouselAsDcMotor = hardwareMap.get(DcMotor.class, "carouselAsDcMotor");
    armAsDcMotor = hardwareMap.get(DcMotor.class, "armAsDcMotor");
    frontleft = hardwareMap.get(DcMotor.class, "front left");
    backleft = hardwareMap.get(DcMotor.class, "back left");
    backright = hardwareMap.get(DcMotor.class, "back right");
    frontrightAsDcMotor = hardwareMap.get(DcMotor.class, "frontrightAsDcMotor");

    // Put initialization blocks here.
    waitForStart();
    intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    carouselAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    armAsDcMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        if (gamepad1.right_stick_x < -0.1) {
          frontleft.setPower(0.9);
          backleft.setPower(0.9);
          backright.setPower(0.9);
          frontrightAsDcMotor.setPower(0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.1) {
          frontleft.setPower(-0.9);
          backleft.setPower(-0.9);
          backright.setPower(-0.9);
          frontrightAsDcMotor.setPower(-0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          backright.setPower(0);
          frontrightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
          frontleft.setPower(0.9);
          backleft.setPower(0.9);
          backright.setPower(-0.9);
          frontrightAsDcMotor.setPower(-0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
          frontrightAsDcMotor.setPower(0);
          backright.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontleft.setPower(-0.9);
          backleft.setPower(-0.9);
          backright.setPower(0.9);
          frontrightAsDcMotor.setPower(0.9);
        } else {
          frontleft.setPower(0);
          backleft.setPower(0);
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
        if (gamepad2.right_bumper) {
          armAsDcMotor.setPower(1);
        } else {
          armAsDcMotor.setPower(0);
        }
        if (gamepad2.left_bumper) {
          armAsDcMotor.setPower(1);
        } else {
          armAsDcMotor.setPower(0);
        }
        if (gamepad2.b) {
          intake.setPower(0.8);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.x) {
          intake.setPower(-0.8);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.a) {
          carouselAsDcMotor.setPower(1);
        } else {
          carouselAsDcMotor.setPower(0);
        }
        if (gamepad2.y) {
          carouselAsDcMotor.setPower(-1);
        } else {
          carouselAsDcMotor.setPower(0);
        }
        telemetry.update();
      }
      // Put run blocks here.
    }
  }
}
