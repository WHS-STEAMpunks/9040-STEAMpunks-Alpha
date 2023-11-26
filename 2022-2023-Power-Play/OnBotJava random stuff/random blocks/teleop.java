package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "teleop (Blocks to Java)")
@Disabled
public class teleop extends LinearOpMode {

  private Servo fingerAsServo;
  private DcMotor intake;
  private DcMotor flywheelAsDcMotor;
  private DcMotor frontLeftAsDcMotor;
  private DcMotor frontRightAsDcMotor;
  private DcMotor backLeftAsDcMotor;
  private DcMotor backRightAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    fingerAsServo = hardwareMap.get(Servo.class, "fingerAsServo");
    intake = hardwareMap.get(DcMotor.class, "intake");
    flywheelAsDcMotor = hardwareMap.get(DcMotor.class, "flywheelAsDcMotor");
    frontLeftAsDcMotor = hardwareMap.get(DcMotor.class, "frontLeftAsDcMotor");
    frontRightAsDcMotor = hardwareMap.get(DcMotor.class, "frontRightAsDcMotor");
    backLeftAsDcMotor = hardwareMap.get(DcMotor.class, "backLeftAsDcMotor");
    backRightAsDcMotor = hardwareMap.get(DcMotor.class, "backRightAsDcMotor");

    // Put initialization blocks here.
    fingerAsServo.setPosition(1);
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Put loop blocks here.
        if (gamepad2.a) {
          intake.setPower(0.9);
        } else {
          intake.setPower(0);
        }
        if (gamepad2.dpad_up) {
          flywheelAsDcMotor.setPower(-0.95);
        } else {
          flywheelAsDcMotor.setPower(0);
        }
        if (gamepad2.x) {
          fingerAsServo.setPosition(0.81);
        } else {
          fingerAsServo.setPosition(1);
        }
        if (gamepad1.right_bumper) {
          frontLeftAsDcMotor.setPower(-0.8);
          frontRightAsDcMotor.setPower(-0.8);
          backLeftAsDcMotor.setPower(0.8);
          backRightAsDcMotor.setPower(0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_bumper) {
          frontLeftAsDcMotor.setPower(0.8);
          frontRightAsDcMotor.setPower(0.8);
          backLeftAsDcMotor.setPower(-0.8);
          backRightAsDcMotor.setPower(-0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y < -0.1) {
          frontLeftAsDcMotor.setPower(-0.8);
          frontRightAsDcMotor.setPower(0.8);
          backLeftAsDcMotor.setPower(-0.8);
          backRightAsDcMotor.setPower(0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.left_stick_y > 0.1) {
          frontLeftAsDcMotor.setPower(0.8);
          frontRightAsDcMotor.setPower(-0.8);
          backLeftAsDcMotor.setPower(0.8);
          backRightAsDcMotor.setPower(-0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_stick_x < -0.1) {
          frontLeftAsDcMotor.setPower(0.8);
          frontRightAsDcMotor.setPower(0.8);
          backLeftAsDcMotor.setPower(0.8);
          backRightAsDcMotor.setPower(0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        if (gamepad1.right_stick_x > 0.1) {
          frontLeftAsDcMotor.setPower(-0.8);
          frontRightAsDcMotor.setPower(-0.8);
          backLeftAsDcMotor.setPower(-0.8);
          backRightAsDcMotor.setPower(-0.8);
        } else {
          frontLeftAsDcMotor.setPower(0);
          frontRightAsDcMotor.setPower(0);
          backLeftAsDcMotor.setPower(0);
          backRightAsDcMotor.setPower(0);
        }
        telemetry.update();
      }
    }
  }
}
