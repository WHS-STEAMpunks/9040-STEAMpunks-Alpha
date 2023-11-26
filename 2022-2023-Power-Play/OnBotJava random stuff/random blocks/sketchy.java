package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "sketchy (Blocks to Java)")
@Disabled
public class sketchy extends LinearOpMode {

  private DcMotor testerAsDcMotor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    testerAsDcMotor = hardwareMap.get(DcMotor.class, "testerAsDcMotor");

    // Put initialization blocks here.
    testerAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    testerAsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    testerAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    testerAsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    waitForStart();
    testerAsDcMotor.setPower(0.8);
    testerAsDcMotor.setPower(0.8);
    testerAsDcMotor.setPower(0.8);
    testerAsDcMotor.setPower(0.8);
    sleep(3000);
    testerAsDcMotor.setPower(0);
    testerAsDcMotor.setPower(0);
    testerAsDcMotor.setPower(0);
    testerAsDcMotor.setPower(0);
  }
}
