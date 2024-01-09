package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name = "rainwithane")
public class rainewithane extends LinearOpMode
{
    private Servo HiTec;
    private DcMotorEx intake;
    private DcMotorEx actuator;
    public static final double HOME = 0.15;
    public static final double RANGE = 0.8;


    @Override
    public void runOpMode()
    {
        HiTec = hardwareMap.get(Servo.class, "HiTec");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        actuator = hardwareMap.get(DcMotorEx.class, "actuator");
        actuator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        actuator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        actuator.setZeroPowerBehavior((DcMotorEx.ZeroPowerBehavior.BRAKE));
        actuator.setDirection(DcMotorEx.Direction.FORWARD);
        telemetry.addData("C4", "Ready");
        telemetry.update();
        HiTec.setPosition(HOME);
        telemetry.addData("Wifi", "Works");
        telemetry.update();
        sleep(1000);
        HiTec.setPosition(RANGE);
        telemetry.addData("Wifi", "Works");
        telemetry.update();
        sleep(1000);
        HiTec.setPosition(HOME);
        telemetry.addData("Wifi", "Works");
        telemetry.update();
        sleep(1000);
        telemetry.addData("HiTec", "Stopped");
        telemetry.update();
        waitForStart();
        while (opModeIsActive())
        {
            boolean actuatorUp = gamepad2.right_stick_y > 0.25;
            boolean actuatorDown = gamepad2.right_stick_y < -0.25;
            boolean actuatorOff = gamepad1.right_stick_y < -0.25;
            if(gamepad2.left_stick_y > 0.25) {intake.setPower(1);}
            else if (gamepad2.left_stick_y < -0.25) {intake.setPower(-1);}
            else {intake.setPower(0);}
            if (actuatorUp)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((-5.64 / ((0.5) * Math.PI)) * 537.7));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff))
                {
                    actuator.setPower(-1);
                    actuatorOff = gamepad1.right_stick_y < -0.25;
                }
                actuator.setPower(0);
            }
            else if (actuatorDown)
            {
                actuator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                actuator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                actuator.setTargetPosition((int)((5.6 / ((0.5) * Math.PI)) * 537.7));
                actuator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                while(actuator.isBusy() && !(actuatorOff))
                {
                    actuator.setPower(1);
                    actuatorOff = gamepad1.right_stick_y < -0.25;
                }
                actuator.setPower(0);
            }
            if (actuatorOff) {actuator.setPower(0);}
        }
    }
}
