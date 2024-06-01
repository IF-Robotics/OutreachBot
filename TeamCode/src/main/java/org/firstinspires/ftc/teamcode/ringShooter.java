package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ringShooter")
public class ringShooter extends LinearOpMode {

    //instantiate objects
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor FR, FL, BR, BL;
    private DcMotor shooter, intake;
    private Servo pusher;
    private IMU imu;

    double pushPos = .5;
    boolean shooting = false;


    @Override
    public void runOpMode(){
        //timer
        timer.reset();

        //drivetrain
        FR = hardwareMap.get(DcMotor.class, "Front Right");
        FL = hardwareMap.get(DcMotor.class, "Front Left");
        BR = hardwareMap.get(DcMotor.class, "Back Right");
        BL = hardwareMap.get(DcMotor.class, "Back Left");
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);


        //accessories
        shooter = hardwareMap.get(DcMotor.class, "Shotter");
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        pusher = hardwareMap.get(Servo.class, "Pusher");
        pusher.setPosition(0.3);

        //imu
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
              RevHubOrientationOnRobot.LogoFacingDirection.UP,
              RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);

        waitForStart();

        while(opModeIsActive()){
            //drive
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double dtp = 0.3;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator * dtp;
            double backLeftPower = (y - x + rx) / denominator * dtp;
            double frontRightPower = (y - x - rx) / denominator * dtp;
            double backRightPower = (y + x - rx) / denominator * dtp;

            FL.setPower(frontLeftPower);
            BL.setPower(backLeftPower);
            FR.setPower(frontRightPower);
            BR.setPower(backRightPower);

            //intake
            if(gamepad1.right_bumper) {
                intake.setPower(1);
            }else {
                intake.setPower(0);
            }

            //shooting
            if(gamepad1.left_bumper) {
                shooter.setPower(1);
                sleep(3000);
                for(int i=1; i<4; i++) {
                    pusher.setPosition(.5);
                    sleep(500);
                    pusher.setPosition(.3);
                    sleep(500);
                }
                shooter.setPower(0);
            }


            telemetry.addData("Timer", timer.seconds());
            telemetry.update();
        }
    }

}
