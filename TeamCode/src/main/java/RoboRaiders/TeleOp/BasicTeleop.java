package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import RoboRaiders.Robots.Pirsus;
import RoboRaiders.Utilities.Logger.Logger;

// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Basic Teleop")

public class BasicTeleop extends OpMode {


    // Create an instance of the robot and store it in Pirsus
    public Pirsus robot = new Pirsus();
    public Logger myLogger =  new Logger("TestBotTeleop");
    public Logger dtLogger = new Logger("DT");   // Drive train logger



    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialized
        robot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }


    @Override
    public void loop() {



        /**
         * very basic teleop to run all the movements manually
         */

        telemetry.addData("        Gamepad2 controls ", "as follows:");
        telemetry.addData("+-------------------------", "-------------------------+");
        telemetry.addData("| Gamepad2 right stick X: ", "lift deposit             |");
        telemetry.addData("| Gamepad2 Y button:      ", "deposit pixels           |");
        telemetry.addData("| Gamepad2 X button:      ", "flip deposit             |");
        telemetry.addData("| Gamepad2 right trigger: ", "intake in                |");
        telemetry.addData("| Gamepad2 left trigger:  ", "intake out               |");
        telemetry.addData("| Gamepad2 right bumper:  ", "lift robot (hold)        |");
        telemetry.addData("| Gamepad2 B button:      ", "drone safety off         |");
        telemetry.addData("| Gamepad2 left bumper:   ", "fire drone               |");
        telemetry.addData("+-------------------------", "-------------------------+");

    }


    public void doDrive() {
        //double autoHeading = RoboRaidersProperties.getHeading();
        // Read inverse IMU heading, as the IMU heading is CW positive

        double botHeading = robot.getHeading();

        double y = gamepad1.left_stick_y; // Remember, this is reversed!`
        double x = -gamepad1.left_stick_x; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // denominator is the largest motor power (absolute value) or 1
        // this ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double lTriggerD = gamepad1.left_trigger;
        double rTriggerD = gamepad1.right_trigger;

        // drone launch buttons
        boolean lBumper = gamepad2.left_bumper;
        boolean bButton = gamepad2.b;

        // intake
        double rTriggerG = gamepad2.right_trigger;
        double lTriggerG = gamepad2.left_trigger;

        // deposit
        boolean yButton = gamepad2.y;
        boolean xButton = gamepad2.x;
        double rStickY = gamepad2.right_stick_y;

        // lift
        boolean rBumper = gamepad2.right_bumper;


//        telemetry.addLine("Variables");
//        telemetry.addData("botHeading", String.valueOf(botHeading));
//        telemetry.addData("y", String.valueOf(y));
//        telemetry.addData("x", String.valueOf(x));
//        telemetry.addData("rx", String.valueOf(rx));
//        telemetry.addData("rotX", String.valueOf(rotX));
//        telemetry.addData("rotY", String.valueOf(rotY));
//        telemetry.addData("denominator", String.valueOf(denominator));
//        telemetry.addData("frontLeftPower", String.valueOf(frontLeftPower));
//        telemetry.addData("backLeftPower", String.valueOf(backLeftPower));
//        telemetry.addData("frontRightPower", String.valueOf(frontRightPower));
//        telemetry.addData("backRightPower", String.valueOf(backRightPower));
//        telemetry.addData("auto heading: ", RoboRaidersProperties.getHeading());

        // speed changer
        if(lTriggerD > 0.0) {
            frontLeftPower = (frontLeftPower*0.65) - (0.2 * lTriggerD);
            frontRightPower = (frontLeftPower*0.65) - (0.2 * lTriggerD);
            backLeftPower = (frontLeftPower*0.65) - (0.2 * lTriggerD);
            backRightPower = (frontLeftPower*0.65) - (0.2 * lTriggerD);
        }
        else if(rTriggerD > 0.0) {
            frontLeftPower = (frontLeftPower*0.65) + (0.2 * lTriggerD);
            frontRightPower = (frontLeftPower*0.65) + (0.2 * lTriggerD);
            backLeftPower = (frontLeftPower*0.65) + (0.2 * lTriggerD);
            backRightPower = (frontLeftPower*0.65) + (0.2 * lTriggerD);
        }
        if(rTriggerG > 0.0) {
            robot.setIntakeMotorPower(rTriggerG);
        }

        else if(lTriggerG > 0.0) {
            robot.setIntakeMotorPower(-lTriggerG);
        }




        robot.setDriveMotorPower(
                frontLeftPower*0.45,
                frontRightPower*0.45,
                backLeftPower*0.45,
                backRightPower*0.45
        );
        //               dtLogger);
    }

}