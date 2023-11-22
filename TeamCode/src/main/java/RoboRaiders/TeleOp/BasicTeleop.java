package RoboRaiders.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    // triggers for driver to adjust speed
    public double lTriggerD;
    public double rTriggerD;

    // drone launch buttons
    public boolean lBumper;
    public boolean bButton;

    // intake triggers for gunner
    public double rTriggerG;
    public double lTriggerG;

    // deposit
    public boolean yButton;
    public boolean xButton;
    public double rStickY;
    public double rStickX;

    // lift
    public double leftStickY;

    //Timer
    public ElapsedTime droneTimer;
    public boolean endGame = false;  //This checks whether we have elapsed enough time to be in endgame

    public int armMotorEncoder;
    public int targetArmMotorEncoder;

    @Override
    public void init() {

        // initialise robot and tell user that the robot is initialized
        robot.initialize(hardwareMap);
        telemetry.addData("Robot Initialized waiting your command", true);
        telemetry.update();
    }

    @Override
    public void start() {

        //Timer for drone launch safety
        droneTimer = new ElapsedTime();
        droneTimer.reset();

    }



    @Override
    public void loop() {

        lTriggerD = gamepad1.left_trigger;
        rTriggerD = gamepad1.right_trigger;

        // drone launch buttons
        lBumper = gamepad2.left_bumper;
        bButton = gamepad2.b;

        // intake
        rTriggerG = gamepad2.right_trigger;
        lTriggerG = gamepad2.left_trigger;

        // deposit
        yButton = gamepad2.y;
        xButton = gamepad2.x;
        rStickY = gamepad2.right_stick_y;
        rStickX = gamepad2.right_stick_x;


        // lift
        leftStickY = gamepad2.left_stick_y;

        //Get Encoders for arm motor to track stopping
        armMotorEncoder = robot.getArmEncoders();
        targetArmMotorEncoder = 100;



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


        if(droneTimer.seconds() >= 90.0){
            endGame = true;
        }

        // The following lines of code added by Steeeve
        // Handle the game pad inputs
        // -----------------------------------------------------------------------------------------
        // doDrive       - GP1 is used for drive train input
        // doIntake      - GP2-Right Trigger and GP2-Left Trigger are used for intake input
        // doDeposit     - GP2-Y and GP2-X are used for deposit input
        // doDroneLaunch - must be in endgame (see droneTimer and endGame variables) and
        //                 GP2-B and GP-Left Bumper are pushed together
        // doLiftRobot   - GP2-Right Bumper is used for lift input

        doDrive();            // Drive Train
        doIntake();           // Intake
        doDeposit();          // Deposit
        doDroneLaunch();      // Launch the drone - only if in end game (last 30 seconds and GP2-B and GP-Left Bumper are pushed together)
        doLiftRobot();        // Lift the robot

        // End code added by Steeeve
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




        robot.setDriveMotorPower(
                frontLeftPower*0.45,
                frontRightPower*0.45,
                backLeftPower*0.45,
                backRightPower*0.45
        );
        //               dtLogger);
    }


    public void doIntake() {

        if(rTriggerG > 0.0) {
            robot.setIntakeMotorPower(rTriggerG);
        }

        else if(lTriggerG > 0.0) {
            robot.setIntakeMotorPower(-lTriggerG);
        }

    }

    public void doDroneLaunch() {
        if (endGame && bButton && lBumper) {
            robot.fireDroneTrigger(1.0);
        }

        else {
            robot.fireDroneTrigger(0.0);
        }

    }

    public void doDeposit() {
        if(armMotorEncoder >= targetArmMotorEncoder){
            robot.armMotor.setPower(0.0);

        }
        else {
            robot.armMotor.setPower(0.5 * rStickX);
        }

        if(xButton) {
            robot.adjustBucketPosition(1.0);
            robot.resetArmMotorEncoders();
        }
        else if(yButton) {
            robot.adjustBucketPosition(0.5);
        }
    }

    public void doLiftRobot() {
        if(leftStickY > 0.0){
            robot.liftUp();
        }
        else if(leftStickY < 0.0){
            robot.liftDown();
        }

        else {
            robot.liftStop();
        }
    }



}