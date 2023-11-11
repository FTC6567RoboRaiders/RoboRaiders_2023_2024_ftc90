package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import RoboRaiders.Robots.IntakeBot;
import RoboRaiders.Robots.IntakeBot;
import RoboRaiders.Utilities.Logger.Logger;

// This line establishes this op mode as a teleop op mode and allows for it to be displayed
// in the drop down list on the Driver Station phone to be chosen to run.
@TeleOp (name = "Basic Teleop")

public class IntakeTest extends OpMode {


    // Create an instance of the robot and store it in Pirsus
    public IntakeBot robot = new IntakeBot();
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

        // drone launch buttons
        boolean lBumper = gamepad2.left_bumper;
        boolean bButton = gamepad2.b;

        // intake
        double rTrigger = gamepad2.right_trigger;
        double lTrigger = gamepad2.left_trigger;

        // deposit
        boolean yButton = gamepad2.y;
        boolean xButton = gamepad2.x;
        double rStickY = gamepad2.right_stick_y;

        // lift
        boolean rBumper = gamepad2.right_bumper;


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

        double intakePower = rTrigger;


    }
    /*
    param
     */
//    public void intakeMotors {
//        robot.setIntakeMotorPower(
//                robot.rIntakeMotor
//        )
//    }
}