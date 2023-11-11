package RoboRaiders.Robots;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import RoboRaiders.Utilities.Logger.Logger;

public class IntakeBot {
    /* Robot Motors, Servos, CR Servos and Sensors */
    public DcMotorEx rIntakeMotor = null;
    public DcMotorEx lIntakeMotor = null;

    public Servo launchTrigger = null;

    public DcMotorEx armMotor = null;
    public Servo bucketPositioner = null;
    public Servo entrapmentServo1 = null;
    public Servo entrapmentServo2 = null;

    public BNO055IMU imu;

    /* Local OpMode Members */
    public HardwareMap hwMap = null;

    /* Public Variables */
    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    public Orientation angles;
    public double integratedZAxis;
    public double iza_lastHeading = 0.0;
    public double iza_deltaHeading;
    public float iza_newHeading;
    public Orientation iza_angles;

    public static double robotHeading;
    public boolean firstTimeCalled = true;

    /**
     * Constructor for Robot class, current does nothing but is needed since every class needs a constructor
     */
    public IntakeBot() {

    }

    /**
     * This method will initialize the robot
     *
     * @param ahwMap hardware map for the robot
     */
    public void initialize(HardwareMap ahwMap) {

        // save reference to hardware map
        hwMap = ahwMap;


        // intake motors
        rIntakeMotor = hwMap.get(DcMotorEx.class, "rIntakeMotor");
        lIntakeMotor = hwMap.get(DcMotorEx.class, "lIntakeMotor");

        // drone launch catch
        launchTrigger = hwMap.get(Servo.class, "launchTrigger");

        // intake motor and servos
        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        bucketPositioner = hwMap.get(Servo.class, "bucketPositioner");
        entrapmentServo1 = hwMap.get(Servo.class, "entrapmentServo1");
        entrapmentServo2 = hwMap.get(Servo.class, "entrapmentServo2");


        // defines the directions the motors will spin
        rIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        lIntakeMotor.setDirection(DcMotor.Direction.REVERSE);

        // have the motors on the drivetrain break here
        rIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // set all motors to zero power
        rIntakeMotor.setPower(0.0);
        lIntakeMotor.setPower(0.0);


        // set all motors to run without encoders
        rIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // Define and initialize sensors
        imu = hwMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        //parameters.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(parameters);

    }


    //**********************************************************************************************
    //
    // INTAKE METHODS
    //
    //***********************************************************************************************/

    public void runWithoutIntakeEncoders() {

        rIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lIntakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * This method will set the power for the drive motors
     *
     * @param intakePower power setting for the intake motors
     */
    public void setIntakePower(double intakePower) {

    }
}