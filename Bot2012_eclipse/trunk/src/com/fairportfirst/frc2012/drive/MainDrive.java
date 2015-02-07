package com.fairportfirst.frc2012.drive;

import com.fairportfirst.frc2012.structures.*;
import com.fairportfirst.frc2012.ui.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DriverStationLCD.Line;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
/**
 * The supreme central class whence all other classes are called
 * @author brennan
 */
public class MainDrive extends IterativeRobot
{
    // Statics for CANJaguar
    private static int JAG_LEFT_A = 2;
    private static int JAG_LEFT_B = 4;
    private static int JAG_RIGHT_A = 3;
    private static int JAG_RIGHT_B = 5;
    private static int JAG_SHOOTER_WHEELS_A = 6;
    private static int JAG_SHOOTER_WHEELS_B = 7;
    private static int JAG_TURRET = 8;

    // Statics for PWM (Victors)
    private static int VIC_WEDGE = 1;
    private static int VIC_LIFT = 2;
    private static int VIC_BALLPICKUP = 3;

    // Statics for Digital I/O (Limit Switches)
    private static int DIO_TURRET_CW_LIMIT = 1;
    private static int DIO_TURRET_CCW_LIMIT = 2;
    private static int DIO_WEDGE_UP_LIMIT = 3;
    private static int DIO_WEDGE_DOWN_LIMIT = 4;

    // Statics for Drive
    private static int TANK_DRIVE = 1;
    private static int TILT_DRIVE = 2;
    private static int TWIST_DRIVE = 3;
    DriveType driveType;
    private double left = 0;
    private double right = 0;

    AutoSpeed autoSpeed;
    // Statics for Shooter Wheel Speed
    private static double PERCENT_100 = 1;
    private static double PERCENT_66 = 2/3;
    private static double PERCENT_33 = 1/3;
    private static double PERCENT_0 = 0;

    DriverStationLCD dslcd;

    DriveTrain driveTrain;
    ShooterStick shooterStick;

    Lift mrLift;
    Shooter mrShooter;
    Wedge mrWedge;
    BallPickup mrPickup;

    private DigitalInput turretCwLimit;
    private DigitalInput turretCcwLimit;
    private DigitalInput wedgeUpLimit;
    private DigitalInput wedgeDownLimit;

    private SendableChooser controlChooser;
    private SendableChooser autoSpeedChooser;
    private Joystick driveStick1;
    private Joystick driveStick2;
    private Joystick gamepad;  // always the gamepad even with one joystick

    Gyro myGyro;



    /**
     * This method is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit()
    {
        NetworkTable.initialize();
        dslcd = DriverStationLCD.getInstance();
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "instantiating Bot2012");

        // Limit switches
        turretCwLimit = new DigitalInput(DIO_TURRET_CW_LIMIT);
        turretCcwLimit = new DigitalInput(DIO_TURRET_CCW_LIMIT);
        wedgeUpLimit = new DigitalInput(DIO_WEDGE_UP_LIMIT);
        wedgeDownLimit = new DigitalInput(DIO_WEDGE_DOWN_LIMIT);

        // CAN IDs  left: 2,4  right: 3,5
        driveTrain = new DriveTrain(JAG_LEFT_A, JAG_RIGHT_A, JAG_LEFT_B, JAG_RIGHT_B);

        // CAN IDs shooter wheels: 6,7  turret: 8
        mrShooter = new Shooter(JAG_SHOOTER_WHEELS_A, JAG_SHOOTER_WHEELS_B, JAG_TURRET, turretCwLimit, turretCcwLimit);

        // Victors - PWM slots on Digital Sidecar
        mrWedge = new Wedge(VIC_WEDGE, wedgeUpLimit, wedgeDownLimit);
        mrLift = new Lift(VIC_LIFT);
        mrPickup = new BallPickup(VIC_BALLPICKUP);

        // UI devices
        driveStick1 = new Joystick(1);
        driveStick2 = new Joystick(2);
        try {
            gamepad = new Joystick(3);
        } catch(NullPointerException e) {
            e.printStackTrace();
        }
        myGyro = new SendableGyro(1);

        //adds the drive options to the sendable chooser, with TankDrive as the default
        controlChooser = new SendableChooser();
        controlChooser.addDefault("Default Tank Drive", new DriveType(TANK_DRIVE));
        controlChooser.addObject("X-Y Tilt Drive", new DriveType(TILT_DRIVE));
        controlChooser.addObject("Twist Stick Drive", new DriveType(TWIST_DRIVE));
        SmartDashboard.putData("Drive Chooser", controlChooser);

        //adds the speed options to the sendable chooser, with 66% as the default
        autoSpeedChooser = new SendableChooser();
        autoSpeedChooser.addDefault("Default 66%", new AutoSpeed(PERCENT_66));
        autoSpeedChooser.addObject("100%", new AutoSpeed(PERCENT_100));
        autoSpeedChooser.addObject("33%", new AutoSpeed(PERCENT_33));
        autoSpeedChooser.addObject("0%", new AutoSpeed(PERCENT_0));
        SmartDashboard.putData("Autonomous Speed", autoSpeedChooser);

        SmartDashboard.putDouble("Gyro", myGyro.getAngle());
    }

    public void autonomousInit() {
        autoSpeed = (AutoSpeed)autoSpeedChooser.getSelected();
    }

    /**
     * This method is called periodically when the robot is in autonomous
     */
    public void autonomousPeriodic()
    {
        //implement the camera tracking code in here before you shoot

        //prepare for the shot (line up the drive train)

        // Start the shooter wheels at 66% power
        mrShooter.setWheelSpeed(autoSpeed.getSpeed());
        mrShooter.wheelsOn();

        // delay for wheel spinup
        Timer.delay(3.0);
        mrLift.up();
    }

    public void teleopInit() {
        driveType = (DriveType)controlChooser.getSelected();
    }


    /**
     * This method is called periodically when the robot is in teleop
     */
    public void teleopPeriodic()
    {
        // Move the robot
        moveRobot();

        // Set the shooter wheel speed and start the wheels
        setShooter();

        // Adjust Turret
        aimTurret();

        // Pickup or eject balls
        pickupBalls();

        // Lift/feed balls into shooter
        feedBalls();

        //add automatic code here on the side to line up the robot to shoot

        //put wedge code here.
        moveWedge();

        updateDashboard();
    }


    public void moveRobot() {
        if(driveType.getDriveType() == TANK_DRIVE) {
            left = driveStick1.getY();
            right = driveStick2.getY();
        } else if(driveType.getDriveType() == TILT_DRIVE) {
            left = roundDirectionNum(driveStick1.getY() - driveStick1.getX()) * (driveStick1.getThrottle() + 1) / 2;
            right = roundDirectionNum(-driveStick1.getY() - driveStick1.getX());
        } else if(driveType.getDriveType() == TWIST_DRIVE) {
            left = driveStick1.getX() + driveStick1.getTwist();
            right = driveStick1.getX() - driveStick1.getTwist();
        }

        driveTrain.setSpeed(left, right);
    }

    public void setShooter() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if(gamepad.getRawButton(1)) {
            mrShooter.setWheelSpeed(PERCENT_0);
        } else if(gamepad.getRawButton(2)) {
            mrShooter.setWheelSpeed(PERCENT_33);
        } else if(gamepad.getRawButton(3)) {
            mrShooter.setWheelSpeed(PERCENT_66);
        } else if(gamepad.getRawButton(4)) {
            mrShooter.setWheelSpeed(PERCENT_100);
        }
        mrShooter.wheelsOn();
    }

    /*
     *   Not used at this time...
     */
    public void setWheelMode() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

//        if(mrJoystick3.getAxisChannel(Joystick.AxisType.kNumAxis)) {
//
//        }
    }

    /**
     * Read digital (left) joystick on gamepad (X)
     */
    public void aimTurret() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        double leftGamepadStick = gamepad.getAxis(Joystick.AxisType.kX);
        if (leftGamepadStick > 0) {
        mrShooter.turretClockwise();
        }
        else {
        mrShooter.turretCounterClockwise();
        }

    }

    public void pickupBalls() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if(gamepad.getRawButton(5)) {
            mrPickup.forward();
        } else if(gamepad.getRawButton(7) && !gamepad.getRawButton(5)){
            mrPickup.reverse();
        }
    }

    public void feedBalls() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if(gamepad.getRawButton(6)) {
            mrLift.up();
        } else if(gamepad.getRawButton(8) && !gamepad.getRawButton(6)){
            mrLift.down();
        }
    }

    public void moveWedge() {
        // Readdress gamepad if single drive stick is being used
        if(driveType.getDriveType() != TANK_DRIVE) {
            gamepad = driveStick2;
        }

        if(gamepad.getRawButton(9)) {
            mrWedge.down();
        } else if(gamepad.getRawButton(10) && !gamepad.getRawButton(9)){
            mrWedge.up();
        }
    }

    public void updateDashboard()
    {
      if(driveType.getDriveType() == TANK_DRIVE) {
        dslcd.println(DriverStationLCD.Line.kMain6, 1, "Output to left side: " + driveStick1.getY());
        dslcd.println(DriverStationLCD.Line.kUser2, 1, "Output to right side: " + driveStick2.getY());
      }
      else if(driveType.getDriveType() == TILT_DRIVE) {
        dslcd.println(DriverStationLCD.Line.kMain6, 1,
        "Output to left side: " + driveStick1.getX());
        dslcd.println(DriverStationLCD.Line.kUser2, 1,
        "Output to right side: " + (driveStick1.getX()));
      }
      else if(driveType.getDriveType() == TWIST_DRIVE) {
        dslcd.println(DriverStationLCD.Line.kMain6, 1,
        "Output to left side: " + driveStick1.getX() + driveStick1.getTwist());
        dslcd.println(DriverStationLCD.Line.kUser2, 1,
        "Output to right side: " + (driveStick1.getX() - driveStick1.getTwist()));
      }
      dslcd.updateLCD();
    }

    /**
     * Used by SmartDashboard to record Drive Type selected by User
     */
    public class DriveType {
        public int driveType;
        public DriveType(int dType) {
            driveType = dType;
        }

        public int getDriveType() {
            return driveType;
        }
    }

    /**
     * Used by SmartDashboard to capture Autonomous Shooter Wheel Speed set by User
     */
    public class AutoSpeed {
        public double autoSpeed;
        public AutoSpeed(double spd) {
            autoSpeed = spd;
        }

        public double getSpeed() {
            return autoSpeed;
        }
    }

    public double roundDirectionNum(double num) {
        if (num > 1) {
            num = 1;

        }
        if (num < -1) {
            num = -1;

        }
        return num;
    }
}
