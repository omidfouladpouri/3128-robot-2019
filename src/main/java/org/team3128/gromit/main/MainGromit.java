package org.team3128.gromit.main;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.team3128.common.hardware.motor.LazyTalonSRX;
import org.team3128.common.hardware.motor.LazyVictorSPX;

import org.team3128.common.hardware.limelight.StreamMode;
import org.team3128.common.hardware.misc.Piston;
import org.team3128.common.narwhaldashboard.NarwhalDashboard;
import org.team3128.common.utility.units.Angle;
import org.team3128.common.utility.units.Length;
import org.team3128.gromit.mechanisms.Climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team3128.gromit.constants.GromitConstants;
import org.team3128.common.generics.RobotConstants;

public class MainGromit extends MainDeepSpaceRobot {
    // Pnuematics
    Piston placeholder;

    // Climber
    public Climber climber;
    public Piston climb_Piston;
    public LazyTalonSRX climbMotor;

    private CommandGroup climbCommand;

    @Override
    protected void constructHardware() {
        // Setting constants. This shouldn't be edited.
        wheelbase = GromitConstants.WHEEL_CIRCUMFERENCE;
        driveMaxSpeed = GromitConstants.DRIVE_MAX_SPEED;
        wheelCirc = GromitConstants.WHEEL_CIRCUMFERENCE;
        leftSpeedScalar = GromitConstants.LEFT_SPEED_SCALAR;
        rightSpeedScalar = GromitConstants.RIGHT_SPEED_SCALAR;
        shiftUpSpeed = GromitConstants.SHIFT_UP_SPEED;
        shiftDownSpeed = GromitConstants.SHIFT_DOWN_SPEED;

        // Pnuematics Constants
        gearshift_Piston = new Piston(GromitConstants.GEARSHIFT_SOL_A, GromitConstants.GEARSHIFT_SOL_B);
        gearshift_Piston.setPistonOn();

        climb_Piston = new Piston(GromitConstants.CLIMB_SOL_A, GromitConstants.CLIMB_SOL_B);
        climb_Piston.setPistonOff();

        hatchIntake_Piston = new Piston(GromitConstants.HATCH_INTAKE_SOL_A, GromitConstants.HATCH_INTAKE_SOL_B);

        placeholder = new Piston(GromitConstants.PLACEHOLDER_SOL_A, GromitConstants.PLACEHOLDER_SOL_B);
        placeholder.setPistonOn();

        // Setting Mechanism Constants
        liftLimitSwitch = new DigitalInput(GromitConstants.LIFT_LIMIT_SWITCH);
        liftSwitchPosition = GromitConstants.LIFT_SWITCH_POSITION;// 170;
        liftMaxVelocity = GromitConstants.LIFT_MAX_VELOCITY;

        fourBarLimitSwitch = new DigitalInput(GromitConstants.FOURBAR_LIMIT_SWITCH);
        fourBarRatio = GromitConstants.FOURBAR_RATIO;
        fourBarSwitchPosition = GromitConstants.FOURBAR_SWITCH_POSITION;
        fourBarMaxVelocity = GromitConstants.FOURBAR_MAX_VELOCITY;

        bottomLLHeight = GromitConstants.BOTTOM_LIMELIGHT_HEIGHT;
        bottomLLAngle = GromitConstants.BOTTOM_LIMELIGHT_ANGLE;

        topLLHeight = GromitConstants.TOP_LIMELIGHT_HEIGHT;
        topLLAngle = -12.0 * Angle.DEGREES;

        // Construct and Configure Drivetrain
        leftDriveLeader = new LazyTalonSRX(10);
        leftDriveFollower = new LazyVictorSPX(11);
        rightDriveLeader = new LazyTalonSRX(15);
        rightDriveFollower = new LazyVictorSPX(16);

        super.constructHardware();

        topLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);
        bottomLimelight.setStreamMode(StreamMode.DRIVER_CAMERA);

        leftDriveLeader.setInverted(InvertType.None);
        leftDriveFollower.setInverted(InvertType.FollowMaster);
        leftDriveLeader.setSensorPhase(false);

        rightDriveLeader.setInverted(InvertType.InvertMotorOutput);
        rightDriveFollower.setInverted(InvertType.FollowMaster);
        rightDriveLeader.setSensorPhase(true);

        // Create the Climber
        climbMotor = new LazyTalonSRX(40);
        Climber.initialize(climb_Piston, climbMotor);
        climber = Climber.getInstance();

        NarwhalDashboard.addButton("climb_12", (boolean down) -> {
            if (down) {
                if (climbCommand != null)
                    climbCommand.cancel();

                climbCommand = climber.new CmdClimb1to2();
                climbCommand.start();
            }
        });
        NarwhalDashboard.addButton("climb_23", (boolean down) -> {
            if (down) {
                if (climbCommand != null)
                    climbCommand.cancel();

                climbCommand = climber.new CmdClimb2to3();
                climber.new CmdClimb2to3().start();
            }
        });
        NarwhalDashboard.addButton("cancel_climb", (boolean down) -> {
            if (down) {
                if (climbCommand != null)
                    climbCommand.cancel();
                climbCommand = null;
            }
        });

        // Debug
        NarwhalDashboard.addButton("rezero_backleg", (boolean down) -> {
            if (down) {
                climbMotor.set(ControlMode.PercentOutput, -0.8);
            } else {
                climbMotor.setSelectedSensorPosition(0);
                climbMotor.set(ControlMode.PercentOutput, 0);
            }
        });

        // Lift Inverts
        liftMotorLeader.setInverted(false);
        liftMotorLeader.setSensorPhase(true);

        liftMotorFollower.setInverted(false);

        // Lift Intake Invert
        liftIntakeMotor.setInverted(false);

        // FourBar Invert
        fourBarMotor.setInverted(true);
        fourBarMotor.setSensorPhase(false);

        // Climber Invert
        climbMotor.setSensorPhase(true);
    }

    @Override
    protected void setupListeners() {
        super.setupListeners();

        // listenerLeft.nameControl(new Button(9), "ClimbPistonExtend");
        // listenerLeft.addButtonDownListener("ClimbPistonExtend", () -> {
        // climbPiston.setPistonOn();
        // });

        // listenerLeft.nameControl(new Button(10), "ClimbPistonRetract");
        // listenerLeft.addButtonDownListener("ClimbPistonRetract", () -> {
        // climbPiston.setPistonOff();
        // });

        // listenerLeft.nameControl(new Button(11), "BackLegDown");
        // listenerLeft.nameControl(new Button(12), "BackLegUp");
        // listenerLeft.addMultiListener(() -> {
        // if (listenerLeft.getButton("BackLegDown") &&
        // !listenerLeft.getButton("BackLegUp")) {
        // climbMotor.set(ControlMode.PercentOutput, +1.0);
        // }
        // else if (listenerLeft.getButton("BackLegUp") &&
        // !listenerLeft.getButton("BackLegDown")) {
        // climbMotor.set(ControlMode.PercentOutput, -1.0);
        // }
        // else {
        // climbMotor.set(ControlMode.PercentOutput, 0.0);
        // }
        // }, "BackLegDown", "BackLegUp");
    }

    @Override
    protected void updateDashboard() {
        super.updateDashboard();

        SmartDashboard.putNumber("Back Leg Position (nu)", climbMotor.getSelectedSensorPosition());
    }

    public static void main(String... args) {
        RobotBase.startRobot(MainGromit::new);
    }
}