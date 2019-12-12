package org.team3128.aramis.autonomous;

import org.team3128.common.util.enums.Direction;
import org.team3128.common.util.units.Length;

import org.team3128.common.drive.SRXTankDrive;

import edu.wpi.first.wpilibj.command.CommandGroup;


public class CmdCustom extends CommandGroup {
    public CmdCustom() {
        SRXTankDrive drive = SRXTankDrive.getInstance();

        /**
        addSequential(drive.new CmdDriveStraight(125 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(360, Direction.LEFT, 1.0, 10000));
        addSequential(drive.new CmdArcTurn(36 * Length.in, 90, Direction.LEFT, .75, 10000));
            **/
        //addSequential(drive.new CmdMoveDistance(MoveEndMode.EITHER, 100, 100, true, .75, true, 10000));
        addSequential(drive.new CmdDriveStraight(98 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(45, Direction.RIGHT, 1.0, 10000));
        addSequential(drive.new CmdDriveStraight(38 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(105, Direction.LEFT, 1.0, 10000));
        addSequential(drive.new CmdDriveStraight(58 * Length.in, .5, 10000));
        addSequential(drive.new CmdArcTurn(47 * Length.in, 90, Direction.RIGHT, .75, 10000));
        addSequential(drive.new CmdDriveStraight(37 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(45, Direction.RIGHT, 1.0, 10000));
        addSequential(drive.new CmdDriveStraight(27 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(90, Direction.RIGHT, 1.0, 10000));
        addSequential(drive.new CmdDriveStraight(21 * Length.in, .5, 10000));
        addSequential(SRXTankDrive.getInstance().new CmdInPlaceTurn(90, Direction.LEFT, 1.0, 10000));
        addSequential(drive.new CmdDriveStraight(116 * Length.in, .5, 10000));




        //addSequential(drive.new CmdMoveDistance(MoveEndMode.BOTH, -254*Length.cm, -254*Length.cm, true, 1, false, 10000));
        //drive.getLeftMotors().set(ControlMode.PercentOutput, -100);
        //drive.getRightMotors().set(ControlMode.PercentOutput, -100);
    }
}