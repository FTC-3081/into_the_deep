package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public abstract class Command {

    protected boolean finished = true;

    /**
     * Executes the command
     */
    abstract void execute();

    final boolean isFinished(){
        return finished;
    }

}
