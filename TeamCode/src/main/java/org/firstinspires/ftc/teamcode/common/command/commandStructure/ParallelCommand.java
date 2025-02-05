package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public class ParallelCommand extends Command{

    Command[] commands;

    /**
     * Intanitates a command that runs subcommands in parallel until both end
     * @param commands the commands in the sequence
     */
    public ParallelCommand(Command... commands){
        this.commands = commands;
    }

    public void execute(){
        finished = true;
        for(Command command : commands){
            command.execute();
            finished = finished && command.isFinished();
        }
    }
}
