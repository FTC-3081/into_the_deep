package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public class ParallelRaceCommand extends Command{

    Command[] commands;

    /**
     * Intanitates a command that runs subcommands in parallel until 1 ends
     * @param commands the commands in the sequence
     */
    public ParallelRaceCommand(Command... commands){
        this.commands = commands;
    }

    public void execute(){
        finished = false;
        for(Command command : commands){
            command.execute();
            finished = finished || command.isFinished();
        }
    }
}
