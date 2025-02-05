package org.firstinspires.ftc.teamcode.common.command.commandStructure;

public class InstantCommand extends Command{

    private final Runnable method;

    public InstantCommand(Runnable method){
        this.method = method;
    }

    public void execute(){
        method.run();
    }
}
