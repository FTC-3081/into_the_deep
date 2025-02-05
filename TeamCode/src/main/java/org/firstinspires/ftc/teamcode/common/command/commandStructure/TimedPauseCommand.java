package org.firstinspires.ftc.teamcode.common.command.commandStructure;

import com.qualcomm.robotcore.util.ElapsedTime;

public class TimedPauseCommand extends Command{

    double pauseTime;
    private final ElapsedTime timer;

    /**
     * Instantiates a command that pauses the sequence for some amount of time
     * @param pauseTime amount of time paused for
     */
    public TimedPauseCommand(double pauseTime){
        this.pauseTime = pauseTime;
        timer = new ElapsedTime();
    }

    public void execute(){
        if(finished) timer.reset();
        finished = timer.time() > pauseTime;
    }

}
