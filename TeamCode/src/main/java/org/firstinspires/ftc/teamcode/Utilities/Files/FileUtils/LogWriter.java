package org.firstinspires.ftc.teamcode.Utilities.Files.FileUtils;

public interface LogWriter {
    void writeLine(String line);
    void stop();
    boolean isWriting();
}
