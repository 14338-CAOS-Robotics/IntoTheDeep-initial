package org.firstinspires.ftc.teamcode.Utilities;

public class AlphaNoiseFilter {

    double predictedCurrent;
    double innovationGain;
    double n;

    public AlphaNoiseFilter(double initialGuess, double innovationGain){
        this.predictedCurrent = initialGuess;
        this.innovationGain = innovationGain;
        this.n = 1;
    }

    public double update(double measurement){
        double innovation = measurement - predictedCurrent;
        predictedCurrent = predictedCurrent + (innovationGain * innovation);
        return predictedCurrent;
    }

    public double getPredictedState(){
        return predictedCurrent;
    }

    public void setPredictedState(double guess){
        predictedCurrent = guess;
        n = 1;
    }

    public double updateAsNFilter(double measurement){
        double innovation = measurement - predictedCurrent;
        predictedCurrent = predictedCurrent + ((1 / n) * innovation);
        n++;
        return predictedCurrent;
    }

    public void setInnovationGain(double innovationGain){
        this.innovationGain = innovationGain;
    }

}
