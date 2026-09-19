package frc.robot.commands;

import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.BallClusterDetection;

public class AISetHeading extends SequentialCommandGroup{
    public AISetHeading(PhotonCamera lowAI, PhotonCamera highAI, DoubleSupplier headingSupplier, DoubleConsumer headingSetter){

        addCommands(
            new InstantCommand(() -> {
      
        var lowResults = lowAI.getAllUnreadResults();
        var highResults = highAI.getAllUnreadResults();
        LinkedList<LinkedList<Pair<Integer, Integer>>> coordsList = new LinkedList<LinkedList<Pair<Integer, Integer>>>();
        LinkedList<Double> headingsList = new LinkedList<Double>();
        if(!lowResults.isEmpty()){
          for(var result : lowResults){
            if(result.hasTargets()){
              for(var target : result.getTargets()){
                LinkedList<Pair<Integer, Integer>> box = new LinkedList<>();
                List<TargetCorner> corners = target.detectedCorners;
                for(TargetCorner corner : corners){
                  box.add(new Pair<Integer, Integer>((int)(corner.x), (int)(corner.y)));
                }
                coordsList.add(box);
                headingsList.add(target.getYaw());
              }
            }
          }
        }
        if(!highResults.isEmpty()){
          for(var result : highResults){
            if(result.hasTargets()){
              for(var target : result.getTargets()){
                LinkedList<Pair<Integer, Integer>> box = new LinkedList<>();
                List<TargetCorner> corners = target.detectedCorners;
                for(TargetCorner corner : corners){
                  box.add(new Pair<Integer, Integer>((int)(corner.x), (int)(corner.y+240)));
                }
                coordsList.add(box);
                headingsList.add(target.getYaw());
              }
            }
          }
        }
        int[][][] detections = new int[coordsList.size()][4][2];
        int i = 0;
        for(var box : coordsList){
          int j = 0;
          for(var coords : box){
            detections[i][j][0] = coords.getFirst();
            detections[i][j][1] = coords.getSecond();
            j++;
          }
          i++;
        }
        int index = BallClusterDetection.detectBallCluster(detections);
        if(index != -1){
          double heading = headingsList.get(index);
          headingSetter.accept(headingSupplier.getAsDouble()-heading);
        }
    })
        );
    }
}
