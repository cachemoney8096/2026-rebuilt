package frc.robot.subsystems.indexer;

import frc.robot.Constants;

public class IndexerCal {

  public static final double INDEXER_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;
  public static final double INDEXER_STATOR_SUPPLY_CURRENT_LIMIT_AMPS = 40.0;

  public static final double INDEXER_P = 0.1;
  public static final double INDEXER_I = 0.0;
  public static final double INDEXER_D = 0.0;
  public static final double INDEXER_FF = 0.0;
  public static final double INDEXER_SPEED = -0.1; // TODO TUNE THIS

  public static final double KICKER_P = 4.0; // TODO CHECK THIS
  public static final double KICKER_I = 0.0;
  public static final double KICKER_D = 0.0;
  public static final double KICKER_FF = 0.0;
  public static final double KICKER_SPEED = -1.0; // TODO TUNE THIS
}
