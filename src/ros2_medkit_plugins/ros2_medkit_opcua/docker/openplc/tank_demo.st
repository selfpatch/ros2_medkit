PROGRAM TankSimulation
  VAR
    TankLevel       AT %MD0  : REAL := 500.0;
    TankTemperature AT %MD1  : REAL := 25.0;
    TankPressure    AT %MD2  : REAL := 1.0;
    PumpSpeed       AT %MD3  : REAL := 0.0;
    PumpRunning     AT %MX16.0 : BOOL := FALSE;
    ValvePosition   AT %MD5  : REAL := 0.0;
    ValveOpen       AT %MX24.0 : BOOL := FALSE;
    HighTempAlarm   AT %MX24.1 : BOOL := FALSE;
    LowLevelAlarm   AT %MX24.2 : BOOL := FALSE;
    OverpressAlarm  AT %MX24.3 : BOOL := FALSE;
  END_VAR
  VAR
    flow_in  : REAL;
    flow_out : REAL;
    heat_rate : REAL;
  END_VAR

  PumpRunning := PumpSpeed > 0.5;

  IF PumpRunning THEN
    flow_in := PumpSpeed * 0.5;
  ELSE
    flow_in := 0.0;
  END_IF;

  ValveOpen := ValvePosition > 0.5;

  IF ValveOpen THEN
    flow_out := ValvePosition * 0.3 * TankLevel / 1000.0;
  ELSE
    flow_out := 0.0;
  END_IF;

  TankLevel := TankLevel + (flow_in - flow_out) * 0.1;

  IF TankLevel > 1000.0 THEN
    TankLevel := 1000.0;
  ELSIF TankLevel < 0.0 THEN
    TankLevel := 0.0;
  END_IF;

  heat_rate := 0.05;
  IF flow_in > 0.0 THEN
    heat_rate := heat_rate - flow_in * 0.002;
  END_IF;
  IF flow_out > 0.0 THEN
    heat_rate := heat_rate - flow_out * 0.001;
  END_IF;

  TankTemperature := TankTemperature + heat_rate * 0.1;

  IF TankTemperature > 120.0 THEN
    TankTemperature := 120.0;
  ELSIF TankTemperature < 0.0 THEN
    TankTemperature := 0.0;
  END_IF;

  TankPressure := 1.0 + TankLevel / 1000.0 * 2.0 + TankTemperature / 100.0 * 3.0;

  IF TankPressure > 10.0 THEN
    TankPressure := 10.0;
  ELSIF TankPressure < 0.0 THEN
    TankPressure := 0.0;
  END_IF;

  HighTempAlarm  := TankTemperature > 80.0;
  LowLevelAlarm  := TankLevel < 100.0;
  OverpressAlarm := TankPressure > 5.0;

END_PROGRAM


CONFIGURATION Config0
  RESOURCE Res0 ON PLC
    TASK MainTask(INTERVAL := T#100ms, PRIORITY := 0);
    PROGRAM Inst0 WITH MainTask : TankSimulation;
  END_RESOURCE
END_CONFIGURATION
