#ifndef CAN_ENUM_H
#define CAN_ENUM_H

enum Device_ID{
    /***All Range***/
        CAN_All,
    /*************RPi range*************/
        CAN_RPi = 0x100,
    /*************Controller 1 range*************/
        CAN_STM1 = CAN_RPi + 0x200,
    /*************Controller 2 range*************/
        CAN_STM2 = CAN_STM1 + 0x80,
    /*************Controller 3 range*************/
        CAN_STM3 = CAN_STM2 + 0x80,
    /*************Controller 4 range*************/
        CAN_STM4 = CAN_STM3 + 0x80,
    /*************Controller 5 range*************/
        CAN_STM5 = CAN_STM4 + 0x80,
    /*************Controller 6 range*************/
        CAN_STM6 = CAN_STM5 + 0x80

};

enum RPiCommand{
        All,
        Heartbeat,
    //Common commands
        Init,
        DeInit,
        Error,
        EmergencyStop,
    //Robot control commands
        MovingStart,
        MovingStop,
    //GUI commands
		R_AllDataQuery,
		R_CleanPlot,
        T_CleanPlot

};


enum ControllerCommand{
        ConnectionAcknowledge = T_CleanPlot + 1,
        Disconnection,
    //Robot control commands
        MotorMoved,
        MotorStopped,
    //Controller commands
        HeartbeatRespond,
        RegulatorRatioReceiveRequest,
        RegulatorRatioReceiveAcknowledge,
        RegulatorRatioTransmitRequest,
        RegulatorRatioTransmitAcknowledge,
        MovingStartedAcknowledge,
        MovingFinishedAcknowedge,
		ToggleLockKey,
        ToggleStopDriver
};

enum ControllerData{
		R_Position = ToggleStopDriver + 1,
		R_Speed,
        T_Position,
        T_Speed,
		T_Current,

        R_PositionProportionalRatio,
        R_PositionIntegralRatio,
        R_PositionDifferentialRatio,
        R_SpeedProportionalRatio,
        R_SpeedIntegralRatio,
        R_SpeedDifferentialRatio,

        T_PositionProportionalRatio,
        T_PositionIntegralRatio,
        T_PositionDifferentialRatio,
        T_SpeedProportionalRatio,
        T_SpeedIntegralRatio,
        T_SpeedDifferentialRatio,

		T_DriverState
};

enum DriverState{
       STOP,
       START
};



#endif // CAN_ENUM_H
