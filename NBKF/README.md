Features:
    9-state vector: position (x,y,z), velocity (vx,vy,vz), attitude (roll,pitch,yaw)
    Fuses IMU data (accelerometer, gyroscope, magnetometer) with GPS
    Provides position estimation (lat, lon, altitude)
    Provides velocity estimation (Vx, Vy, Vz)
    Configurable process and measurement noise
    Coordinate transformations between GPS and local ENU frame

Usage:
    Initialize with NBKF_Init()
    Call NBKF_Predict() with IMU data at high frequency
    Call NBKF_Update() when GPS data is available
    Get results with NBKF_GetOutput()