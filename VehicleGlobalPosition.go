package main

type VehicleGlobalPosition struct {
    Timestamp uint64
    Lat float64
    Lon float64
    Alt float32
    AltEllipsoid float32
    DeltaAlt float32
    LatLonResetCounter uint8
    AltResetCounter uint8
    Eph float32
    Epv float32
    TerrainAlt float32
    TerrainAltValid bool
    DeadReckoning bool
}
