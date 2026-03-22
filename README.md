# NEO — Corrections firmware + ROS2
## Récapitulatif des modifications
-------------------------------------------------------------
Firmware Arduino

Calibration IMU non-bloquante (machine d'état)
500000 baud
Wire timeout 80ms
Serial.flush() supprimé

ROS2

MultiThreadedExecutor 8 threads
Rates optimisés pour LiDAR 10Hz
Mutex timeout dynamique
imu_controller timer cancel/reset

Dynamixel

latency_timer 1ms sur tous les FTDI
50 Hz joint_states std dev 0.4ms
Script de reprogrammation baud

udev

Règles propres pour tous les devices
latency_timer automatique au branchement

--------------------------------------------------------------

## 📁 Arduino/

### `ArduBot.h`
| # | Modification | Ligne(s) | Raison |
|---|---|---|---|
| 1 | Ajout enum `CalibrationState` (IDLE/RUNNING/DONE) | ~60 | Machine d'état non-bloquante |
| 2 | Ajout variables internes `calibState_`, `calibMean_`, etc. | ~130 | Stockage état calibration |
| 3 | Remplacement `calibrate()` par `calibrateRequest()` | ~170 | Point d'entrée protocole série |
| 4 | Ajout `calibrationStep()` | ~200 | 1 échantillon/cycle dans spinOnce() |
| 5 | `#define CALIB_BUFFERSIZE 500` | ~62 | 500 × 5ms = 2.5s sans blocage |

### `ArduBot.cpp`
| # | Modification | Raison |
|---|---|---|
| 1 | `Serial.begin(115200)` → `Serial.begin(500000)` | Transmission 4.3× plus rapide |
| 2 | `Wire.setWireTimeout(25000)` → `Wire.setWireTimeout(80000)` | SRF10 prend 65ms, 25ms trop court |
| 3 | Ajout `calibrationStep()` dans `spinOnce()` | Collecte 1 échantillon IMU par cycle 5ms |

### `serialProtocol.cpp`
| # | Modification | Raison |
|---|---|---|
| 1 | `Serial.begin(115200)` → `Serial.begin(500000)` | Cohérence avec ArduBot.cpp |
| 2 | `Serial.flush()` supprimé dans `sendResponse()` | Bloquait ~1.7ms/réponse inutilement |
| 3 | `Serial.flush()` supprimé dans `sendNack()` | Idem |
| 4 | `case CALIBRATE_IMU` → `robot->calibrateRequest()` | Nouveau protocole non-bloquant |

---

## 📁 ROS2/

### `qboards_config.yaml`
| # | Paramètre | Avant | Après | Raison |
|---|---|---|---|---|
| 1 | `baud1` | 115200 | 500000 | Cohérence Arduino |
| 2 | `timeout1` | 3.0 | 10.0 | Calibration IMU peut durer >3s |
| 3 | `battery rate` | .0 | 1.0 | Division par zéro dans create_wall_timer |
| 4 | `base_ctrl rate` | 30.0 | 20.0 | Optimal pour LiDAR 10Hz |
| 5 | `imu_ctrl rate` | 30.0 | 20.0 | Cohérent avec base_ctrl |
| 6 | `lcd_ctrl rate` | 3.0 | 1.0 | Réduit charge QBoard1 |
| 7 | `sens_ctrl rate` | 15.0 | 5.0 | Aligné sur le LiDAR ET DIVISE PAR 2|

### `qboduino_driver.cpp`
| # | Modification | Raison |
|---|---|---|
| 1 | Mutex timeout `500ms` → `timeout1_ + 100ms` dynamique | Calibration IMU tient le mutex >500ms |

### `qbo_arduqbo.cpp`
| # | Modification | Raison |
|---|---|---|
| 1 | `SingleThreadedExecutor` → `MultiThreadedExecutor(8)` | Rate effectif 15Hz → 20Hz, un thread/cœur |

### `imu_controller.cpp`
| # | Modification | Raison |
|---|---|---|
| 1 | `timer_->cancel()` décommenté | Libère mutex QBoard1 pendant calibration |
| 2 | Boucle `while` restaurée avec sémantique `0xFF` | Polling légitimé par nouveau protocole firmware |
| 3 | `timer_->reset()` dans les deux chemins | Timer jamais redémarré si calibration réussissait |

---

## Protocole calibration non-bloquante

```
ROS2 calibrateService()
  │
  ├─ timer_->cancel()
  │
  ├─ calibrateIMU() ──► Arduino calibrateRequest()
  │                       └─ IDLE → RUNNING, répond 0xFF
  │
  ├─ while (result == 0xFF) [poll toutes les 500ms]
  │    └─ calibrateIMU() ──► calibrateRequest()
  │                            ├─ RUNNING → 0xFF (spinOnce collecte 1 sample)
  │                            └─ DONE    → 1/0, reset état
  │
  └─ timer_->reset()

Pendant la calibration (~2.5s) :
  ✅ loop() tourne normalement
  ✅ spinOnce() → PID moteurs à jour
  ✅ processSerial() → autres commandes traitées
```

---

## Budget bus QBoard1 final

```
base_ctrl   20 Hz × 5ms  = 100ms
imu_ctrl    20 Hz × 5ms  = 100ms
lcd_ctrl     1 Hz × 3ms  =   3ms
battery      1 Hz × 3ms  =   3ms
────────────────────────────────
Total QBoard1            = 206ms  → 79% de marge ✅
```
