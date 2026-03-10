بِسْمِ اللَّهِ الرَّحْمَنِ الرَّحِيمِ

 *"And He it is Who made the stars for you that you might follow the right direction thereby in the darkness of the land and the sea."* — Quran 6:97
# FGO-PPP — Factor Graph Optimization for Precise Point Positioning

A high-accuracy GNSS positioning engine implementing **Precise Point Positioning (PPP)** and **PPP-RTK** through **Factor Graph Optimization (FGO)**, with optional tight-coupling to inertial and other sensors.

---

## Project Goals

| Goal | Method |
|------|--------|
| Centimeter-level positioning | PPP / PPP-RTK with integer ambiguity resolution |
| Robust state estimation | Factor Graph Optimization (iSAM2 / sliding-window) |
| Multi-constellation support | GPS, GLONASS, Galileo, BeiDou, QZSS |
| Sensor fusion ready | IMU tight-coupling via pre-integration factors |
| Real-time & post-processing | Dual-mode pipeline |

---

## Technology Stack

| Layer | Library / Tool |
|-------|---------------|
| Language | C++20 |
| Build system | CMake >= 3.25 |
| Package manager | Conan 2 |
| Linear algebra | Eigen 3 |
| Factor graph | GTSAM 4 (or custom, TBD) |
| Unit testing | Google Test |
| Logging | spdlog |
| Utilities | fmt |

---

## Current Status

### Completed

- **Project scaffold**: CMake + Conan 2 build system, Eigen3/spdlog/fmt/GTest integration
- **Common GNSS types** (`include/common/`):
  - `SatId` — constellation + PRN packed into uint16_t, O(1) hash
  - `ObsCode` — 3-char observation code packed into uint32_t, open-ended (no enum brittleness)
  - `ObsMeasurement` — value + LLI + signal strength per observation slot
  - `Constellation` — GPS, GLONASS, Galileo, BeiDou, QZSS, SBAS, NavIC
- **Time types** (`include/time/`):
  - `GnssTime` — epoch struct with full validation (year >= 1980, leap second support)
- **RINEX 3 observation parser** (`include/input/rinex/`, `src/gnss/input/rinex/`):
  - `RinexObsParser` — stateless parser for RINEX 3.03/3.04 observation files
  - Full header parsing: all standard records including `SYS / # / OBS TYPES` with continuation lines
  - Epoch parsing: `>` marker line with column-exact field extraction per RINEX 3 spec
  - Satellite data line parsing: N x 16-char observation blocks (14-char value + LLI + SNR)
  - Constellation filtering, malformed-line tolerance, pre-allocation hints
  - `ParseResult<T>` — discriminated union (variant) for error handling without exceptions
  - Slot-indexed observation storage: O(1) access by column position, no nested maps
  - Tested with real multi-constellation RINEX 3.04 MIXED files (GPS/GLONASS/Galileo/BeiDou)
- **Example app** (`src/apps/rinex_reader/`):
  - Takes a RINEX 3 `.obs` file path and prints header, observation types, epoch summary, first-epoch detail
- **RINEX 3 navigation parser** (`include/input/rinex/`, `src/gnss/input/rinex/`):
  - `RinexNavParser` — stateless parser for RINEX 3.03/3.04 GPS navigation files
  - GPS broadcast ephemeris: all 28 Keplerian orbital + clock + metadata fields
  - Klobuchar ionospheric model coefficients (GPSA/GPSB) from IONOSPHERIC CORR header records
  - Fortran D19.12 double-precision field parsing (D/d exponent notation)
  - Non-GPS constellation records skipped with correct orbit line counts (3 for GLO/SBAS, 7 for GPS/GAL/BDS/QZSS/NavIC)
  - Constellation filtering, malformed-record tolerance, pre-allocation hints
  - Tested with TLSE (Toulouse) RINEX 3.04 GPS nav file: 211 ephemeris records parsed
- **Shared parse result types** (`include/input/rinex/rinexParseResult.hpp`):
  - `ParseResult<T>`, `ParseError`, `ParseErrorCode` shared between obs and nav parsers
- **Example app** (`src/apps/nav_reader/`):
  - Takes a RINEX 3 `.nav`/`.rnx` file path and prints header, Klobuchar coefficients, ephemeris table

### Next Steps — Road to First SPP Fix

1. **Satellite orbit computation** — Keplerian propagation from broadcast ephemeris to ECEF position at transmission time
2. **Satellite clock correction** — af0/af1/af2 polynomial + relativistic correction + TGD
3. **Coordinate frame transforms** — ECEF / LLA / ENU conversions
4. **Tropospheric correction** — Saastamoinen zenith delay + elevation mapping function
5. **Ionospheric correction** — Klobuchar model (single-frequency) or iono-free L1/L2 combination (dual-frequency)
6. **Earth rotation correction** — Sagnac effect during signal travel time
7. **Elevation/azimuth computation** — ECEF to local ENU, elevation mask
8. **SPP observation model** — pseudorange equation linearised around approximate position
9. **Weighted Least Squares solver** — iterate design matrix H, weight by elevation, converge to ECEF + receiver clock

---

## High-Level Architecture

```
+----------------------------------------------------------------------+
|                          FGO-PPP Engine                              |
|                                                                      |
|  +-------------+   +--------------+   +--------------------------+  |
|  |  Data Layer |-->|  Processing  |-->|  Factor Graph Optimizer  |  |
|  |  (Parsers)  |   |  Pipeline    |   |  (State Estimator)       |  |
|  +-------------+   +--------------+   +--------------------------+  |
|         |                 |                        |                 |
|         v                 v                        v                 |
|  +-------------+   +--------------+   +--------------------------+  |
|  |  Corrections|   |  Observation |   |  Output / Results        |  |
|  |  & Models   |   |  Models      |   |  (Position, Cov, NMEA)   |  |
|  +-------------+   +--------------+   +--------------------------+  |
+----------------------------------------------------------------------+
```

---

## Repository Layout

```
FGO-PPP/
├── CMakeLists.txt
├── conanfile.py
├── .clang-format
├── README.md
│
├── include/
│   ├── common/            # SatId, ObsCode, ObsMeasurement, Constellation
│   ├── time/              # GnssTime
│   ├── geodesy/           # CoordFrame (ECEF/ENU/LLA transforms)
│   └── input/
│       └── rinex/         # rinexObsTypes, RinexObsParser, RinexNavParser, rinexParseResult
│
├── src/
│   ├── gnss/
│   │   ├── input/         # Parser implementations
│   │   │   └── rinex/     # RinexObsParser.cpp, RinexNavParser.cpp
│   │   ├── time/          # GnssTime.cpp, CoordFrame.cpp
│   │   ├── satellite/     # Orbit & clock computation (planned)
│   │   ├── corrections/   # Atmospheric, hardware corrections (planned)
│   │   ├── preprocessing/ # Cycle slip, outlier, smoothing (planned)
│   │   ├── observation/   # Measurement models (planned)
│   │   ├── ambiguity/     # Float estimation & integer fixing (planned)
│   │   └── solution/      # Output, quality indicators (planned)
│   │
│   ├── fgo/               # Factor graph engine (planned)
│   ├── sensor_fusion/     # IMU pre-integration (planned)
│   │
│   └── apps/
│       ├── rinex_reader/  # RINEX 3 observation file reader
│       └── nav_reader/    # RINEX 3 navigation file reader
│
├── tests/                 # Google Test unit & integration tests
├── data/
│   └── rinex/
│       ├── obs/           # RINEX 3 observation files (.rnx, .obs)
│       └── nav/           # RINEX 3 navigation files (.rnx, .nav)
└── docs/                  # Algorithm documentation
```

---

## Implementation Roadmap

### Phase 1 — Foundation
- [x] CMake + Conan 2 project scaffold
- [x] Common GNSS types (SatId, ObsCode, ObsMeasurement)
- [x] GnssTime with epoch validation
- [x] RINEX 3 observation parser (3.03/3.04, multi-constellation)
- [x] RINEX 3 navigation parser (GPS broadcast ephemeris, Klobuchar iono)
- [x] Shared `ParseResult<T>` error handling (rinexParseResult.hpp)
- [ ] SP3 precise orbit parser
- [ ] Time system utilities (GPST/UTC/GST/BDT conversions)
- [ ] Coordinate frame transforms (ECEF / LLA / ENU)

### Phase 2 — SPP Baseline
- [ ] Broadcast orbit propagation (Keplerian elements to ECEF)
- [ ] Satellite clock correction (polynomial + relativistic + TGD)
- [ ] Transmission time iteration
- [ ] Earth rotation correction (Sagnac)
- [ ] Saastamoinen tropospheric model
- [ ] Klobuchar ionospheric model
- [ ] Elevation / azimuth computation
- [ ] SPP weighted least squares solver
- [ ] Basic outlier rejection

### Phase 3 — PPP Float
- [ ] SP3 + RINEX clock precise products
- [ ] ANTEX PCO/PCV corrections
- [ ] Phase wind-up, solid Earth tide
- [ ] Uncombined PPP observation model
- [ ] Factor graph integration (GTSAM or Eigen-based)
- [ ] ZTD + receiver clock as states
- [ ] Float ambiguity states
- [ ] Cycle slip detection (MW + GF)

### Phase 4 — Ambiguity Resolution (PPP-AR)
- [ ] FCB/UPD product parser
- [ ] LAMBDA integer search engine
- [ ] Wide-lane / narrow-lane cascaded fixing
- [ ] Ratio test & partial fixing
- [ ] PPP-AR validation against IGS reference stations

### Phase 5 — RTK & PPP-RTK
- [ ] Double-difference observation model
- [ ] SSR decoder (RTCM 3 SSR messages)
- [ ] Network augmentation (OSR / SSR phase biases)
- [ ] PPP-RTK fast convergence

### Phase 6 — Sensor Fusion
- [ ] IMU pre-integration factor
- [ ] GNSS/IMU tight-coupled FGO
- [ ] Sliding-window smoother (iSAM2)
- [ ] Real-time pipeline

---

## Building

```bash
# Install dependencies
conan install . --output-folder=build --build=missing

# Configure and build
cmake --preset conan-default
cmake --build build/build --config Release

# Run the RINEX observation reader
./build/build/src/apps/Release/rinex_reader <path-to-rinex3-obs-file>

# Run the RINEX navigation reader
./build/build/src/apps/Release/nav_reader <path-to-rinex3-nav-file>
```

---

## Key References

- Teunissen, P.J.G. & Montenbruck, O. (Eds.) *Springer Handbook of Global Navigation Satellite Systems*, 2017
- Forster, C. et al. "IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation", RSS 2015
- Kaplan, E. & Hegarty, C. *Understanding GPS/GNSS: Principles and Applications*, 3rd Ed.
- Dellaert, F. & Kaess, M. "Factor Graphs for Robot Perception", *Foundations and Trends in Robotics*, 2017
- Zumberge, J.F. et al. "Precise point positioning for the efficient and robust analysis of GPS data from large networks", *JGR*, 1997
- Ge, M. et al. "Resolution of GPS carrier-phase ambiguities in Precise Point Positioning with daily observations", *JoG*, 2008
