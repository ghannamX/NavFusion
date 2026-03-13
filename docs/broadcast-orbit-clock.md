# GPS Broadcast Orbit & Clock Computation

Reference: IS-GPS-200 (Interface Specification), Springer Handbook of GNSS Ch.3

---

## 1. Ephemeris Selection

A navigation file contains multiple ephemeris records per satellite (typically every 2 hours). For a given observation time `t`, select the record with:

- **Closest Toe** (Time of Ephemeris) to `t`
- **Within validity**: `|t - Toe| <= 2 hours` (GPS broadcast validity interval)
- **Health = 0** (satellite healthy)
- **Latest IODE** if duplicates exist at same Toe

---

## 2. Keplerian Orbit Propagation (Broadcast Ephemeris → ECEF)

Given the 16 orbital parameters from the broadcast ephemeris, compute satellite ECEF position at time `t`.

### 2.1 Time from Ephemeris Reference Epoch

```
tk = t - Toe
```

Account for GPS week rollover: if `tk > 302400`, subtract `604800`; if `tk < -302400`, add `604800`.

### 2.2 Mean Motion

```
n0 = sqrt(mu / a^3)          # computed mean motion [rad/s]
n  = n0 + delta_n             # corrected mean motion
```

Where:
- `mu = 3.986005e14 m^3/s^2` (WGS-84 gravitational parameter)
- `a = sqrtA^2` (semi-major axis from sqrt(A) in ephemeris)
- `delta_n` = mean motion difference from computed value

### 2.3 Kepler's Equation (Mean Anomaly → Eccentric Anomaly)

```
M = M0 + n * tk               # mean anomaly at time t
```

Solve iteratively for eccentric anomaly `E`:

```
E(0) = M
E(i+1) = M + e * sin(E(i))    # iterate until |E(i+1) - E(i)| < 1e-12
```

Typically converges in 5-10 iterations. This is the only iterative step in orbit computation.

### 2.4 True Anomaly

```
sin_v = (sqrt(1 - e^2) * sin(E)) / (1 - e * cos(E))
cos_v = (cos(E) - e)           / (1 - e * cos(E))
v     = atan2(sin_v, cos_v)
```

### 2.5 Argument of Latitude (with harmonic corrections)

```
phi = v + omega                # uncorrected argument of latitude

# Second-harmonic corrections
delta_u = Cus * sin(2*phi) + Cuc * cos(2*phi)   # argument of latitude correction
delta_r = Crs * sin(2*phi) + Crc * cos(2*phi)   # radius correction
delta_i = Cis * sin(2*phi) + Cic * cos(2*phi)   # inclination correction

u = phi + delta_u              # corrected argument of latitude
r = a * (1 - e * cos(E)) + delta_r              # corrected radius
i = i0 + delta_i + iDot * tk                    # corrected inclination
```

### 2.6 Positions in Orbital Plane

```
x_op = r * cos(u)             # position in orbital plane
y_op = r * sin(u)
```

### 2.7 Corrected Longitude of Ascending Node

```
Omega = Omega0 + (OmegaDot - omega_e) * tk - omega_e * Toe
```

Where `omega_e = 7.2921151467e-5 rad/s` (WGS-84 Earth rotation rate).

### 2.8 Earth-Centered Earth-Fixed (ECEF) Coordinates

```
X = x_op * cos(Omega) - y_op * cos(i) * sin(Omega)
Y = x_op * sin(Omega) + y_op * cos(i) * cos(Omega)
Z = y_op * sin(i)
```

Output: `[X, Y, Z]` in metres (WGS-84 ECEF).

---

## 3. Satellite Clock Correction

### 3.1 Clock Polynomial

```
dt_sv = af0 + af1 * (t - Toc) + af2 * (t - Toc)^2
```

Where `Toc` = Time of Clock (reference epoch for clock polynomial, from ephemeris record line 0).

### 3.2 Relativistic Correction

The satellite clock runs at different rates due to orbital eccentricity (special + general relativity combined):

```
dt_rel = F * e * sqrtA * sin(E)
```

Where:
- `F = -2 * sqrt(mu) / c^2 = -4.442807633e-10 s/m^(1/2)`
- `e` = eccentricity
- `sqrtA` = square root of semi-major axis
- `E` = eccentric anomaly (already computed in orbit step)
- `c = 299792458 m/s`

This correction oscillates with orbital period. For a typical GPS orbit (e ≈ 0.01), the amplitude is about ±7 ns (≈ 2 m range equivalent).

### 3.3 Total Group Delay (TGD)

For single-frequency L1 C/A users:

```
dt_clock = dt_sv + dt_rel - TGD
```

TGD accounts for the differential delay between the L1 and L2 signals in the satellite hardware. It is provided in the broadcast ephemeris (typically a few nanoseconds).

For dual-frequency (iono-free) users, TGD is not applied (it cancels in the L1/L2 combination).

### 3.4 Clock Correction Applied to Pseudorange

The clock correction is subtracted from the pseudorange measurement:

```
P_corrected = P_measured + c * dt_clock
```

Note the sign: a positive `dt_clock` means the satellite clock is ahead of GPS system time, so the pseudorange is too short and must be increased.

---

## 4. Transmission Time Iteration

The satellite position must be computed at the signal **transmission** time, not reception time. But transmission time depends on the geometric range, which depends on satellite position — circular dependency.

### Algorithm

```
t_tx(0) = t_rx - P / c                    # initial estimate

for iteration = 1 to 3:
    compute satellite position at t_tx(i)
    compute satellite clock correction dt_clock at t_tx(i)
    t_tx(i+1) = t_rx - P / c - dt_clock   # refine with clock correction
```

Converges in 2-3 iterations (each iteration refines by ~1 ms → ~300 m → ~1 mm).

---

## 5. Earth Rotation Correction (Sagnac Effect)

During signal travel time `tau = |r_sat - r_rx| / c`, the Earth rotates. The satellite ECEF coordinates at transmission time must be rotated to align with the ECEF frame at reception time:

```
          | cos(omega_e * tau)   sin(omega_e * tau)   0 |
R_sagnac =| -sin(omega_e * tau)  cos(omega_e * tau)   0 |
          | 0                    0                     1 |

r_sat_corrected = R_sagnac * r_sat
```

For GPS orbit altitude, `omega_e * tau ≈ 5e-9 rad`, giving a correction of ~30 m in position. This is not negligible for metre-level SPP.

---

## 6. WGS-84 Constants Summary

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Gravitational parameter | mu | 3.986005 × 10^14 | m^3/s^2 |
| Earth rotation rate | omega_e | 7.2921151467 × 10^-5 | rad/s |
| Speed of light | c | 299792458 | m/s |
| Relativistic constant | F | -4.442807633 × 10^-10 | s/m^(1/2) |
| Pi (GPS-defined) | pi | 3.1415926535898 | - |

Note: GPS uses its own defined value of pi (IS-GPS-200), not the C++ `M_PI` constant, to ensure bit-exact reproducibility across implementations.

---

## 7. Typical Magnitudes (Sanity Checks)

| Quantity | Typical Value |
|----------|--------------|
| Semi-major axis (a) | ~26,560 km |
| Orbital period | ~11 h 58 min |
| Satellite altitude | ~20,200 km |
| Geometric range (receiver to satellite) | ~20,000 – 26,000 km |
| Signal travel time | ~67 – 87 ms |
| Satellite clock bias (af0) | ~10^-5 to 10^-3 s (3 – 300 km equivalent) |
| Relativistic correction | ±7 ns (±2 m) |
| TGD | ~1-10 ns (0.3 – 3 m) |
| Sagnac correction | ~30 m |
| Earth rotation during travel | ~5 × 10^-9 rad |
