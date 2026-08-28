# PMSM Drive — Governing Equations

Equations as implemented in `PMSM_Drive` (all-Simulink, no Simscape).
Signal flow per motor:

```
trqR ──► iq_ref ──► [iq limit] ──► current PI ──► [averaged inverter] ──► vd,vq
                                        ▲                                    │
                                     id,iq ◄──────── PMSM dq electrical ◄─────┘
                                                            │
                                     Te ──► mechanics ──► ω ──► θ ──► inverse Park ──► ia,ib,ic
```

---

## 1. PMSM stator equations (rotor dq frame)

Electrical angular speed:

$$\omega_e = p\,\omega_m$$

Stator voltage equations (state variables $i_d, i_q$):

$$\frac{di_d}{dt}=\frac{v_d-R_s i_d+\omega_e L_q i_q}{L_d}$$

$$\frac{di_q}{dt}=\frac{v_q-R_s i_q-\omega_e\left(L_d i_d+\psi_m\right)}{L_q}$$

The $\omega_e L_q i_q$ and $\omega_e L_d i_d$ terms are the cross-coupling between axes;
$\omega_e\psi_m$ is the back-EMF.

## 2. Electromagnetic torque

$$T_e=\tfrac{3}{2}\,p\left[\psi_m i_q+(L_d-L_q)\,i_d i_q\right]$$

With a **non-salient (surface-PM) machine** $L_d=L_q$, so the reluctance term vanishes:

$$T_e=\tfrac{3}{2}p\,\psi_m i_q=K_t\,i_q,\qquad K_t=\tfrac{3}{2}p\,\psi_m$$

## 3. Mechanics

$$\frac{d\omega_m}{dt}=\frac{T_e-T_{drag}-B_m\omega_m}{J}$$

$T_{drag}$ is the rotor aerodynamic torque supplied by the flight model
(in the standalone testbench $T_{drag}=k_{drag}\,\omega_m|\omega_m|$).

Rotor electrical angle:

$$\theta_e=\int p\,\omega_m\,dt$$

## 4. Three-phase currents (inverse Park + inverse Clarke)

$$i_a=i_d\cos\theta_e-i_q\sin\theta_e$$
$$i_b=i_d\cos\!\left(\theta_e-\tfrac{2\pi}{3}\right)-i_q\sin\!\left(\theta_e-\tfrac{2\pi}{3}\right)$$
$$i_c=i_d\cos\!\left(\theta_e+\tfrac{2\pi}{3}\right)-i_q\sin\!\left(\theta_e+\tfrac{2\pi}{3}\right)$$

Amplitude-invariant convention: with $i_d=0$ the phase-current peak equals $i_q$,
so $I_{rms}=i_q/\sqrt{2}$.

---

## 5. Field-oriented control

**Current references** (torque request from the flight speed PID):

$$i_q^{*}=\frac{T_{req}}{K_t},\qquad i_d^{*}=0$$

$i_d^{*}=0$ is the MTPA condition for a non-salient machine.

**Voltage-feasible current limit** — the largest $i_q$ the DC bus can actually
drive at the present speed (with $i_d=0$, neglecting $R_s i_q$):

$$V_{max}=\frac{V_{dc}}{\sqrt{3}},\qquad E_b=\omega_e\psi_m$$

$$i_{q,lim}=\min\!\left(I_{max},\ \frac{\sqrt{V_{max}^{2}-E_b^{2}}}{\omega_e L_q}\right)$$

Derived from the voltage-limit circle $\left(E_b+R_s i_q\right)^2+\left(\omega_e L_q i_q\right)^2\le V_{max}^2$.

**Current loop** — PI with pole/zero cancellation, closed-loop bandwidth $\omega_{bi}$:

$$K_{p,i}=L_d\,\omega_{bi},\qquad K_{i,i}=R_s\,\omega_{bi}$$

---

## 6. Averaged inverter

No switching is modelled (no PWM ripple); the commanded dq voltage is applied
directly, with two corrections.

**(a) dq decoupling feed-forward** — cancels the cross-coupling and back-EMF so
the PI outputs only cover the resistive drop and transients:

$$v_d^{c}=v_d^{PI}-\omega_e L_q i_q$$
$$v_q^{c}=v_q^{PI}+\omega_e\left(L_d i_d+\psi_m\right)$$

**(b) Voltage clamp, d-axis priority** — SVPWM linear range $V_{max}=V_{dc}/\sqrt3$:

$$v_d=\mathrm{sat}\!\left(v_d^{c},\ \pm V_{max}\right)$$
$$v_q=\mathrm{sat}\!\left(v_q^{c},\ \pm\sqrt{V_{max}^{2}-v_d^{2}}\right)$$

The d-axis keeps priority so flux control is retained at the voltage limit
(a common-factor scaling of $v_d,v_q$ loses $i_d$ control and the flux runs away).

## 7. DC side (power balance)

$$P_{ac}=\tfrac{3}{2}\left(v_d i_d+v_q i_q\right),\qquad i_{dc}=\frac{P_{ac}}{V_{dc}}$$

This ties the AC side to the battery: $P_{dc}=P_{ac}$ (lossless averaged converter).

## 8. Battery — SA88 pack, 1-RC equivalent circuit

$$V_{dc}=\mathrm{OCV}(SOC)-i_{dc}R_0(SOC)-V_1$$
$$\frac{dV_1}{dt}=\frac{i_{dc}}{C_1}-\frac{V_1}{R_1C_1}$$
$$SOC=SOC_0-\frac{1}{3600\,C_{pack}}\int i_{dc}\,dt$$

$\mathrm{OCV},R_0,R_1,C_1$ are measured SA88 cell look-up tables vs SOC, scaled to the
pack: $\mathrm{OCV}_{pack}=N_s\,\mathrm{OCV}_{cell}$, $R_{pack}=\frac{N_s}{N_p}R_{cell}$,
$C_{pack}=\frac{N_p}{N_s}C_{cell}$.

---

## 9. Post-processing reconstruction (full mission)

For the full mission the detailed drive is too slow, so the phase currents are
reconstructed algebraically from the logged torque and speed:

$$i_q=\frac{T_e}{K_t},\quad i_d=0,\quad \theta_e=\int p\,\omega\,dt$$
$$i_a=-i_q\sin\theta_e,\quad i_{b,c}=-i_q\sin\!\left(\theta_e\mp\tfrac{2\pi}{3}\right)$$

Energy-consistent with the shaft: $\tfrac{3}{2}E_b i_q=\tfrac{3}{2}\omega_e\psi_m i_q=T_e\omega_m$.

**Validation.** Measured on a 1.0 s takeoff run of the integrated model, with the
D1500 pair of section 10 and the 200S21P pack, by logging the drive's own $i_q$
and comparing it against $T_{drag}/K_t$ on the same run.

| Window | detailed $i_q$ | reconstructed | error |
|---|---|---|---|
| settled, t > 0.90 s | 628.5 A mean | 628.3 A | −0.16 A, **0.026 %** |
| peak over the run | 631.6 A | 631.4 A | −0.2 A |
| whole run, spin-up included | 562.7 A mean | 560.0 A | rms 10.1 A, worst −61.7 A |

The reconstruction is exact where it matters and wrong only during fast
acceleration, because $T_{drag}$ stands in for $T_e$ and the two differ by the
inertia term:

$$T_e = T_{drag} + B_m\omega + J\frac{d\omega}{dt}$$

The worst point is t = 0.010 s, 23 rpm into the spin-up at 944 rad/s², where the
drive draws 75.3 A (271 N·m) against 49 N·m of rotor drag. $J\,d\omega/dt$ accounts
for 85 N·m of the gap. The rest is not fully explained and is most likely the
four speed filters on the feedback path plus the current-loop transient, so treat
the transient error as a bound rather than a model.

None of this affects a mission figure. Hover and cruise are quasi-steady, and the
peak current agrees to 0.2 A, so both the envelope and the battery sizing that
follows from it are safe. Only low-current fast transients read low, and they set
no limit.

An earlier version of this file claimed 2.1 % from 831.9 A against 830.9 A. Those
two numbers differ by 0.12 %, not 2.1 %, and they belong to the D500-class
parameter set. The table above replaces them.

---

## 10. Parameters

Motor: two Evolito D1500 stacked per rotor, axial flux, about 80 kg, up to
2500 rpm, rated near 2880 N·m peak and 540 kW. Evolito does not publish the
electrical detail, so $R_s$, $L_d$, $L_q$, $\psi_m$ and $p$ are estimates chosen
to keep the back-EMF inside the bus up to 1500 rpm. Values are those in
`pmsm_batt_testbench_data.m`.

| Symbol | Value | Unit | Meaning |
|---|---|---|---|
| $R_s$ | 0.010 | Ω | stator resistance / phase (estimate) |
| $L_d=L_q$ | 200 | µH | stator inductance, non-salient (estimate) |
| $\psi_m$ | 0.16 | Wb | PM flux linkage (estimate) |
| $p$ | 15 | – | pole pairs (estimate) |
| $K_t=\tfrac32 p\psi_m$ | 3.6 | N·m/A | torque constant |
| $I_{max}$ | 800 | A | peak current, $K_t I_{max}$ = 2880 N·m |
| $J_m$ | 0.09 | kg·m² | shaft 0.009 plus propeller 0.08 |
| $V_{dc}$ | 690 | V | DC bus, see the note below |
| $\omega_{bi}$ | 2π·1000 | rad/s | current-loop bandwidth |
| $N_s\times N_p$ | 200 × 21 | – | SA88 pack, 690 V nominal, 220.5 Ah, 152.1 kWh |

The pack matches the flight model, which sets $N_s=200$, $N_p=21$ and a 10.5 Ah
SA88 cell. An earlier version of this file used 200 × 200 with a 3.5 Ah cell,
which was the LG placeholder the flight side replaced on 14 August 2026. That
pack held 3.2 times the capacity and a ninth of the resistance, so its bus never
sagged.

$V_{dc}$ is still a constant, which is a stopgap. The flight model measures the
pack at 824 V full and 666 V at the P4 landing peak, so a fixed 690 V neither
follows the state of charge nor reproduces the sag under load. Section 5 of
`INTEGRATION_GUIDE.md` describes the fix, which is to sum $i_{dc}$ across the four
motors into the battery block of section 8 and broadcast its terminal voltage
back as $V_{dc}$.

**Back-EMF headroom.** $E_b=\omega_e\psi_m$ against the SVPWM ceiling
$V_{max}=V_{dc}/\sqrt3$:

| Speed | $E_b$ | $V_{max}$ at 824 V | at 690 V | at 666 V |
|---|---|---|---|---|
| 1000 rpm | 251 V | 476 V | 398 V | 384 V |
| 1265 rpm (measured peak) | 318 V | 476 V | 398 V | 384 V |
| 1500 rpm (RPMMAX) | 377 V | 476 V | 398 V | 384 V |

At the measured peak rotor speed there is 66 V of headroom on a depleted pack.
At RPMMAX there is 7 V, so a mission that reaches the speed ceiling on a low pack
would need field weakening.

## 11. References (equation → source)

**PMSM dq model, torque, Park transform** (§1, §2, §4)
- R. H. Park, "Two-reaction theory of synchronous machines," *AIEE Trans.*, vol. 48, pp. 716–727, 1929. — original dq transformation.
- R. Krishnan, *Permanent Magnet Synchronous and Brushless DC Motor Drives*, CRC Press, 2010, Ch. 3–4. — standard dq voltage/torque equations for PMSM.
- D. W. Novotny and T. A. Lipo, *Vector Control and Dynamics of AC Drives*, Oxford Univ. Press, 1996.
- B. K. Bose, *Modern Power Electronics and AC Drives*, Prentice Hall, 2002, Ch. 8.

**Zero d-axis control for a non-salient machine** (§5)
- MathWorks, *PMSM Field-Oriented Control* block — current-reference strategy "Zero d-axis control" is the default for non-salient machines.
  https://www.mathworks.com/help/sps/ref/pmsmfieldorientedcontrol.html

**Voltage-limit / current-limit constraint and the feasible $i_q$** (§5)
- S. Morimoto, Y. Takeda, T. Hirasa, K. Taniguchi, "Expansion of operating limits for permanent magnet motor by current vector control considering inverter capacity," *IEEE Trans. Ind. Appl.*, vol. 26, no. 5, pp. 866–871, 1990. — **the classic derivation of the voltage-limit ellipse / current-limit circle** that our $i_{q,lim}$ comes from.
- T. M. Jahns, "Flux-weakening regime operation of an interior permanent magnet synchronous motor drive," *IEEE Trans. Ind. Appl.*, vol. IA-23, no. 3, pp. 681–689, 1987.
- M. Nicola et al., "A review about flux-weakening operating limits and control techniques for synchronous motor drives," *Energies*, vol. 15, no. 5, 1930, 2022. — modern review of the same constraints.
- MathWorks, *PMSM Constraint Curves and Their Application*.
  https://www.mathworks.com/help/mcb/gs/pmsm-constraint-curves-and-their-application.html

**dq decoupling / pre-control feed-forward** (§6a)
- L. Harnefors and H.-P. Nee, "Model-based current control of AC machines using the internal model control method," *IEEE Trans. Ind. Appl.*, vol. 34, no. 1, pp. 133–141, 1998. — basis for both the decoupling terms and the pole-cancelling PI tuning.
- T. M. Rowan and R. J. Kerkman, "A new synchronous current regulator and an analysis of current-regulated PWM inverters," *IEEE Trans. Ind. Appl.*, vol. IA-22, no. 4, pp. 678–690, 1986.
- MathWorks, *PMSM Current Controller with Pre-Control* — "pre-control voltage" is the same feed-forward.
  https://www.mathworks.com/help/sps/ref/pmsmcurrentcontrollerwithprecontrol.html

**Voltage clamp with axis prioritisation (d-axis priority)** (§6b)
- MathWorks, *PMSM Field-Oriented Control* block, parameter **"Axis prioritization": `q-axis` | `d-axis` | `d-q equivalence`** — "Prioritize or maintain ratio between d- and q-axis when the block limits voltage." Our implementation is the `d-axis` option.
- Anti-windup treatment of the same saturation: J.-M. Kim, S.-K. Sul, "Speed control of interior permanent magnet synchronous motor drive for the flux weakening operation," *IEEE Trans. Ind. Appl.*, vol. 33, no. 1, pp. 43–48, 1997.

**SVPWM linear range $V_{max}=V_{dc}/\sqrt3$** (§6b)
- H. W. van der Broeck, H.-C. Skudelny, G. V. Stanke, "Analysis and realization of a pulsewidth modulator based on voltage space vectors," *IEEE Trans. Ind. Appl.*, vol. 24, no. 1, pp. 142–150, 1988.
- D. G. Holmes and T. A. Lipo, *Pulse Width Modulation for Power Converters: Principles and Practice*, IEEE Press/Wiley, 2003, Ch. 6. — the inscribed circle of the hexagon gives peak phase voltage $0.577\,V_{dc}=V_{dc}/\sqrt3$.

**Averaged (switching-function-free) converter model** (§6, §7)
- MathWorks, *Average-Value Inverter (Three-Phase)* / SPS averaged converter documentation — power-invariant DC/AC relation $P_{dc}=P_{ac}$ used in §7.

**Battery 1-RC equivalent-circuit model** (§8)
- G. L. Plett, *Battery Management Systems, Vol. I: Battery Modeling*, Artech House, 2015, Ch. 2–3.
- M. Chen and G. A. Rincón-Mora, "Accurate electrical battery model capable of predicting runtime and I–V performance," *IEEE Trans. Energy Conversion*, vol. 21, no. 2, pp. 504–511, 2006.

## 12. Modelling assumptions

- Non-salient (surface-PM) machine: $L_d=L_q$, no reluctance torque, $i_d^*=0$.
- Averaged inverter: no PWM switching, therefore **no current ripple and no
  switching losses**; the fundamental current is exact.
- Linear magnetics: no saturation of $\psi_m$, $L_d$, $L_q$.
- Lossless DC/AC power balance (copper loss appears through $R_s$, iron loss not modelled).
