# FL-EHGO-LFA MPPT for Grid-Tied PV Systems

MATLAB/Simulink implementation of a soft-computing maximum power point tracking (MPPT) algorithm based on Lyapunov-fuzzy auxiliary (LFA) control within a feedback-linearization and extended high-gain observer (FL-EHGO) structure.

The repository contains the models and scripts used for processor-in-the-loop (PIL) evaluation, embedded execution-time analysis, and dc-side experimental validation.

## Overview

The proposed MPPT algorithm combines a lookup-table-based MPP current-reference generator with an FL-EHGO structure for current tracking. A Lyapunov-fuzzy auxiliary controller adjusts the closed-loop decay rate through a two-rule Tsukamoto fuzzy inference system while maintaining the decay rate within the bounds established by the Lyapunov-based design.

The proposed algorithm is evaluated against three benchmark MPPT implementations:

- PI-MPPT
- SF-MPPT
- FL-EHGO-MPPT
- Proposed FL-EHGO-LFA-based MPPT

The repository includes the 10 kW grid-tied PV system used for PIL evaluation, embedded models used for execution-time measurements, and the models used for the laboratory-scale dc-side experimental validation.

## Repository Structure

### MATLAB scripts

| File | Description |
|---|---|
| `M_PARAM_V1.m` | Main parameter and controller-design script for the 10 kW grid-tied PV system. It defines the PV array, boost converter, inverter, PI and SF controllers, FL-EHGO parameters, observer gains, LFA decay-rate bounds, and MPPT-related parameters. |
| `M_PIL_PVSISTEM_2026_V1.m` | Parameters and initialization associated with the PIL implementation of the grid-tied PV system. |
| `ModTusk_V1.m` | Implementation and visualization of the two-rule Tsukamoto fuzzy inference system used by the LFA controller, including the input/output membership functions and the resulting adaptive decay-rate mapping. |

### Main simulation and PIL models

| File | Description |
|---|---|
| `S_PARAM_V1.slx` | DC-side simulation model used for controller parametrization and evaluation. It includes the equivalent PV source, boost converter, battery-based dc-side load, and the four evaluated MPPT algorithms. |
| `S_PIL_PVSISTEM_2026_V1.slx` | Main 10 kW grid-tied PV system used for PIL evaluation. It includes the PV array, DC-DC boost converter, dc bus, three-phase inverter, output filter, electrical grid, inverter control, and the four MPPT algorithms. |

The PIL model allows the selected MPPT algorithm to be replaced by its corresponding PIL block for execution on the target processor while the remaining power system is simulated in Simulink.

### Embedded execution-time models

| File | Description |
|---|---|
| `S_ET_PI_V1.slx` | Embedded execution-time model for PI-MPPT. |
| `S_ET_SF_V1.slx` | Embedded execution-time model for SF-MPPT. |
| `S_ET_FL_EHGO_P_V1.slx` | Embedded execution-time model for FL-EHGO-MPPT. |
| `S_ET_PROP_V1.slx` | Embedded execution-time model for the proposed FL-EHGO-LFA-based MPPT algorithm. |

These models implement the corresponding MPPT algorithms on the C2806x/F28069M target. They include ADC acquisition, signal conversion/scaling, MPPT computation, and ePWM generation. GPIO signals are used to delimit the execution interval for experimental execution-time measurement.

### Experimental embedded models

| File | Description |
|---|---|
| `S_EXP_PI_V1.slx` | Embedded experimental implementation of PI-MPPT. |
| `S_EXP_SF_V1.slx` | Embedded experimental implementation of SF-MPPT. |
| `S_EXP_FL_EHGO_P_V1.slx` | Embedded experimental implementation of FL-EHGO-MPPT. |
| `S_EXP_PROP_V1.slx` | Embedded experimental implementation of the proposed FL-EHGO-LFA-based MPPT algorithm. |
| `S_EXP_PROP_FORMATO_V1.slx` | Supporting version of the proposed experimental model. |

The experimental embedded models coordinate ADC acquisition and synchronous controller execution through the ADC interrupt. The controller is executed within the basic control loop, while the LUT-based MPPT reference generation is handled separately. A Rate Transition block decouples data transmission from the synchronous control sequence.

### Experimental data acquisition and visualization

| File | Description |
|---|---|
| `S_GRAF_EXP_PI_V1.slx` | Serial data acquisition, processing, and visualization for PI-MPPT experiments. |
| `S_GRAF_EXP_SF_V1.slx` | Serial data acquisition, processing, and visualization for SF-MPPT experiments. |
| `S_GRAF_EXP_FL_EHGO_P_V1.slx` | Serial data acquisition, processing, and visualization for FL-EHGO-MPPT experiments. |
| `S_GRAF_EXP_PROP_V1.slx` | Serial data acquisition, processing, and visualization for the proposed algorithm. |

These models receive the experimental signals through serial communication and reconstruct the variables used for analysis, including PV-side voltage, MPP current reference, measured current, extracted power, and control signal.

## PIL Evaluation

The PIL evaluation is performed using a 10 kW PV array connected to the grid through a DC-DC boost converter and a three-phase DC-AC inverter.

The main system includes:

- PV array under variable irradiance
- DC-DC boost converter
- DC-bus dynamics
- Three-phase grid-connected inverter
- Output filtering
- Grid synchronization through a PLL
- dq-axis inverter control
- LUT-based MPP current-reference generation
- PI-MPPT, SF-MPPT, FL-EHGO-MPPT, and the proposed algorithm

The selected MPPT algorithm can be executed on the embedded target through PIL while the remaining grid-tied PV system is simulated in Simulink.

## Proposed FL-EHGO-LFA MPPT

The proposed structure uses the measured PV current as feedback for the FL-EHGO structure. The MPP current reference is obtained from the LUT-based MPPT stage.

The LFA controller adapts the closed-loop decay rate according to the tracking-error magnitude. A two-rule Tsukamoto fuzzy inference system maps the absolute tracking error to an admissible decay rate:

- Small tracking error -> lower decay rate
- Large tracking error -> higher decay rate

The resulting decay rate is constrained between `alpha_min` and `alpha_max`, which are determined from the selected settling-time bounds.

The adaptive decay rate is then used to update the controller and observer dynamics while preserving the conditions imposed by the Lyapunov-based design.

## Experimental Validation

The dc-side experimental implementation uses a laboratory-scale 20 W setup controlled by the F28069M board.

The experimental platform includes:

- Equivalent Thevenin source representing the PV array
- DC-DC boost converter
- PV-current measurement
- PV-voltage measurement
- F28069M digital controller
- Battery replacing the grid-side inverter and maintaining the dc-bus voltage approximately constant
- Serial communication for experimental data acquisition, logging, and monitoring

The converter operates at a switching frequency of 5 kHz, corresponding to a sampling period of 200 us.

The experimental models evaluate the response of the four MPPT algorithms under successive MPP current-reference variations.

## Embedded Implementation

The embedded architecture separates the controller execution from the LUT-based MPPT reference generation.

The ADC interrupt synchronizes:

1. ADC acquisition and scaling
2. Controller execution
3. ePWM update

The LUT-based MPPT stage is executed separately from the synchronous controller sequence. Experimental signals are transferred through a Rate Transition block before serial transmission, preventing the communication task from being included in the controller sampling sequence.

This architecture is used consistently for the evaluated MPPT algorithms.

## Execution-Time Evaluation

Dedicated `S_ET_*` models are provided to evaluate the execution time of each complete MPPT implementation on the embedded target.

GPIO signals delimit the evaluated code section, allowing its execution time to be measured externally.

The repository provides execution-time models for all four evaluated algorithms so that the computational requirements can be compared under equivalent embedded conditions.

## Software and Hardware

The repository was developed using MATLAB and Simulink and includes models targeting the Texas Instruments C2000 platform.

Main hardware used in the experimental implementation:

- Texas Instruments F28069M
- DC-DC boost converter
- Current and voltage sensing circuits
- Equivalent PV source
- Battery-based dc-side setup
- Serial communication with the host computer

Appropriate MATLAB/Simulink support packages for the C2000 target are required for embedded execution.

## Recommended Workflow

For simulation and PIL evaluation:

1. Run `M_PARAM_V1.m` to initialize the system and controller parameters.
2. Open `S_PIL_PVSISTEM_2026_V1.slx`.
3. Select the MPPT algorithm to be evaluated.
4. For PIL operation, replace the corresponding controller subsystem with its generated PIL block.
5. Run the irradiance profile and record the required PV-side and grid-side variables.

For experimental validation:

1. Open the corresponding `S_EXP_*` model.
2. Build and deploy the model to the F28069M target.
3. Open the corresponding `S_GRAF_EXP_*` model on the host computer.
4. Configure the serial communication port when required.
5. Run the experiment and acquire the transmitted variables.

## Citation

If you use the models or algorithms contained in this repository in academic work, please cite the associated paper:

> D. R. Lopez-Flores et al., "Soft-computing MPPT algorithm for a grid-tied PV system using Lyapunov-fuzzy auxiliary control within a feedback-linearization and extended high-gain observer structure: PIL and experimental validation."

Complete bibliographic information and DOI will be added after publication.

## License

Please refer to the repository license for the applicable terms of use.
