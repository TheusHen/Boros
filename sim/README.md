# Boros Simulacoes (Python + MATLAB/Octave)

Este diretorio tem dois fluxos:

- `sim/flight`: simulador 2D em Python (vento, rajada, trilho, empuxo C6, paraquedas, telemetria estilo firmware e Monte Carlo).
- `sim/3d`: analise MATLAB/Octave baseada no STL (geometria, Cd/Re/Mach, estabilidade e simulacao).

## Python (`sim/flight`)

### Rodar simulacao nominal

```powershell
py -3 sim/flight/simulate.py --mass-total-g 80 --wind-profile linear --wind0-mps 1 --windtop-mps 4 --gust-sigma-mps 0.6
```

### Rodar Monte Carlo

```powershell
py -3 sim/flight/monte_carlo.py --iterations 300
```

### Testes rapidos

```powershell
py -3 sim/flight/test_no_drag.py
py -3 sim/flight/test_constant_drag.py
```

### Saidas principais

- `sim/flight/out/flight_timeseries.csv`
- `sim/flight/out/summary.json`
- `sim/flight/out/cd_table.csv`
- `sim/flight/out/firmware_like_log.csv`
- `sim/flight/out/monte_carlo/*`

## MATLAB/Octave (`sim/3d`)

### Pipeline CI-friendly (compativel com MATLAB e Octave)

```matlab
cd('sim/3d');
test_aero_limits
run_ci_sim
```

### Saidas principais

- `sim/3d/out/cd_table_matlab.csv`
- `sim/3d/out/trajectory_matlab.csv`
- `sim/3d/out/report_matlab_summary.txt`
- `sim/3d/out/report_matlab.json` (quando `jsonencode` estiver disponivel)

## CI (GitHub Actions)

- Workflow: `.github/workflows/ci.yml`
- Jobs:
  - `repo-check`
  - `firmware-build`
  - `python-sim`
  - `matlab-or-octave` (fallback automatico)

### Fallback MATLAB -> Octave

- Se existir `MLM_LICENSE_TOKEN` (ou `MATLAB_LICENSE_TOKEN`) nos secrets do repo, o CI roda MATLAB.
- Se nao existir token, o CI instala Octave e roda `run_ci_sim.m`.

### Cache usado no CI

- Toolchain ARM GCC (`actions/cache`) no job de firmware.
- Dependencias Python (`setup-python` com cache `pip`).
- Instalacao do MATLAB (`setup-matlab` com `cache: true`).
